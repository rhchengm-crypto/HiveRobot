import copy
import json
import math
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

from left_arm_fault_support import FaultSupport, BoardWatchdog, MotionStop, segment_timing
import left_arm_board_grasp as board
import chessboard_vision_v3_0_web_control as web


class Clock:
    now = 0.
    def __call__(self):
        return self.now


class FaultSupportTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.names = ['elbow', 'wrist']
        self.status = {'elbow': {'pos': -.9, 'vel': .01, 'tau': 3.2},
                       'wrist': {'pos': -2., 'vel': -.212, 'tau': -.2}}
        self.arm = mock.Mock()
        self.arm.motors = {n: n for n in self.names}
        self.arm.read_status.side_effect = lambda names: copy.deepcopy(self.status)
        self.reports = []
        self.gains = {n: {'kp': 6., 'kd': .8} for n in self.names}
        self.support = FaultSupport(self.arm, self.names, self.gains,
                                    clock=self.clock, emit=self.reports.append)
        self.support.activate(self.status)

    def test_first_net_torque_preserves_measured_load_and_targets_freeze(self):
        self.support.latch(MotionStop('velocity', {'joint': 'wrist'}))
        calls = self.arm.ctrl.controlMIT.call_args_list
        for call in calls:
            name, kp, kd, target, dq, ff = call.args
            self.assertEqual(target, self.status[name]['pos'])
            self.assertEqual(dq, 0.)
            self.assertAlmostEqual(ff-kd*self.status[name]['vel'], self.status[name]['tau'])
        self.status['elbow']['pos'] += .2
        self.clock.now = 3600.
        self.support.observe(self.status)
        self.support.step()
        self.assertEqual(self.support.targets['elbow'], -.9)
        for call in self.arm.ctrl.controlMIT.call_args_list[-2:]:
            name, _, _, _, _, ff = call.args
            self.assertEqual(ff, self.support.support_tau[name])
        self.assertGreater(self.reports[-1]['max_drift_deg']['elbow'], 10)
        self.arm.enable.assert_not_called()
        self.arm.disable.assert_not_called()
        self.arm.close.assert_not_called()

    def test_real_incident_velocity_latches_triggering_snapshot(self):
        self.support.last_commands['wrist'] = {'kp':22.,'kd':1.4,'q':-2.,'dq':0.,'tau_ff':.339}
        watchdog = BoardWatchdog(self.arm, self.support, .2, 5.)
        with self.assertRaisesRegex(MotionStop, 'wrist -0.212') as raised:
            watchdog({n: self.status[n]['pos'] for n in self.names})
        self.support.latch(raised.exception)
        self.assertEqual(self.reports[0]['diagnostics']['velocity_rad_s'], -.212)
        self.assertEqual(self.support.snapshot['wrist']['vel'], -.212)
        self.assertEqual(raised.exception.trace[-1]['last_sent_commands']['wrist']['tau_ff'], .339)

    def test_record_only_allows_large_speed_and_tracking_errors_and_keeps_logging(self):
        watchdog=board.motion_watchdog(self.arm,self.support,.2,5.,{})
        watchdog.check_every_calls=1
        self.assertTrue(watchdog.record_only)
        reference={'elbow':-.9,'wrist':-1.5}
        with mock.patch('builtins.print') as log:
            for i in range(30):
                self.clock.now=.14*i
                self.status['wrist']['vel']=.212 if i<15 else -.6
                watchdog(reference)
        self.assertFalse(self.support.latched)
        self.assertGreater(log.call_count,4)

        record=json.loads(log.call_args.args[0].split('= ',1)[1])
        self.assertEqual(record['mode'],'record-only')
        self.assertFalse(record['protection_triggered'])
        self.assertGreater(record['error_deg']['wrist']**2,25.)
        self.assertTrue(record['violations'])
        self.arm.disable.assert_not_called()
        self.arm.close.assert_not_called()

    def test_audit_timestamps_bracket_the_actual_feedback_read(self):
        watchdog = BoardWatchdog(self.arm, self.support, .3, 5., record_only=True)
        with mock.patch('left_arm_fault_support.time.time', side_effect=[100., 100.02]), \
                mock.patch('builtins.print') as output:
            watchdog({n: self.status[n]['pos'] for n in self.names})
        row = json.loads(output.call_args.args[0].split('= ', 1)[1])
        self.assertEqual(row['feedback_read_started_wall_s'], 100.)
        self.assertEqual(row['feedback_read_finished_wall_s'], 100.02)
        self.assertEqual(row['feedback_sample_monotonic_s'], self.clock.now)
        self.arm.read_status.assert_called_once_with(self.support.names)
        self.arm.disable.assert_not_called()

    def test_record_only_replays_actual_wrist_incident_without_triggering_support(self):
        path=Path(__file__).parent/'data/left_arm_v2_8_full_runs/arm_run_1793142788195_52694.txt'
        trace=next(json.loads(line.split('= ',1)[1]) for line in path.read_text(encoding='utf8').splitlines()
                   if line.startswith('board watchdog trace= '))
        watchdog=board.motion_watchdog(self.arm,self.support,.2,5.,{})
        watchdog.check_every_calls=1
        with mock.patch('builtins.print'):
            for frame in trace:
                self.clock.now=frame['time']
                for n in self.names:self.status[n]=frame['feedback'][n]
                watchdog(frame['reference_rad'])
        self.assertEqual(watchdog.velocity_warning_count['wrist'],1)
        self.assertFalse(self.support.latched)

    def test_record_only_log_failure_cannot_abort_motion(self):
        watchdog=board.motion_watchdog(self.arm,self.support,.2,5.,{})
        with mock.patch('builtins.print',side_effect=BrokenPipeError()):
            watchdog({'elbow':-.9,'wrist':-1.})
        self.assertEqual(watchdog.warning_log_failures,1)
        self.assertFalse(self.support.latched)

    def test_record_only_does_not_change_reference_speed(self):
        for delta in (.01,.1,1.,2.):
            self.assertEqual(segment_timing(delta,.2),board.motion_timing(delta,.2))

    def test_tracking_and_invalid_feedback_abort_without_corrupting_last_support(self):
        watchdog = BoardWatchdog(self.arm, self.support, .3, 5.)
        with self.assertRaisesRegex(MotionStop, 'tracking'):
            watchdog({'elbow': -.5, 'wrist': -2.})
        previous = copy.deepcopy(self.support.snapshot)
        self.status['elbow']['tau'] = float('nan')
        with self.assertRaises(ValueError):
            self.support.observe(self.status)
        self.assertEqual(self.support.snapshot, previous)

    def test_failed_io_refreshes_other_joints_and_never_reenables(self):
        self.arm.ctrl.controlMIT.side_effect = lambda name, *args: (
            (_ for _ in ()).throw(OSError('CAN unavailable')) if name == 'elbow' else None)
        self.arm.read_status.side_effect = OSError('read failed')
        self.support.latch(RuntimeError('motion error'))
        self.assertEqual(self.arm.ctrl.controlMIT.call_args_list[-1].args[0], 'wrist')
        self.assertEqual(self.reports[-1]['state'], 'degraded')
        self.assertGreater(self.reports[-1]['io_failure_count'], 0)
        self.arm.enable.assert_not_called()
        self.arm.disable.assert_not_called()
        self.arm.close.assert_not_called()

    def test_broken_log_pipe_cannot_exit_support(self):
        self.support.emit = mock.Mock(side_effect=BrokenPipeError())
        self.support.latch(RuntimeError('stop'))
        self.clock.now = 10.
        self.support.step()
        self.assertEqual(self.support.logging_failure_count, 2)
        self.assertEqual(self.arm.ctrl.controlMIT.call_count, 4)

    def test_run_has_no_duration_exit_and_explicit_interrupt_still_works(self):
        iterations = []
        def tick(seconds):
            self.clock.now += 10.
            iterations.append(seconds)
            if len(iterations) == 8:
                raise KeyboardInterrupt()
        self.support.sleep = tick
        with self.assertRaises(KeyboardInterrupt):
            self.support.run(RuntimeError('software stop'))
        self.assertGreater(self.reports[-1]['hold_elapsed_s'], 60.)
        self.assertEqual(self.reports[-1]['state'], 'support-commanding')
        self.assertFalse(self.reports[-1]['driver_enable_verified'])
        self.arm.close.assert_not_called()
        self.arm.disable.assert_not_called()

    def test_quintic_reference_keeps_margin_without_relaxing_feedback_guard(self):
        for degrees in (.01, 1, 5, 15):
            result = segment_timing(math.radians(degrees))
            self.assertLessEqual(result['reference_peak_velocity_rad_s'], .07+1e-12)
            self.assertLessEqual(result['reference_peak_acceleration_rad_s2'], .1+1e-12)
            self.assertEqual(result['feedback_velocity_limit_rad_s'], .2)
        with self.assertRaises(ValueError):
            segment_timing(float('nan'))

    def test_zero_trajectory_cli_dry_run_needs_neither_models_nor_serial(self):
        result = subprocess.run([sys.executable, board.__file__, '--test-protective-stop',
                                 '--model', 'missing.json'], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(json.loads(result.stdout)['motion_issued'])

    def test_main_injected_stop_keeps_same_arm_until_explicit_interrupt(self):
        model = mock.Mock(names=self.names)
        # Mock.names conflicts with Mock constructor; assign explicitly.
        model.names = self.names
        self.status['wrist']['vel'] = 0.
        legacy = types.SimpleNamespace(LeftArmV2=lambda: self.arm)
        def persistent_hold(supervisor, cause):
            supervisor.latch(cause)
            self.arm.close.assert_not_called()
            self.arm.disable.assert_not_called()
            self.arm.enable.assert_not_called()
            self.arm.move_targets_with_holds.assert_not_called()
            raise KeyboardInterrupt()
        here = Path(board.__file__).parent
        cfg = json.loads((here/'data/left_arm_board_grasp.json').read_text())
        with mock.patch.object(board.RigidBodyModel, 'load', return_value=model), \
             mock.patch.object(board, 'load_board_config', return_value=cfg), \
             mock.patch.dict(sys.modules, {'left_arm_v2_6': legacy}), \
             mock.patch('left_arm_general_motion.load_configuration'), \
             mock.patch.object(board.FaultSupport, 'run', persistent_hold), \
             mock.patch.object(board.subprocess, 'run') as post_hold, \
             mock.patch.dict(board.os.environ, {}, clear=False):
            with self.assertRaises(KeyboardInterrupt):
                board.main(['--execute', '--test-protective-stop'])
        self.arm.close.assert_called_once()  # Only after operator interruption.
        post_hold.assert_not_called()

    def test_web_reports_latched_support_as_fault_still_running(self):
        with tempfile.TemporaryDirectory() as directory:
            state = web.OperationalRunState(str(Path(directory)/'runs.jsonl'), web.build_parser().parse_args([]))
            state.current = {'action': 'dynamics:board-grasp', 'running': True, 'ok': True,
                'stdout': 'board protective hold= '+json.dumps({'state': 'holding', 'reason': 'velocity'})+'\n'}
            snapshot = state.snapshot()
        self.assertFalse(snapshot['running'])
        self.assertTrue(snapshot['support_running'])
        self.assertFalse(snapshot['ok'])
        self.assertEqual(snapshot['control_state'], 'protective-hold')
        self.assertIsNone(snapshot['post_move_model_hold']['timeout_seconds'])

    def test_web_hold_exit_snapshot_is_consistent_before_waiter_annotation(self):
        with tempfile.TemporaryDirectory() as directory:
            state = web.OperationalRunState(str(Path(directory)/'runs.jsonl'), web.build_parser().parse_args([]))
            state.last = {'action':'dynamics:model-hold-after:home','running':False,'ok':True,
                          'returncode':0,'post_move_model_hold':{'state':'running'}}
            self.assertEqual(state.snapshot()['post_move_model_hold']['state'], 'complete')

    def test_transport_exception_still_routes_to_support_without_disable(self):
        model = mock.Mock()
        model.names = self.names
        self.status['wrist']['vel'] = 0.
        self.arm.positions.return_value = {n: row['pos'] for n, row in self.status.items()}
        legacy = types.SimpleNamespace(LeftArmV2=lambda: self.arm, HOME_GAINS=self.gains)
        row = {'encoder_rad': dict(self.arm.positions.return_value), 'stage': 'source_above',
               'target_board_mm': [137.5, 192.5, 160.]}
        route = {'stage': 'above', 'waypoints': [row, copy.deepcopy(row)]}
        def move(*args, **kwargs):
            self.status['wrist']['vel'] = -.51
            kwargs['watchdog'](row['encoder_rad'])
            raise OSError('simulated transport failure')
        self.arm.move_targets_with_holds.side_effect = move
        observed = []
        def persistent_hold(supervisor, cause):
            supervisor.latch(cause)
            self.arm.close.assert_not_called()
            self.arm.disable.assert_not_called()
            self.assertEqual(self.arm.move_targets_with_holds.call_count, 1)
            self.assertEqual(supervisor.snapshot['wrist']['vel'], -.51)
            self.assertIsInstance(cause, OSError)
            observed.append(cause)
            raise KeyboardInterrupt()
        here = Path(board.__file__).parent
        cfg = json.loads((here/'data/left_arm_board_grasp.json').read_text())
        with mock.patch.object(board.RigidBodyModel, 'load', return_value=model), \
             mock.patch.object(board, 'load_board_config', return_value=cfg), \
             mock.patch.object(board, 'validate_request'), \
             mock.patch.object(board, 'plan', return_value=route), \
             mock.patch.object(board, 'check_transit_edge'), \
             mock.patch.dict(sys.modules, {'left_arm_v2_6': legacy}), \
             mock.patch('left_arm_general_motion.load_configuration'), \
             mock.patch.object(board.FaultSupport, 'run', persistent_hold), \
             mock.patch.dict(board.os.environ, {}, clear=False):
            with self.assertRaises(KeyboardInterrupt):
                board.main(['--x-mm', '137.5', '--y-mm', '192.5', '--execute'])
        self.assertEqual(len(observed), 1)
        self.arm.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
