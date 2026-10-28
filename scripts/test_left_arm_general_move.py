import io
import json
import math
from pathlib import Path
import types
import tempfile
import unittest
from unittest import mock
from contextlib import redirect_stdout

import left_arm_general_move as move
from test_left_arm_v31_motion_dynamics import load_legacy_without_hardware


class GeneralTargetTests(unittest.TestCase):
    def setUp(self):
        self.start = dict(zip(move.JOINTS, [.77, -2.84, 2.36, -1.28, -2.67, .166, -2.08]))

    def test_unseen_targets_do_not_need_named_move_or_local_samples(self):
        for k in range(1, 10):
            target = {n: q + math.sin(k+i)*.1 for i, (n, q) in enumerate(self.start.items())}
            plan = move.plan_move(self.start, target, seconds=1., max_velocity=.1)
            self.assertEqual(plan['targets_rad'], target)
            self.assertLessEqual(plan['reference_peak_velocity_rad_s'], .07500001)
        plan = move.plan_move(self.start, {'wrist': -2.0})
        self.assertEqual(plan['held_joints'], list(move.JOINTS[:-1]))
        self.assertEqual(plan['targets_rad']['elbow'], self.start['elbow'])

    def test_bad_targets_and_limits_fail_before_hardware(self):
        for target in ({}, {'claw': 0.}, {'wrist': float('nan')}, {'wrist': True}, {'wrist': 13.}):
            with self.assertRaises(ValueError):
                move.clean_targets(target)
        with self.assertRaises(ValueError):
            move.plan_move(self.start, {'wrist': -1.}, max_delta_deg=5.)
        with self.assertRaises(ValueError):
            move.plan_move(self.start, {'wrist': -2.}, seconds=float('nan'))

    def test_feedback_failure_stops_trajectory_before_completion(self):
        arm = mock.Mock()
        arm.read_status.return_value = {n: {'pos': q, 'vel': 0.} for n, q in self.start.items()}
        watchdog = move.TrackingWatchdog(arm, .3, 2.)
        watchdog(self.start)
        arm.read_status.return_value['elbow']['vel'] = .4
        with self.assertRaisesRegex(RuntimeError, 'velocity watchdog'):
            for _ in range(8):
                watchdog(self.start)
        arm.read_status.return_value['elbow']['vel'] = 0.
        arm.read_status.return_value['elbow']['pos'] += .1
        watchdog.calls = 0
        with self.assertRaisesRegex(RuntimeError, 'tracking watchdog'):
            watchdog(self.start)

    def test_shared_group_adapter_carries_entry_load_and_holds_omitted_joints(self):
        arm = mock.Mock()
        legacy = types.SimpleNamespace(HOME_GAINS={n: {'kp': 10., 'kd': 1.} for n in move.JOINTS})
        status = {n: {'pos': q, 'vel': 0., 'tau': .2} for n, q in self.start.items()}
        plan = move.plan_move(self.start, {'wrist': -2.})
        move.execute_plan(arm, legacy, plan, status, .3, 5.)
        call = arm.move_targets_with_holds.call_args
        self.assertEqual(call.args[0], {'wrist': -2.})
        self.assertEqual(set(call.kwargs['hold_targets']), set(move.JOINTS)-{'wrist'})
        self.assertEqual(call.kwargs['move_tau_ff'], dict.fromkeys(move.JOINTS, .2))
        self.assertEqual(call.kwargs['trajectory'], 'smootherstep')
        status['elbow']['vel'] = .2
        arm.reset_mock()
        with self.assertRaisesRegex(ValueError, 'stationary'):
            move.execute_plan(arm, legacy, plan, status, .3, 5.)
        arm.enable.assert_not_called()

    def test_group_watchdog_hook_is_optional_and_reaches_real_shared_loop(self):
        legacy = load_legacy_without_hardware()
        arm = legacy.LeftArmV2.__new__(legacy.LeftArmV2)
        arm.validate = lambda names: list(names)
        arm.positions = lambda names: {n: self.start[n] for n in names}
        arm.ctrl = mock.Mock()
        arm.motors = {n: n for n in move.JOINTS}
        arm.motion_dynamics_model = None
        def fail(reference):
            self.assertEqual(set(reference), {'wrist', 'elbow'})
            raise RuntimeError('watchdog stopped')
        with self.assertRaisesRegex(RuntimeError, 'watchdog stopped'):
            arm.move_targets_with_holds({'wrist': -2.}, 1., {}, {'elbow': -1.28}, {},
                                        2., .4, watchdog=fail)

    def test_offline_preview_cannot_issue_motion(self):
        model = Path(__file__).resolve().parents[1]/'outputs/left_arm_general_model/left_arm_general_dynamics.json'
        with redirect_stdout(io.StringIO()) as output:
            move.main(['--model', str(model), '--start-json', json.dumps(self.start),
                       '--targets-json', '{"wrist":-2.0}'])
        result = json.loads(output.getvalue().splitlines()[-1])
        self.assertFalse(result['motion_issued'])
        self.assertIn('plan', result)

    def test_unregistered_point_reaches_actual_dynamics_loop_and_releases(self):
        from left_arm_general_motion import load_configuration
        from test_left_arm_general_motion import DeterministicWorker
        legacy = load_legacy_without_hardware()
        arm = legacy.LeftArmV2.__new__(legacy.LeftArmV2)
        arm.specs = dict.fromkeys(move.JOINTS)
        arm.motors = dict(zip(move.JOINTS, move.JOINTS))
        pose = dict(self.start)
        arm.positions = lambda names: {n: pose[n] for n in names}
        arm.read_status = lambda names: {n: {'pos': pose[n], 'vel': 0., 'tau': .2} for n in names}
        arm.enable = mock.Mock()
        sent = []
        def send(name, kp, kd, q, dq, tau):
            pose[name] = q
            sent.append((name, tau))
        arm.ctrl = types.SimpleNamespace(controlMIT=send)
        clock = types.SimpleNamespace(now=0.)
        def sleep(dt):
            clock.now += dt
        model = Path(__file__).resolve().parents[1]/'outputs/left_arm_general_model/left_arm_general_dynamics.json'
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            config = load_configuration(model, 'apply')
            config['worker'] = DeterministicWorker(config['model'].data)
            log = Path(folder)/'runs.jsonl'
            config['log_path'] = str(log)
            arm.motion_dynamics_model = {'rigid_body_configuration': config}
            plan = move.plan_move(pose, {'elbow': pose['elbow']+.08, 'wrist': pose['wrist']+.06}, seconds=1.)
            with (mock.patch.object(legacy.time, 'time', lambda: clock.now),
                  mock.patch.object(legacy.time, 'monotonic', lambda: clock.now),
                  mock.patch.object(legacy.time, 'sleep', sleep)):
                move.execute_plan(arm, legacy, plan, arm.read_status(move.JOINTS), .3, 5.)
            report = json.loads(log.read_text(encoding='utf-8'))
        self.assertEqual(report['mode'], 'apply')
        self.assertEqual(report['prediction_status'], 'available')
        self.assertTrue(report['terminal_correction_zero'])
        self.assertTrue(any(abs(tau-.2) > .0001 for _, tau in sent))
        self.assertEqual(pose, plan['targets_rad'])

    def test_new_point_hold_coverage_uses_general_model_without_modifying_history(self):
        from left_arm_v3_1_model_control import empty_model, gravity_prediction, inertia_prediction
        from left_arm_rigid_body import RigidBodyModel
        import copy
        model = empty_model()
        before = copy.deepcopy(model)
        path = Path(__file__).resolve().parents[1]/'outputs/left_arm_general_model/left_arm_general_dynamics.json'
        rigid = RigidBodyModel.load(path)
        result, supplied = move.supply_hold_coverage(model, rigid, self.start, move.JOINTS)
        self.assertEqual(model, before)
        self.assertEqual(len(supplied), 14)
        q = rigid.encoder_to_model([self.start[n] for n in rigid.names])
        expected = rigid.signs * rigid.gravity_torques(q)
        for i, n in enumerate(rigid.names):
            self.assertAlmostEqual(gravity_prediction(result, n, self.start)[0], expected[i])
            self.assertGreater(inertia_prediction(result, n, self.start)[0], 0.)
        unchanged, supplied = move.supply_hold_coverage(result, rigid, self.start, move.JOINTS)
        self.assertIs(unchanged, result)
        self.assertEqual(supplied, [])


if __name__ == '__main__':
    unittest.main()
