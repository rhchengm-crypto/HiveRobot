import json
import subprocess
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

import left_arm_board_grasp as board
import chessboard_vision_v3_0_web_control as web


class BoardGraspTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.here = Path(__file__).resolve().parent
        cls.model = board.RigidBodyModel.load(cls.here/'data/left_arm_general_dynamics.json')
        cls.cfg = board.load_board_config(cls.here/'data/left_arm_board_grasp.json', cls.model)

    def test_step_datum_is_not_bounding_box_or_claimed_contact_calibration(self):
        datum = self.cfg['tcp_datum']
        self.assertAlmostEqual(datum['wrist_axis_distance_mm'], 201.22366344844, places=5)
        self.assertEqual(datum['mount_cylinder_step_entity'], 193498)
        self.assertFalse(datum['semantic_contact_centre_verified'])
        self.assertFalse(self.cfg['independently_validated'])

    def test_fractional_non_square_coordinate_has_full_geometric_path(self):
        start = self.cfg['profiles']['white_knight']['seed_encoder_rad']
        result = board.plan(self.model, self.cfg, start, 139.2, 194.7, 'white_knight', 'grasp')
        self.assertTrue(result['ok'])
        self.assertEqual(result['target_board_mm'][:2], [139.2, 194.7])
        self.assertEqual(set(r['stage'] for r in result['waypoints']), {'source_above', 'contact', 'lift'})
        contact = [r for r in result['waypoints'] if r['stage']=='contact'][-1]
        np.testing.assert_allclose(contact['target_board_mm'], [139.2, 194.7, 55], atol=1e-8)
        self.assertFalse(result['motion_issued'])

    def test_reported_home_start_gets_checked_raised_entry_without_rotate_move(self):
        start = dict(zip(self.model.names, [
            .7696269154548645, -2.853627920150757, 2.342832088470459,
            -1.2834744453430176, -2.6575493812561035, .16308079659938812,
            -2.0708401203155518]))
        result = board.plan(self.model, self.cfg, start, 137.5, 192.5, 'white_knight', 'above')
        self.assertEqual(result['entry_route'], ['entry_clearance', 'source_above'])
        previous = np.array([start[n] for n in self.model.names])
        for row in result['waypoints']:
            target = np.array([row['encoder_rad'][n] for n in self.model.names])
            board.check_transit_edge(self.model, previous, target, self.cfg)
            if row['stage'] == 'entry_clearance':
                self.assertEqual(row['encoder_rad']['shoulder_rotate'], start['shoulder_rotate'])
            previous = target
        np.testing.assert_allclose(result['waypoints'][-1]['target_board_mm'],
                                   [137.5, 192.5, 160], atol=.5)

    def test_transit_check_samples_do_not_restart_the_controller(self):
        start = dict(zip(self.model.names, [
            .8291370868682861, -2.853627920150757, 2.3653390407562256,
            -1.2796597480773926, -2.6651790142059326, .16880293190479279,
            -2.0708401203155518]))
        result = board.plan(self.model, self.cfg, start, 137.5, 192.5, 'white_knight', 'above')
        groups = board.execution_waypoints(result)
        self.assertEqual(len(result['waypoints']), 52)
        self.assertEqual(len(groups), 2)
        self.assertEqual([(r['first_original_index'], r['original_index']) for r in groups], [(0,24),(25,51)])
        previous = np.array([start[n] for n in self.model.names])
        for group in groups:
            target = np.array([group['encoder_rad'][n] for n in self.model.names])
            for row in result['waypoints'][group['first_original_index']:group['original_index']+1]:
                point = np.array([row['encoder_rad'][n] for n in self.model.names])
                alpha = float((point-previous)@(target-previous)/np.sum((target-previous)**2))
                np.testing.assert_allclose(point, previous+alpha*(target-previous), atol=1e-9)
            board.check_transit_edge(self.model, previous, target, self.cfg)
            previous = target

    def test_curved_transit_is_not_shortcut_by_grouping(self):
        route = {'start_rad': {'wrist': 0., 'elbow': 0.}, 'waypoints': [
            {'stage': 'source_above', 'encoder_rad': {'wrist': 1., 'elbow': 0.}},
            {'stage': 'source_above', 'encoder_rad': {'wrist': 1., 'elbow': 1.}}]}
        self.assertEqual(len(board.execution_waypoints(route)), 2)

    def test_executor_runs_two_model_controlled_transit_groups(self):
        start = dict(zip(self.model.names, [
            .8291370868682861, -2.853627920150757, 2.3653390407562256,
            -1.2796597480773926, -2.6651790142059326, .16880293190479279,
            -2.0708401203155518]))
        route = board.plan(self.model, self.cfg, start, 137.5,192.5,'white_knight','above')
        arm = mock.Mock()
        arm.motors = {n: object() for n in self.model.names}
        status = {n: {'pos': start[n], 'vel': 0., 'tau': 0.} for n in self.model.names}
        arm.read_status.side_effect = lambda names: {n: dict(status[n]) for n in names}
        def complete(target, **kwargs):
            for n in target:
                status[n]['pos'] = target[n]
        arm.move_targets_with_holds.side_effect = complete
        legacy = types.SimpleNamespace(HOME_GAINS={n:{'kp':22.,'kd':1.4} for n in self.model.names})
        result = board.execute(arm, legacy, route, geometry=(self.model,self.cfg))
        self.assertTrue(result['ok'])
        self.assertEqual(arm.move_targets_with_holds.call_count, 2)
        for call in arm.move_targets_with_holds.call_args_list:
            self.assertIsInstance(call.kwargs['watchdog'], board.BoardWatchdog)
            self.assertEqual(call.kwargs['trajectory'], 'smootherstep')
            self.assertLessEqual(call.kwargs['control_dt'], .01)
        arm.disable.assert_not_called()

    def test_touch_is_small_vertical_descent_with_fixed_current_orientation(self):
        seed = self.cfg['profiles']['white_knight']['seed_encoder_rad']
        above = board.plan(self.model, self.cfg, seed, 137.5, 192.5, 'white_knight', 'above')
        start = above['waypoints'][-1]['encoder_rad']
        result = board.plan(self.model, self.cfg, start, 137.5, 192.5, 'white_knight', 'touch', 140.)
        start_rotation = board.tcp_pose(self.model, np.array([start[n] for n in self.model.names]),
                                       np.array(self.cfg['tcp_in_wrist_frame_m']))[:3,:3]
        heights = []
        for row in result['waypoints']:
            self.assertEqual(row['stage'], 'contact')
            np.testing.assert_allclose(row['target_board_mm'][:2], [137.5,192.5], atol=.5)
            pose = board.tcp_pose(self.model, np.array([row['encoder_rad'][n] for n in self.model.names]),
                                  np.array(self.cfg['tcp_in_wrist_frame_m']))
            self.assertLess(np.linalg.norm(board.rotation_error(start_rotation, pose[:3,:3])), .005)
            heights.append(row['target_board_mm'][2])
        self.assertTrue(all(a>b for a,b in zip(heights, heights[1:])))
        self.assertAlmostEqual(heights[-1], 140.)
        for x,y,h in ((137.5,192.5,55.), (150.,192.5,140.)):
            with self.assertRaisesRegex(board.BoardPlanningError, 'no feasible complete board path'):
                board.plan(self.model, self.cfg, start, x,y, 'white_knight','touch',h)

    def test_restored_endpoint_settle_is_bounded_and_can_be_suppressed(self):
        incident=json.loads((self.here.parent/'outputs/left_arm_general_model/board_v7_last_step_diagnosis.json').read_text(encoding='utf8'))
        before=incident['phases']['source_above:0']['last']
        measured=before['joint_pos_rad']
        # Read the final reference from the source log, independently of the
        # measured hover the operator identified as centred on C4.
        target=None
        for line in (self.here/'data/left_arm_v2_8_full_runs'/incident['source_log']).read_text(encoding='utf8').splitlines():
            if line.startswith('board tracking audit= '):
                row=json.loads(line.split('= ',1)[1])
                if row['stage']=='source_above':target=row['reference_rad']
        names=list(self.model.names)
        status={n:dict(pos=measured[n],vel=0.,tau=before['command_tau_ff'][n]) for n in names}
        arm=mock.Mock();arm.motors={n:object() for n in names}
        arm.read_status.return_value=status
        legacy=types.SimpleNamespace(HOME_GAINS={n:dict(kp=22.,kd=1.4) for n in names})
        route={'stage':'above','waypoints':[dict(stage='source_above',encoder_rad=target,
                                               target_board_mm=[132.5,192.5,160.])]}
        result=board.execute(arm,legacy,route)
        self.assertEqual(arm.move_targets_with_holds.call_count,4)
        self.assertTrue(result['endpoint']['automatic_endpoint_correction'])
        self.assertEqual(result['endpoint']['endpoint_settle_attempts'],3)
        for call in arm.move_targets_with_holds.call_args_list[1:]:
            self.assertEqual(call.args[0],target)
            self.assertEqual(call.kwargs['move_gains'],legacy.HOME_GAINS)
            self.assertIsInstance(call.kwargs['watchdog'],board.BoardWatchdog)
            self.assertEqual(call.kwargs['trajectory'],'smootherstep')
        self.assertFalse(result['endpoint']['nominal_joint_target_reached'])
        self.assertGreater(result['endpoint']['max_nominal_error_deg'],2.)
        self.assertEqual(result['endpoint']['measured_pose_rad'],measured)
        self.assertEqual(result['final'],status)
        arm.disable.assert_not_called()
        arm.close.assert_not_called()
        arm.move_targets_with_holds.reset_mock()
        route['endpoint_settle_enabled']=False
        disabled=board.execute(arm,legacy,route)
        arm.move_targets_with_holds.assert_called_once()
        self.assertFalse(disabled['endpoint']['automatic_endpoint_correction'])

    def test_blocked_reference_is_not_used_to_bypass_board_check(self):
        start = self.model.reference.copy()
        with mock.patch.object(board, 'check_transit_edge', side_effect=board.BoardPlanningError(
                'blocked', {'code': 'test_board_intersection'})):
            route, failures = board.find_entry_transit(self.model, start, [(1., start+.01)], self.cfg)
        self.assertIsNone(route)
        self.assertTrue(any(r.get('leg') == 'start_to_reference' for r in failures))

    def test_planning_failure_retains_ik_and_transit_diagnostics(self):
        start = self.cfg['profiles']['white_knight']['seed_encoder_rad']
        with mock.patch.object(board, 'find_entry_transit', return_value=(None, [
                {'code': 'tool_board_intersection', 'transit_fraction': .4}])):
            with self.assertRaises(board.BoardPlanningError) as error:
                board.plan(self.model, self.cfg, start, 137.5, 192.5, 'white_knight', 'above')
        self.assertEqual(error.exception.diagnostics['start_rad'], start)
        details = error.exception.diagnostics['orientation_attempts'][0]['details']
        self.assertEqual(details['code'], 'above_transit_blocked')
        self.assertGreater(details['ik_solutions'], 0)
        self.assertEqual(details['blocked_paths'][0]['transit_fraction'], .4)

    def test_preview_does_not_load_can_or_enable_motors(self):
        cmd = [sys.executable, str(self.here/'left_arm_board_grasp.py'),
               '--x-mm', '137.5', '--y-mm', '192.5', '--piece', 'white_knight',
               '--start-json', json.dumps(self.cfg['profiles']['white_knight']['seed_encoder_rad'])]
        run = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertIn('"motion_issued": false', run.stdout)
        self.assertNotIn('Serial port is open', run.stdout)
        run = subprocess.run(cmd+['--execute'], capture_output=True, text=True)
        self.assertNotEqual(run.returncode, 0)
        self.assertIn('cannot execute', run.stderr)

    def test_cli_planning_failure_reports_structured_error_without_traceback(self):
        start = dict(self.cfg['profiles']['white_knight']['seed_encoder_rad'])
        start['shoulder_front'] = 8.
        run = subprocess.run([sys.executable, str(self.here/'left_arm_board_grasp.py'),
            '--x-mm', '137.5', '--y-mm', '192.5', '--piece', 'white_knight',
            '--start-json', json.dumps(start)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 1)
        self.assertIn('start_outside_operating_bounds', run.stdout)
        self.assertNotIn('Traceback', run.stderr)
        self.assertNotIn('Serial port is open', run.stdout)

    def test_bad_requests_and_nonfinite_data_rejected(self):
        for x, y, stage in ((-1, 20, 'above'), (441, 20, 'above'), (float('nan'), 20, 'above'),
                            (20, 20, 'drop')):
            with self.assertRaises(ValueError):
                board.validate_request(x, y, 'white_knight', stage, None, self.cfg)
        with self.assertRaises(ValueError):
            board.validate_request(20, 20, 'unknown', 'above', None, self.cfg)

    def _fake_execution(self, fail_contact=False):
        names = list(self.model.names)
        pose = self.cfg['profiles']['white_knight']['seed_encoder_rad']
        arm = mock.Mock()
        arm.motors = {n: object() for n in names}
        arm.read_status.return_value = {n: {'pos': pose[n], 'vel': 0, 'tau': 0} for n in names}
        legacy = types.SimpleNamespace(HOME_GAINS={n: {'kp': 6, 'kd': .8} for n in names},
                                       CLAW_KP_HOLD=14, CLAW_KD_HOLD=.7)
        route = {'stage': 'grasp', 'waypoints': [
            {'stage': label, 'encoder_rad': pose, 'target_board_mm': [137.5, 192.5, 55]}
            for label in ('contact', 'lift')]}
        with mock.patch('left_arm_v2_8_move_library.claw_home_while_holding_arm',
                        return_value={'target_rad': .67}), \
             mock.patch.object(board, 'BOARD_EXECUTION_SUSPENDED', False), \
             mock.patch.object(board, 'SUPPORTED_EXECUTION_STAGES', ('above', 'grasp')), \
             mock.patch('left_arm_v2_8_move_library.close_claw_while_holding_arm',
                        side_effect=RuntimeError('no contact') if fail_contact else None,
                        return_value=1.8) as close:
            if fail_contact:
                with self.assertRaisesRegex(RuntimeError, 'no contact'):
                    board.execute(arm, legacy, route)
                self.assertEqual(arm.move_targets_with_holds.call_count, 1)
            else:
                self.assertTrue(board.execute(arm, legacy, route)['grasp_contact_confirmed'])
                self.assertEqual(arm.move_targets_with_holds.call_count, 2)
                self.assertEqual(arm.move_targets_with_holds.call_args.kwargs['hold_targets']['claw'], 1.8)
            self.assertTrue(close.call_args.kwargs['require_contact'])

    def test_pressure_contact_required_before_lift(self):
        self._fake_execution(True)

    def test_claw_target_refreshed_during_model_controlled_lift(self):
        self._fake_execution(False)

    def test_web_gating_model_and_post_hold_routing(self):
        args = web.build_parser().parse_args([])
        args.rigid_body_model = str(self.here/'data/left_arm_general_dynamics.json')
        args.enable_execute = True
        args.rigid_body_mode = 'apply'
        preview = web.build_dynamics_command(args, 'board-grasp', {'x_mm': 137.5, 'y_mm': 192.5})
        self.assertNotIn('--execute', preview)
        actual = web.build_dynamics_command(args, 'board-grasp',
                 {'x_mm': 137.5, 'y_mm': 192.5, 'execute': True, 'test_protective_stop': True})
        self.assertIn('--execute', actual)
        self.assertIn('--test-protective-stop', actual)
        touch = web.build_dynamics_command(args, 'board-grasp',
                {'x_mm':137.5,'y_mm':192.5,'stage':'touch','grip_height_mm':140.,'execute':True})
        self.assertIn('--execute', touch)
        self.assertEqual(touch[touch.index('--grip-height-mm')+1], '140.0')
        with self.assertRaises(PermissionError):
            web.build_dynamics_command(args, 'board-grasp',
                    {'x_mm': 139.2, 'y_mm': 194.7, 'stage': 'grasp', 'execute': True})
        self.assertTrue(web.is_post_move_hold_action('dynamics:board-grasp'))
        for token in preview:
            self.assertNotIn('no-adaptive', token)
        with self.assertRaises(ValueError):
            web.build_dynamics_command(args, 'board-grasp', {'x_mm': 450, 'y_mm': 10})

    def test_direct_executor_rejected_without_hardware_calls(self):
        arm = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'Only Above Only'):
            board.execute(arm, mock.Mock(), {'stage': 'grasp'})
        self.assertEqual(arm.mock_calls, [])

    def test_cli_execution_lock_precedes_even_model_loading(self):
        run = subprocess.run([sys.executable, str(self.here/'left_arm_board_grasp.py'),
            '--model', 'intentionally-missing-model.json', '--x-mm', '137.5', '--y-mm', '192.5',
            '--stage', 'grasp', '--execute'], capture_output=True, text=True)
        self.assertEqual(run.returncode, 1)
        self.assertIn('Only Above Only', run.stdout)
        self.assertNotIn('Serial port is open', run.stdout)
        self.assertNotIn('general motion configuration', run.stdout)


if __name__ == '__main__':
    unittest.main()
