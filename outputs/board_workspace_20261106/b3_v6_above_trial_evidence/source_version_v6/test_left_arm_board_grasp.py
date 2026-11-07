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

    def test_c5_feedback_seed_roundoff_does_not_relax_posture_targets(self):
        cfg = json.loads(json.dumps(self.cfg))
        q = dict(zip(self.model.names, [.50030517578125, -2.8444724082946777,
            2.427138090133667, .4228656589984894, -2.800602674484253,
            -.02841992862522602, -2.9345006942749023]))
        cfg['joint_limits_encoder_rad'][self.model.names.index('shoulder_side')][0] = -2.97684
        cfg['joint_limits_encoder_rad'][self.model.names.index('shoulder_rotate')][1] = 2.42638
        with self.assertRaisesRegex(ValueError, 'seed outside'):
            board.feedback_contact_path(self.model, cfg, q, [137.5, 247.5, 30.2])
        cfg['ik_soft_posture_seed_tolerance_deg'] = {'shoulder_side': .1, 'shoulder_rotate': .1}
        rows, audit = board.feedback_contact_path(self.model, cfg, q, [137.5, 247.5, 30.2])
        self.assertEqual(audit['start_measured_rad'], q)
        self.assertGreater(len(rows), 1)
        for row in rows:
            self.assertLessEqual(row['encoder_rad']['shoulder_rotate'], 2.42638)
            self.assertGreaterEqual(row['encoder_rad']['shoulder_side'], -2.97684)
        self.assertLess(rows[0]['seed_projection_delta_deg']['shoulder_rotate'], 0)

    def test_c5_live_waypoint_resolve_projects_feedback_inside_soft_bounds(self):
        cfg = json.loads(json.dumps(self.cfg))
        start = dict(zip(self.model.names, [.49458304047584534, -2.852102041244507,
            2.427138090133667, .4236285984516144, -2.792973279953003,
            -.031090257689356804, -2.9345006942749023]))
        cfg['joint_limits_encoder_rad'][self.model.names.index('shoulder_side')][0] = -2.97684
        cfg['joint_limits_encoder_rad'][self.model.names.index('shoulder_rotate')][1] = 2.42638
        cfg['ik_soft_posture_seed_tolerance_deg'] = {'shoulder_side': .1, 'shoulder_rotate': .1}
        status = {n: {'pos': start[n], 'vel': 0., 'tau': 0.} for n in self.model.names}
        arm = mock.Mock()
        arm.motors = {n: object() for n in self.model.names}
        arm.read_status.side_effect = lambda names: {n: dict(status[n]) for n in names}
        def complete(target, **kwargs):
            for n in target:
                status[n]['pos'] = target[n]
            # Simulate a persistent small tracking offset on every live edge.
            status['shoulder_rotate']['pos'] += np.radians(.043)
        arm.move_targets_with_holds.side_effect = complete
        route = {'stage': 'approach', 'target_board_mm': [137.5,247.5,30.2],
                 'waypoints': [{'stage': 'contact', 'target_board_mm': [137.5,247.5,30.2],
                                'encoder_rad': start}]}
        legacy = types.SimpleNamespace(HOME_GAINS={n: {'kp':22.,'kd':1.4} for n in self.model.names})
        result = board.execute(arm, legacy, route, geometry=(self.model,cfg))
        self.assertTrue(result['ok'])
        self.assertGreater(arm.move_targets_with_holds.call_count, 10)
        for call in arm.move_targets_with_holds.call_args_list:
            self.assertLessEqual(call.args[0]['shoulder_rotate'], 2.42638)
            self.assertGreaterEqual(call.args[0]['shoulder_side'], -2.97684)
        arm.disable.assert_not_called()

    def test_c5_saturated_shoulder_redistributes_exact_failed_alignment_edge(self):
        cfg = json.loads(json.dumps(self.cfg))
        cfg['joint_limits_encoder_rad'][self.model.names.index('shoulder_side')][0] = -2.97684
        cfg['joint_limits_encoder_rad'][self.model.names.index('shoulder_rotate')][1] = 2.42638
        cfg['ik_soft_posture_seed_tolerance_deg'] = {'shoulder_side': .1, 'shoulder_rotate': .1}
        measured = np.array([.49076828360557556, -2.890249490737915, 2.4267566204071045,
                             .41142138838768005, -2.7548255920410156,
                             -.02536812424659729, -2.9345006942749023])
        wrist_r = [[.4065771870404572,.8404592876260505,.35822224501192274],
                   [.023858035178180685,-.4017260091179695,.9154490743649196],
                   [.9133048697645858,-.3636542306109775,-.18338433799818424]]
        target = board.board_point_to_cad([137.5,247.5,131.3224118542743],
            cfg['board_to_cad_rotation'], cfg['board_to_cad_translation_m'])
        answer = board.solve_ik(self.model, target, measured, cfg['tcp_in_wrist_frame_m'],
            cfg['joint_limits_encoder_rad'], wrist_r, position_tolerance_m=.0005,
            rotation_tolerance_rad=.005,
            seed_projection_tolerance_rad=board.soft_posture_seed_tolerance(self.model,cfg))
        self.assertTrue(answer['success'])
        self.assertLess(answer['position_error_mm'], .5)
        self.assertLessEqual(answer['encoder_rad']['shoulder_rotate'], 2.42638)
        self.assertGreaterEqual(answer['encoder_rad']['shoulder_side'], -2.97684)

    def test_fractional_non_square_coordinate_has_full_geometric_path(self):
        start = self.cfg['profiles']['white_knight']['seed_encoder_rad']
        result = board.plan(self.model, self.cfg, start, 139.2, 194.7, 'white_knight', 'grasp')
        self.assertTrue(result['ok'])
        self.assertEqual(result['target_board_mm'][:2], [139.2, 194.7])
        self.assertEqual(set(r['stage'] for r in result['waypoints']), {'source_above', 'contact', 'lift'})
        contact = [r for r in result['waypoints'] if r['stage']=='contact'][-1]
        np.testing.assert_allclose(contact['target_board_mm'], [139.2, 194.7, 55], atol=1e-8)
        self.assertFalse(result['motion_issued'])

    def test_common_planner_checks_cartesian_edges_with_executor_transit_checker(self):
        cfg = json.loads(json.dumps(self.cfg))
        cfg['workspace_planning_policy'] = 'shared-body-constrained-IK'
        start = cfg['profiles']['white_knight']['seed_encoder_rad']
        with mock.patch.object(board, 'check_transit_edge', wraps=board.check_transit_edge) as checks:
            result = board._plan_fixed_orientation(self.model, cfg, start,
                139.2, 194.7, 'white_knight', 'grasp')
        cartesian = [row for row in result['waypoints'] if row['stage'] in ('contact', 'lift')]
        self.assertGreater(len(cartesian), 2)
        for row in cartesian:
            goal = np.array([row['encoder_rad'][name] for name in self.model.names])
            self.assertTrue(any(np.allclose(call.args[2], goal, atol=1e-10)
                                for call in checks.call_args_list), row['stage'])

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

    def test_search_samples_merge_until_the_turn_without_cutting_it(self):
        route={'start_rad':{'wrist':0.,'elbow':0.},'waypoints':[
            {'stage':'entry_search','encoder_rad':{'wrist':.5,'elbow':0.}},
            {'stage':'entry_search','encoder_rad':{'wrist':1.,'elbow':0.}},
            {'stage':'entry_search','encoder_rad':{'wrist':1.,'elbow':.5}},
            {'stage':'entry_search','encoder_rad':{'wrist':1.,'elbow':1.}}]}
        groups=board.execution_waypoints(route)
        self.assertEqual([r['original_index'] for r in groups],[1,3])
        self.assertEqual(groups[0]['encoder_rad'],{'wrist':1.,'elbow':0.})

    def test_support_anchor_does_not_integrate_repeated_endpoint_pd_load(self):
        positions=dict(zip(self.model.names,self.model.reference.tolist()))
        tau=dict.fromkeys(self.model.names,0.)
        tau['elbow']=4.
        status={n:{'pos':positions[n],'vel':0.,'tau':tau[n]} for n in positions}
        gains={n:{'kp':70.,'kd':3.} for n in positions}
        limits={n:[-5.4,5.4] for n in positions}
        anchor=board.TrajectorySupportBaseline(self.model,positions,tau,status,gains,limits)
        status['elbow']['vel']=.01
        first,_=anchor.command(positions,status,gains)
        self.assertAlmostEqual(first['elbow']['start_tau'],4.03)
        self.assertAlmostEqual(first['elbow']['end_tau'],4.)
        status['elbow']['vel']=0.
        for later_feedback in (8.5,12.4,15.7,18.6):
            status['elbow']['tau']=later_feedback
            command,audit=anchor.command(positions,status,gains)
            self.assertAlmostEqual(command['elbow']['end_tau'],4.)
            self.assertFalse(audit['elbow']['clamped'])
        moved=dict(positions,elbow=positions['elbow']+.1)
        command,audit=anchor.command(moved,status,gains)
        expected=4.+anchor.gravity(moved)['elbow']-anchor.initial_gravity['elbow']
        self.assertAlmostEqual(command['elbow']['end_tau'],min(5.4,max(-5.4,expected)))
        tau['elbow']=18.6
        with self.assertRaisesRegex(RuntimeError,'entry support exceeds'):
            board.TrajectorySupportBaseline(self.model,positions,tau,status,gains,limits)

    def test_entry_support_preserves_observed_wrist_load_without_expanding_from_later_pd(self):
        positions=dict(zip(self.model.names,[1.94,-2.72,2.36,.817,-2.65,.307,-1.029]))
        tau=dict.fromkeys(self.model.names,0.);tau['wrist']=.7350427508354187
        status={n:{'pos':positions[n],'vel':0.,'tau':tau[n]} for n in positions}
        gains={n:{'kp':22.,'kd':1.4} for n in positions}
        limits={n:[-.7,.7] for n in positions};hardware=dict.fromkeys(positions,10.)
        anchor=board.TrajectorySupportBaseline(self.model,positions,tau,status,gains,limits,hardware)
        command,audit=anchor.command(positions,status,gains)
        self.assertAlmostEqual(command['wrist']['end_tau'],tau['wrist'])
        self.assertEqual(audit['wrist']['trajectory_tau_limits'],(-.7,tau['wrist']))
        status['wrist']['tau']=9.
        command,audit=anchor.command(positions,status,gains)
        self.assertAlmostEqual(command['wrist']['end_tau'],tau['wrist'])
        self.assertEqual(audit['wrist']['trajectory_tau_limits'],(-.7,tau['wrist']))
        tau['wrist']=10.1
        with self.assertRaisesRegex(RuntimeError,'valid torque range for wrist'):
            board.TrajectorySupportBaseline(self.model,positions,tau,status,gains,limits,hardware)

    def test_common_posture_preference_penalizes_raised_loaded_elbow(self):
        def frame(q):
            value=np.eye(4)[None].copy()
            value[0,2,3]=q[0]
            return value
        model=types.SimpleNamespace(names=['elbow'],signs=np.ones(1),
            encoder_to_model=lambda q:np.asarray(q),fk=frame,
            gravity_torques=lambda q:np.array([10.*q[0]]))
        cfg={'board_to_cad_rotation':np.eye(3).tolist(),'board_to_cad_translation_m':[0.,0.,0.],
             'posture_cost_weights':{'elbow_height_per_m':10.,'nominal_gravity_squared_per_Nm2':.02}}
        low,audit=board.posture_load_cost(model,cfg,{'elbow':.4},[{'encoder_rad':{'elbow':.2}}])
        high,_=board.posture_load_cost(model,cfg,{'elbow':.4},[{'encoder_rad':{'elbow':.8}}])
        self.assertLess(low,high)
        self.assertAlmostEqual(audit['elbow_height_terminal_mm'],200.)
        self.assertFalse(audit['physical_load_independently_verified'])

    def test_full_orientation_rejects_v4_1_axis_only_b3_posture(self):
        cfg=board.load_board_config(self.here/'data/left_arm_board_workspace_candidate.json',self.model)
        cfg['tool_orientation_constraint']='full-wrist'
        wrong=dict(zip(self.model.names,[.7142831932318399,-2.9484186672817776,2.228697831568297,
            .838674008846283,-3.141778538949578,.3118984046474499,-3.4836426130046085]))
        reference=[[.3597132909743766,.42957112159554606,-.8282964443890417],
                   [.5212442546931378,.6437686794338732,.560237730190823],
                   [.7738932583023205,-.6332697204754972,.00766067119162169]]
        route={'selected_wrist_rotation_in_board':reference,
               'waypoints':[{'stage':'source_above','encoder_rad':wrong}]}
        with self.assertRaisesRegex(board.BoardPlanningError,'complete gripper orientation'):
            board.check_planned_tool_orientation(self.model,cfg,route)
        # The same numeric pose passes only when its FULL frame is the task.
        q=np.array([wrong[n] for n in self.model.names])
        route['selected_wrist_rotation_in_board']=(np.asarray(cfg['board_to_cad_rotation']).T@
            board.tcp_pose(self.model,q,cfg['tcp_in_wrist_frame_m'])[:3,:3]).tolist()
        self.assertLess(board.check_planned_tool_orientation(self.model,cfg,route)['max_error_deg'],1e-6)

    def test_low_elbow_refinement_preserves_tcp_axis_bounds_and_clearances(self):
        cfg=board.load_board_config(self.here/'data/left_arm_board_workspace_candidate.json',self.model)
        goal=np.array([.8911695966980924,-2.18090009074116,2.266346044042885,
            .7304842309004096,-3.1270939099447004,-.39477146087124493,-2.5784475117080303])
        before=board.tcp_pose(self.model,goal,cfg['tcp_in_wrist_frame_m'])
        after_q=board.lower_elbow_goal(self.model,cfg,goal,before[:3,3],before[:3,:3])
        after=board.tcp_pose(self.model,after_q,cfg['tcp_in_wrist_frame_m'])
        axis=np.asarray(cfg['tool_axis_in_wrist_frame'])
        self.assertLess(np.linalg.norm(after[:3,3]-before[:3,3]),.0005)
        self.assertLess(np.linalg.norm(after[:3,:3]@axis-before[:3,:3]@axis),.005)
        if cfg.get('tool_orientation_constraint') != 'axis-only':
            self.assertLess(np.linalg.norm(board.rotation_error(before[:3,:3],after[:3,:3])),.005)
        limits=np.asarray(cfg['joint_limits_encoder_rad'])
        self.assertTrue(np.all(after_q>=limits[:,0]) and np.all(after_q<=limits[:,1]))
        board.check_board_intersection(self.model,after_q,cfg)
        up=np.asarray(cfg['board_to_cad_rotation'])[:,2]
        index=self.model.names.index('elbow')
        height=lambda q:up@self.model.fk(self.model.encoder_to_model(q))[index,:3,3]
        self.assertLess(height(after_q),height(goal)-.0001)

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

    def test_candidate_validation_truncates_before_descent_without_mutating_plan(self):
        cfg = {'workspace_planning_policy': 'shared-body-constrained-IK',
               'body_collision': {'registration_status': 'nominal'}}
        plan = {'stage': 'approach', 'target_board_mm': [27.5, 27.5, 50.2],
                'waypoints': [{'stage': 'entry_search'},
                              {'stage': 'source_above', 'target_board_mm': [27.5, 27.5, 140.2]},
                              {'stage': 'contact', 'target_board_mm': [27.5, 27.5, 50.2]}]}
        trial = board.candidate_above_trial(plan, cfg)
        self.assertEqual(trial['stage'], 'above')
        self.assertEqual([r['stage'] for r in trial['waypoints']], ['entry_search', 'source_above'])
        self.assertEqual(trial['requested_contact_board_mm'], plan['target_board_mm'])
        self.assertFalse(trial['validation_trial']['descent_executed'])
        self.assertFalse(trial['validation_trial']['claw_commanded'])
        self.assertEqual(len(plan['waypoints']), 3)
        self.assertEqual(plan['stage'], 'approach')
        plan['waypoints'][1]['target_board_mm'][2] = 89.
        with self.assertRaises(ValueError):
            board.candidate_above_trial(plan, cfg)
        plan['stage'] = 'grasp'
        with self.assertRaises(ValueError):
            board.candidate_above_trial(plan, cfg)

    def test_candidate_web_trial_is_explicit_and_cannot_request_descent(self):
        args = web.build_parser().parse_args([])
        args.rigid_body_model = str(self.here/'data/left_arm_general_dynamics.json')
        payload = {'x_mm': 27.5, 'y_mm': 27.5, 'stage': 'approach',
                   'validate_candidate_above': True}
        cmd = web.build_dynamics_command(args, 'board-grasp', payload)
        self.assertIn('--validate-candidate-above', cmd)
        self.assertEqual(cmd[cmd.index('--board-model')+1], web.BOARD_WORKSPACE_CANDIDATE)
        self.assertIn('--plan-output', cmd)
        payload['validate_candidate_above'] = False
        cmd = web.build_dynamics_command(args, 'board-grasp', payload)
        self.assertEqual(cmd[cmd.index('--board-model')+1], web.BOARD_GRASP_MODEL)
        for invalid in ('true', 1, None):
            payload['validate_candidate_above'] = invalid
            with self.assertRaises(ValueError):
                web.build_dynamics_command(args, 'board-grasp', payload)
        payload['validate_candidate_above'] = True
        for stage in ('touch', 'grasp', 'above', 'cartesian-check'):
            payload['stage'] = stage
            with self.assertRaises(ValueError):
                web.build_dynamics_command(args, 'board-grasp', payload)

    def test_candidate_touch_web_trial_cannot_expand_to_grasp_or_above_trial(self):
        args=web.build_parser().parse_args([])
        args.rigid_body_model=str(self.here/'data/left_arm_general_dynamics.json')
        payload=dict(x_mm=82.5,y_mm=137.5,stage='touch',grip_height_mm=111.5,validate_candidate_touch=True)
        cmd=web.build_dynamics_command(args,'board-grasp',payload)
        self.assertIn('--validate-candidate-touch',cmd)
        self.assertEqual(cmd[cmd.index('--board-model')+1],web.BOARD_WORKSPACE_CANDIDATE)
        for extra in ({'stage':'grasp'},{'stage':'approach'},{'validate_candidate_above':True},
                      {'test_protective_stop':True},{'validate_candidate_touch':'true'}):
            with self.assertRaises(ValueError):web.build_dynamics_command(args,'board-grasp',dict(payload,**extra))

    def test_candidate_touch_trial_requires_small_vertical_contact_only_path(self):
        cfg=board.load_board_config(self.here/'data/left_arm_board_workspace_candidate.json',self.model)
        start=dict(zip(self.model.names,[.6387808322906494,-2.8715572357177734,1.9457160234451294,
            .6528953909873962,-3.11570143699646,-.6906614899635315,-3.466277599334717]))
        pose=board.tcp_pose(self.model,np.array(list(start.values())),cfg['tcp_in_wrist_frame_m'])
        xyz=(pose[:3,3]-np.asarray(cfg['board_to_cad_translation_m']))@np.asarray(cfg['board_to_cad_rotation'])*1000
        route={'stage':'touch','start_rad':start,'target_board_mm':[*xyz[:2],xyz[2]-20.],
               'waypoints':[{'stage':'contact'}]}
        trial=board.candidate_touch_trial(self.model,route,cfg)
        self.assertTrue(trial['validation_trial']['touch_only'])
        self.assertFalse(trial['validation_trial']['claw_commanded'])
        for end in ([*xyz[:2],xyz[2]-30.],[xyz[0]+6.,xyz[1],xyz[2]-20.]):
            with self.assertRaises(ValueError):board.candidate_touch_trial(self.model,dict(route,target_board_mm=end),cfg)
        with self.assertRaises(ValueError):
            board.candidate_touch_trial(self.model,dict(route,waypoints=[{'stage':'lift'}]),cfg)

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
