import copy
import json
import math
from pathlib import Path
import sys
import unittest
from unittest import mock

import numpy as np

import left_arm_board_grasp as board
from left_arm_board_kinematics import solve_ik, tcp_pose, rotation_error
from left_arm_board_task_pose import batch_tool_frames, endpoint_seeds, global_task_seeds, task_settings
from left_arm_rigid_body import axis_rotation


class TaskPoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.here = Path(__file__).parent
        cls.model = board.RigidBodyModel.load(cls.here/'data/left_arm_general_dynamics.json')
        cls.cfg = board.load_board_config(
            cls.here/'data/left_arm_board_workspace_continuous_candidate.json', cls.model)

    def test_axis_cone_accepts_tilt_without_pretending_full_frame_is_matched(self):
        q = self.model.reference.copy()
        tcp = np.array([.03, -.07, -.14])
        actual = tcp_pose(self.model, q, tcp)
        desired = axis_rotation(actual[:3, :3]@np.array([1., 0., 0.]),
                                math.radians(5))@actual[:3, :3]
        bounds = np.column_stack([q-1e-5, q+1e-5])
        result = solve_ik(self.model, actual[:3, 3], q, tcp, bounds, desired,
            tool_axis_local=[0, 0, 1], tool_axis_cone_rad=math.radians(10), max_iterations=4)
        self.assertTrue(result['success'])
        self.assertAlmostEqual(result['tool_axis_deviation_deg'], 5.)
        # A fixed-orientation request must still reject this same locked pose.
        fixed = solve_ik(self.model, actual[:3, 3], q, tcp, bounds, desired,
                          max_iterations=4, rotation_tolerance_rad=.001)
        self.assertFalse(fixed['success'])

    def test_cone_does_not_accept_outside_axis_or_unreachable_position(self):
        q = self.model.reference.copy()
        tcp = np.array([.03, -.07, -.14])
        actual = tcp_pose(self.model, q, tcp)
        bounds = np.column_stack([q-1e-5, q+1e-5])
        desired = axis_rotation(actual[:3, :3]@np.array([1., 0., 0.]),
                                math.radians(15))@actual[:3, :3]
        result = solve_ik(self.model, actual[:3, 3], q, tcp, bounds, desired,
            tool_axis_local=[0, 0, 1], tool_axis_cone_rad=math.radians(10), max_iterations=4)
        self.assertFalse(result['success'])
        result = solve_ik(self.model, [5, 5, 5], q, tcp, bounds, actual[:3, :3],
            tool_axis_local=[0, 0, 1], tool_axis_cone_rad=math.radians(10), max_iterations=4)
        self.assertFalse(result['success'])
        with self.assertRaises(ValueError):
            solve_ik(self.model, actual[:3, 3], q, tcp, bounds, desired, tool_axis_cone_rad=.1)

    def test_common_seed_bank_is_deterministic_bounded_and_does_not_mutate_config(self):
        original = copy.deepcopy(self.cfg)
        a = endpoint_seeds(self.model, self.cfg, self.model.reference, 12)
        b = endpoint_seeds(self.model, self.cfg, self.model.reference, 12)
        np.testing.assert_array_equal(a, b)
        self.assertEqual(self.cfg, original)
        limits = np.asarray(self.cfg['joint_limits_encoder_rad'])
        self.assertTrue(np.all(np.asarray(a) >= limits[:, 0]))
        self.assertTrue(np.all(np.asarray(a) <= limits[:, 1]))

    def test_vectorized_search_fk_agrees_with_independent_scalar_fk(self):
        rng = np.random.default_rng(97)
        angles = self.model.reference+rng.uniform(-2., 2., (19, self.model.n))
        tcp = np.asarray(self.cfg['tcp_in_wrist_frame_m'])
        frames = batch_tool_frames(self.model, angles, tcp)
        for row, q in zip(frames, angles):
            np.testing.assert_allclose(row, tcp_pose(self.model, q, tcp), atol=1e-12)

    def test_global_search_proposals_obey_bounds_and_remain_unvalidated(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['continuous_grasp_search']['global_seed_generations'] = 3
        pose = tcp_pose(self.model, self.model.reference, cfg['tcp_in_wrist_frame_m'])
        seeds, audit = global_task_seeds(self.model, cfg, pose[:3, 3], pose[:3, :3],
                                         np.array([0., 0., 1.]), .1, lambda _: None)
        limits = np.asarray(cfg['joint_limits_encoder_rad'])
        self.assertFalse(audit['collision_acceptance'])
        self.assertTrue(np.all(np.asarray(seeds) >= limits[:, 0]))
        self.assertTrue(np.all(np.asarray(seeds) <= limits[:, 1]))

    def test_invalid_task_settings_fail_instead_of_dropping_orientation_constraint(self):
        for key, value in [('axis_cone_deg', 90), ('position_tolerance_mm', 20),
                           ('endpoint_seed_count', 1000)]:
            cfg = copy.deepcopy(self.cfg)
            cfg['continuous_grasp_search'][key] = value
            with self.assertRaises(ValueError):
                task_settings(cfg)
        cfg = copy.deepcopy(self.cfg)
        cfg['tool_orientation_constraint'] = 'axis-only'
        with self.assertRaises(ValueError):
            task_settings(cfg)
        cfg = copy.deepcopy(self.cfg)
        cfg['profiles']['white_knight']['wrist_rotation_in_board'][0][0] = 5.
        with self.assertRaises(ValueError):
            task_settings(cfg)

    def test_bounded_descent_audit_recomputes_fk_and_rejects_corrupt_pose(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['continuous_grasp_search']['descent_axis_cone_deg'] = 10.
        q = self.model.reference.copy()
        pose = tcp_pose(self.model, q, cfg['tcp_in_wrist_frame_m'])
        R = np.asarray(cfg['board_to_cad_rotation'])
        preferred = R.T@pose[:3, :3]
        cfg['profiles']['white_knight']['wrist_rotation_in_board'] = preferred.tolist()
        xyz = (pose[:3, 3]-cfg['board_to_cad_translation_m'])@R*1000
        row = dict(stage='contact', target_board_mm=xyz.tolist(),
                   encoder_rad=dict(zip(self.model.names, q.tolist())))
        result = dict(piece='white_knight', target_board_mm=xyz.tolist(),
                      selected_wrist_rotation_in_board=preferred.tolist(), waypoints=[row],
                      task_pose_search={'descent_axis_cone_deg':10.})
        audit = board.check_planned_tool_orientation(self.model, cfg, result)
        self.assertTrue(audit['passed'])
        self.assertEqual(audit['constraint'], 'bounded-axis-with-fixed-terminal-frame')
        result['waypoints'][0]['encoder_rad']['shoulder_front'] += .3
        result['waypoints'][0]['position_error_mm'] = 0.  # Claimed error is not evidence.
        with self.assertRaises(board.BoardPlanningError):
            board.check_planned_tool_orientation(self.model, cfg, result)

    def test_fixed_descent_checks_between_matching_endpoint_rotations(self):
        cfg=copy.deepcopy(self.cfg)
        cfg['continuous_grasp_search']['descent_axis_cone_deg']=0.
        q=self.model.reference.copy()
        pose=tcp_pose(self.model,q,cfg['tcp_in_wrist_frame_m'])
        R=np.asarray(cfg['board_to_cad_rotation'])
        # Same endpoint rotation, but the intermediate joint interpolation
        # would spin the wrist. Endpoint-only auditing must not accept it.
        end=q.copy();end[-1]+=2*math.pi
        route={'selected_wrist_rotation_in_board':(R.T@pose[:3,:3]).tolist(),
               'waypoints':[{'stage':s,'encoder_rad':dict(zip(self.model.names,v.tolist()))}
                            for s,v in [('source_above',q),('contact',end)]]}
        with self.assertRaises(board.BoardPlanningError) as caught:
            board.check_planned_tool_orientation(self.model,cfg,route)
        self.assertEqual(caught.exception.diagnostics['code'],'full_tool_orientation_mismatch')
        self.assertGreater(caught.exception.diagnostics['checked_task_edge_samples'],0)

    def test_strict_terminal_reference_is_resolved_with_fixed_descent(self):
        from left_arm_board_task_pose import plan_continuous_approach
        cfg=copy.deepcopy(self.cfg)
        cfg['continuous_grasp_search'].update(descent_axis_cone_deg=0.,terminal_reference_tolerance_deg=.2865)
        start=json.loads((self.here/'data/left_arm_v2_table_clearance.json').read_text())['joints']
        with mock.patch('left_arm_board_task_pose.solve_ik',return_value={'success':False}) as ik, \
             mock.patch('left_arm_board_task_pose.global_task_seeds',return_value=([],{})), \
             mock.patch('left_arm_board_task_pose.endpoint_seeds',return_value=[]):
            with self.assertRaises(board.BoardPlanningError):
                plan_continuous_approach(self.model,cfg,start,110.,165.,'white_knight',50.2,board)
        ik.assert_called_once()
        expected=np.asarray(cfg['board_to_cad_rotation'])@np.asarray(cfg['profiles']['white_knight']['wrist_rotation_in_board'])
        np.testing.assert_allclose(ik.call_args.args[5],expected)
        self.assertNotIn('tool_axis_cone_rad',ik.call_args.kwargs)

    def test_common_reference_bank_orders_by_fk_and_passes_requested_xyz(self):
        from left_arm_board_task_pose import plan_from_approved_references
        cfg = copy.deepcopy(self.cfg)
        q = np.array([cfg['seeds']['white_bishop_place'][n] for n in self.model.names])
        q2 = q.copy();q2[0] += .2
        frame = tcp_pose(self.model, q, cfg['tcp_in_wrist_frame_m'])
        R = np.asarray(cfg['board_to_cad_rotation'])
        xyz = (frame[:3, 3]-cfg['board_to_cad_translation_m'])@R*1000
        cfg['approved_task_pose_references'] = [dict(name='other',seed_encoder_rad=dict(zip(self.model.names,q2))),
            dict(name='near',seed_encoder_rad=dict(zip(self.model.names,q)))]
        for ref in cfg['approved_task_pose_references']:
            ref['wrist_rotation_in_board'] = (R.T@frame[:3,:3]).tolist()
        original = copy.deepcopy(cfg)
        with mock.patch('left_arm_board_task_pose.plan_continuous_approach',return_value={}) as solve, \
             mock.patch.object(board,'check_planned_tool_orientation',return_value={'passed':True}):
            result = plan_from_approved_references(self.model,cfg,{},*xyz[:2],'white_knight',xyz[2],board)
        self.assertEqual(result['approved_task_pose_reference'],'near')
        self.assertEqual(solve.call_args.args[3:5],tuple(xyz[:2]))
        self.assertFalse(result['reference_selection']['saved_angles_replayed'])
        self.assertEqual(cfg,original)
        from left_arm_board_task_pose import approved_profile
        with self.assertRaises(ValueError):approved_profile(cfg,'white_knight','unregistered')

    def test_mathematical_candidate_rejects_execute_before_opening_serial(self):
        args = ['board', '--model', str(self.here/'data/left_arm_general_dynamics.json'),
                '--board-model', str(self.here/'data/left_arm_board_workspace_continuous_candidate.json'),
                '--x-mm', '27.5', '--y-mm', '412.5', '--piece', 'white_knight',
                '--stage', 'approach', '--validate-candidate-approach', '--execute']
        with mock.patch.object(sys, 'argv', args), self.assertRaisesRegex(RuntimeError, 'offline-only'):
            board.main()

    def test_failed_primary_uses_nearest_validated_xy_and_resolves_new_target(self):
        from left_arm_board_task_pose import plan_from_approved_references
        cfg=copy.deepcopy(self.cfg)
        q=cfg['seeds']['white_bishop_place']
        frame=cfg['profiles']['white_knight']['wrist_rotation_in_board']
        cfg['validated_path_reference_policy']='primary-then-nearest-validated-path'
        cfg['reference_attempt_budget_s']=45.
        cfg['approved_task_pose_references']=[
            dict(name='far',reference_kind='validated_path',reference_board_xy_mm=[200.,300.],
                 seed_encoder_rad=q,wrist_rotation_in_board=frame),
            dict(name='primary',seed_encoder_rad=q,wrist_rotation_in_board=frame),
            dict(name='near',reference_kind='validated_path',reference_board_xy_mm=[100.,160.],
                 seed_encoder_rad=q,wrist_rotation_in_board=frame,
                 path_seed_encoder_rad=[q])]
        original=copy.deepcopy(cfg)
        with mock.patch('left_arm_board_task_pose.plan_continuous_approach',
                side_effect=[board.BoardPlanningError('unsolved',{'code':'test-unsolved'}),{}]) as solve, \
             mock.patch.object(board,'check_planned_tool_orientation',return_value={'passed':True}):
            result=plan_from_approved_references(self.model,cfg,{},110.,165.,'white_knight',30.2,board)
        self.assertEqual(result['approved_task_pose_reference'],'near')
        self.assertEqual(result['reference_selection']['search_phase'],'nearest-validated-path')
        self.assertEqual(result['reference_selection']['failed_references'][0]['reference'],'primary')
        for call in solve.call_args_list:
            self.assertEqual(call.args[3:7],(110.,165.,'white_knight',30.2))
        self.assertEqual(solve.call_args.args[1]['_reference_path_seeds'],[q])
        self.assertEqual(cfg,original)

    def test_reference_timeout_continues_but_total_deadline_still_stops(self):
        from left_arm_board_task_pose import plan_from_approved_references
        import time
        cfg=copy.deepcopy(self.cfg)
        cfg['validated_path_reference_policy']='primary-then-nearest-validated-path'
        cfg['reference_attempt_budget_s']=1.
        cfg['approved_task_pose_references']=[dict(name=n,
            seed_encoder_rad=cfg['seeds']['white_bishop_place'],
            wrist_rotation_in_board=cfg['profiles']['white_knight']['wrist_rotation_in_board'])
            for n in ('first','second')]
        with mock.patch('left_arm_board_task_pose.plan_continuous_approach',
                side_effect=[board.PlanningBudgetExceeded(),{}]), \
             mock.patch.object(board,'check_planned_tool_orientation',return_value={'passed':True}):
            result=plan_from_approved_references(self.model,cfg,{},110.,165.,'white_knight',30.2,board)
        self.assertEqual(result['reference_selection']['failed_references'][0]['details']['code'],
                         'reference_attempt_timeout')
        cfg['_planning_deadline']=time.monotonic()-1
        with mock.patch('left_arm_board_task_pose.plan_continuous_approach') as solve:
            with self.assertRaises(board.PlanningBudgetExceeded):
                plan_from_approved_references(self.model,cfg,{},110.,165.,'white_knight',30.2,board)
            solve.assert_not_called()


class CompleteTaskBranchTests(unittest.TestCase):
    def test_real_model_c6_whole_branch_has_fixed_frame_xy_and_checked_edges(self):
        here = Path(__file__).parent
        model = board.RigidBodyModel.load(here/'data/left_arm_general_dynamics.json')
        cfg = board.load_board_config(here/'data/left_arm_board_workspace_continuous_candidate.json', model)
        start = json.loads((here/'data/left_arm_v2_table_clearance.json').read_text())['joints']
        start['shoulder_rotate'] = 2.341306209564209
        original = copy.deepcopy(cfg)
        result = board.plan(model, cfg, start, 137.5, 302.5, 'white_knight', 'approach', 30.2)
        result = board.candidate_approach_trial(result, cfg)
        self.assertEqual(cfg, original)
        self.assertFalse(result['motion_issued'])
        self.assertFalse(result['posture_selection']['per_square_parameters'])
        self.assertEqual(result['posture_selection']['score'],
                         min(result['task_pose_search']['complete_branch_scores']))
        self.assertIn('source_above', result['entry_route'])
        self.assertEqual(result['waypoints'][-1]['stage'], 'contact')
        desired = np.asarray(cfg['board_to_cad_rotation'])@np.asarray(result['selected_wrist_rotation_in_board'])
        previous = np.array([start[n] for n in model.names])
        heights = []
        for row in result['waypoints']:
            q = np.array([row['encoder_rad'][n] for n in model.names])
            board.check_transit_edge(model, previous, q, cfg)
            if row['stage'] == 'contact':
                pose = tcp_pose(model, q, cfg['tcp_in_wrist_frame_m'])
                self.assertLess(np.linalg.norm(rotation_error(desired, pose[:3, :3])), .005+1e-8)
                xyz = (pose[:3, 3]-cfg['board_to_cad_translation_m'])@np.asarray(cfg['board_to_cad_rotation'])*1000
                self.assertLessEqual(np.linalg.norm(xyz-row['target_board_mm']), 1.084+1e-6)
                np.testing.assert_allclose(row['target_board_mm'][:2], [137.5, 302.5], atol=1e-8)
                heights.append(row['target_board_mm'][2])
            previous = q
        self.assertTrue(np.all(np.diff(heights) < 0))
        self.assertAlmostEqual(heights[-1], 30.2)
        # The held frame can differ from every taught full frame.
        distances = [np.linalg.norm(rotation_error(
            np.asarray(cfg['board_to_cad_rotation'])@np.asarray(h['wrist_rotation_in_board']), desired))
            for h in cfg['orientation_hints']]
        self.assertGreater(min(distances), .005)


if __name__ == '__main__':
    unittest.main()
