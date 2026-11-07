import copy
import json
from pathlib import Path
import unittest
from unittest import mock

import numpy as np

from left_arm_board_collision import BodyCollisionScene, segment_box_clearance
from left_arm_board_kinematics import solve_ik
import left_arm_board_grasp as board
from left_arm_board_transit import connect_path
from left_arm_rigid_body import RigidBodyModel


class LinearModel:
    """Two redundant Cartesian coordinates, only for constraint tests."""
    n = 2
    names = ('a', 'b')
    signs = np.ones(2)

    def encoder_to_model(self, q):
        return np.asarray(q)

    def fk(self, q):
        frames = np.array([np.eye(4), np.eye(4)])
        frames[0, 0, 3] = q[0]
        frames[1, 0, 3] = sum(q)
        return frames

    def point_jacobian(self, q, index, point):
        return np.array([[1., 1.], [0., 0.], [0., 0.]]), np.zeros((3, 2))


class CollisionTests(unittest.TestCase):
    def test_whole_segment_crossing_detected_when_endpoints_are_clear(self):
        gap, fraction = segment_box_clearance([-2., 0, 0], [2., 0, 0],
                                              np.zeros(3), np.eye(3), np.ones(3))
        self.assertAlmostEqual(gap, -1.)
        self.assertAlmostEqual(fraction, .5)
        gap, _ = segment_box_clearance([2., 2., 0], [3., 2., 0],
                                       np.zeros(3), np.eye(3), np.ones(3))
        self.assertEqual(gap, 1.)  # Conservative vs Euclidean sqrt(2).

    def test_rotated_box_uses_declared_frame(self):
        axes = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        a = np.array([2., .4, 0.])
        center = np.array([.1, .3, .4])
        gap, _ = segment_box_clearance(axes@a+center, axes@a+center,
                                       center, axes, np.array([1., .1, .5]))
        self.assertAlmostEqual(gap, 1.)

    def test_ik_redistributes_redundant_coordinates_for_collision_constraint(self):
        m = LinearModel()
        bounds = np.array([[-1., 1.], [-1., 1.]])
        result = solve_ik(m, [0., 0., 0.], [-.01, .01], [0., 0., 0.], bounds,
                          clearance_function=lambda q: np.array([q[0]]),
                          position_tolerance_m=1e-5)
        self.assertTrue(result['success'], result)
        self.assertGreaterEqual(result['encoder_rad']['a'], 0)
        self.assertAlmostEqual(sum(result['encoder_rad'].values()), 0., places=5)
        self.assertTrue(result['body_constraints_checked'])

    def test_unavoidable_collision_never_reports_success(self):
        m = LinearModel()
        result = solve_ik(m, [0., 0., 0.], [0., 0.], [0., 0., 0.],
                          [[-1., 1.], [-1., 1.]],
                          clearance_function=lambda q: np.array([-1.]))
        self.assertFalse(result['success'])

    def test_invalid_nonfinite_collision_function_rejected(self):
        with self.assertRaisesRegex(ValueError, 'clearance function'):
            solve_ik(LinearModel(), [0., 0., 0.], [0., 0.], [0., 0., 0.],
                     [[-1., 1.], [-1., 1.]], clearance_function=lambda q: [np.nan])

    def test_body_scene_validation_and_whole_transit(self):
        cfg = {'tcp_in_wrist_frame_m': [0., 0., .02],
               'board_to_cad_rotation': np.eye(3).tolist(),
               'board_to_cad_translation_m': [0., 0., -10.],
               'body_collision': {'schema': 1, 'enabled': True, 'frame': 'cad_base',
                   'obstacles': [dict(name='test', center_m=[0., 0., 0.],
                                     axes_in_cad=np.eye(3).tolist(), half_extents_m=[.1, .1, .1])],
                   'capsules': [dict(name='tool', start_index=1, end_index=2,
                                     radius_m=.01, margin_m=.005)]}}
        m = LinearModel()
        scene = BodyCollisionScene(m, cfg)
        self.assertTrue(np.all(scene([-1., 0.]) > 0))
        self.assertTrue(np.all(scene([1., 0.]) > 0))
        with self.assertRaises(board.BoardPlanningError) as caught:
            board.check_transit_edge(m, np.array([-1., 0.]), np.array([1., 0.]), cfg)
        self.assertEqual(caught.exception.diagnostics['code'], 'body_collision')
        self.assertTrue(0 < caught.exception.diagnostics['transit_fraction'] < 1)
        broken = copy.deepcopy(cfg)
        broken['body_collision']['obstacles'][0]['axes_in_cad'][0][0] = 2.
        with self.assertRaises(ValueError):
            BodyCollisionScene(m, broken)

    def test_offline_scene_rejected_before_any_enable(self):
        arm = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'offline candidate'):
            board.execute(arm, None, {}, geometry=(None, {
                'body_collision': {'enabled': True, 'physically_validated': False}}))
        arm.enable.assert_not_called()
        arm.disable.assert_not_called()

    def test_analytic_clearance_jacobian_matches_encoder_perturbations(self):
        here = Path(__file__).parent
        model = RigidBodyModel.load(here/'data/left_arm_general_dynamics.json')
        cfg = json.loads((here/'data/left_arm_board_workspace_candidate.json').read_text(encoding='utf-8'))
        scene = BodyCollisionScene(model, cfg)
        rng = np.random.default_rng(381)
        for _ in range(8):
            q = model.reference+rng.uniform(-.3, .3, model.n)
            numerical = np.column_stack([(scene(q+np.eye(model.n)[k]*1e-6)
                -scene(q-np.eye(model.n)[k]*1e-6))/2e-6 for k in range(model.n)])
            np.testing.assert_allclose(scene.jacobian(q), numerical, atol=1e-6, rtol=1e-4)

    def test_transit_search_detours_without_shortcutting_obstacle(self):
        def clear(a, b):
            points = np.linspace(a, b, 101)
            return not np.any((np.abs(points[:, 0]) < .15) & (np.abs(points[:, 1]) < .5))
        start, goal = np.array([-.8, 0.]), np.array([.8, 0.])
        self.assertFalse(clear(start, goal))
        route = connect_path(start, goal, np.array([[-1., 1.], [-1., 1.]]), clear)
        self.assertIsNotNone(route)
        np.testing.assert_array_equal(route[0], start)
        np.testing.assert_allclose(route[-1], goal, atol=1e-12)
        self.assertGreater(len(route), 2)
        for a, b in zip(route, route[1:]):
            self.assertTrue(clear(a, b))
            self.assertTrue(np.all(np.abs(b) <= 1.))


if __name__ == '__main__':
    unittest.main()
