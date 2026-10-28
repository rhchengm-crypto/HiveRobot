import unittest
from pathlib import Path

import numpy as np

from left_arm_board_kinematics import (board_point_to_cad, estimate_board_and_tcp,
                                      plan_board_approach, rotation_error, solve_ik, tcp_pose)
from left_arm_rigid_body import RigidBodyModel, axis_rotation


class BoardKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RigidBodyModel.load(Path(__file__).parent/'data/left_arm_general_dynamics.json')
        cls.tcp = np.array([.03, -.07, -.14])
        # Synthetic test bounds, not deployment mechanical limits.
        cls.bounds = np.column_stack([cls.model.reference-1.5, cls.model.reference+1.5])

    def test_board_coordinates_are_metric_and_not_image_pixels(self):
        r = axis_rotation([0, 0, 1], np.pi/2)
        np.testing.assert_allclose(board_point_to_cad([55, 110, 20], r, [.1, .2, .3]),
                                   [-.01, .255, .32], atol=1e-12)
        for point in ([-1, 0, 55], [441, 0, 55], [0, 0, float('nan')]):
            with self.assertRaises(ValueError):
                board_point_to_cad(point, r, [0, 0, 0])

    def test_rotation_error_at_pi_is_not_zero(self):
        for axis in ([1, 0, 0], [0, 1, 1]):
            error = rotation_error(axis_rotation(axis, np.pi), np.eye(3))
            self.assertAlmostEqual(np.linalg.norm(error), np.pi)

    def test_ik_recovers_cartesian_pose_with_encoder_signs(self):
        rng = np.random.default_rng(89)
        for _ in range(12):
            goal = self.model.reference + rng.uniform(-.4, .4, self.model.n)
            frame = tcp_pose(self.model, goal, self.tcp)
            seed = goal + rng.uniform(-.08, .08, self.model.n)
            result = solve_ik(self.model, frame[:3, 3], seed, self.tcp, self.bounds,
                              frame[:3, :3], position_tolerance_m=.0001,
                              rotation_tolerance_rad=.001)
            self.assertTrue(result['success'], result)
            self.assertLess(result['position_error_mm'], .1)
            self.assertFalse(result['motion_issued'])

    def test_unreachable_target_does_not_return_success(self):
        result = solve_ik(self.model, [5, 5, 5], self.model.reference,
                          self.tcp, self.bounds)
        self.assertFalse(result['success'])
        angles = np.array(list(result['encoder_rad'].values()))
        self.assertTrue(np.all(angles >= self.bounds[:, 0]))
        self.assertTrue(np.all(angles <= self.bounds[:, 1]))

    def test_invalid_limits_and_seed_rejected(self):
        for limits, seed in ((None, self.model.reference),
                             (self.bounds, self.model.reference+4)):
            with self.assertRaises(ValueError):
                solve_ik(self.model, [0, 0, 0], seed, self.tcp, limits)

    def test_board_descent_and_lift_use_continuous_oriented_ik(self):
        seed = self.model.reference + np.array([.2, .1, -.2, .4, .1, -.2, .1])
        frame = tcp_pose(self.model, seed, self.tcp)
        board_point = np.array([137.5, 192.5, 55.])
        config = {'board_to_cad_rotation': np.eye(3).tolist(),
                  'board_to_cad_translation_m': (frame[:3, 3]-board_point/1000).tolist(),
                  'tcp_in_wrist_frame_m': self.tcp.tolist(),
                  'grasp_wrist_rotation_in_cad': frame[:3, :3].tolist(),
                  'joint_limits_encoder_rad': self.bounds.tolist()}
        plan = plan_board_approach(self.model, board_point, seed, config, clearance_mm=65)
        self.assertTrue(plan['success'], plan)
        self.assertEqual(set(p['stage'] for p in plan['waypoints']), {'source_above', 'contact', 'lift'})
        self.assertTrue(all(p['orientation_constrained'] for p in plan['waypoints']))
        self.assertFalse(plan['executable'])

    def test_joint_board_tcp_fit_on_independent_synthetic_correspondences(self):
        rng = np.random.default_rng(73)
        r = axis_rotation([1, 2, 3], .4)
        t = np.array([.1, -.2, .03])
        rows = []
        for i in range(18):
            enc = self.model.reference + rng.uniform(-.7, .7, self.model.n)
            world = tcp_pose(self.model, enc, self.tcp)[:3, 3]
            rows.append({'id': str(i), 'board_tcp_mm': ((world-t) @ r * 1000).tolist(),
                         'pose_rad': dict(zip(self.model.names, enc.tolist()))})
        fit = estimate_board_and_tcp(self.model, rows)
        np.testing.assert_allclose(fit['tcp_in_wrist_frame_m'], self.tcp, atol=1e-7)
        np.testing.assert_allclose(fit['board_to_cad_rotation'], r, atol=1e-7)
        self.assertLess(max(fit['fit_errors_mm'].values()), .001)
        self.assertFalse(fit['runtime_enabled'])


if __name__ == '__main__':
    unittest.main()
