import unittest

from left_arm_v2_8_claw import lead_limited_target, resisting_torque


class GuardedClawTests(unittest.TestCase):
    def test_positive_close_uses_negative_torque_as_resistance(self):
        self.assertAlmostEqual(resisting_torque(-0.31, 1.0), 0.31)
        self.assertEqual(resisting_torque(0.31, 1.0), 0.0)

    def test_target_is_limited_by_schedule_and_live_position(self):
        self.assertAlmostEqual(lead_limited_target(-4.9, -4.7, 1.1, 1.0), -4.8)
        self.assertAlmostEqual(lead_limited_target(-4.9, -4.88, 1.1, 1.0), -4.88)

    def test_negative_direction_has_symmetric_limits(self):
        self.assertAlmostEqual(lead_limited_target(4.9, 4.7, -1.1, -1.0), 4.8)


if __name__ == "__main__":
    unittest.main()
