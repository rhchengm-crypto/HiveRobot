import math
import tempfile
import unittest
from pathlib import Path

from left_arm_v2_8_move_library import (
    LocalTargetBias,
    JOINTS,
    blocking_joint_errors,
    final_blocking_joint_errors,
    pose_distance_deg,
    verify_pre_wrist_or_learn,
)


def pose(offset_deg=0.0):
    return {joint: math.radians(index * 5.0 + offset_deg) for index, joint in enumerate(JOINTS)}


class LocalBiasTests(unittest.TestCase):
    def test_nearby_pose_reuses_anchor_and_distant_pose_does_not(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bias.json"
            first = LocalTargetBias(path, pose(), "pawn01")
            first.update({"elbow": 2.0})
            nearby = LocalTargetBias(path, pose(2.0), "bishop01")
            self.assertEqual(nearby.anchor_id, first.anchor_id)
            self.assertAlmostEqual(math.degrees(nearby.bias_rad("elbow")), 0.75)
            distant = LocalTargetBias(path, pose(30.0), "far")
            self.assertNotEqual(distant.anchor_id, first.anchor_id)
            self.assertEqual(distant.bias_rad("elbow"), 0.0)

    def test_backoff_uses_best_bias_when_error_gets_worse(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "move")
            local.update({"wrist": 2.0})
            update = local.update({"wrist": 3.0})["wrist"]
            self.assertEqual(update["learning_state"], "backoff")
            self.assertLess(update["step_scale"], 1.0)
            self.assertLessEqual(abs(update["bias_deg"]), 5.0)

    def test_pose_distance_is_in_degrees(self):
        rms, maximum = pose_distance_deg(pose(), pose(4.0))
        self.assertAlmostEqual(rms, 4.0)
        self.assertAlmostEqual(maximum, 4.0)

    def test_half_degree_learning_deadband(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "move")
            self.assertEqual(local.update({"elbow": 0.49}), {})
            self.assertIn("elbow", local.update({"elbow": 0.51}))

    def test_pre_wrist_gate_excludes_wrist_and_reports_other_joints(self):
        blockers = blocking_joint_errors({"shoulder_front": -0.52, "elbow": 0.5, "wrist": 3.0})
        self.assertEqual(blockers, {"shoulder_front": -0.52})

    def test_pre_wrist_gate_allows_half_encoder_count_at_boundary(self):
        blockers = blocking_joint_errors({"shoulder_front": -0.5027087531021085})
        self.assertEqual(blockers, {})

    def test_final_gate_includes_wrist(self):
        blockers = final_blocking_joint_errors({"shoulder_front": 0.5, "wrist": 0.52})
        self.assertEqual(blockers, {"wrist": 0.52})

    def test_pre_wrist_gate_learns_without_issuing_motion(self):
        class Arm:
            def __init__(self):
                self.current = {joint: 0.0 for joint in JOINTS}
                self.current["shoulder_front"] = math.radians(1.0)
            def positions(self, joints):
                return {joint: self.current[joint] for joint in joints}
        arm = Arm()
        target_pose = {joint: 0.0 for joint in JOINTS}
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", target_pose, "move")
            with self.assertRaisesRegex(RuntimeError, "no extra correction motion"):
                verify_pre_wrist_or_learn(arm, target_pose, local)
            self.assertIn("shoulder_front", local.anchor["joint_bias"])

    def test_pre_wrist_gate_allows_wrist_when_all_joints_are_inside_tolerance(self):
        class Arm:
            def positions(self, joints):
                return {joint: 0.0 for joint in joints}
        target_pose = {joint: 0.0 for joint in JOINTS}
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", target_pose, "move")
            errors = verify_pre_wrist_or_learn(Arm(), target_pose, local)
            self.assertEqual(blocking_joint_errors(errors), {})

    def test_equal_wrist_induced_errors_continue_integrating(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "move")
            first = local.update_hold_bias("wrist", {"elbow": 0.72})["elbow"]
            second = local.update_hold_bias("wrist", {"elbow": 0.72})["elbow"]
            self.assertGreater(second["bias_deg"], first["bias_deg"])
            self.assertEqual(second["learning_state"], "integrating")

    def test_wrist_hold_bias_is_context_specific_and_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "move")
            for _ in range(30):
                local.update_hold_bias("wrist", {"shoulder_rotate": 2.0})
            self.assertAlmostEqual(math.degrees(local.hold_bias_rad("wrist", "shoulder_rotate")), 3.0)
            self.assertEqual(local.hold_bias_rad("wrist_side", "shoulder_rotate"), 0.0)

    def test_validation_is_stored_per_move_even_when_anchor_is_shared(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bias.json"
            first = LocalTargetBias(path, pose(), "pawn01")
            first.record_move_validation({joint: 0.1 for joint in JOINTS})
            second = LocalTargetBias(path, pose(2.0), "bishop01")
            second.record_move_validation({**{joint: 0.1 for joint in JOINTS}, "wrist": 0.8})
            validations = second.anchor["move_validation"]
            self.assertEqual(validations["pawn01"]["status"], "validated")
            self.assertEqual(validations["bishop01"]["status"], "training")


if __name__ == "__main__":
    unittest.main()
