import unittest
import tempfile
import types
from pathlib import Path
from unittest.mock import patch

from left_arm_v2_8 import (
    clearance_errors_deg,
    clearance_validation_joints,
    captured_home_transition_limit,
    is_fixed_prehome,
    option_value,
    remove_flag,
    replace_option,
    run_trained_clearance,
)
from left_arm_v2_8_move_library import JOINTS


class HomeWrapperTests(unittest.TestCase):
    def test_only_home_with_clearance_is_intercepted(self):
        self.assertTrue(is_fixed_prehome(["home", "--prehome-clearance", "--execute"]))
        self.assertFalse(is_fixed_prehome(["home", "--execute"]))
        self.assertFalse(is_fixed_prehome(["clearance", "--execute"]))

    def test_force_pass_and_correction_arguments(self):
        original = ["home", "--deadband-deg", "0.5", "--prehome-clearance", "--execute"]
        forced = replace_option(original, "--deadband-deg", "-1.0")
        self.assertEqual(option_value(forced, "--deadband-deg", "missing"), "-1.0")
        correction = remove_flag(original, "--prehome-clearance")
        self.assertNotIn("--prehome-clearance", correction)
        self.assertEqual(option_value(correction, "--deadband-deg", "missing"), "0.5")

    def test_missing_option_is_added(self):
        result = replace_option(["home"], "--deadband-deg", "-1.0")
        self.assertEqual(result, ["home", "--deadband-deg", "-1.0"])

    def test_clearance_validation_omits_intentionally_skipped_rotate(self):
        class Legacy:
            CLEARANCE_JOINT_DEADBANDS_DEG = {"shoulder_rotate": 180.0}
        joints = clearance_validation_joints(Legacy, ["shoulder_side", "shoulder_rotate", "elbow"])
        self.assertEqual(joints, ["shoulder_side", "elbow"])

    def test_clearance_errors_compare_with_nominal_capture(self):
        errors = clearance_errors_deg(
            {"shoulder_side": 1.0},
            {"shoulder_side": 1.0 - 0.017453292519943295},
            ["shoulder_side"],
        )
        self.assertAlmostEqual(errors["shoulder_side"], 1.0)

    def test_captured_home_transition_gets_small_boundary_margin(self):
        self.assertAlmostEqual(captured_home_transition_limit(120.0, [20.0, 121.02]), 121.12)
        self.assertEqual(captured_home_transition_limit(120.0, [126.0]), 120.0)

    def test_replace_option_expands_clearance_command_limit(self):
        original = ["clearance", "--max-delta-deg", "120", "--execute"]
        expanded = replace_option(original, "--max-delta-deg", str(120.0 + 5.0))
        self.assertEqual(option_value(expanded, "--max-delta-deg", "missing"), "125.0")

    def test_history_recovery_returns_before_opening_motor_controller(self):
        target = {
            "shoulder_front": 1.9357976913452148,
            "shoulder_side": -2.7143893241882324,
            "elbow": 0.8165484070777893,
            "shoulder_rotate": 1.9323643445968628,
            "arm_roll": -2.649538516998291,
            "wrist_side": 0.3057526648044586,
            "wrist": -1.0313191413879395,
        }
        called = []
        parsed = types.SimpleNamespace(
            max_delta_deg=120.0,
            clearance_file="clearance.json",
            joints=",".join(JOINTS),
        )
        parser = types.SimpleNamespace(parse_args=lambda argv: parsed)
        legacy = types.SimpleNamespace(
            build_parser=lambda: parser,
            load_pose=lambda path: target,
            parse_joints=lambda value: list(JOINTS),
            CLEARANCE_JOINT_DEADBANDS_DEG={joint: 0.0 for joint in JOINTS},
            main=lambda: called.append("motor-controller-opened"),
        )
        with tempfile.TemporaryDirectory() as directory, patch(
            "left_arm_v2_8.CLEARANCE_BIAS_PATH", Path(directory) / "bias.json"
        ):
            run_trained_clearance(["clearance", "--execute"], legacy)
        self.assertEqual(called, [])


if __name__ == "__main__":
    unittest.main()
