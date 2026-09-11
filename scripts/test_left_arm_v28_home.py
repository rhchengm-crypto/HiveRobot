import unittest
import tempfile
import types
from pathlib import Path
from unittest.mock import patch

from left_arm_v2_8 import (
    clearance_errors_deg,
    clearance_validation_joints,
    captured_home_transition_limit,
    ensure_clearance_best_snapshot,
    fine_correct_clearance_wrist_side,
    is_fixed_prehome,
    option_value,
    remove_flag,
    replace_option,
    run_trained_clearance,
)
from left_arm_v2_8_move_library import JOINTS
from left_arm_v2_8_move_library import LocalTargetBias


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

    def test_best_snapshot_is_created_once(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", {
                joint: 0.0 for joint in JOINTS
            }, "clearance:test")
            local.anchor.setdefault("hold_bias", {}).setdefault("clearance", {})["wrist_side"] = {
                "bias_deg": 0.65
            }
            self.assertTrue(ensure_clearance_best_snapshot(local))
            local.anchor["hold_bias"]["clearance"]["wrist_side"]["bias_deg"] = 1.2
            self.assertFalse(ensure_clearance_best_snapshot(local))
            saved = local.anchor["best_snapshots"]["clearance"]["hold_bias"]
            self.assertEqual(saved["wrist_side"]["bias_deg"], 0.65)

    def test_wrist_side_fine_correction_holds_every_other_joint(self):
        class Arm:
            def __init__(self):
                self.current = {joint: 0.0 for joint in JOINTS}
                self.call = None

            def positions(self, names):
                return {name: self.current[name] for name in names}

            def move_target_with_holds(self, name, target, **kwargs):
                self.call = (name, target, kwargs)
                self.current[name] = math.radians(1.0)

        import math
        arm = Arm()
        nominal = {joint: 0.0 for joint in JOINTS}
        nominal["wrist_side"] = math.radians(1.0)
        legacy = types.SimpleNamespace(
            DEFAULT_JOINTS=list(JOINTS),
            CLEARANCE_HOLD_GAINS={joint: {"kp": 2.0, "kd": 1.0} for joint in JOINTS},
            CLEARANCE_BASE_HOLD_GAINS={joint: {"kp": 1.0, "kd": 1.0} for joint in JOINTS},
            CLEARANCE_JOINT_HOLD_TAU={},
            COUPLED_CLEARANCE_CONTROL_DT=0.01,
        )
        result = fine_correct_clearance_wrist_side(arm, nominal, list(JOINTS), legacy)
        self.assertAlmostEqual(result["wrist_side"], nominal["wrist_side"])
        self.assertEqual(arm.call[0], "wrist_side")
        self.assertEqual(set(arm.call[2]["hold_targets"]), set(JOINTS) - {"wrist_side"})


if __name__ == "__main__":
    unittest.main()
