import json
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from left_arm_v2_8_move_library import (
    LocalTargetBias,
    JOINTS,
    PLACEMENT1_MOVE_NAME,
    blocking_joint_errors,
    build_parser,
    close_claw_while_holding_arm,
    final_blocking_joint_errors,
    pose_distance_deg,
    verify_pre_wrist_or_learn,
    run_placement1_on_arm,
    rollback_rejected_placement1_learning,
)


def pose(offset_deg=0.0):
    return {joint: math.radians(index * 5.0 + offset_deg) for index, joint in enumerate(JOINTS)}


class LocalBiasTests(unittest.TestCase):
    def test_placement1_cli_is_an_explicit_replay_option(self):
        args = build_parser().parse_args([
            "replay-move", "--name", PLACEMENT1_MOVE_NAME, "--placement1"
        ])
        self.assertTrue(args.placement1)
        self.assertEqual(args.name, "bishop01")

    def test_placement1_carries_claw_hold_into_coupled_clearance(self):
        class Arm:
            def __init__(self):
                self.current = {joint: 0.0 for joint in JOINTS}
                self.hold_targets = None

            def positions(self, names):
                return {name: self.current[name] for name in names}

            def enable(self, names):
                self.enabled = list(names)

            def move_targets_with_holds(self, targets, **kwargs):
                self.current.update(targets)
                self.hold_targets = dict(kwargs["hold_targets"])

            def hold_positions_with_gains(self, *args, **kwargs):
                pass

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            clearance_path = root / "clearance.json"
            bias_path = root / "placement-bias.json"
            clearance = {joint: 0.1 for joint in JOINTS}
            clearance_path.write_text(json.dumps({"joints": clearance}), encoding="utf-8")
            arm = Arm()
            api = types.SimpleNamespace(
                load_pose=lambda path: json.loads(Path(path).read_text(encoding="utf-8"))["joints"],
                DEFAULT_CLEARANCE_ORDER=list(JOINTS),
                DEFAULT_JOINTS=list(JOINTS),
                CLEARANCE_JOINT_DEADBANDS_DEG={"shoulder_rotate": 180.0},
                HOME_GAINS={joint: {"seconds": 1.0} for joint in JOINTS},
                COUPLED_CLEARANCE_MAX_SECONDS=12.0,
                CLEARANCE_BASE_HOLD_GAINS={joint: {"kp": 1.0, "kd": 1.0} for joint in JOINTS},
                CLEARANCE_HOLD_GAINS={joint: {"kp": 2.0, "kd": 1.0} for joint in JOINTS},
                CLEARANCE_JOINT_HOLD_TAU={},
                CLAW_KP_HOLD=14.0,
                CLAW_KD_HOLD=2.0,
                COUPLED_CLEARANCE_MOVE_GAINS={joint: {"kp": 1.0, "kd": 1.0} for joint in JOINTS},
                CLEARANCE_MOVE_GAINS={joint: {"kp": 1.0, "kd": 1.0} for joint in JOINTS},
                COUPLED_CLEARANCE_PROGRESS_WINDOWS={},
                COUPLED_CLEARANCE_PRE_WINDOW_GAINS={},
                COUPLED_CLEARANCE_CONTROL_DT=0.01,
                COUPLED_CLEARANCE_VELOCITY_FF_JOINTS=set(),
                COUPLED_CLEARANCE_MOVE_TAU_FF={},
                COUPLED_CLEARANCE_TRAJECTORY="smoothstep",
                COUPLED_CLEARANCE_LINEAR_BLEND=0.0,
                COUPLED_CLEARANCE_SETTLE_SECONDS=0.1,
                CLEARANCE_WRIST_FINE_DEADBAND_DEG=0.5,
            )
            with patch.dict(sys.modules, {"left_arm_v2_6": api}), patch(
                "left_arm_v2_8.CLEARANCE_BIAS_PATH", bias_path
            ), patch(
                "left_arm_v2_8_move_library.close_claw_while_holding_arm", return_value=1.25
            ):
                run_placement1_on_arm(arm, str(clearance_path), 3.0, 0.3)
            self.assertEqual(arm.hold_targets["claw"], 1.25)
            self.assertIn("shoulder_rotate", arm.hold_targets)
            self.assertIn("claw", arm.enabled)

    def test_gross_placement_failure_rolls_back_its_clearance_learning(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bias.json"
            target = {joint: 0.0 for joint in JOINTS}
            local = LocalTargetBias(path, target, "clearance:clearance.json")
            local.update_hold_bias(
                "clearance", {"wrist": 20.0, "arm_roll": -1.0},
                label="placement1-clearance-final",
            )
            local.record_move_validation({"wrist": 89.0, "arm_roll": -1.0})
            restored = rollback_rejected_placement1_learning(local)
            self.assertEqual(set(restored), {"wrist", "arm_roll"})
            self.assertAlmostEqual(local.hold_bias_rad("clearance", "wrist"), 0.0)
            self.assertAlmostEqual(local.hold_bias_rad("clearance", "arm_roll"), 0.0)

    def test_placement1_stops_when_claw_pressure_contact_is_not_detected(self):
        clock = [0.0]

        class Ctrl:
            def controlMIT(self, *args):
                pass

        class Arm:
            ctrl = Ctrl()
            motors = {"elbow": object(), "claw": object()}

            def enable(self, names):
                pass

            def claw_status(self):
                return {"pos": 1.0, "vel": 1.0, "tau": 0.0}

        api = types.SimpleNamespace(
            load_pose=lambda path: {"claw": 0.0},
            CLAW_HOME_PATH="claw.json",
            CLEARANCE_HOLD_GAINS={"elbow": {"kp": 2.0, "kd": 1.0}},
            CLEARANCE_BASE_HOLD_GAINS={"elbow": {"kp": 1.0, "kd": 1.0}},
            CLAW_CLOSE_OFFSET=1.0,
            CLAW_CLOSE_SECONDS=0.03,
            cosine_smoothstep=lambda value: value,
            CLAW_KP_MOVE=1.0,
            CLAW_KD_MOVE=1.0,
            CLAW_TAU_THRESHOLD=10.0,
            CLAW_VEL_STALL_THRESHOLD=0.01,
            CLAW_STALL_TAU_THRESHOLD=10.0,
            CLAW_CONFIRM_COUNT_NEEDED=2,
            CLAW_BACKOFF=0.1,
            CLAW_KP_HOLD=1.0,
            CLAW_KD_HOLD=1.0,
        )

        def sleep(seconds):
            clock[0] += seconds

        with patch("left_arm_v2_8_move_library.time.time", side_effect=lambda: clock[0]), patch(
            "left_arm_v2_8_move_library.time.sleep", side_effect=sleep
        ):
            with self.assertRaisesRegex(RuntimeError, "pressure contact was not detected"):
                close_claw_while_holding_arm(
                    Arm(), {"elbow": 0.5}, api,
                    arm_gains={"elbow": {"kp": 2.0, "kd": 1.0}},
                    arm_tau={"elbow": 2.0},
                )

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
