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
    WHITE_BISHOP_PLACE_MOVE_NAME,
    PLACEMENT1_CONTAMINATED_CLEARANCE_ANCHOR,
    PLACEMENT1_PRECONTAMINATION_CLEARANCE_BIASES_DEG,
    blocking_joint_errors,
    build_parser,
    close_claw_while_holding_arm,
    claw_home_while_holding_arm,
    final_blocking_joint_errors,
    pose_distance_deg,
    recover_placement1_shared_clearance_contamination,
    replay_move_tau_with_wrist_side_support,
    verify_pre_wrist_or_learn,
    run_placement1_on_arm,
    run_white_bishop_place_on_arm,
    rollback_rejected_placement1_learning,
)


def pose(offset_deg=0.0):
    return {joint: math.radians(index * 5.0 + offset_deg) for index, joint in enumerate(JOINTS)}


class LocalBiasTests(unittest.TestCase):
    def test_white_bishop_claw_home_keeps_arm_held_through_open_and_settle(self):
        class Clock:
            now = 0.0
            def time(self):
                return self.now
            def sleep(self, duration):
                self.now += duration

        class Arm:
            def __init__(self):
                self.motors = {name: name for name in (*JOINTS, 'claw')}
                self.ctrl = types.SimpleNamespace(controlMIT=self.command)
                self.commands = []
                self.claw_pos = -1.0
            def command(self, motor, kp, kd, target, velocity, tau):
                self.commands.append((motor, target, tau))
                if motor == 'claw':
                    self.claw_pos = target
            def enable(self, names):
                self.enabled = list(names)
            def positions(self, names):
                return {name: .25 for name in names}
            def claw_status(self):
                return {'pos': self.claw_pos, 'vel': 0.0, 'tau': 0.0}

        arm = Arm()
        clock = Clock()
        api = types.SimpleNamespace(
            load_pose=lambda path: {'claw': 1.0},
            CLAW_HOME_PATH='claw-home.json',
            DEFAULT_JOINTS=JOINTS,
            CLEARANCE_HOLD_GAINS={name: {'kp': 2.0, 'kd': 1.0} for name in JOINTS},
            CLEARANCE_BASE_HOLD_GAINS={},
            CLAW_MOVE_SECONDS=.04,
            CLAW_KP_OPEN=22.0,
            CLAW_KD_OPEN=.8,
            CLAW_KP_HOLD=14.0,
            CLAW_KD_HOLD=.7,
            cosine_smoothstep=lambda fraction: min(1.0, max(0.0, fraction)),
        )
        with patch('left_arm_v2_8_move_library.time.time', clock.time), \
             patch('left_arm_v2_8_move_library.time.sleep', clock.sleep):
            status = claw_home_while_holding_arm(
                arm, api, 3.0, .3,
                carry_hold={'hold_tau': {'wrist': .55},
                            'hold_gains': {'wrist': {'kp': 8.0, 'kd': 1.4}}},
            )
        self.assertEqual(status['pos'], 1.0)
        self.assertEqual(arm.enabled, [*JOINTS, 'claw'])
        claw_indices = [i for i, command in enumerate(arm.commands)
                        if command[0] == 'claw']
        self.assertGreater(len(claw_indices), 3)
        for index in claw_indices:
            held = arm.commands[index-len(JOINTS):index]
            self.assertEqual([command[0] for command in held], list(JOINTS))
            self.assertEqual(held[-1][2], .55)
        self.assertEqual(arm.commands[-1][:2], ('claw', 1.0))

    def test_placement1_cli_is_an_explicit_replay_option(self):
        args = build_parser().parse_args([
            "replay-move", "--name", PLACEMENT1_MOVE_NAME, "--placement1"
        ])
        self.assertTrue(args.placement1)
        self.assertFalse(args.white_bishop_placement)
        self.assertEqual(args.name, "bishop01")

    def test_white_bishop_placement_cli_is_separate_and_implies_placement1_flow(self):
        args = build_parser().parse_args([
            "replay-move", "--name", PLACEMENT1_MOVE_NAME,
            "--white-bishop-placement",
        ])
        self.assertFalse(args.placement1)
        self.assertTrue(args.white_bishop_placement)

    def test_white_bishop_place_follows_clearance_and_holds_claw(self):
        class Ctrl:
            def controlMIT(self, *args):
                pass

        class Arm:
            def __init__(self):
                self.current = {joint: 0.0 for joint in JOINTS}
                self.ctrl = Ctrl()
                self.motors = {joint: object() for joint in (*JOINTS, "claw")}
                self.calls = []

            def enable(self, names):
                self.enabled = list(names)

            def positions(self, names):
                return {name: self.current[name] for name in names}

            def move_target_with_holds(self, name, target, **kwargs):
                self.calls.append(("single", name, dict(kwargs)))
                self.current[name] = target

            def move_targets_with_holds(self, targets, **kwargs):
                self.calls.append(("group", tuple(targets), dict(kwargs)))
                self.current.update(targets)

        target = {joint: 0.1 for joint in JOINTS}
        arm = Arm()
        api = types.SimpleNamespace(
            LeftArmV2=Arm,
            DEFAULT_JOINTS=list(JOINTS),
            CLEARANCE_HOLD_GAINS={joint: {"kp": 2.0, "kd": 1.0} for joint in JOINTS},
            CLEARANCE_BASE_HOLD_GAINS={joint: {"kp": 1.0, "kd": 1.0} for joint in JOINTS},
            adaptive_hold_gains_for=lambda active, gains, low: gains,
            CLAW_KP_HOLD=14.0,
            CLAW_KD_HOLD=2.0,
        )

        def run_non_wrist(current_arm, pose_value, replay_order, *args):
            targets = {joint: pose_value[joint] for joint in replay_order}
            current_arm.move_targets_with_holds(
                targets,
                hold_targets={"wrist": current_arm.current["wrist"]},
                hold_gains={"wrist": {"kp": 1.0, "kd": 1.0}},
                hold_tau={"wrist": 0.1},
            )
            return targets

        def run_final(current_arm, pose_value, *args):
            current_arm.move_target_with_holds(
                "wrist", pose_value["wrist"],
                hold_targets={joint: pose_value[joint] for joint in JOINTS[:-1]},
                hold_gains={joint: {"kp": 1.0, "kd": 1.0} for joint in JOINTS[:-1]},
                hold_tau={joint: 0.1 for joint in JOINTS[:-1]},
            )

        legacy = types.SimpleNamespace(
            load_moves=lambda path: {"moves": {
                WHITE_BISHOP_PLACE_MOVE_NAME: {"pose": target}
            }},
            normalize_move_name=lambda name: name,
            REPLAY_REQUIRED_JOINTS=list(JOINTS),
            resolve_replay_order=lambda record: (list(JOINTS[:-1]), "wrist"),
            is_low_shoulder_pose=lambda pose_value: False,
            REPLAY_ENTRY_HOLD_TAU={},
            REPLAY_ENTRY_TAKEOVER_SECONDS=0.0,
            REPLAY_ENTRY_TAKEOVER_CONTROL_DT=0.01,
            print_replay_error_report=lambda *args: {},
            run_adaptive_non_wrist_replay=run_non_wrist,
            run_replay_settle_pass=lambda *args, **kwargs: {},
            run_replay_correction_pass=lambda arm_value, pose_value, errors, *args: errors,
            run_final_wrist_move=run_final,
        )
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(
                Path(directory) / "bias.json", target, WHITE_BISHOP_PLACE_MOVE_NAME
            )
            result = run_white_bishop_place_on_arm(
                arm, "moves.json", 3.0, 0.3, 0.5,
                api, legacy, local, {"claw_hold_pos": -0.2},
            )
        self.assertEqual(result["status"], "validated")
        self.assertEqual(len(arm.calls), 2)
        for call in arm.calls:
            self.assertEqual(call[2]["hold_targets"]["claw"], -0.2)
            self.assertEqual(call[2]["hold_gains"]["claw"]["kp"], 14.0)
        self.assertIn("claw", arm.enabled)

    def test_placement1_carries_claw_hold_into_coupled_clearance(self):
        class Arm:
            def __init__(self):
                self.current = {joint: 0.0 for joint in JOINTS}
                self.hold_targets = None
                self.group_kwargs = None
                self.settle_kwargs = None
                self.events = []

            def positions(self, names):
                return {name: self.current[name] for name in names}

            def enable(self, names):
                self.enabled = list(names)

            def read_status(self, names):
                return {
                    name: {"pos": self.current[name], "vel": 0.0, "tau": 0.1}
                    for name in names
                }

            def move_target_with_holds(self, name, target, **kwargs):
                self.events.append(("single", name, target))
                self.current[name] = target

            def move_targets_with_holds(self, targets, **kwargs):
                self.events.append(("group", tuple(targets)))
                self.current.update(targets)
                self.hold_targets = dict(kwargs["hold_targets"])
                self.group_kwargs = kwargs

            def hold_positions_with_gains(self, *args, **kwargs):
                self.settle_kwargs = kwargs

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            clearance_path = root / "clearance.json"
            shared_bias_path = root / "shared-bias.json"
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
                CLEARANCE_JOINT_HOLD_TAU={"shoulder_front": 3.0},
                CLAW_KP_HOLD=14.0,
                CLAW_KD_HOLD=2.0,
                COUPLED_CLEARANCE_MOVE_GAINS={joint: {"kp": 1.0, "kd": 1.0} for joint in JOINTS},
                CLEARANCE_MOVE_GAINS={joint: {"kp": 1.0, "kd": 1.0} for joint in JOINTS},
                COUPLED_CLEARANCE_PROGRESS_WINDOWS={},
                COUPLED_CLEARANCE_PRE_WINDOW_GAINS={},
                COUPLED_CLEARANCE_CONTROL_DT=0.01,
                COUPLED_CLEARANCE_VELOCITY_FF_JOINTS=set(),
                COUPLED_CLEARANCE_MOVE_TAU_FF={"shoulder_front": 1.0},
                COUPLED_CLEARANCE_TRAJECTORY="smoothstep",
                COUPLED_CLEARANCE_LINEAR_BLEND=0.0,
                COUPLED_CLEARANCE_SETTLE_SECONDS=0.1,
                CLEARANCE_WRIST_FINE_DEADBAND_DEG=0.5,
            )
            inherited_targets = {joint: 0.2 + index * 0.01 for index, joint in enumerate(JOINTS)}
            inherited_gains = {joint: {"kp": 3.0, "kd": 0.5} for joint in JOINTS}
            inherited_tau = {joint: 0.1 + index * 0.01 for index, joint in enumerate(JOINTS)}
            with patch.dict(sys.modules, {"left_arm_v2_6": api}), patch(
                "left_arm_v2_8_move_library.PLACEMENT1_CLEARANCE_BIAS_PATH", bias_path
            ), patch(
                "left_arm_v2_8.CLEARANCE_BIAS_PATH", shared_bias_path
            ), patch(
                "left_arm_v2_8_move_library.close_claw_while_holding_arm", return_value=1.25
            ) as close_mock:
                result = run_placement1_on_arm(
                    arm, str(clearance_path), 3.0, 0.3,
                    carry_hold={
                        "hold_targets": inherited_targets,
                        "hold_gains": inherited_gains,
                        "hold_tau": inherited_tau,
                    },
                )
            self.assertEqual(result["status"], "training complete")
            self.assertEqual(result["blocking_errors_deg"], {})
            self.assertEqual(close_mock.call_args.args[1], inherited_targets)
            self.assertEqual(close_mock.call_args.kwargs["arm_tau"], inherited_tau)
            self.assertEqual(arm.events[0][0:2], ("single", "wrist"))
            self.assertEqual(arm.events[1][0], "group")
            self.assertEqual(arm.hold_targets["claw"], 1.25)
            self.assertIn("wrist", arm.hold_targets)
            self.assertNotIn("shoulder_rotate", arm.hold_targets)
            self.assertIn("shoulder_rotate", arm.events[1][1])
            self.assertIn("claw", arm.enabled)
            self.assertEqual(
                arm.settle_kwargs["gains"]["shoulder_front"],
                api.COUPLED_CLEARANCE_MOVE_GAINS["shoulder_front"],
            )
            self.assertEqual(arm.settle_kwargs["hold_tau"]["shoulder_front"], 1.0)
            self.assertNotEqual(arm.settle_kwargs["hold_tau"]["shoulder_front"], 3.0)
            self.assertTrue(bias_path.exists())
            shared = LocalTargetBias(shared_bias_path, clearance, "clearance:" + clearance_path.name)
            self.assertEqual(shared.anchor.get("hold_bias", {}).get("clearance", {}), {})

    def test_recovers_known_shared_clearance_contamination_once(self):
        target = {
            "shoulder_front": 1.9357976913452148,
            "shoulder_side": -2.7143893241882324,
            "elbow": 0.8165484070777893,
            "shoulder_rotate": 1.9323643445968628,
            "arm_roll": -2.649538516998291,
            "wrist_side": 0.3057526648044586,
            "wrist": -1.0313191413879395,
        }
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", target, "clearance:test")
            self.assertEqual(local.anchor_id, PLACEMENT1_CONTAMINATED_CLEARANCE_ANCHOR)
            rules = local.anchor.setdefault("hold_bias", {}).setdefault("clearance", {})
            for joint, value in {"arm_roll": 0.28, "wrist_side": 1.25, "wrist": -0.66}.items():
                rules[joint] = {"bias_deg": value, "samples": 8, "last_error_deg": 1.5}
            local._save()
            recovered = recover_placement1_shared_clearance_contamination(local)
            self.assertEqual(set(recovered), {"arm_roll", "wrist_side", "wrist"})
            for joint, expected in PLACEMENT1_PRECONTAMINATION_CLEARANCE_BIASES_DEG.items():
                self.assertAlmostEqual(math.degrees(local.hold_bias_rad("clearance", joint)), expected)
            self.assertEqual(recover_placement1_shared_clearance_contamination(local), {})
            forced = recover_placement1_shared_clearance_contamination(local, force=True)
            self.assertEqual(set(forced), {"arm_roll", "wrist_side", "wrist"})

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
            first = local.update({"wrist": 2.0})["wrist"]
            update = local.update({"wrist": 3.0})["wrist"]
            self.assertEqual(update["learning_state"], "backoff")
            self.assertLess(update["step_scale"], 1.0)
            self.assertEqual(update["previous_bias_deg"], first["bias_deg"])
            self.assertEqual(update["bias_deg"], update["best_bias_deg"])
            self.assertEqual(update["delta_bias_deg"], -first["bias_deg"])

    def test_old_backoff_step_is_restored_when_bias_file_is_loaded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bias.json"
            local = LocalTargetBias(path, pose(), "move")
            local.anchor["joint_bias"]["wrist_side"] = {
                "bias_deg": -0.12021312231688155,
                "previous_bias_deg": 0.0,
                "delta_bias_deg": -0.12021312231688155,
                "last_error_deg": -1.2021312231688155,
                "best_error_deg": 0.5245662122197955,
                "best_bias_deg": 0.0,
                "step_scale": 0.2,
                "learning_state": "backoff",
                "samples": 6,
            }
            local._save()
            reloaded = LocalTargetBias(path, pose(), "move")
            record = reloaded.anchor["joint_bias"]["wrist_side"]
            self.assertEqual(record["bias_deg"], 0.0)
            self.assertEqual(record["previous_bias_deg"], -0.12021312231688155)
            self.assertEqual(record["learning_state"], "legacy_backoff_restored")

    def test_wrist_side_bias_is_reset_once_for_active_tau_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "move")
            local.anchor.setdefault("joint_bias", {})["wrist_side"] = {
                "bias_deg": -0.12,
                "best_bias_deg": 0.0,
                "samples": 7,
            }
            local._save()
            reset = local.reset_wrist_side_bias_for_active_tau()
            self.assertEqual(reset, {"previous_bias_deg": -0.12, "new_bias_deg": 0.0})
            self.assertEqual(local.anchor["joint_bias"]["wrist_side"]["bias_deg"], 0.0)
            self.assertEqual(local.reset_wrist_side_bias_for_active_tau(), {})

    def test_low_shoulder_wrist_side_uses_existing_support_tau(self):
        legacy = types.SimpleNamespace(REPLAY_LOW_SHOULDER_WRIST_SIDE_HOLD_TAU=-0.35)
        original = lambda name, delta, low: 0.0
        self.assertEqual(
            replay_move_tau_with_wrist_side_support(
                original, legacy, "wrist_side", -7.4, True
            ),
            -0.35,
        )
        self.assertEqual(
            replay_move_tau_with_wrist_side_support(
                original, legacy, "elbow", -7.4, True
            ),
            0.0,
        )

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

    def test_placement_hold_bias_backs_off_when_error_worsens(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "placement")
            first = local.update_hold_bias(
                "clearance", {"wrist_side": 3.4}, backoff_on_worse=True
            )["wrist_side"]
            second = local.update_hold_bias(
                "clearance", {"wrist_side": 3.7}, backoff_on_worse=True
            )["wrist_side"]
            self.assertEqual(second["learning_state"], "backoff")
            self.assertEqual(second["bias_deg"], first["best_bias_deg"])
            self.assertLess(second["step_scale"], first["step_scale"])

    def test_recovers_observed_placement_wrist_side_worsening_once(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "placement")
            rules = local.anchor.setdefault("hold_bias", {}).setdefault("clearance", {})
            rules["wrist_side"] = {
                "bias_deg": 1.2,
                "previous_bias_deg": 0.8,
                "last_error_deg": 3.650109441515467,
                "samples": 3,
            }
            local._save()
            recovered = local.recover_known_placement1_wrist_side_worsening()
            self.assertEqual(recovered["restored_bias_deg"], 0.8)
            self.assertEqual(rules["wrist_side"]["bias_deg"], 0.8)
            self.assertEqual(rules["wrist_side"]["step_scale"], 0.5)
            self.assertEqual(local.recover_known_placement1_wrist_side_worsening(), {})

    def test_rejected_placement_wrist_side_step_is_not_retried_at_full_scale(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "placement")
            rules = local.anchor.setdefault("hold_bias", {}).setdefault("clearance", {})
            rules["wrist_side"] = {
                "bias_deg": 1.2,
                "previous_bias_deg": 0.8,
                "last_error_deg": 3.43,
                "best_error_deg": 3.45,
                "best_bias_deg": 0.8,
                "step_scale": 1.0,
            }
            local.anchor.setdefault("migrations", {})[
                "restore-placement1-wrist-side-20260911-2043-v1"
            ] = {"rejected_bias_deg": 1.2, "restored_bias_deg": 0.8}
            local._save()
            recovered = local.avoid_repeating_rejected_placement1_wrist_side_step()
            self.assertEqual(recovered["restored_bias_deg"], 0.8)
            self.assertEqual(recovered["step_scale"], 0.5)
            self.assertEqual(rules["wrist_side"]["bias_deg"], 0.8)

    def test_wrist_hold_bias_is_context_specific_and_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "move")
            for _ in range(30):
                local.update_hold_bias("wrist", {"shoulder_rotate": 2.0})
            self.assertAlmostEqual(math.degrees(local.hold_bias_rad("wrist", "shoulder_rotate")), 3.0)
            self.assertEqual(local.hold_bias_rad("wrist_side", "shoulder_rotate"), 0.0)

    def test_placement_hold_bias_can_use_five_degree_local_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            local = LocalTargetBias(Path(directory) / "bias.json", pose(), "placement")
            for _ in range(20):
                local.update_hold_bias(
                    "clearance", {"shoulder_rotate": -2.0},
                    bias_limit_deg=5.0,
                )
            self.assertAlmostEqual(
                math.degrees(local.hold_bias_rad("clearance", "shoulder_rotate", limit_deg=5.0)),
                -5.0,
            )
            self.assertAlmostEqual(
                math.degrees(local.hold_bias_rad("clearance", "shoulder_rotate")),
                -3.0,
            )

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
