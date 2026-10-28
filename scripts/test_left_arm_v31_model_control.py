import math
import tempfile
import types
import unittest
from unittest import mock
from pathlib import Path

from left_arm_v3_1_model_control import (
    JOINTS,
    SlewLimitedFilter,
    empty_model,
    gravity_prediction,
    inertia_prediction,
    load_model,
    pose_distance_deg,
    save_model,
    scheduled_terms,
    static_hold_bias,
    hold_load_support,
    supported_hold_terms,
    preview_schedule,
    configure,
    configure_friction,
    build_parser,
    analyze_inertia_trace,
    inertia_passive_hold_torques,
    reprocess_inertia,
    remove_latest_gravity,
    remove_latest_inertia,
    validate_ready,
    wait_until_still,
    confirm_velocity_watchdog,
    model_hold,
)


def pose(offset=0.0):
    return {joint: offset + index * 0.05 for index, joint in enumerate(JOINTS)}


class ModelControlTests(unittest.TestCase):
    def populated_model(self):
        model = empty_model()
        target = pose()
        for index, joint in enumerate(JOINTS):
            for delta in (-0.1, 0.0, 0.1):
                sample_pose = dict(target)
                sample_pose[joint] += delta
                model["joints"][joint]["gravity_samples"].append(
                    {"pose_rad": sample_pose, "tau": 0.2 + index * 0.01 + delta}
                )
            model["joints"][joint]["inertia_samples"].append(
                {"pose_rad": target, "effective_inertia": 2.0 + index * 0.1}
            )
        return model, target

    def test_pose_distance_uses_full_pose(self):
        left = pose()
        right = pose()
        right["elbow"] += math.radians(14.0)
        self.assertAlmostEqual(pose_distance_deg(left, right), math.sqrt(14.0 ** 2 / 7.0), places=6)

    def test_nearest_gravity_and_inertia_predictions(self):
        model, target = self.populated_model()
        gravity, gdiag = gravity_prediction(model, "elbow", target)
        inertia, idiag = inertia_prediction(model, "elbow", target)
        self.assertAlmostEqual(gravity, 0.23)
        self.assertAlmostEqual(inertia, 2.3)
        self.assertEqual(gdiag["nearest_distance_deg"], 0.0)
        self.assertEqual(idiag["nearest_distance_deg"], 0.0)

    def test_prediction_excludes_neighbors_beyond_distance_limit(self):
        target = pose()
        near_pose = dict(target)
        far_pose = dict(target)
        near_pose["elbow"] += math.radians(7.0)
        far_pose["elbow"] += math.radians(70.0)
        samples = [
            {"pose_rad": near_pose, "tau": 1.0},
            {"pose_rad": far_pose, "tau": 100.0},
        ]
        from left_arm_v3_1_model_control import inverse_distance_prediction
        value, diagnostic = inverse_distance_prediction(
            samples, target, "tau", neighbors=2, max_distance_deg=10.0
        )
        self.assertAlmostEqual(value, 1.0)
        self.assertEqual(diagnostic["neighbors_used"], 1)
        self.assertEqual(len(diagnostic["neighbor_distances_deg"]), 1)

    def test_formula_and_clamps_are_applied(self):
        model, target = self.populated_model()
        scheduler = model["joints"]["elbow"]["scheduler"]
        scheduler.update({"omega_n": 3.0, "zeta": 0.8, "kp_min": 0.0, "kp_max": 100.0, "kd_min": 0.0, "kd_max": 100.0})
        result = scheduled_terms(model, "elbow", target)
        self.assertAlmostEqual(result["kp"], 3.0 ** 2 * 2.3)
        self.assertAlmostEqual(result["kd"], 5.0)
        self.assertTrue(result["kd_clamped"])
        self.assertAlmostEqual(result["tau_ff"], 0.23)

    def test_minimum_gain_fallback_requires_explicit_joint_configuration(self):
        model, target = self.populated_model()
        remote_pose = pose(math.radians(100.0))
        model["joints"]["shoulder_rotate"]["inertia_samples"][0]["pose_rad"] = remote_pose
        with self.assertRaisesRegex(ValueError, "nearest effective_inertia sample"):
            scheduled_terms(model, "shoulder_rotate", target)

        scheduler = model["joints"]["shoulder_rotate"]["scheduler"]
        scheduler["inertia_fallback"] = "minimum-gains"
        result = scheduled_terms(model, "shoulder_rotate", target)
        self.assertEqual(result["gain_source"], "minimum-gains-without-local-inertia")
        self.assertIsNone(result["effective_inertia"])
        self.assertEqual(result["kp"], scheduler["kp_min"])
        self.assertEqual(result["kd"], scheduler["kd_min"])
        self.assertEqual(result["inertia_diagnostic"]["source"], "minimum-gains-fallback")

    def test_minimum_gain_fallback_does_not_bypass_gravity_coverage(self):
        model, target = self.populated_model()
        model["joints"]["shoulder_rotate"]["scheduler"]["inertia_fallback"] = "minimum-gains"
        model["joints"]["shoulder_rotate"]["gravity_samples"] = []
        with self.assertRaisesRegex(ValueError, "no usable tau samples"):
            scheduled_terms(model, "shoulder_rotate", target)

    def test_coulomb_friction_blends_through_zero_velocity(self):
        model, target = self.populated_model()
        model["joints"]["wrist"]["friction"].update({
            "coulomb": 0.17, "viscous": 0.0, "identified": True,
        })
        stopped = scheduled_terms(model, "wrist", target, velocity=0.0)
        quantized = scheduled_terms(model, "wrist", target, velocity=0.002)
        moving = scheduled_terms(model, "wrist", target, velocity=0.5)
        self.assertAlmostEqual(stopped["friction_tau"], 0.0)
        self.assertLess(abs(quantized["friction_tau"]), 0.01)
        self.assertAlmostEqual(moving["friction_tau"], 0.17, places=6)

    def test_static_hold_bias_preserves_measured_friction_branch(self):
        model, target = self.populated_model()
        model["joints"]["wrist"]["friction"].update({
            "coulomb": 0.17, "viscous": 0.0, "identified": True,
        })
        base = scheduled_terms(model, "wrist", target, 0.0)
        result = static_hold_bias(model, "wrist", target, base["tau_ff"] - 0.13)
        self.assertAlmostEqual(result["static_hold_bias_tau"], -0.13)
        self.assertAlmostEqual(result["tau_ff"], base["tau_ff"] - 0.13)
        self.assertFalse(result["static_hold_bias_clamped"])

    def test_static_hold_bias_is_limited_by_identified_coulomb(self):
        model, target = self.populated_model()
        model["joints"]["wrist"]["friction"].update({
            "coulomb": 0.17, "viscous": 0.0, "identified": True,
        })
        base = scheduled_terms(model, "wrist", target, 0.0)
        result = static_hold_bias(model, "wrist", target, base["tau_ff"] + 1.0)
        self.assertAlmostEqual(result["static_hold_bias_tau"], 0.17)
        self.assertAlmostEqual(result["tau_ff"], base["tau_ff"] + 0.17)
        self.assertTrue(result["static_hold_bias_clamped"])

    def test_configure_rejects_kd_above_mit_protocol_range(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            save_model(path, empty_model())
            args = types.SimpleNamespace(
                model=str(path), joint="shoulder_front",
                omega_n=None, zeta=None, kp_min=None, kp_max=None,
                kd_min=None, kd_max=5.1, inertia_fallback=None,
                tau_min=None, tau_max=None,
            )
            with self.assertRaisesRegex(ValueError, "MIT protocol range"):
                configure(args)

    def test_configure_updates_tau_limit_within_motor_protocol_range(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            save_model(path, empty_model())
            args = types.SimpleNamespace(
                model=str(path), joint="shoulder_front",
                omega_n=None, zeta=None, kp_min=None, kp_max=None,
                kd_min=None, kd_max=None, inertia_fallback=None,
                tau_min=-4.8, tau_max=None,
            )
            result = configure(args)
            self.assertEqual(result["tau_limits"], [-4.8, 3.8])
            self.assertEqual(load_model(path)["joints"]["shoulder_front"]["tau_limits"], [-4.8, 3.8])

    def test_configure_rejects_tau_limit_outside_motor_protocol_range(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            save_model(path, empty_model())
            args = types.SimpleNamespace(
                model=str(path), joint="shoulder_front",
                omega_n=None, zeta=None, kp_min=None, kp_max=None,
                kd_min=None, kd_max=None, inertia_fallback=None,
                tau_min=-28.1, tau_max=None,
            )
            with self.assertRaisesRegex(ValueError, "MIT protocol range"):
                configure(args)

    def test_inertia_passive_hold_prefers_identified_gravity(self):
        model = empty_model()
        target = pose()
        model["joints"]["shoulder_front"]["gravity_samples"].append(
            {"pose_rad": target, "tau": 2.7}
        )
        torques, diagnostics = inertia_passive_hold_torques(model, "elbow", target)
        self.assertAlmostEqual(torques["shoulder_front"], 2.7)
        self.assertEqual(diagnostics["shoulder_front"]["source"], "identified_gravity")
        self.assertAlmostEqual(torques["wrist"], 0.55)
        self.assertEqual(diagnostics["wrist"]["source"], "legacy_fallback")

    def test_slew_limiter_prevents_gain_and_torque_steps(self):
        filt = SlewLimitedFilter(cutoff_hz=100.0, kp_slew=2.0, kd_slew=1.0, tau_slew=0.5)
        filt.seed("elbow", 2.0, 0.5, 0.0)
        result = filt.update("elbow", {"kp": 20.0, "kd": 5.0, "tau_ff": 2.0}, 0.1)
        self.assertAlmostEqual(result["kp"], 2.2)
        self.assertAlmostEqual(result["kd"], 0.6)
        self.assertAlmostEqual(result["tau_ff"], 0.05)

    def test_execution_requires_data_and_unit_verification(self):
        model, _ = self.populated_model()
        with self.assertRaisesRegex(ValueError, "gain_units_verified"):
            validate_ready(model, JOINTS)
        model["gain_units_verified"] = True
        validate_ready(model, JOINTS)
        model["joints"]["wrist"]["gravity_samples"] = []
        with self.assertRaisesRegex(ValueError, "wrist"):
            validate_ready(model, JOINTS)

    def test_model_hold_uses_validated_watchdog_defaults(self):
        args = build_parser().parse_args(["model-hold"])
        self.assertEqual(args.max_velocity, 0.3)
        self.assertEqual(args.max_error_deg, 2.5)

    def test_model_round_trip(self):
        model, _ = self.populated_model()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            save_model(path, model)
            loaded = load_model(path)
        self.assertEqual(loaded["schema"], 1)
        self.assertEqual(len(loaded["joints"]["elbow"]["gravity_samples"]), 3)

    def test_remove_latest_gravity_removes_only_selected_joint_without_motion(self):
        model, _ = self.populated_model()
        expected = model["joints"]["shoulder_side"]["gravity_samples"][-1]
        elbow_count = len(model["joints"]["elbow"]["gravity_samples"])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            save_model(path, model)
            result = remove_latest_gravity(types.SimpleNamespace(
                model=str(path), joint="shoulder_side",
            ))
            loaded = load_model(path)
        self.assertTrue(result["removed"])
        self.assertFalse(result["motion_issued"])
        self.assertEqual(result["sample"], expected)
        self.assertEqual(result["remaining"], 2)
        self.assertEqual(len(loaded["joints"]["elbow"]["gravity_samples"]), elbow_count)

    def test_friction_configuration_rejects_negative_magnitudes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            save_model(path, empty_model())
            args = types.SimpleNamespace(
                model=str(path), joint="elbow", coulomb=-0.1, viscous=0.2,
                method="test",
            )
            with self.assertRaisesRegex(ValueError, "non-negative"):
                configure_friction(args)

    def test_unit_verification_flags_work_without_boolean_optional_action(self):
        parser = build_parser()
        self.assertTrue(parser.parse_args(["set-units-verified"]).verified)
        self.assertTrue(parser.parse_args(["set-units-verified", "--verified"]).verified)
        self.assertFalse(parser.parse_args(["set-units-verified", "--no-verified"]).verified)

    def test_inertia_trace_estimator_recovers_second_order_mass(self):
        dt = 0.02
        mass = 2.0
        kp = 40.0
        kd = 3.0
        target = math.radians(2.0)
        pos = 0.0
        vel = 0.0
        trace = []
        for index in range(40):
            trace.append({
                "t": index * dt, "start_pos": 0.0, "target": target,
                "pos": pos, "vel": vel, "tau": 0.0,
            })
            acceleration = (kp * (target - pos) - kd * vel) / mass
            vel += acceleration * dt
            pos += vel * dt
        result = analyze_inertia_trace(trace, step_rad=target, test_kp=kp, test_kd=kd)
        self.assertAlmostEqual(result["effective_inertia"], mass, delta=0.35)
        self.assertGreater(result["usable_samples"], 3)

    def test_inertia_estimator_removes_static_feedforward_offset(self):
        dt = 0.02
        mass = 2.0
        kp = 40.0
        kd = 3.0
        step = math.radians(2.0)
        baseline_target = -0.04
        start_pos = 0.0
        pos = start_pos
        vel = 0.0
        trace = []
        for index in range(40):
            trace.append({
                "t": index * dt, "start_pos": start_pos,
                "baseline_target": baseline_target,
                "target": baseline_target + step,
                "pos": pos, "vel": vel, "tau": 0.0,
            })
            incremental_error = ((baseline_target + step) - pos) - (baseline_target - start_pos)
            acceleration = (kp * incremental_error - kd * vel) / mass
            vel += acceleration * dt
            pos += vel * dt
        result = analyze_inertia_trace(trace, step_rad=step, test_kp=kp, test_kd=kd)
        self.assertAlmostEqual(result["effective_inertia"], mass, delta=0.35)

    def test_inertia_shape_fit_handles_quantized_velocity(self):
        dt = 0.02
        mass = 0.2
        kp = 40.0
        kd = 3.0
        step = math.radians(2.0)
        pos = 0.0
        vel = 0.0
        trace = []
        for index in range(60):
            trace.append({
                "t": index * dt, "start_pos": 0.0, "baseline_target": 0.0,
                "target": step, "pos": pos, "vel": 0.0, "tau": 0.0,
            })
            for _ in range(10):
                sub_dt = dt / 10.0
                acceleration = (kp * (step - pos) - kd * vel) / mass
                vel += acceleration * sub_dt
                pos += vel * sub_dt
        result = analyze_inertia_trace(trace, step_rad=step, test_kp=kp, test_kd=kd)
        self.assertEqual(result["estimator"], "normalized_closed_loop_step_fit")
        self.assertAlmostEqual(result["effective_inertia"], mass, delta=0.05)

    def test_partial_noisy_step_rejects_acceleration_ratio(self):
        dt = 0.02
        mass = 0.2
        kp = 40.0
        kd = 3.0
        step = math.radians(-2.0)
        pos = 0.0
        vel = 0.0
        trace = []
        for index in range(60):
            trace.append({
                "t": index * dt, "start_pos": 0.0, "baseline_target": 0.0,
                "target": step, "pos": pos,
                "vel": vel + (0.05 if index % 3 == 0 else -0.03), "tau": 0.0,
            })
            for _ in range(10):
                sub_dt = dt / 10.0
                acceleration = (kp * (step - pos) - kd * vel) / mass
                vel += acceleration * sub_dt
                pos += vel * sub_dt
        result = analyze_inertia_trace(trace, step_rad=step, test_kp=kp, test_kd=kd)
        self.assertEqual(result["estimator"], "normalized_closed_loop_step_fit")
        self.assertAlmostEqual(result["effective_inertia"], mass, delta=0.06)

    def test_incomplete_step_is_rejected_before_shape_fit(self):
        step = math.radians(-2.0)
        trace = []
        for index in range(60):
            fraction = min(0.55, index / 35.0 * 0.55)
            trace.append({
                "t": index * 0.02, "start_pos": 0.0, "baseline_target": 0.0,
                "target": step, "pos": step * fraction, "vel": 0.0, "tau": 0.0,
            })
        with self.assertRaisesRegex(ValueError, "55.0%"):
            analyze_inertia_trace(trace, step_rad=step, test_kp=40.0, test_kd=3.0)

    def test_shape_fit_rejects_search_boundary_solution(self):
        step = math.radians(2.0)
        trace = []
        for index in range(60):
            trace.append({
                "t": index * 0.02, "start_pos": 0.0, "baseline_target": 0.0,
                "target": step, "pos": 0.0 if index == 0 else step,
                "vel": 0.0, "tau": 0.0,
            })
        with self.assertRaisesRegex(ValueError, "search boundary|physically implausible"):
            analyze_inertia_trace(trace, step_rad=step, test_kp=40.0, test_kd=3.0)

    def test_loaded_offset_preflight_rejects_high_load_step(self):
        from left_arm_v3_1_model_control import validate_inertia_loaded_offset

        with self.assertRaisesRegex(ValueError, "loaded position offset"):
            validate_inertia_loaded_offset(
                start_pos=math.radians(1.6),
                baseline_target=0.0,
                step_rad=math.radians(2.0),
            )
        offset = validate_inertia_loaded_offset(
            start_pos=math.radians(0.2),
            baseline_target=0.0,
            step_rad=math.radians(2.0),
        )
        self.assertAlmostEqual(offset, 0.2)

    def test_reprocess_replaces_prior_sample_from_same_capture(self):
        dt = 0.02
        mass = 0.2
        kp = 40.0
        kd = 3.0
        step = math.radians(-2.0)
        pos = 0.0
        vel = 0.0
        trace = []
        for index in range(60):
            trace.append({
                "t": index * dt, "start_pos": 0.0, "baseline_target": 0.0,
                "target": step, "pos": pos, "vel": 0.0, "tau": 0.0,
            })
            for _ in range(10):
                sub_dt = dt / 10.0
                acceleration = (kp * (step - pos) - kd * vel) / mass
                vel += acceleration * sub_dt
                pos += vel * sub_dt
        captured_at = "2026-10-13 17:40:01"
        record = {
            "joint": "elbow", "captured_at": captured_at,
            "pose_rad": pose(), "step_deg": -2.0,
            "test_kp": kp, "test_kd": kd, "trace": trace,
        }
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "model.json"
            log_path = Path(directory) / "traces.jsonl"
            model = empty_model()
            model["joints"]["elbow"]["inertia_samples"].append({
                "captured_at": captured_at, "effective_inertia": 4.1,
                "method": "guarded_small_step_acceleration_fit",
            })
            save_model(model_path, model)
            log_path.write_text(__import__("json").dumps(record) + "\n", encoding="utf-8")
            result = reprocess_inertia(types.SimpleNamespace(
                model=str(model_path), log=str(log_path), joint="elbow", label="corrected",
            ))
            saved = load_model(model_path)["joints"]["elbow"]["inertia_samples"]
        self.assertEqual(result["replaced"], 1)
        self.assertFalse(result["motion_issued"])
        self.assertEqual(len(saved), 1)
        self.assertEqual(saved[0]["method"], "normalized_closed_loop_step_fit")

    def test_remove_latest_inertia_is_no_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "model.json"
            model = empty_model()
            model["joints"]["elbow"]["inertia_samples"] = [
                {"captured_at": "first", "effective_inertia": 0.05},
                {"captured_at": "second", "effective_inertia": 0.005},
            ]
            save_model(model_path, model)
            result = remove_latest_inertia(types.SimpleNamespace(
                model=str(model_path), joint="elbow",
            ))
            saved = load_model(model_path)["joints"]["elbow"]["inertia_samples"]
        self.assertFalse(result["motion_issued"])
        self.assertEqual(result["sample"]["captured_at"], "second")
        self.assertEqual(len(saved), 1)

    def test_measure_inertia_parser_has_guarded_defaults(self):
        args = build_parser().parse_args(["measure-inertia", "--joint", "elbow"])
        self.assertEqual(args.step_deg, 2.0)
        self.assertEqual(args.test_kp, 40.0)
        self.assertFalse(args.execute)

    def test_guarded_nudge_parser_is_low_gain_and_opt_in(self):
        args = build_parser().parse_args(["guarded-nudge", "--joint", "wrist"])
        self.assertEqual(args.step_deg, 2.0)
        self.assertEqual(args.test_kp, 6.0)
        self.assertEqual(args.test_kd, 0.8)
        self.assertEqual(args.max_gravity_distance_deg, 5.0)
        self.assertEqual(args.max_active_gravity_tau, 0.15)
        self.assertFalse(args.execute)

    def test_model_hold_parser_preflight_is_opt_in(self):
        args = build_parser().parse_args(["model-hold"])
        self.assertEqual(args.preflight_still_timeout, 0.0)
        self.assertEqual(args.preflight_still_seconds, 0.5)
        self.assertEqual(args.preflight_max_velocity, 0.1)
        self.assertEqual(args.preflight_max_drift_deg, 0.25)
        self.assertFalse(args.motors_already_enabled)
        self.assertEqual(args.velocity_spike_max_drift_deg, 0.1)

        takeover = build_parser().parse_args(
            ["model-hold", "--motors-already-enabled"]
        )
        self.assertTrue(takeover.motors_already_enabled)

    def test_reused_motor_takeover_sends_bridge_without_enabling(self):
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "model.json"
            log_path = Path(directory) / "runs.jsonl"
            model = empty_model()
            model["gain_units_verified"] = True
            sample_pose = {joint: 0.0 for joint in JOINTS}
            joint_model = model["joints"]["shoulder_rotate"]
            joint_model["gravity_samples"] = [
                {"pose_rad": sample_pose, "tau": 0.0} for _ in range(3)
            ]
            joint_model["inertia_samples"] = [
                {"pose_rad": sample_pose, "effective_inertia": 0.01}
            ]
            save_model(model_path, model)
            args = build_parser().parse_args([
                "--model", str(model_path), "model-hold",
                "--joints", "shoulder_rotate", "--seconds", "0",
                "--log", str(log_path), "--motors-already-enabled", "--execute",
            ])
            status = {
                joint: {"pos": 0.0, "vel": 0.0, "tau": 0.0}
                for joint in JOINTS
            }
            arm = mock.Mock()
            arm.read_status.return_value = status
            arm.motors = {joint: object() for joint in JOINTS}
            with mock.patch(
                "left_arm_v3_1_model_control.open_arm", return_value=(mock.Mock(), arm)
            ):
                result = model_hold(args)
        arm.enable.assert_not_called()
        arm.ctrl.controlMIT.assert_called_once()
        self.assertTrue(result["seamless_takeover"]["enabled"])
        self.assertEqual(
            result["seamless_takeover"]["mode"],
            "immediate-current-state-bridge",
        )

    def test_hold_support_preserves_load_beyond_model_limits_for_both_signs(self):
        model, target = self.populated_model()
        cases = {"shoulder_rotate": -2.43, "shoulder_side": 1.88, "wrist_side": 0.97}
        for joint, measured in cases.items():
            for sign in (1.0, -1.0):
                with self.subTest(joint=joint, sign=sign):
                    measured_tau = measured * sign
                    original_limits = list(model["joints"][joint]["tau_limits"])
                    support = hold_load_support(model, joint, target, measured_tau, 0.02, 0.4)
                    self.assertAlmostEqual(support["bridge_tau_ff"] - 0.4 * 0.02, measured_tau)
                    steady = supported_hold_terms(model, joint, target, support)
                    self.assertAlmostEqual(steady["tau_ff"], measured_tau)
                    self.assertFalse(steady["load_support_clamped"])
                    self.assertEqual(model["joints"][joint]["tau_limits"], original_limits)
                    self.assertLessEqual(support["hold_tau_limits"][1], max(original_limits[1], support["bridge_tau_ff"], measured_tau))

    def test_hold_support_rejects_invalid_feedback_before_sending_commands(self):
        model, target = self.populated_model()
        for torque in (float("nan"), float("inf"), 10.1):
            with self.subTest(torque=torque), self.assertRaises(ValueError):
                hold_load_support(model, "shoulder_rotate", target, torque, 0.0, 0.4)

    def test_load_residual_keeps_model_corrections_inside_observed_envelope(self):
        model, target = self.populated_model()
        support = hold_load_support(model, "shoulder_rotate", target, -2.43, 0.0, 0.4)
        baseline = scheduled_terms(model, "shoulder_rotate", target)["tau_ff"]
        # Gravity prediction may change as other joints settle. Retain its
        # correction, while never demanding more inherited negative support
        # than was observed when the hold began.
        with mock.patch("left_arm_v3_1_model_control.scheduled_terms") as terms:
            terms.return_value = dict(scheduled_terms(model, "shoulder_side", target))
            terms.return_value["tau_ff"] = baseline + 0.1
            corrected = supported_hold_terms(model, "shoulder_rotate", target, support)
            self.assertAlmostEqual(corrected["tau_ff"], -2.33)
            terms.return_value["tau_ff"] = baseline - 0.1
            bounded = supported_hold_terms(model, "shoulder_rotate", target, support)
            self.assertAlmostEqual(bounded["tau_ff"], -2.43)
            self.assertTrue(bounded["load_support_clamped"])

    def test_loaded_takeover_holds_physics_for_full_duration_without_torque_decay(self):
        # Reproduce the three loads clipped in the 2026-10-20 Placement1 log.
        # An inertial plant accelerates immediately if takeover discards load.
        loads = {"shoulder_rotate": -2.43, "shoulder_side": 1.88, "wrist_side": 0.97}
        loads.update({joint: 0.1 for joint in JOINTS if joint not in loads})
        class Plant:
            def __init__(self):
                self.now = 0.0
                self.q = dict.fromkeys(JOINTS, 0.0)
                self.v = dict.fromkeys(JOINTS, 0.0)
                self.motors = {joint: joint for joint in JOINTS}
                self.ctrl = types.SimpleNamespace(controlMIT=self.command)
                self.commands = {}
                self.history = []
                self.enable = mock.Mock()
                self.close = mock.Mock()
            def command(self, joint, kp, kd, q, dq, tau):
                self.commands[joint] = (kp, kd, q, tau)
                self.history.append((self.now, joint, kp, kd, q, tau))
            def read_status(self, joints):
                result = {}
                for joint in joints:
                    output = loads[joint]
                    if joint in self.commands:
                        kp, kd, target, tau = self.commands[joint]
                        output = kp * (target - self.q[joint]) - kd * self.v[joint] + tau
                    result[joint] = {"pos": self.q[joint], "vel": self.v[joint], "tau": output}
                return result
            def sleep(self, seconds):
                steps = max(1, math.ceil(seconds / 0.0005))
                dt = seconds / steps
                for _ in range(steps):
                    for joint, (kp, kd, target, tau) in self.commands.items():
                        output = kp * (target - self.q[joint]) - kd * self.v[joint] + tau
                        self.v[joint] += (output - loads[joint]) / 0.02 * dt
                        self.q[joint] += self.v[joint] * dt
                    self.now += dt
        plant = Plant()
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "model.json"
            model = empty_model()
            model["gain_units_verified"] = True
            zero_pose = dict.fromkeys(JOINTS, 0.0)
            gravity = {"shoulder_rotate": -0.12454, "shoulder_side": 0.67692,
                       "wrist_side": 0.21490}
            coulomb = {"shoulder_rotate": 0.22, "wrist_side": 0.17}
            for joint in JOINTS:
                model["joints"][joint]["gravity_samples"] = [
                    {"pose_rad": zero_pose, "tau": gravity.get(joint, 0.1)} for _ in range(3)
                ]
                model["joints"][joint]["friction"]["coulomb"] = coulomb.get(joint, 0.0)
                model["joints"][joint]["inertia_samples"] = [
                    {"pose_rad": zero_pose, "effective_inertia": 0.02}
                ]
            save_model(model_path, model)
            original_model = model_path.read_bytes()
            args = build_parser().parse_args([
                "--model", str(model_path), "model-hold", "--seconds", "5",
                "--log", str(Path(directory) / "runs.jsonl"),
                "--motors-already-enabled", "--execute",
            ])
            with mock.patch("left_arm_v3_1_model_control.open_arm", return_value=(None, plant)), \
                 mock.patch("left_arm_v3_1_model_control.time.time", side_effect=lambda: plant.now), \
                 mock.patch("left_arm_v3_1_model_control.time.sleep", side_effect=plant.sleep):
                result = model_hold(args)
            self.assertEqual(model_path.read_bytes(), original_model)
        self.assertTrue(result["ok"])
        self.assertGreaterEqual(result["duration_s"], 5.0)
        plant.enable.assert_not_called()
        for joint, load in loads.items():
            self.assertLess(result["max_abs_error_deg"][joint], 0.01)
            self.assertLess(result["max_velocity_rad_s"][joint], 0.001)
            outputs = [row for row in plant.history if row[1] == joint]
            self.assertGreater(len(outputs), 400)
            for row in (outputs[0], outputs[len(outputs)//2], outputs[-1]):
                self.assertAlmostEqual(row[-1], load)

    def test_preview_matches_loaded_hold_and_explicit_pose_needs_no_arm(self):
        import json
        model, target = self.populated_model()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            save_model(path, model)
            args = build_parser().parse_args(["--model", str(path), "preview",
                                              "--joints", "shoulder_rotate"])
            arm = mock.Mock()
            arm.read_status.return_value = {
                joint: {"pos": target[joint], "vel": 0.0, "tau": -2.43}
                for joint in JOINTS
            }
            with mock.patch("left_arm_v3_1_model_control.open_arm", return_value=(None, arm)):
                result = preview_schedule(args)
            entry = result["model_hold_takeover"]["shoulder_rotate"]
            self.assertAlmostEqual(entry["takeover_tau_ff"], -2.43)
            self.assertAlmostEqual(entry["supported_hold_tau_ff"], -2.43)
            args.pose_json = json.dumps(target)
            with mock.patch("left_arm_v3_1_model_control.open_arm") as open_mock:
                result = preview_schedule(args)
            open_mock.assert_not_called()
            self.assertEqual(result["model_hold_takeover"], {})

    def test_velocity_watchdog_rejects_one_unconfirmed_feedback_spike(self):
        reference = {joint: 0.0 for joint in JOINTS}
        first = {
            joint: {"pos": 0.0, "vel": 0.374 if joint == "shoulder_rotate" else 0.0, "tau": 0.0}
            for joint in JOINTS
        }
        confirmation = {
            joint: {"pos": 0.0, "vel": 0.02 if joint == "shoulder_rotate" else 0.0, "tau": 0.0}
            for joint in JOINTS
        }
        arm = mock.Mock()
        arm.read_status.return_value = confirmation
        resolved, diagnostic = confirm_velocity_watchdog(
            arm, first, JOINTS, max_velocity=0.3,
            reference_position=reference, max_unconfirmed_drift_deg=0.1,
        )
        self.assertIs(resolved, confirmation)
        self.assertIn("shoulder_rotate", diagnostic["first_velocity_rad_s"])

    def test_velocity_watchdog_stops_when_encoder_confirms_motion(self):
        reference = {joint: 0.0 for joint in JOINTS}
        first = {
            joint: {"pos": 0.0, "vel": 0.374 if joint == "shoulder_rotate" else 0.0, "tau": 0.0}
            for joint in JOINTS
        }
        confirmation = {
            joint: {
                "pos": math.radians(0.2) if joint == "shoulder_rotate" else 0.0,
                "vel": 0.02 if joint == "shoulder_rotate" else 0.0,
                "tau": 0.0,
            }
            for joint in JOINTS
        }
        arm = mock.Mock()
        arm.read_status.return_value = confirmation
        with self.assertRaisesRegex(RuntimeError, "velocity watchdog confirmed"):
            confirm_velocity_watchdog(
                arm, first, JOINTS, max_velocity=0.3,
                reference_position=reference, max_unconfirmed_drift_deg=0.1,
            )

    def test_wait_until_still_requires_a_continuous_quiet_window(self):
        def status(shoulder_rotate_velocity):
            return {
                joint: {
                    "pos": 0.0,
                    "vel": shoulder_rotate_velocity if joint == "shoulder_rotate" else 0.0,
                    "tau": 0.0,
                }
                for joint in JOINTS
            }

        arm = mock.Mock()
        arm.read_status.side_effect = [
            status(0.37), status(0.02), status(0.02), status(0.02),
        ]
        with (
            mock.patch(
                "left_arm_v3_1_model_control.time.monotonic",
                side_effect=[0.0, 0.0, 0.2, 0.4, 0.8],
            ),
            mock.patch("left_arm_v3_1_model_control.time.sleep"),
        ):
            result = wait_until_still(
                arm,
                JOINTS,
                max_velocity=0.25,
                max_drift_deg=0.25,
                stable_seconds=0.5,
                timeout=2.0,
            )
        self.assertAlmostEqual(result["waited_s"], 0.8)
        self.assertAlmostEqual(
            result["peak_velocity_rad_s"]["shoulder_rotate"], 0.37
        )
        self.assertEqual(arm.read_status.call_count, 4)

    def test_wait_until_still_resets_when_encoder_position_drifts(self):
        def status(shoulder_rotate_position):
            return {
                joint: {
                    "pos": shoulder_rotate_position if joint == "shoulder_rotate" else 0.0,
                    "vel": 0.02 if joint == "shoulder_rotate" else 0.0,
                    "tau": 0.0,
                }
                for joint in JOINTS
            }

        arm = mock.Mock()
        arm.read_status.side_effect = [
            status(0.0), status(0.01), status(0.01), status(0.01),
        ]
        with (
            mock.patch(
                "left_arm_v3_1_model_control.time.monotonic",
                side_effect=[0.0, 0.0, 0.2, 0.4, 0.8],
            ),
            mock.patch("left_arm_v3_1_model_control.time.sleep"),
        ):
            result = wait_until_still(
                arm,
                JOINTS,
                max_velocity=0.25,
                max_drift_deg=0.25,
                stable_seconds=0.5,
                timeout=2.0,
            )
        self.assertEqual(result["last_reset"]["reason"], "position_drift")
        self.assertGreater(
            result["max_drift_observed_deg"]["shoulder_rotate"], 0.5
        )


if __name__ == "__main__":
    unittest.main()
