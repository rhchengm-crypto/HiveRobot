#!/usr/bin/env python3
"""Model-based left-arm identification and low-vibration holding controller.

This file is deliberately independent from the v2.6/v2.8/v3.0 motion flows.
It adds a guarded path for:

* quasi-static gravity samples, stored against the full arm pose;
* effective joint inertia samples at representative poses;
* online gain scheduling using kp = omega_n**2 * M(q) and
  kd = 2 * zeta * omega_n * M(q);
* gravity and optional friction feed-forward;
* bounded, rate-limited handoff to the scheduled controller.

No command moves or holds the arm unless ``--execute`` is supplied.  The
existing v2.6 controller is imported lazily so model inspection and tests do
not require the motor CAN stack.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = SCRIPT_DIR / "data" / "left_arm_v3_1_dynamics.json"
DEFAULT_LOG_PATH = SCRIPT_DIR / "data" / "left_arm_v3_1_model_runs.jsonl"
DEFAULT_INERTIA_LOG_PATH = SCRIPT_DIR / "data" / "left_arm_v3_1_inertia_runs.jsonl"
MODEL_SCHEMA = 1
MIT_KP_MIN = 0.0
MIT_KP_MAX = 500.0
MIT_KD_MIN = 0.0
MIT_KD_MAX = 5.0
MIT_TAU_MAX = {
    "shoulder_front": 28.0,
    "shoulder_side": 28.0,
    "shoulder_rotate": 10.0,
    "elbow": 28.0,
    "arm_roll": 10.0,
    "wrist_side": 10.0,
    "wrist": 10.0,
}
FRICTION_VELOCITY_SCALE = 0.05

JOINTS = (
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
    "elbow",
    "arm_roll",
    "wrist_side",
    "wrist",
)

# These are protective envelopes, not identified gains.  Scheduled values are
# clamped here even if a bad inertia sample or frequency is entered.
DEFAULT_SCHEDULER = {
    "shoulder_front": {"omega_n": 2.5, "zeta": 0.90, "kp_min": 2.0, "kp_max": 75.0, "kd_min": 0.3, "kd_max": 5.0},
    "shoulder_side": {"omega_n": 2.5, "zeta": 0.90, "kp_min": 2.0, "kp_max": 65.0, "kd_min": 0.3, "kd_max": 5.0},
    "shoulder_rotate": {"omega_n": 2.0, "zeta": 0.90, "kp_min": 1.0, "kp_max": 24.0, "kd_min": 0.3, "kd_max": 4.5},
    "elbow": {"omega_n": 2.5, "zeta": 0.90, "kp_min": 2.0, "kp_max": 65.0, "kd_min": 0.3, "kd_max": 5.0},
    "arm_roll": {"omega_n": 2.0, "zeta": 0.90, "kp_min": 1.0, "kp_max": 18.0, "kd_min": 0.3, "kd_max": 5.0},
    "wrist_side": {"omega_n": 2.0, "zeta": 0.90, "kp_min": 1.0, "kp_max": 16.0, "kd_min": 0.3, "kd_max": 3.0},
    "wrist": {"omega_n": 2.0, "zeta": 0.90, "kp_min": 1.0, "kp_max": 22.0, "kd_min": 0.3, "kd_max": 4.7},
}

DEFAULT_TAU_LIMITS = {
    "shoulder_front": (-3.8, 3.8),
    "shoulder_side": (-1.0, 1.0),
    "shoulder_rotate": (-1.0, 1.0),
    "elbow": (-5.4, 5.4),
    "arm_roll": (-0.8, 0.8),
    "wrist_side": (-0.45, 0.45),
    "wrist": (-0.7, 0.7),
}

INERTIA_PASSIVE_TAU_FALLBACKS = {
    "shoulder_front": 1.2,
    "wrist": 0.55,
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def inertia_passive_hold_torques(model: Mapping, active_joint: str,
                                 pose: Mapping[str, float]) -> Tuple[dict, dict]:
    """Use nearby identified gravity torque for passive inertia-test joints."""
    torques = {}
    diagnostics = {}
    for joint in JOINTS:
        if joint == active_joint:
            continue
        fallback = float(INERTIA_PASSIVE_TAU_FALLBACKS.get(joint, 0.0))
        source = "legacy_fallback"
        distance = None
        value = fallback
        if model["joints"][joint].get("gravity_samples"):
            try:
                value, prediction = gravity_prediction(model, joint, pose)
                source = "identified_gravity"
                distance = prediction.get("nearest_distance_deg")
            except ValueError:
                pass
        low, high = map(float, model["joints"][joint]["tau_limits"])
        torques[joint] = clamp(value, low, high)
        diagnostics[joint] = {
            "tau": torques[joint], "source": source,
            "nearest_distance_deg": distance,
        }
    return torques, diagnostics


def parse_joints(value: str) -> List[str]:
    names = [item.strip() for item in value.split(",") if item.strip()]
    unknown = sorted(set(names) - set(JOINTS))
    if unknown:
        raise ValueError("unknown joints: " + ", ".join(unknown))
    if not names:
        raise ValueError("at least one joint is required")
    return names


def wait_until_still(arm, joints: Sequence[str], *, max_velocity: float,
                     max_drift_deg: float, stable_seconds: float, timeout: float,
                     poll_interval: float = 0.05) -> dict:
    """Read feedback only until every selected joint remains slow enough.

    This gate runs before Model Hold enables or commands any motor. It lets a
    completed trajectory shed residual velocity without weakening the active
    hold watchdog.
    """
    if max_velocity <= 0.0:
        raise ValueError("preflight max velocity must be positive")
    if stable_seconds <= 0.0:
        raise ValueError("preflight stable seconds must be positive")
    if max_drift_deg <= 0.0:
        raise ValueError("preflight max drift must be positive")
    if timeout < stable_seconds:
        raise ValueError("preflight timeout must be at least the stable window")
    started = time.monotonic()
    stable_since = None
    peak_velocity = {joint: 0.0 for joint in joints}
    last_velocity = {joint: 0.0 for joint in joints}
    reference_position = None
    max_drift_observed_deg = {joint: 0.0 for joint in joints}
    last_reset = None
    while True:
        status = arm.read_status(JOINTS)
        now = time.monotonic()
        last_velocity = {joint: float(status[joint]["vel"]) for joint in joints}
        position = {joint: float(status[joint]["pos"]) for joint in joints}
        for joint, velocity in last_velocity.items():
            peak_velocity[joint] = max(peak_velocity[joint], abs(velocity))
        velocity_violations = {
            joint: velocity for joint, velocity in last_velocity.items()
            if abs(velocity) > max_velocity
        }
        if velocity_violations:
            stable_since = None
            reference_position = None
            last_reset = {"reason": "velocity", "values_rad_s": velocity_violations}
        else:
            if stable_since is None:
                stable_since = now
                reference_position = position
            drift_deg = {
                joint: abs(math.degrees(position[joint] - reference_position[joint]))
                for joint in joints
            }
            for joint, drift in drift_deg.items():
                max_drift_observed_deg[joint] = max(
                    max_drift_observed_deg[joint], drift
                )
            drift_violations = {
                joint: drift for joint, drift in drift_deg.items()
                if drift > max_drift_deg
            }
            if drift_violations:
                stable_since = now
                reference_position = position
                last_reset = {
                    "reason": "position_drift",
                    "values_deg": drift_violations,
                }
            elif now - stable_since >= stable_seconds:
                return {
                    "waited_s": now - started,
                    "stable_seconds": stable_seconds,
                    "max_velocity_rad_s": max_velocity,
                    "max_drift_deg": max_drift_deg,
                    "final_velocity_rad_s": last_velocity,
                    "peak_velocity_rad_s": peak_velocity,
                    "max_drift_observed_deg": max_drift_observed_deg,
                    "last_reset": last_reset,
                }
        if now - started >= timeout:
            raise RuntimeError(
                "model hold preflight stillness timeout: " + json.dumps({
                    "final_velocity_rad_s": last_velocity,
                    "peak_velocity_rad_s": peak_velocity,
                    "max_drift_observed_deg": max_drift_observed_deg,
                    "last_reset": last_reset,
                })
            )
        time.sleep(max(0.001, poll_interval))


def confirm_velocity_watchdog(
    arm,
    status: Mapping[str, Mapping[str, float]],
    joints: Sequence[str],
    *,
    max_velocity: float,
    reference_position: Mapping[str, float],
    max_unconfirmed_drift_deg: float,
) -> Tuple[Mapping[str, Mapping[str, float]], Optional[dict]]:
    """Confirm a velocity trip against a second frame and encoder motion.

    DM velocity feedback is quantized and can occasionally contain an isolated
    high sample. A real joint motion must still trip when the next velocity
    frame remains high or when encoder displacement confirms motion.
    """
    first_violations = {
        joint: float(status[joint]["vel"])
        for joint in joints
        if abs(float(status[joint]["vel"])) > max_velocity
    }
    if not first_violations:
        return status, None

    confirmation = arm.read_status(JOINTS)
    confirmed_velocity = {
        joint: float(confirmation[joint]["vel"])
        for joint in joints
        if abs(float(confirmation[joint]["vel"])) > max_velocity
    }
    drift_deg = {
        joint: abs(math.degrees(
            float(confirmation[joint]["pos"]) - float(reference_position[joint])
        ))
        for joint in first_violations
    }
    confirmed_drift = {
        joint: drift
        for joint, drift in drift_deg.items()
        if drift > max_unconfirmed_drift_deg
    }
    diagnostic = {
        "first_velocity_rad_s": first_violations,
        "confirmation_velocity_rad_s": {
            joint: float(confirmation[joint]["vel"]) for joint in first_violations
        },
        "encoder_drift_deg": drift_deg,
        "max_unconfirmed_drift_deg": max_unconfirmed_drift_deg,
    }
    if confirmed_velocity or confirmed_drift:
        diagnostic["confirmed_velocity_violations_rad_s"] = confirmed_velocity
        diagnostic["confirmed_encoder_drift_deg"] = confirmed_drift
        raise RuntimeError(
            "velocity watchdog confirmed: " + json.dumps(diagnostic, sort_keys=True)
        )
    return confirmation, diagnostic


def atomic_write_json(path: Path, payload: Mapping) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def append_jsonl(path: Path, payload: Mapping) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, ensure_ascii=False) + "\n")


def empty_model() -> dict:
    return {
        "schema": MODEL_SCHEMA,
        "type": "left_arm_v3_1_identified_dynamics",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "updated_at": None,
        "gain_units_verified": False,
        "notes": [
            "Verify that DM MIT kp/kd/tau use the expected joint-side physical units before execution.",
            "Gravity samples include friction and sensor bias unless captured in both travel directions.",
        ],
        "settings": {
            "gravity_neighbors": 6,
            "max_gravity_distance_deg": 25.0,
            "inertia_neighbors": 3,
            "max_inertia_distance_deg": 45.0,
            "gain_filter_hz": 2.0,
            "kp_slew_per_second": 20.0,
            "kd_slew_per_second": 5.0,
            "tau_slew_per_second": 1.5,
        },
        "joints": {
            name: {
                "gravity_samples": [],
                "gravity_validation": None,
                "inertia_samples": [],
                "friction": {"coulomb": 0.0, "viscous": 0.0, "identified": False},
                "scheduler": dict(DEFAULT_SCHEDULER[name]),
                "tau_limits": list(DEFAULT_TAU_LIMITS[name]),
            }
            for name in JOINTS
        },
    }


def load_model(path: Path, create: bool = False) -> dict:
    if not path.exists():
        if create:
            model = empty_model()
            atomic_write_json(path, model)
            return model
        raise FileNotFoundError(f"dynamics model not found: {path}")
    model = json.loads(path.read_text(encoding="utf-8"))
    if model.get("schema") != MODEL_SCHEMA or not isinstance(model.get("joints"), dict):
        raise ValueError(f"invalid v3.1 dynamics model: {path}")
    return model


def save_model(path: Path, model: MutableMapping) -> None:
    model["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    atomic_write_json(path, model)


def pose_from_status(status: Mapping[str, Mapping[str, float]]) -> Dict[str, float]:
    return {name: float(status[name]["pos"]) for name in JOINTS if name in status}


def wrapped_difference(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose_distance_deg(left: Mapping[str, float], right: Mapping[str, float]) -> float:
    common = [name for name in JOINTS if name in left and name in right]
    if not common:
        return float("inf")
    squared = [math.degrees(wrapped_difference(float(left[name]), float(right[name]))) ** 2 for name in common]
    # Penalize incomplete stored poses so old single-axis records cannot look close.
    missing_penalty = (len(JOINTS) - len(common)) * 30.0 ** 2
    return math.sqrt((sum(squared) + missing_penalty) / len(JOINTS))


def inverse_distance_prediction(samples: Sequence[Mapping], pose: Mapping[str, float], value_key: str,
                                neighbors: int, max_distance_deg: float) -> tuple[float, dict]:
    ranked = []
    for sample in samples:
        sample_pose = sample.get("pose_rad", {})
        if not isinstance(sample_pose, dict) or value_key not in sample:
            continue
        distance = pose_distance_deg(pose, sample_pose)
        if math.isfinite(distance):
            ranked.append((distance, float(sample[value_key]), sample))
    if not ranked:
        raise ValueError(f"no usable {value_key} samples")
    ranked.sort(key=lambda item: item[0])
    nearest = ranked[0][0]
    if nearest > max_distance_deg:
        raise ValueError(
            f"nearest {value_key} sample is {nearest:.2f}deg away; limit is {max_distance_deg:.2f}deg"
        )
    # The distance threshold is a sample-admission boundary, not merely a
    # nearest-neighbor preflight.  Mixing a far pose into a valid local fit
    # leaks Home, Clearance, and board-square dynamics into one another.
    eligible = [item for item in ranked if item[0] <= max_distance_deg]
    selected = eligible[:max(1, int(neighbors))]
    if nearest < 1e-9:
        prediction = selected[0][1]
    else:
        weights = [1.0 / max(distance, 0.25) ** 2 for distance, _, _ in selected]
        prediction = sum(weight * value for weight, (_, value, _) in zip(weights, selected)) / sum(weights)
    return prediction, {
        "nearest_distance_deg": nearest,
        "neighbors_used": len(selected),
        "neighbor_distances_deg": [round(item[0], 4) for item in selected],
    }


def gravity_prediction(model: Mapping, joint: str, pose: Mapping[str, float]) -> tuple[float, dict]:
    settings = model["settings"]
    samples = model["joints"][joint]["gravity_samples"]
    return inverse_distance_prediction(
        samples,
        pose,
        "tau",
        int(settings["gravity_neighbors"]),
        float(settings["max_gravity_distance_deg"]),
    )


def inertia_prediction(model: Mapping, joint: str, pose: Mapping[str, float]) -> tuple[float, dict]:
    settings = model["settings"]
    samples = model["joints"][joint]["inertia_samples"]
    inertia, diagnostic = inverse_distance_prediction(
        samples,
        pose,
        "effective_inertia",
        int(settings["inertia_neighbors"]),
        float(settings["max_inertia_distance_deg"]),
    )
    if inertia <= 0.0 or not math.isfinite(inertia):
        raise ValueError(f"invalid effective inertia for {joint}: {inertia}")
    return inertia, diagnostic


def scheduled_terms(model: Mapping, joint: str, pose: Mapping[str, float], velocity: float = 0.0) -> dict:
    joint_model = model["joints"][joint]
    scheduler = joint_model["scheduler"]
    omega_n = float(scheduler["omega_n"])
    zeta = float(scheduler["zeta"])
    if omega_n <= 0.0 or not 0.5 <= zeta <= 1.5:
        raise ValueError(f"invalid scheduler configuration for {joint}")
    kp_low = max(MIT_KP_MIN, float(scheduler["kp_min"]))
    kp_high = min(MIT_KP_MAX, float(scheduler["kp_max"]))
    kd_low = max(MIT_KD_MIN, float(scheduler["kd_min"]))
    kd_high = min(MIT_KD_MAX, float(scheduler["kd_max"]))
    if kp_low > kp_high or kd_low > kd_high:
        raise ValueError(f"scheduler gain limits exceed the MIT protocol range for {joint}")
    gravity, gravity_diag = gravity_prediction(model, joint, pose)
    inertia_fallback = str(scheduler.get("inertia_fallback", "reject"))
    if inertia_fallback not in {"reject", "minimum-gains"}:
        raise ValueError(f"invalid inertia_fallback for {joint}: {inertia_fallback}")
    try:
        inertia, inertia_diag = inertia_prediction(model, joint, pose)
    except ValueError as exc:
        coverage_error = str(exc).startswith((
            "nearest effective_inertia sample is ",
            "no usable effective_inertia samples",
        ))
        if inertia_fallback != "minimum-gains" or not coverage_error:
            raise
        # Some joints cannot be safely excited in every work pose.  Keep the
        # local-inertia admission limit intact and use only the explicitly
        # configured, previously validated minimum gains.  Gravity and static
        # friction remain pose-local and are still required above.
        inertia = None
        inertia_diag = {
            "source": "minimum-gains-fallback",
            "available": False,
            "reason": str(exc),
        }
    if inertia is None:
        kp_unclamped = None
        kd_unclamped = None
        kp = kp_low
        kd = kd_low
        gain_source = "minimum-gains-without-local-inertia"
    else:
        kp_unclamped = omega_n * omega_n * inertia
        kd_unclamped = 2.0 * zeta * omega_n * inertia
        kp = clamp(kp_unclamped, kp_low, kp_high)
        kd = clamp(kd_unclamped, kd_low, kd_high)
        gain_source = "identified-inertia"
    friction = joint_model.get("friction", {})
    velocity = float(velocity)
    friction_tau = float(friction.get("viscous", 0.0)) * velocity
    # Motor velocity feedback is quantized around zero.  A hard sign switch
    # would turn that sensor jitter into full Coulomb torque in either
    # direction, so blend continuously through zero instead.
    friction_tau += float(friction.get("coulomb", 0.0)) * math.tanh(
        velocity / FRICTION_VELOCITY_SCALE
    )
    tau_low, tau_high = map(float, joint_model["tau_limits"])
    tau_ff = clamp(gravity + friction_tau, tau_low, tau_high)
    return {
        "kp": kp,
        "kd": kd,
        "tau_ff": tau_ff,
        "effective_inertia": inertia,
        "gravity_tau": gravity,
        "friction_tau": friction_tau,
        "omega_n": omega_n,
        "zeta": zeta,
        "gain_source": gain_source,
        "kp_unclamped": kp_unclamped,
        "kd_unclamped": kd_unclamped,
        "kp_clamped": kp_unclamped is None or kp != kp_unclamped,
        "kd_clamped": kd_unclamped is None or kd != kd_unclamped,
        "tau_clamped": tau_ff != gravity + friction_tau,
        "gravity_diagnostic": gravity_diag,
        "inertia_diagnostic": inertia_diag,
    }


def static_hold_bias(model: Mapping, joint: str, pose: Mapping[str, float],
                     measured_tau: float) -> dict:
    """Select the static-friction branch already supporting a stationary joint."""
    desired = scheduled_terms(model, joint, pose, 0.0)
    joint_model = model["joints"][joint]
    coulomb = abs(float(joint_model.get("friction", {}).get("coulomb", 0.0)))
    raw_bias = float(measured_tau) - float(desired["tau_ff"])
    bias = clamp(raw_bias, -coulomb, coulomb)
    tau_low, tau_high = map(float, joint_model["tau_limits"])
    tau_without_bias = float(desired["tau_ff"])
    desired["tau_ff"] = clamp(tau_without_bias + bias, tau_low, tau_high)
    desired["static_hold_bias_tau"] = bias
    desired["static_hold_bias_raw_tau"] = raw_bias
    desired["tau_ff_without_static_bias"] = tau_without_bias
    desired["static_hold_bias_clamped"] = bias != raw_bias
    desired["tau_clamped"] = desired["tau_clamped"] or (
        desired["tau_ff"] != tau_without_bias + bias
    )
    return desired


def hold_load_support(model: Mapping, joint: str, pose: Mapping[str, float],
                      measured_tau: float, measured_velocity: float,
                      takeover_kd: float) -> dict:
    """Capture existing load support for this fixed-pose hold only.

    Model tau_limits bound the identified feed-forward component, not the
    preceding controller's total PD + feed-forward output. Preserve that
    observed output as a separate, non-learning load residual. Its envelope
    extends only to the observed support, never to the full hardware range.
    """
    measured_tau, measured_velocity = float(measured_tau), float(measured_velocity)
    hardware_max = MIT_TAU_MAX[joint]
    if not all(math.isfinite(value) for value in (measured_tau, measured_velocity)):
        raise ValueError(f"non-finite takeover feedback for {joint}")
    if abs(measured_tau) > hardware_max:
        raise ValueError(f"takeover torque exceeds MIT protocol range for {joint}")
    initial = static_hold_bias(model, joint, pose, measured_tau)
    # q_target equals q_measured on the first frame. Cancel that frame's
    # damping contribution so its net torque matches the preceding output.
    bridge_tau = clamp(measured_tau + takeover_kd * measured_velocity,
                       -hardware_max, hardware_max)
    low, high = map(float, model["joints"][joint]["tau_limits"])
    return {
        "measured_tau": measured_tau,
        "model_tau_with_static_bias": float(initial["tau_ff"]),
        "static_hold_bias_tau": float(initial["static_hold_bias_tau"]),
        "load_support_tau": measured_tau - float(initial["tau_ff"]),
        "bridge_tau_ff": bridge_tau,
        "model_tau_limits": [low, high],
        "hold_tau_limits": [max(-hardware_max, min(low, measured_tau, bridge_tau)),
                            min(hardware_max, max(high, measured_tau, bridge_tau))],
        "scope": "current-fixed-pose-hold-only",
    }


def supported_hold_terms(model: Mapping, joint: str, pose: Mapping[str, float],
                         support: Mapping) -> dict:
    """Schedule gains and model torque without discarding inherited load."""
    desired = scheduled_terms(model, joint, pose, 0.0)
    base_tau = float(desired["tau_ff"])
    low, high = map(float, support["model_tau_limits"])
    static_tau = float(support["static_hold_bias_tau"])
    model_tau = clamp(base_tau + static_tau, low, high)
    load_tau = float(support["load_support_tau"])
    requested_tau = model_tau + load_tau
    desired.update({
        "tau_ff": clamp(requested_tau, *support["hold_tau_limits"]),
        "tau_ff_without_static_bias": base_tau,
        "static_hold_bias_tau": static_tau,
        "load_support_tau": load_tau,
        "hold_tau_limits": list(support["hold_tau_limits"]),
    })
    desired["tau_clamped"] = bool(desired["tau_clamped"] or model_tau != base_tau + static_tau)
    desired["load_support_clamped"] = desired["tau_ff"] != requested_tau
    return desired


@dataclass
class SlewLimitedFilter:
    cutoff_hz: float
    kp_slew: float
    kd_slew: float
    tau_slew: float
    state: Dict[str, Dict[str, float]] = field(default_factory=dict)

    def seed(self, joint: str, kp: float, kd: float, tau_ff: float) -> None:
        self.state[joint] = {"kp": float(kp), "kd": float(kd), "tau_ff": float(tau_ff)}

    def update(self, joint: str, desired: Mapping[str, float], dt: float) -> dict:
        if joint not in self.state:
            self.seed(joint, desired["kp"], desired["kd"], desired["tau_ff"])
            return dict(self.state[joint])
        dt = max(1e-4, float(dt))
        alpha = 1.0 - math.exp(-2.0 * math.pi * max(0.01, self.cutoff_hz) * dt)
        current = self.state[joint]
        limits = {"kp": self.kp_slew, "kd": self.kd_slew, "tau_ff": self.tau_slew}
        for key, rate in limits.items():
            filtered = current[key] + alpha * (float(desired[key]) - current[key])
            delta = clamp(filtered - current[key], -abs(rate) * dt, abs(rate) * dt)
            current[key] += delta
        return dict(current)


def validate_ready(model: Mapping, joints: Iterable[str]) -> None:
    if not model.get("gain_units_verified"):
        raise ValueError("gain_units_verified is false; verify MIT gain and torque units before execution")
    failures = []
    for joint in joints:
        data = model["joints"][joint]
        if len(data.get("gravity_samples", [])) < 3:
            failures.append(f"{joint}: needs at least 3 gravity samples")
        if len(data.get("inertia_samples", [])) < 1:
            failures.append(f"{joint}: needs at least 1 inertia sample")
    if failures:
        raise ValueError("model is not ready: " + "; ".join(failures))


def robust_status_sample(arm, joints: Sequence[str], count: int, interval: float,
                         max_velocity: float) -> tuple[dict, dict]:
    readings = []
    for _ in range(max(3, count)):
        reading = arm.read_status(JOINTS)
        if all(abs(float(reading[name]["vel"])) <= max_velocity for name in joints):
            readings.append(reading)
        time.sleep(max(0.001, interval))
    if len(readings) < 3:
        raise RuntimeError("not enough quasi-static samples; joint velocity remained above the limit")
    pose = {
        name: statistics.median(float(item[name]["pos"]) for item in readings)
        for name in JOINTS
    }
    result = {
        name: {
            "pos": pose[name],
            "vel": statistics.median(float(item[name]["vel"]) for item in readings),
            "tau": statistics.median(float(item[name]["tau"]) for item in readings),
            "tau_mad": statistics.median(
                abs(float(item[name]["tau"]) - statistics.median(float(row[name]["tau"]) for row in readings))
                for item in readings
            ),
        }
        for name in joints
    }
    return pose, result


def open_arm():
    import left_arm_v2_6 as legacy
    return legacy, legacy.LeftArmV2()


def capture_gravity(args) -> dict:
    model_path = Path(args.model)
    model = load_model(model_path, create=True)
    _, arm = open_arm()
    joints = parse_joints(args.joints)
    try:
        pose, status = robust_status_sample(
            arm, joints, args.samples, args.interval, args.max_velocity
        )
    finally:
        arm.close()
    now = time.strftime("%Y-%m-%d %H:%M:%S")
    added = {}
    for joint in joints:
        sample = {
            "captured_at": now,
            "pose_rad": pose,
            "tau": status[joint]["tau"],
            "tau_mad": status[joint]["tau_mad"],
            "direction": args.direction,
            "source": args.source,
        }
        model["joints"][joint]["gravity_samples"].append(sample)
        added[joint] = sample
    save_model(model_path, model)
    return {"ok": True, "model": str(model_path), "added": added}


def add_inertia(args) -> dict:
    model_path = Path(args.model)
    model = load_model(model_path, create=True)
    joint = args.joint
    if joint not in JOINTS:
        raise ValueError(f"unknown joint: {joint}")
    if args.pose_json:
        pose = {name: float(value) for name, value in json.loads(args.pose_json).items() if name in JOINTS}
    elif args.from_arm:
        _, arm = open_arm()
        try:
            pose = pose_from_status(arm.read_status(JOINTS))
        finally:
            arm.close()
    else:
        raise ValueError("provide --pose-json or --from-arm")
    if len(pose) != len(JOINTS):
        raise ValueError("inertia samples require the full seven-joint pose")
    if args.effective_inertia is not None:
        inertia = float(args.effective_inertia)
        method = "direct"
    else:
        if args.test_kp is None or args.measured_omega_n is None:
            raise ValueError("provide --effective-inertia or both --test-kp and --measured-omega-n")
        if args.measured_omega_n <= 0.0:
            raise ValueError("--measured-omega-n must be positive")
        inertia = float(args.test_kp) / float(args.measured_omega_n) ** 2
        method = "step_response_kp_over_omega_squared"
    if not 0.0 < inertia <= args.max_inertia:
        raise ValueError(f"effective inertia {inertia} is outside (0, {args.max_inertia}]")
    sample = {
        "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "pose_rad": pose,
        "effective_inertia": inertia,
        "method": method,
        "test_kp": args.test_kp,
        "measured_omega_n": args.measured_omega_n,
        "label": args.label,
    }
    model["joints"][joint]["inertia_samples"].append(sample)
    save_model(model_path, model)
    return {"ok": True, "model": str(model_path), "joint": joint, "sample": sample}


def analyze_inertia_trace(trace: Sequence[Mapping[str, float]], *, step_rad: float,
                          test_kp: float, test_kd: float) -> dict:
    """Estimate effective inertia from the early, acceleration-dominated response."""
    if len(trace) < 7:
        raise ValueError("inertia trace needs at least 7 samples")
    direction = 1.0 if step_rad > 0.0 else -1.0
    start_pos = float(trace[0]["start_pos"])
    baseline_target = float(trace[0].get("baseline_target", start_pos))
    baseline_error = baseline_target - start_pos
    ratios = []
    accelerations = []
    achieved = 0.0
    for row in trace:
        achieved = max(achieved, direction * (float(row["pos"]) - start_pos))
    for index in range(1, len(trace) - 1):
        previous = trace[index - 1]
        row = trace[index]
        following = trace[index + 1]
        dt = float(following["t"]) - float(previous["t"])
        if dt <= 1e-4:
            continue
        acceleration = (float(following["vel"]) - float(previous["vel"])) / dt
        response_fraction = direction * (float(row["pos"]) - start_pos) / abs(step_rad)
        incremental_error = (float(row["target"]) - float(row["pos"])) - baseline_error
        dynamic_tau = test_kp * incremental_error - test_kd * float(row["vel"])
        if not 0.0 <= response_fraction <= 0.80:
            continue
        if direction * acceleration <= 0.05 or direction * dynamic_tau <= 0.02:
            continue
        estimate = dynamic_tau / acceleration
        if 1e-4 < estimate <= 100.0 and math.isfinite(estimate):
            ratios.append(estimate)
            accelerations.append(acceleration)
    achieved_fraction = achieved / abs(step_rad)
    if achieved_fraction < 0.25:
        raise ValueError(f"joint achieved only {achieved_fraction * 100:.1f}% of the test step")
    # The acceleration-ratio estimator is useful only when the move overcame
    # stiction and its point estimates agree.  A partially completed reverse
    # step can otherwise look like an inertia tens of times too large.
    if len(ratios) >= 5 and achieved_fraction >= 0.80:
        inertia = statistics.median(ratios)
        mad = statistics.median(abs(value - inertia) for value in ratios)
        if mad / inertia <= 0.35:
            return {
                "effective_inertia": inertia,
                "inertia_mad": mad,
                "relative_mad": mad / inertia,
                "measured_omega_n": math.sqrt(test_kp / inertia),
                "achieved_fraction": achieved_fraction,
                "usable_samples": len(ratios),
                "peak_velocity": max(abs(float(row["vel"])) for row in trace),
                "peak_acceleration": max(abs(value) for value in accelerations),
                "estimator": "acceleration_ratio",
            }

    if achieved_fraction < 0.80:
        raise ValueError(
            f"joint achieved only {achieved_fraction * 100:.1f}% of the test step; "
            "response is dominated by stiction or insufficient drive and was not saved"
        )

    # DM velocity feedback is quantized on this arm.  When differentiation
    # leaves too few reliable acceleration points, fit M against the complete
    # normalized closed-loop step shape.  Normalization removes the constant
    # gravity/friction offset while retaining kp, kd and the response timing.
    tail_count = max(5, len(trace) // 5)
    plateau = statistics.median(float(row["pos"]) for row in trace[-tail_count:])
    amplitude = plateau - start_pos
    if direction * amplitude <= 0.25 * abs(step_rad):
        raise ValueError("step response did not reach a stable plateau in the commanded direction")
    observations = [(0.0, 0.0)]
    observations.extend(
        (float(row["t"]), (float(row["pos"]) - start_pos) / amplitude)
        for row in trace
    )

    def shape_score(candidate: float) -> float:
        simulated_pos = 0.0
        simulated_vel = 0.0
        previous_t = 0.0
        squared = []
        for sample_t, observed_pos in observations[1:]:
            remaining = max(0.0, sample_t - previous_t)
            while remaining > 1e-9:
                dt = min(0.002, remaining)
                acceleration = (test_kp * (1.0 - simulated_pos) - test_kd * simulated_vel) / candidate
                simulated_vel += acceleration * dt
                simulated_pos += simulated_vel * dt
                remaining -= dt
            previous_t = sample_t
            squared.append((simulated_pos - observed_pos) ** 2)
        return sum(squared) / max(1, len(squared))

    candidates = [10.0 ** (-2.3 + index * (3.6 / 239.0)) for index in range(240)]
    scored = [(shape_score(candidate), candidate) for candidate in candidates]
    best_index, (best_score, inertia) = min(enumerate(scored), key=lambda item: item[1][0])
    if best_score > 0.08:
        raise ValueError(f"normalized step fit error {best_score:.4f} is too high; sample was not saved")
    if best_index in {0, len(candidates) - 1}:
        raise ValueError(
            "normalized step fit saturated at the inertia search boundary; "
            "response is not identifiable and was not saved"
        )
    omega_n = math.sqrt(test_kp / inertia)
    zeta = test_kd / (2.0 * math.sqrt(test_kp * inertia))
    if omega_n > 60.0 or not 0.15 <= zeta <= 2.0:
        raise ValueError(
            f"normalized step fit is physically implausible "
            f"(omega_n={omega_n:.2f}, zeta={zeta:.2f}); sample was not saved"
        )
    return {
        "effective_inertia": inertia,
        "inertia_mad": None,
        "relative_mad": None,
        "measured_omega_n": omega_n,
        "estimated_zeta": zeta,
        "normalized_fit_mse": best_score,
        "plateau_step_deg": math.degrees(amplitude),
        "achieved_fraction": achieved_fraction,
        "usable_samples": len(trace),
        "peak_velocity": max(abs(float(row["vel"])) for row in trace),
        "peak_acceleration": max((abs(value) for value in accelerations), default=None),
        "estimator": "normalized_closed_loop_step_fit",
    }


def validate_inertia_loaded_offset(start_pos: float, baseline_target: float,
                                   step_rad: float) -> float:
    """Reject ordinary steps when static load consumes most of the excitation."""
    offset_rad = wrapped_difference(float(start_pos), float(baseline_target))
    offset_deg = math.degrees(offset_rad)
    limit_deg = max(0.5, 0.5 * abs(math.degrees(step_rad)))
    if abs(offset_deg) > limit_deg:
        raise ValueError(
            f"inertia preflight refused: loaded position offset is {offset_deg:.2f}deg; "
            f"ordinary step limit is {limit_deg:.2f}deg. Use a supported free-space pose "
            "or a dedicated high-load identification method"
        )
    return offset_deg


def reprocess_inertia(args) -> dict:
    """Fit and save the most recent unsaved trace without moving any motor."""
    log_path = Path(args.log)
    if not log_path.is_file():
        raise FileNotFoundError(f"inertia log not found: {log_path}")
    records = []
    with log_path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                records.append(json.loads(line))
    matching = [record for record in records if record.get("joint") == args.joint and record.get("trace")]
    if not matching:
        raise ValueError(f"no recorded inertia trace found for {args.joint}")
    record = matching[-1]
    analysis = analyze_inertia_trace(
        record["trace"], step_rad=math.radians(float(record["step_deg"])),
        test_kp=float(record["test_kp"]), test_kd=float(record["test_kd"]),
    )
    model_path = Path(args.model)
    model = load_model(model_path)
    sample = {
        "captured_at": record["captured_at"],
        "pose_rad": record["pose_rad"],
        "effective_inertia": analysis["effective_inertia"],
        "method": analysis["estimator"],
        "test_kp": record["test_kp"], "test_kd": record["test_kd"],
        "measured_omega_n": analysis["measured_omega_n"],
        "step_deg": record["step_deg"], "quality": analysis,
        "label": args.label or "reprocessed_last_trace",
    }
    samples = model["joints"][args.joint]["inertia_samples"]
    same_capture = [item for item in samples if item.get("captured_at") == sample["captured_at"]]
    duplicate = any(
        item.get("method") == sample["method"]
        and math.isclose(
            float(item.get("effective_inertia", -1.0)),
            float(sample["effective_inertia"]), rel_tol=1e-6, abs_tol=1e-9,
        )
        for item in same_capture
    )
    replaced = 0
    if not duplicate:
        # A trace may already have been saved by an older, overly permissive
        # estimator.  Reprocessing is a correction operation: replace samples
        # from this exact capture instead of leaving contradictory values.
        replaced = len(same_capture)
        if replaced:
            samples[:] = [item for item in samples if item.get("captured_at") != sample["captured_at"]]
        samples.append(sample)
        save_model(model_path, model)
    return {
        "ok": True, "saved": not duplicate, "duplicate": duplicate,
        "replaced": replaced, "motion_issued": False,
        "joint": args.joint, "sample": sample,
    }


def remove_latest_inertia(args) -> dict:
    """Remove the newest stored inertia sample for one joint without motion."""
    model_path = Path(args.model)
    model = load_model(model_path)
    samples = model["joints"][args.joint]["inertia_samples"]
    if not samples:
        raise ValueError(f"{args.joint} has no inertia sample to remove")
    removed = samples.pop()
    save_model(model_path, model)
    return {
        "ok": True,
        "removed": True,
        "motion_issued": False,
        "joint": args.joint,
        "sample": removed,
        "remaining": len(samples),
    }


def remove_latest_gravity(args) -> dict:
    """Remove the newest stored gravity sample for one joint without motion."""
    model_path = Path(args.model)
    model = load_model(model_path)
    samples = model["joints"][args.joint]["gravity_samples"]
    if not samples:
        raise ValueError(f"{args.joint} has no gravity sample to remove")
    removed = samples.pop()
    save_model(model_path, model)
    return {
        "ok": True,
        "removed": True,
        "motion_issued": False,
        "joint": args.joint,
        "sample": removed,
        "remaining": len(samples),
    }


def measure_inertia(args) -> dict:
    """Run one guarded small-angle step, return to the start, and save a valid sample."""
    if not args.execute:
        raise RuntimeError("inertia test execution disabled; add --execute after reviewing the limits")
    if args.joint not in JOINTS:
        raise ValueError(f"unknown joint: {args.joint}")
    if not 0.25 <= abs(args.step_deg) <= 3.0:
        raise ValueError("--step-deg magnitude must be between 0.25 and 3.0 degrees")
    if not 1.0 <= args.test_kp <= 70.0 or not 0.1 <= args.test_kd <= MIT_KD_MAX:
        raise ValueError("test gains are outside the guarded range")

    model_path = Path(args.model)
    model = load_model(model_path)
    joint_model = model["joints"][args.joint]
    if len(joint_model.get("gravity_samples", [])) < 3:
        raise ValueError(f"{args.joint} needs at least 3 gravity samples before inertia testing")
    if not joint_model.get("friction", {}).get("identified"):
        raise ValueError(f"{args.joint} friction must be configured before inertia testing")

    legacy, arm = open_arm()
    initial = None
    targets = None
    sample_pose = None
    trace = []
    step_rad = math.radians(args.step_deg)
    started_at = time.strftime("%Y-%m-%d %H:%M:%S")
    active_tau = 0.0
    recovery_tau = 0.0
    passive_hold_tau = dict(INERTIA_PASSIVE_TAU_FALLBACKS)
    passive_identified = set()

    def command_pose(active_target: float, active_kp: float, active_kd: float,
                     active_tau: float, blend: float = 1.0) -> None:
        for name in JOINTS:
            if name == args.joint:
                kp = 2.0 + blend * (active_kp - 2.0)
                kd = 0.6 + blend * (active_kd - 0.6)
                tau = blend * active_tau
                target = active_target
            else:
                configured = legacy.HOME_BASE_HOLD_GAINS[name]
                kp = 2.0 + blend * (float(configured["kp"]) - 2.0)
                kd = 0.6 + blend * (float(configured["kd"]) - 0.6)
                target = targets[name]
                passive_tau = float(passive_hold_tau.get(name, 0.0))
                # A nearby gravity sample represents the physical support
                # torque already present at this pose.  Apply it immediately
                # during takeover; ramping it from zero lets heavy joints fall
                # before the identification step starts.
                tau = passive_tau if name in passive_identified else blend * passive_tau
            arm.ctrl.controlMIT(arm.motors[name], kp, kd, target, 0.0, tau)

    def read_active_fast() -> dict:
        """Read only the excited joint so the short acceleration phase is sampled densely."""
        motor = arm.motors[args.joint]
        arm.ctrl.refresh_motor_status(motor)
        time.sleep(args.feedback_wait)
        arm.ctrl.recv()
        return {
            "pos": float(motor.getPosition()),
            "vel": float(motor.getVelocity()),
            "tau": float(motor.getTorque()),
        }

    try:
        initial = arm.read_status(JOINTS)
        targets = pose_from_status(initial)
        if len(targets) != len(JOINTS):
            raise RuntimeError("inertia test requires status from all seven joints")
        gravity_tau, _ = gravity_prediction(model, args.joint, targets)
        passive_hold_tau, passive_diagnostics = inertia_passive_hold_torques(
            model, args.joint, targets
        )
        passive_identified = {
            name for name, item in passive_diagnostics.items()
            if item["source"] == "identified_gravity"
        }
        print("v3.1 inertia passive hold=", json.dumps(passive_diagnostics, sort_keys=True))
        direction = 1.0 if step_rad > 0.0 else -1.0
        friction = joint_model["friction"]
        active_tau = gravity_tau + direction * float(friction.get("coulomb", 0.0))
        active_tau = clamp(active_tau, *map(float, joint_model["tau_limits"]))
        recovery_tau = gravity_tau - direction * float(friction.get("coulomb", 0.0))
        recovery_tau = clamp(recovery_tau, *map(float, joint_model["tau_limits"]))
        arm.enable(JOINTS)

        takeover_started = time.time()
        while time.time() - takeover_started < args.settle_seconds:
            progress = clamp((time.time() - takeover_started) / args.settle_seconds, 0.0, 1.0)
            blend = progress * progress * (3.0 - 2.0 * progress)
            command_pose(targets[args.joint], args.test_kp, args.test_kd, active_tau, blend)
            time.sleep(args.control_dt)

        settled = arm.read_status(JOINTS)
        start_pos = float(settled[args.joint]["pos"])
        sample_pose = pose_from_status(settled)
        baseline_target = targets[args.joint]
        validate_inertia_loaded_offset(start_pos, baseline_target, step_rad)
        # Preserve the command target that produced the settled starting
        # position.  The gravity/friction feed-forward creates a repeatable
        # static offset; the test must add the step to this command target,
        # rather than adding it to the measured position and applying the
        # offset a second time.
        targets.update(sample_pose)
        targets[args.joint] = baseline_target
        test_target = baseline_target + step_rad
        test_started = time.time()
        sample_index = 0
        while time.time() - test_started < args.test_seconds:
            command_pose(test_target, args.test_kp, args.test_kd, active_tau)
            time.sleep(args.control_dt)
            row = read_active_fast()
            elapsed = time.time() - test_started
            active_error_deg = math.degrees(wrapped_difference(float(row["pos"]), sample_pose[args.joint]))
            if abs(active_error_deg) > abs(args.step_deg) + args.max_position_error_deg:
                raise RuntimeError(f"position watchdog tripped for {args.joint}: {active_error_deg:.2f}deg")
            if abs(float(row["vel"])) > args.max_velocity:
                raise RuntimeError(f"velocity watchdog tripped for {args.joint}: {row['vel']:.3f}rad/s")
            # Reading all seven motors is slower, so check the passive joints
            # periodically while keeping dense samples for the excited joint.
            if sample_index % 8 == 0:
                watchdog = arm.read_status(JOINTS)
                for name in JOINTS:
                    if name == args.joint:
                        continue
                    error_deg = math.degrees(wrapped_difference(float(watchdog[name]["pos"]), sample_pose[name]))
                    if abs(error_deg) > args.max_position_error_deg:
                        raise RuntimeError(f"position watchdog tripped for {name}: {error_deg:.2f}deg")
                    if abs(float(watchdog[name]["vel"])) > args.max_velocity:
                        raise RuntimeError(f"velocity watchdog tripped for {name}: {watchdog[name]['vel']:.3f}rad/s")
            trace.append({
                "t": elapsed, "start_pos": start_pos, "baseline_target": baseline_target,
                "target": test_target,
                "pos": float(row["pos"]), "vel": float(row["vel"]), "tau": float(row["tau"]),
            })
            sample_index += 1
    finally:
        if initial is not None and targets is not None:
            try:
                recovery_started = time.time()
                recovery_from = trace[-1]["target"] if trace else targets[args.joint]
                while time.time() - recovery_started < args.return_seconds:
                    progress = clamp((time.time() - recovery_started) / args.return_seconds, 0.0, 1.0)
                    blend = progress * progress * (3.0 - 2.0 * progress)
                    target = recovery_from + blend * (targets[args.joint] - recovery_from)
                    command_pose(target, args.test_kp, args.test_kd, recovery_tau)
                    time.sleep(args.control_dt)
            finally:
                arm.close()
        else:
            arm.close()

    record = {
        "captured_at": started_at, "joint": args.joint, "pose_rad": sample_pose,
        "step_deg": args.step_deg, "test_kp": args.test_kp, "test_kd": args.test_kd,
        "trace": trace,
    }
    try:
        analysis = analyze_inertia_trace(trace, step_rad=step_rad, test_kp=args.test_kp, test_kd=args.test_kd)
    except ValueError as exc:
        record.update({"saved": False, "reason": str(exc)})
        append_jsonl(Path(args.log), record)
        return {"ok": False, "saved": False, "reason": str(exc), "diagnostics": record}

    sample = {
        "captured_at": started_at, "pose_rad": sample_pose,
        "effective_inertia": analysis["effective_inertia"],
        "method": analysis["estimator"],
        "test_kp": args.test_kp, "test_kd": args.test_kd,
        "measured_omega_n": analysis["measured_omega_n"],
        "step_deg": args.step_deg, "quality": analysis, "label": args.label,
    }
    model["joints"][args.joint]["inertia_samples"].append(sample)
    save_model(model_path, model)
    record.update({"saved": True, "analysis": analysis})
    append_jsonl(Path(args.log), record)
    return {"ok": True, "saved": True, "model": str(model_path), "joint": args.joint, "sample": sample}


def guarded_nudge(args) -> dict:
    """Move one joint slowly while nearby identified gravity holds the others."""
    if not args.execute:
        raise RuntimeError("guarded nudge execution disabled; add --execute after reviewing the limits")
    if args.joint not in JOINTS:
        raise ValueError(f"unknown joint: {args.joint}")
    if not 0.25 <= abs(args.step_deg) <= 5.0:
        raise ValueError("--step-deg magnitude must be between 0.25 and 5.0 degrees")
    if not 1.0 <= args.test_kp <= 12.0 or not 0.1 <= args.test_kd <= 2.0:
        raise ValueError("guarded nudge gains are outside the low-gain range")
    if args.seconds <= 0.0 or args.settle_seconds <= 0.0 or args.stop_seconds < 0.0:
        raise ValueError("guarded nudge timing values must be positive")
    if args.control_dt <= 0.0 or args.max_velocity <= 0.0 or args.max_position_error_deg <= 0.0:
        raise ValueError("guarded nudge control and watchdog limits must be positive")

    model = load_model(Path(args.model))
    legacy, arm = open_arm()
    targets = None
    passive_hold_tau = {}
    final_status = None
    final_target = None
    failure = None
    motors_enabled = False

    def command_pose(active_target: float, active_kp: float, active_kd: float,
                     blend: float = 1.0) -> None:
        for name in JOINTS:
            if name == args.joint:
                kp = 2.0 + blend * (active_kp - 2.0)
                kd = 0.6 + blend * (active_kd - 0.6)
                target = active_target
                tau = 0.0
            else:
                configured = legacy.HOME_BASE_HOLD_GAINS[name]
                kp = 2.0 + blend * (float(configured["kp"]) - 2.0)
                kd = 0.6 + blend * (float(configured["kd"]) - 0.6)
                target = targets[name]
                tau = float(passive_hold_tau[name])
            arm.ctrl.controlMIT(arm.motors[name], kp, kd, target, 0.0, tau)

    try:
        initial = arm.read_status(JOINTS)
        targets = pose_from_status(initial)
        active_gravity, active_diag = gravity_prediction(model, args.joint, targets)
        active_distance = active_diag.get("nearest_distance_deg")
        if active_distance is None or float(active_distance) > args.max_gravity_distance_deg:
            raise ValueError(
                f"guarded nudge has no nearby gravity sample for {args.joint}; "
                f"nearest distance is {active_distance}deg"
            )
        if abs(active_gravity) > args.max_active_gravity_tau:
            raise ValueError(
                f"guarded nudge requires a low-load pose for {args.joint}; "
                f"predicted gravity torque is {active_gravity:.3f}, limit is "
                f"{args.max_active_gravity_tau:.3f}"
            )
        passive_hold_tau, passive_diagnostics = inertia_passive_hold_torques(
            model, args.joint, targets
        )
        unavailable = {
            name: item for name, item in passive_diagnostics.items()
            if item["source"] != "identified_gravity"
            or item["nearest_distance_deg"] is None
            or float(item["nearest_distance_deg"]) > args.max_gravity_distance_deg
        }
        if unavailable:
            raise ValueError(
                "guarded nudge requires nearby identified gravity for every passive joint: "
                + json.dumps(unavailable, sort_keys=True)
            )
        print("v3.1 guarded nudge active gravity=", json.dumps({
            "joint": args.joint, "tau": active_gravity,
            "nearest_distance_deg": active_distance,
            "applied_tau": 0.0,
        }, sort_keys=True))
        print("v3.1 guarded nudge passive hold=", json.dumps(passive_diagnostics, sort_keys=True))

        arm.enable(JOINTS)
        motors_enabled = True
        takeover_started = time.time()
        while time.time() - takeover_started < args.settle_seconds:
            progress = clamp((time.time() - takeover_started) / args.settle_seconds, 0.0, 1.0)
            blend = progress * progress * (3.0 - 2.0 * progress)
            command_pose(targets[args.joint], args.test_kp, args.test_kd, blend)
            time.sleep(args.control_dt)

        settled = arm.read_status(JOINTS)
        targets = pose_from_status(settled)
        start_pose = dict(targets)
        step_rad = math.radians(args.step_deg)
        final_target = start_pose[args.joint] + step_rad
        move_started = time.time()
        while time.time() - move_started < args.seconds:
            progress = clamp((time.time() - move_started) / args.seconds, 0.0, 1.0)
            blend = progress * progress * (3.0 - 2.0 * progress)
            active_target = start_pose[args.joint] + blend * step_rad
            command_pose(active_target, args.test_kp, args.test_kd)
            time.sleep(args.control_dt)
            status = arm.read_status(JOINTS)
            for name in JOINTS:
                velocity = abs(float(status[name]["vel"]))
                if velocity > args.max_velocity:
                    raise RuntimeError(
                        f"guarded nudge velocity watchdog tripped for {name}: {velocity:.3f}rad/s"
                    )
                displacement = abs(math.degrees(wrapped_difference(
                    float(status[name]["pos"]), start_pose[name]
                )))
                limit = args.max_position_error_deg
                if name == args.joint:
                    limit += abs(args.step_deg)
                if displacement > limit:
                    raise RuntimeError(
                        f"guarded nudge position watchdog tripped for {name}: {displacement:.2f}deg"
                    )
        final_status = arm.read_status(JOINTS)
    except Exception as exc:
        failure = exc
        raise
    finally:
        if motors_enabled and targets is not None and passive_hold_tau:
            try:
                if failure:
                    stop_status = arm.read_status(JOINTS)
                    targets = pose_from_status(stop_status)
                    hold_target = targets[args.joint]
                    hold_kp = 2.0
                    hold_kd = 1.0
                    hold_blend = 0.0
                else:
                    hold_target = final_target
                    hold_kp = args.test_kp
                    hold_kd = args.test_kd
                    hold_blend = 1.0
                stop_started = time.time()
                while time.time() - stop_started < args.stop_seconds:
                    command_pose(hold_target, hold_kp, hold_kd, blend=hold_blend)
                    time.sleep(args.control_dt)
                if not failure:
                    final_status = arm.read_status(JOINTS)
            except Exception as stop_error:
                print("v3.1 guarded nudge soft-stop failed:", stop_error)
        arm.close()

    return {
        "ok": True,
        "motion_issued": True,
        "joint": args.joint,
        "step_deg": args.step_deg,
        "active_tau": 0.0,
        "target_rad": final_target,
        "final": final_status,
    }


def set_verified(args) -> dict:
    path = Path(args.model)
    model = load_model(path, create=True)
    model["gain_units_verified"] = bool(args.verified)
    save_model(path, model)
    return {"ok": True, "gain_units_verified": model["gain_units_verified"], "model": str(path)}


def preview_schedule(args) -> dict:
    model = load_model(Path(args.model))
    joints = parse_joints(args.joints)
    if args.pose_json:
        pose = {name: float(value) for name, value in json.loads(args.pose_json).items() if name in JOINTS}
        velocities = {name: 0.0 for name in joints}
        status = None
    else:
        _, arm = open_arm()
        try:
            status = arm.read_status(JOINTS)
        finally:
            arm.close()
        pose = pose_from_status(status)
        velocities = {name: float(status[name]["vel"]) for name in joints}
    scheduled = {joint: scheduled_terms(model, joint, pose, 0.0) for joint in joints}
    hold_takeover = {}
    for joint in joints if status is not None else []:
        takeover = static_hold_bias(
            model, joint, pose, float(status[joint]["tau"])
        )
        hold_takeover[joint] = {
            "measured_tau": float(status[joint]["tau"]),
            "tau_ff_without_static_bias": takeover["tau_ff_without_static_bias"],
            "static_hold_bias_tau": takeover["static_hold_bias_tau"],
            "static_hold_bias_raw_tau": takeover["static_hold_bias_raw_tau"],
            "static_hold_bias_clamped": takeover["static_hold_bias_clamped"],
            "takeover_tau_ff": takeover["tau_ff"],
        }
        support = hold_load_support(
            model, joint, pose, float(status[joint]["tau"]), velocities[joint], 0.4
        )
        hold_takeover[joint].update({
            "takeover_tau_ff": support["bridge_tau_ff"],
            "load_support_tau": support["load_support_tau"],
            "hold_tau_limits": support["hold_tau_limits"],
            "supported_hold_tau_ff": supported_hold_terms(model, joint, pose, support)["tau_ff"],
        })
    return {
        "ok": True,
        "gain_units_verified": bool(model.get("gain_units_verified")),
        "pose_rad": pose,
        # Preview the same zero target-velocity terms used by model_hold.
        "measured_velocity_rad_s": velocities,
        "scheduled": scheduled,
        "model_hold_takeover": hold_takeover,
    }


def model_hold(args) -> dict:
    if not args.execute:
        raise RuntimeError("execution disabled; add --execute after reviewing the model and preview")
    if not MIT_KP_MIN <= args.takeover_kp <= MIT_KP_MAX:
        raise ValueError(f"--takeover-kp must be in [{MIT_KP_MIN}, {MIT_KP_MAX}]")
    if not MIT_KD_MIN <= args.takeover_kd <= MIT_KD_MAX:
        raise ValueError(f"--takeover-kd must be in [{MIT_KD_MIN}, {MIT_KD_MAX}]")
    model = load_model(Path(args.model))
    joints = parse_joints(args.joints)
    validate_ready(model, joints)
    general_fallback = None
    if getattr(args, 'general_fallback_model', None):
        from left_arm_general_motion import load_configuration
        general_fallback = load_configuration(args.general_fallback_model, 'apply')['model']
    legacy, arm = open_arm()
    log_path = Path(args.log)
    settings = model["settings"]
    controller_filter = SlewLimitedFilter(
        cutoff_hz=float(settings["gain_filter_hz"]),
        kp_slew=float(settings["kp_slew_per_second"]),
        kd_slew=float(settings["kd_slew_per_second"]),
        tau_slew=float(settings["tau_slew_per_second"]),
    )
    started = time.time()
    last = started
    next_log = started
    reason = "duration_complete"
    last_status = {}
    last_errors_deg = {}
    max_velocity_rad_s = {joint: 0.0 for joint in joints}
    max_abs_error_deg = {joint: 0.0 for joint in joints}
    rejected_velocity_spikes = []
    static_bias_tau = {}
    load_support = {}
    preflight_still = {"enabled": False}
    post_enable_still = {"enabled": False}
    seamless_takeover = {"enabled": False}
    try:
        if args.preflight_still_timeout > 0.0 and not args.motors_already_enabled:
            preflight_still = wait_until_still(
                arm,
                joints,
                max_velocity=args.preflight_max_velocity,
                max_drift_deg=args.preflight_max_drift_deg,
                stable_seconds=args.preflight_still_seconds,
                timeout=args.preflight_still_timeout,
                poll_interval=min(0.05, args.control_dt),
            )
            preflight_still["enabled"] = True
            print(
                "v3.1 model hold preflight still= "
                + json.dumps(preflight_still, sort_keys=True),
                flush=True,
            )
        if args.motors_already_enabled:
            # The preceding motion process has stopped transmitting MIT
            # frames. Do not spend a read-only settling window here: at a
            # loaded pose the motor command timeout can release a joint before
            # Model Hold begins. Capture and bridge the live state immediately.
            preflight_still = {
                "enabled": False,
                "reason": "replaced-by-immediate-seamless-takeover",
            }
            print(
                "v3.1 model hold reusing enabled motors; enable frames skipped= "
                + json.dumps(joints),
                flush=True,
            )
        else:
            arm.enable(joints)
        if args.preflight_still_timeout > 0.0 and not args.motors_already_enabled:
            # An enable frame can produce a short feedback transient even
            # after the arm was stationary. Do not capture targets or send
            # MIT commands until feedback and encoder position settle again.
            post_enable_still = wait_until_still(
                arm,
                joints,
                max_velocity=args.preflight_max_velocity,
                max_drift_deg=args.preflight_max_drift_deg,
                stable_seconds=args.preflight_still_seconds,
                timeout=args.preflight_still_timeout,
                poll_interval=min(0.05, args.control_dt),
            )
            post_enable_still["enabled"] = True
            print(
                "v3.1 model hold post-enable still= "
                + json.dumps(post_enable_still, sort_keys=True),
                flush=True,
            )
            # The requested hold duration begins after both handoff gates.
            started = time.time()
            last = started
            next_log = started
        initial = arm.read_status(JOINTS)
        targets = {joint: float(initial[joint]["pos"]) for joint in joints}
        watchdog_reference_position = dict(targets)
        if args.motors_already_enabled:
            try:
                initial, initial_spike = confirm_velocity_watchdog(
                    arm,
                    initial,
                    joints,
                    max_velocity=args.preflight_max_velocity,
                    reference_position=watchdog_reference_position,
                    max_unconfirmed_drift_deg=args.velocity_spike_max_drift_deg,
                )
            except RuntimeError:
                reason = "seamless_takeover_velocity_watchdog"
                raise
            if initial_spike is not None:
                initial_spike["phase"] = "seamless_takeover_initial_read"
                rejected_velocity_spikes.append(initial_spike)
                targets = {joint: float(initial[joint]["pos"]) for joint in joints}
                watchdog_reference_position = dict(targets)
        initial_pose = pose_from_status(initial)
        if general_fallback is not None:
            from left_arm_general_move import supply_hold_coverage
            model, supplied = supply_hold_coverage(model, general_fallback, initial_pose, joints)
            print('v3.1 current-hold general coverage= ' + json.dumps({
                'supplied': supplied, 'persisted': False}), flush=True)
        # The preceding PD controller may provide more load torque than the
        # model feed-forward limits. Preserve that difference for this hold;
        # clipping it, or decaying it to the Coulomb branch, unloads the joint.
        for joint in joints:
            support = hold_load_support(
                model, joint, initial_pose, float(initial[joint]["tau"]),
                float(initial[joint]["vel"]), args.takeover_kd,
            )
            load_support[joint] = support
            static_bias_tau[joint] = float(support["static_hold_bias_tau"])
            controller_filter.seed(
                joint,
                args.takeover_kp,
                args.takeover_kd,
                support["bridge_tau_ff"],
            )
        if args.motors_already_enabled:
            bridge = {}
            for joint in joints:
                tau = load_support[joint]["bridge_tau_ff"]
                arm.ctrl.controlMIT(
                    arm.motors[joint],
                    args.takeover_kp,
                    args.takeover_kd,
                    targets[joint],
                    0.0,
                    tau,
                )
                bridge[joint] = {
                    "target_rad": targets[joint],
                    "measured_velocity_rad_s": float(initial[joint]["vel"]),
                    "takeover_tau": tau,
                    "measured_tau": float(initial[joint]["tau"]),
                    "load_support_tau": load_support[joint]["load_support_tau"],
                    "hold_tau_limits": load_support[joint]["hold_tau_limits"],
                }
            seamless_takeover = {
                "enabled": True,
                "mode": "immediate-current-state-bridge",
                "kp": args.takeover_kp,
                "kd": args.takeover_kd,
                "joints": bridge,
            }
            print(
                "v3.1 model hold seamless takeover bridge= "
                + json.dumps(seamless_takeover, sort_keys=True),
                flush=True,
            )
            # Count the requested hold duration from the completed bridge.
            started = time.time()
            last = started
            next_log = started
        while time.time() - started < args.seconds:
            now = time.time()
            dt = max(1e-4, now - last)
            last = now
            status = arm.read_status(JOINTS)
            for joint in joints:
                max_velocity_rad_s[joint] = max(
                    max_velocity_rad_s[joint], abs(float(status[joint]["vel"]))
                )
            try:
                status, rejected_spike = confirm_velocity_watchdog(
                    arm,
                    status,
                    joints,
                    max_velocity=args.max_velocity,
                    reference_position=watchdog_reference_position,
                    max_unconfirmed_drift_deg=args.velocity_spike_max_drift_deg,
                )
            except RuntimeError:
                reason = "velocity_watchdog"
                raise
            if rejected_spike is not None:
                rejected_spike["time_s"] = now - started
                rejected_velocity_spikes.append(rejected_spike)
                print(
                    "v3.1 model hold rejected isolated velocity spike= "
                    + json.dumps(rejected_spike, sort_keys=True),
                    flush=True,
                )
            pose = pose_from_status(status)
            watchdog_reference_position = dict(pose)
            errors_deg = {joint: math.degrees(targets[joint] - pose[joint]) for joint in joints}
            last_status = {joint: dict(status[joint]) for joint in joints}
            last_errors_deg = dict(errors_deg)
            for joint in joints:
                max_velocity_rad_s[joint] = max(
                    max_velocity_rad_s[joint], abs(float(status[joint]["vel"]))
                )
                max_abs_error_deg[joint] = max(
                    max_abs_error_deg[joint], abs(float(errors_deg[joint]))
                )
            if any(abs(error) > args.max_error_deg for error in errors_deg.values()):
                reason = "position_error_watchdog"
                raise RuntimeError(f"position error watchdog tripped: {json.dumps(errors_deg)}")
            applied = {}
            desired_all = {}
            for joint in joints:
                desired = supported_hold_terms(model, joint, pose, load_support[joint])
                desired_all[joint] = desired
                applied[joint] = controller_filter.update(joint, desired, dt)
                arm.ctrl.controlMIT(
                    arm.motors[joint],
                    applied[joint]["kp"],
                    applied[joint]["kd"],
                    targets[joint],
                    0.0,
                    applied[joint]["tau_ff"],
                )
            if now >= next_log:
                append_jsonl(log_path, {
                    "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "event": "model_hold",
                    "pose_rad": pose,
                    "errors_deg": errors_deg,
                    "applied": applied,
                    "desired": desired_all,
                })
                next_log = now + args.log_interval
            time.sleep(args.control_dt)
    finally:
        arm.close()
        append_jsonl(log_path, {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "event": "model_hold_end",
            "reason": reason,
            "duration_s": time.time() - started,
        })
    return {
        "ok": True,
        "reason": reason,
        "duration_s": time.time() - started,
        "final": last_status,
        "final_error_deg": last_errors_deg,
        "max_velocity_rad_s": max_velocity_rad_s,
        "max_abs_error_deg": max_abs_error_deg,
        "rejected_velocity_spikes": rejected_velocity_spikes,
        "static_hold_bias_tau": static_bias_tau,
        "load_support": load_support,
        "motor_enable_mode": (
            "reused-existing-enable" if args.motors_already_enabled else "enabled-by-model-hold"
        ),
        "seamless_takeover": seamless_takeover,
        "preflight_still": preflight_still,
        "post_enable_still": post_enable_still,
        "log": str(log_path),
    }


def configure(args) -> dict:
    path = Path(args.model)
    model = load_model(path, create=True)
    joint = args.joint
    if joint not in JOINTS:
        raise ValueError(f"unknown joint: {joint}")
    scheduler = model["joints"][joint]["scheduler"]
    for key in ("omega_n", "zeta", "kp_min", "kp_max", "kd_min", "kd_max"):
        value = getattr(args, key)
        if value is not None:
            scheduler[key] = float(value)
    inertia_fallback = getattr(args, "inertia_fallback", None)
    if inertia_fallback is not None:
        scheduler["inertia_fallback"] = str(inertia_fallback)
    tau_limits = list(map(float, model["joints"][joint]["tau_limits"]))
    tau_min = getattr(args, "tau_min", None)
    tau_max = getattr(args, "tau_max", None)
    if tau_min is not None:
        tau_limits[0] = float(tau_min)
    if tau_max is not None:
        tau_limits[1] = float(tau_max)
    if scheduler["omega_n"] <= 0 or not 0.5 <= scheduler["zeta"] <= 1.5:
        raise ValueError("omega_n must be positive and zeta must be in [0.5, 1.5]")
    if scheduler["kp_min"] > scheduler["kp_max"] or scheduler["kd_min"] > scheduler["kd_max"]:
        raise ValueError("gain minimum cannot exceed gain maximum")
    if not MIT_KP_MIN <= scheduler["kp_min"] <= scheduler["kp_max"] <= MIT_KP_MAX:
        raise ValueError(f"kp limits must stay in the MIT protocol range [{MIT_KP_MIN}, {MIT_KP_MAX}]")
    if not MIT_KD_MIN <= scheduler["kd_min"] <= scheduler["kd_max"] <= MIT_KD_MAX:
        raise ValueError(f"kd limits must stay in the MIT protocol range [{MIT_KD_MIN}, {MIT_KD_MAX}]")
    hardware_tau_max = MIT_TAU_MAX[joint]
    if not -hardware_tau_max <= tau_limits[0] < tau_limits[1] <= hardware_tau_max:
        raise ValueError(
            f"tau limits for {joint} must be ordered and stay in the MIT protocol range "
            f"[-{hardware_tau_max}, {hardware_tau_max}]"
        )
    model["joints"][joint]["tau_limits"] = tau_limits
    save_model(path, model)
    return {
        "ok": True,
        "joint": joint,
        "scheduler": scheduler,
        "tau_limits": tau_limits,
        "model": str(path),
    }


def configure_friction(args) -> dict:
    path = Path(args.model)
    model = load_model(path, create=True)
    joint = args.joint
    if joint not in JOINTS:
        raise ValueError(f"unknown joint: {joint}")
    if args.coulomb < 0.0 or args.viscous < 0.0:
        raise ValueError("friction coefficients must be non-negative magnitudes")
    friction = {
        "coulomb": float(args.coulomb),
        "viscous": float(args.viscous),
        "identified": True,
        "method": args.method,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    model["joints"][joint]["friction"] = friction
    save_model(path, model)
    return {"ok": True, "joint": joint, "friction": friction, "model": str(path)}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="v3.1 identified-dynamics, low-vibration arm controller")
    parser.add_argument("--model", default=str(DEFAULT_MODEL_PATH))
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("init-model", help="Create a model file without opening the motor controller")
    sub.add_parser("show-model", help="Print the model file")

    verify = sub.add_parser("set-units-verified", help="Record explicit MIT gain/torque unit verification")
    # BooleanOptionalAction is unavailable in the Python/argparse version on
    # the Jetson.  Keep both spellings with the portable store actions.
    verify_group = verify.add_mutually_exclusive_group()
    verify_group.add_argument("--verified", dest="verified", action="store_true")
    verify_group.add_argument("--no-verified", dest="verified", action="store_false")
    verify.set_defaults(verified=True)

    gravity = sub.add_parser("capture-gravity", help="Capture a stationary gravity/load sample; does not command motion")
    gravity.add_argument("--joints", default=",".join(JOINTS))
    gravity.add_argument("--samples", type=int, default=30)
    gravity.add_argument("--interval", type=float, default=0.02)
    gravity.add_argument("--max-velocity", type=float, default=0.03)
    gravity.add_argument("--direction", choices=("positive", "negative", "stationary"), default="stationary")
    gravity.add_argument("--source", default="manual-quasi-static")

    inertia = sub.add_parser("add-inertia", help="Add an effective-inertia sample from a measured step response")
    inertia.add_argument("--joint", required=True, choices=JOINTS)
    inertia.add_argument("--pose-json")
    inertia.add_argument("--from-arm", action="store_true")
    inertia.add_argument("--effective-inertia", type=float)
    inertia.add_argument("--test-kp", type=float)
    inertia.add_argument("--measured-omega-n", type=float)
    inertia.add_argument("--max-inertia", type=float, default=100.0)
    inertia.add_argument("--label", default="")

    measure = sub.add_parser("measure-inertia", help="Run a guarded small-angle inertia step test")
    measure.add_argument("--joint", required=True, choices=JOINTS)
    measure.add_argument("--step-deg", type=float, default=2.0)
    measure.add_argument("--test-kp", type=float, default=40.0)
    measure.add_argument("--test-kd", type=float, default=3.0)
    measure.add_argument("--settle-seconds", type=float, default=1.5)
    measure.add_argument("--test-seconds", type=float, default=1.2)
    measure.add_argument("--return-seconds", type=float, default=1.5)
    measure.add_argument("--control-dt", type=float, default=0.01)
    measure.add_argument("--feedback-wait", type=float, default=0.004)
    measure.add_argument("--max-velocity", type=float, default=1.0)
    measure.add_argument("--max-position-error-deg", type=float, default=5.0)
    measure.add_argument("--label", default="")
    measure.add_argument("--log", default=str(DEFAULT_INERTIA_LOG_PATH))
    measure.add_argument("--execute", action="store_true")

    nudge = sub.add_parser(
        "guarded-nudge",
        help="Run a slow low-gain nudge using identified gravity on passive joints",
    )
    nudge.add_argument("--joint", required=True, choices=JOINTS)
    nudge.add_argument("--step-deg", type=float, default=2.0)
    nudge.add_argument("--test-kp", type=float, default=6.0)
    nudge.add_argument("--test-kd", type=float, default=0.8)
    nudge.add_argument("--settle-seconds", type=float, default=1.5)
    nudge.add_argument("--seconds", type=float, default=6.0)
    nudge.add_argument("--stop-seconds", type=float, default=0.5)
    nudge.add_argument("--control-dt", type=float, default=0.01)
    nudge.add_argument("--max-velocity", type=float, default=0.3)
    nudge.add_argument("--max-position-error-deg", type=float, default=2.0)
    nudge.add_argument("--max-gravity-distance-deg", type=float, default=5.0)
    nudge.add_argument("--max-active-gravity-tau", type=float, default=0.15)
    nudge.add_argument("--execute", action="store_true")

    reprocess = sub.add_parser("reprocess-inertia", help="Fit the latest logged trace without moving motors")
    reprocess.add_argument("--joint", required=True, choices=JOINTS)
    reprocess.add_argument("--log", default=str(DEFAULT_INERTIA_LOG_PATH))
    reprocess.add_argument("--label", default="")

    remove_inertia = sub.add_parser(
        "remove-latest-inertia", help="Remove the newest stored inertia sample without moving motors"
    )
    remove_inertia.add_argument("--joint", required=True, choices=JOINTS)

    remove_gravity = sub.add_parser(
        "remove-latest-gravity", help="Remove the newest stored gravity sample without moving motors"
    )
    remove_gravity.add_argument("--joint", required=True, choices=JOINTS)

    config = sub.add_parser("configure", help="Configure natural frequency, damping, and gain limits")
    config.add_argument("--joint", required=True, choices=JOINTS)
    for option in ("omega_n", "zeta", "kp_min", "kp_max", "kd_min", "kd_max"):
        config.add_argument("--" + option.replace("_", "-"), dest=option, type=float)
    config.add_argument(
        "--inertia-fallback",
        choices=("reject", "minimum-gains"),
        help="Behavior when no inertia sample is inside the configured distance limit",
    )
    config.add_argument("--tau-min", type=float, help="Joint-side feed-forward torque lower limit")
    config.add_argument("--tau-max", type=float, help="Joint-side feed-forward torque upper limit")

    friction = sub.add_parser("configure-friction", help="Store identified Coulomb and viscous friction magnitudes")
    friction.add_argument("--joint", required=True, choices=JOINTS)
    friction.add_argument("--coulomb", type=float, required=True)
    friction.add_argument("--viscous", type=float, required=True)
    friction.add_argument("--method", default="symmetric-low-speed-identification")

    preview = sub.add_parser("preview", help="Calculate scheduled gains and feed-forward without commanding motors")
    preview.add_argument("--joints", default=",".join(JOINTS))
    preview.add_argument("--pose-json")

    hold = sub.add_parser("model-hold", help="Hold the current pose with identified, scheduled gains")
    hold.add_argument("--joints", default=",".join(JOINTS))
    hold.add_argument("--seconds", type=float, default=5.0)
    hold.add_argument("--control-dt", type=float, default=0.01)
    hold.add_argument("--takeover-kp", type=float, default=2.0)
    hold.add_argument("--takeover-kd", type=float, default=0.4)
    hold.add_argument("--max-velocity", type=float, default=0.3)
    hold.add_argument(
        "--velocity-spike-max-drift-deg",
        type=float,
        default=0.1,
        help="Maximum encoder motion allowed when rejecting one isolated velocity sample",
    )
    hold.add_argument("--max-error-deg", type=float, default=2.5)
    hold.add_argument("--preflight-still-timeout", type=float, default=0.0)
    hold.add_argument("--preflight-still-seconds", type=float, default=0.5)
    hold.add_argument("--preflight-max-velocity", type=float, default=0.1)
    hold.add_argument("--preflight-max-drift-deg", type=float, default=0.25)
    hold.add_argument(
        "--motors-already-enabled",
        action="store_true",
        help=(
            "Reuse motor enable state established by an immediately preceding motion; "
            "skip duplicate enable frames"
        ),
    )
    hold.add_argument("--log", default=str(DEFAULT_LOG_PATH))
    hold.add_argument('--general-fallback-model', help='General-model current-pose priors when local hold samples are absent; no learning-file writes')
    hold.add_argument("--log-interval", type=float, default=0.25)
    hold.add_argument("--execute", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "init-model":
            result = {"ok": True, "model": str(args.model), "data": load_model(Path(args.model), create=True)}
        elif args.command == "show-model":
            result = {"ok": True, "model": str(args.model), "data": load_model(Path(args.model))}
        elif args.command == "set-units-verified":
            result = set_verified(args)
        elif args.command == "capture-gravity":
            result = capture_gravity(args)
        elif args.command == "add-inertia":
            result = add_inertia(args)
        elif args.command == "measure-inertia":
            result = measure_inertia(args)
        elif args.command == "guarded-nudge":
            result = guarded_nudge(args)
        elif args.command == "reprocess-inertia":
            result = reprocess_inertia(args)
        elif args.command == "remove-latest-inertia":
            result = remove_latest_inertia(args)
        elif args.command == "remove-latest-gravity":
            result = remove_latest_gravity(args)
        elif args.command == "configure":
            result = configure(args)
        elif args.command == "configure-friction":
            result = configure_friction(args)
        elif args.command == "preview":
            result = preview_schedule(args)
        elif args.command == "model-hold":
            result = model_hold(args)
        else:
            parser.error("unsupported command")
            return
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except (ValueError, RuntimeError, FileNotFoundError) as exc:
        parser.exit(1, f"error: {exc}\n")


if __name__ == "__main__":
    main()
