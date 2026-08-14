#!/usr/bin/env python3
"""Saved move helpers for left_arm_v2_5_3 web controller 1.2.

This file is intentionally separate from left_arm_v2_5_3.py so the existing
controller stays unchanged while the web controller can add move capture/replay.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from typing import Dict, Iterable


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MOVES_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_v2_5_3_saved_moves.json")
ARM_SCRIPT = os.path.join(SCRIPT_DIR, "left_arm_v2_5_3.py")
TABLE_CLEARANCE_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_v2_table_clearance.json")
DEFAULT_JOINTS = [
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
    "elbow",
    "arm_roll",
    "wrist_side",
    "wrist",
]
REPLAY_ORDER = [
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
    "elbow",
    "arm_roll",
    "wrist_side",
]
REPLAY_FINAL_JOINT = "wrist"
REPLAY_REQUIRED_JOINTS = DEFAULT_JOINTS

REPLAY_COMPLETED_HOLD_GAINS = {
    "shoulder_front": {"kp": 75.0, "kd": 7.0},
    "shoulder_side": {"kp": 65.0, "kd": 4.5},
    "shoulder_rotate": {"kp": 8.0, "kd": 4.0},
    "elbow": {"kp": 78.0, "kd": 7.2},
    "arm_roll": {"kp": 6.0, "kd": 6.0},
    "wrist_side": {"kp": 8.0, "kd": 1.6},
    "wrist": {"kp": 8.0, "kd": 1.4},
}

REPLAY_PROTECTED_COMPLETED_HOLDS = {
    "shoulder_rotate",
}

REPLAY_COMPLETED_HOLD_TAU = {
    "shoulder_front": 2.4,
    "elbow": 5.1,
    "wrist": 0.55,
}

REPLAY_PENDING_HOLD_TAU = {
    "shoulder_front": 2.4,
    "elbow": 1.2,
    "wrist": 0.55,
}

REPLAY_PENDING_HOLD_GAINS_BY_ACTIVE = {
    "shoulder_rotate": {
        "elbow": {"kp": 92.0, "kd": 6.2},
    },
}

REPLAY_PENDING_HOLD_TAU_BY_ACTIVE = {
    "shoulder_rotate": {
        "elbow": 3.2,
    },
}

REPLAY_ACTIVE_GAINS = {
    "elbow": {"kp": 76.0, "kd": 4.0},
}

REPLAY_MIN_SECONDS = {
    "elbow": 3.0,
}

REPLAY_SETTLE_ORDER = [
    "elbow",
    "shoulder_rotate",
    "wrist_side",
    "shoulder_front",
    "shoulder_side",
    "arm_roll",
]

REPLAY_SETTLE_DEADBAND_DEG = 2.0
REPLAY_SETTLE_ENABLED = False
REPLAY_SETTLE_BASE_SECONDS = 3.0
REPLAY_SETTLE_SECONDS_PER_DEG = 0.8
REPLAY_SETTLE_MAX_SECONDS = 8.0

REPLAY_SETTLE_GAINS = {
    "shoulder_front": {"kp": 75.0, "kd": 7.0},
    "shoulder_side": {"kp": 65.0, "kd": 4.5},
    "shoulder_rotate": {"kp": 24.0, "kd": 4.5},
    "elbow": {"kp": 65.0, "kd": 6.5},
    "arm_roll": {"kp": 12.0, "kd": 7.0},
    "wrist_side": {"kp": 16.0, "kd": 2.2},
    "wrist": {"kp": 26.0, "kd": 4.7},
}

REPLAY_COUPLED_SETTLE_GROUP = [
    "shoulder_front",
    "shoulder_side",
    "elbow",
]

REPLAY_COUPLED_SETTLE_ENABLED = False

REPLAY_COUPLED_SETTLE_GAINS = {
    "shoulder_front": {"kp": 75.0, "kd": 7.0},
    "shoulder_side": {"kp": 65.0, "kd": 4.5},
    "elbow": {"kp": 65.0, "kd": 6.5},
}

REPLAY_LOW_SHOULDER_WRIST_SIDE_HOLD_TAU = -0.35
def arm_api():
    import left_arm_v2_5_3

    return left_arm_v2_5_3


def now_string() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def load_moves(path: str = MOVES_PATH) -> dict:
    if not os.path.exists(path):
        return {"type": "left_arm_v2_5_3_saved_moves", "moves": {}}
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if "moves" not in data or not isinstance(data["moves"], dict):
        raise RuntimeError(f"invalid moves file: {path}")
    return data


def save_moves(data: dict, path: str = MOVES_PATH) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")


def normalize_move_name(name: str) -> str:
    normalized = " ".join(name.strip().split())
    if not normalized:
        raise ValueError("move name is required")
    if len(normalized) > 80:
        raise ValueError("move name must be 80 characters or fewer")
    return normalized


def capture_move(name: str, note: str, path: str) -> dict:
    move_name = normalize_move_name(name)
    api = arm_api()
    arm = api.LeftArmV2()
    try:
        print("Serial port is open", flush=True)
        pose = arm.positions(DEFAULT_JOINTS)
    finally:
        arm.close()
    data = load_moves(path)
    replaced = move_name in data.get("moves", {})
    data["moves"][move_name] = {
        "name": move_name,
        "type": "left_arm_v2_5_3_saved_move",
        "created_at": now_string(),
        "note": note,
        "pose": pose,
        "replay_requires_table_clearance": True,
        "replay_order": REPLAY_ORDER,
        "replay_final_joint": REPLAY_FINAL_JOINT,
    }
    save_moves(data, path)
    print("v2.5.3 capture move " + ("overwritten" if replaced else "saved") + ":", move_name, flush=True)
    print(json.dumps(data["moves"][move_name], ensure_ascii=False, indent=2), flush=True)
    return data["moves"][move_name]


def list_moves(path: str) -> dict:
    data = load_moves(path)
    names = sorted(data.get("moves", {}).keys())
    return {"moves": [{"name": name, **data["moves"][name]} for name in names]}


def low_torque(kp: float, kd: float, control_dt: float, trace_interval: float) -> None:
    api = arm_api()
    arm = api.LeftArmV2()
    try:
        print("Serial port is open", flush=True)
        arm.enable(DEFAULT_JOINTS)
        print(
            "v2.5.3 low torque mode active",
            "joints=",
            ",".join(DEFAULT_JOINTS),
            "kp=",
            kp,
            "kd=",
            kd,
            "control_dt=",
            control_dt,
            flush=True,
        )
        next_trace = time.time()
        while True:
            current = arm.positions(DEFAULT_JOINTS)
            for name in DEFAULT_JOINTS:
                arm.ctrl.controlMIT(arm.motors[name], kp, kd, current[name], 0, 0)
            now = time.time()
            if now >= next_trace:
                print("v2.5.3 low torque status=", json.dumps(current, ensure_ascii=False), flush=True)
                next_trace = now + trace_interval
            time.sleep(control_dt)
    finally:
        arm.close()


def run_table_clearance(python_bin: str, clearance_file: str) -> None:
    cmd = [
        python_bin,
        "-u",
        ARM_SCRIPT,
        "clearance",
        "--clearance-file",
        clearance_file,
        "--deadband-deg",
        "0.5",
        "--max-delta-deg",
        "120",
        "--step-deg",
        "5",
        "--execute",
    ]
    print("v2.5.3 replay pre-step table clearance command=", " ".join(cmd), flush=True)
    completed = subprocess.run(cmd, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"table clearance failed with returncode {completed.returncode}")


def active_replay_gains(name: str, fallback_kp: float, fallback_kd: float) -> Dict[str, float]:
    api = arm_api()
    if name in REPLAY_ACTIVE_GAINS:
        return REPLAY_ACTIVE_GAINS[name]
    if name == "wrist":
        return {
            "kp": api.COUPLED_CLEARANCE_WRIST_FINE_GAINS["kp"],
            "kd": api.COUPLED_CLEARANCE_WRIST_FINE_GAINS["kd"],
        }
    if name == "wrist_side":
        return api.PREHOME_CLEARANCE_GAINS.get("wrist_side", api.COUPLED_CLEARANCE_MOVE_GAINS["wrist_side"])
    return api.NUDGE_GAINS.get(name, api.CLEARANCE_MOVE_GAINS.get(name, {"kp": fallback_kp, "kd": fallback_kd}))


def replay_seconds(name: str, delta_deg: float) -> float:
    api = arm_api()
    if name == "wrist_side":
        return api.prehome_clearance_seconds_for(name, delta_deg)
    if name == "wrist":
        base = api.COUPLED_CLEARANCE_WRIST_FINE_GAINS["seconds"]
    else:
        base = max(REPLAY_MIN_SECONDS.get(name, 2.2), api.NUDGE_MIN_SECONDS.get(name, 2.2), 2.2)
    return min(20.0, max(base, base * abs(delta_deg) / 5.0))


def apply_target_limits(name: str, target: float) -> float:
    api = arm_api()
    limits = api.NUDGE_TARGET_LIMITS.get(name)
    if not limits:
        return target
    limited = target
    min_target = limits.get("min")
    max_target = limits.get("max")
    if min_target is not None and limited < min_target:
        limited = min_target
    if max_target is not None and limited > max_target:
        limited = max_target
    if limited != target:
        print(
            "v2.5.3 replay target clamped",
            name,
            "requested=",
            target,
            "target=",
            limited,
            "limits=",
            json.dumps(limits, ensure_ascii=False),
            flush=True,
        )
    return limited


def is_low_shoulder_pose(pose: Dict[str, float]) -> bool:
    api = arm_api()
    shoulder_front = pose.get("shoulder_front")
    return shoulder_front is not None and shoulder_front < api.NUDGE_WRIST_ACTIVE_LOW_SHOULDER_FRONT_THRESHOLD


def build_hold_gains(
    active: str,
    hold_names: Iterable[str],
    completed_names: Iterable[str],
    fallback_kp: float,
    fallback_kd: float,
) -> Dict[str, Dict[str, float]]:
    api = arm_api()
    completed = set(completed_names)
    hold_gains = {
        hold_name: (
            REPLAY_COMPLETED_HOLD_GAINS.get(
                hold_name,
                api.CLEARANCE_BASE_HOLD_GAINS.get(hold_name, {"kp": fallback_kp, "kd": fallback_kd}),
            )
            if hold_name in completed
            else api.CLEARANCE_HOLD_GAINS.get(
                hold_name,
                api.CLEARANCE_BASE_HOLD_GAINS.get(hold_name, {"kp": fallback_kp, "kd": fallback_kd}),
            )
        )
        for hold_name in hold_names
    }
    for compliant_name in api.CLEARANCE_COMPLIANT_HOLDS_BY_ACTIVE.get(active, set()):
        if compliant_name in hold_gains:
            if compliant_name in completed and compliant_name in REPLAY_PROTECTED_COMPLETED_HOLDS:
                continue
            hold_gains[compliant_name] = api.CLEARANCE_COMPLIANT_HOLD_GAINS[compliant_name]
    nudge_compliant_gains = api.NUDGE_COMPLIANT_HOLD_GAINS_BY_ACTIVE.get(active, api.NUDGE_COMPLIANT_HOLD_GAINS)
    for compliant_name in api.NUDGE_COMPLIANT_HOLDS_BY_ACTIVE.get(active, set()):
        if compliant_name in hold_gains:
            if compliant_name in completed and compliant_name in REPLAY_PROTECTED_COMPLETED_HOLDS:
                continue
            hold_gains[compliant_name] = nudge_compliant_gains[compliant_name]
    for hold_name, gains in REPLAY_PENDING_HOLD_GAINS_BY_ACTIVE.get(active, {}).items():
        if hold_name in hold_gains and hold_name not in completed:
            hold_gains[hold_name] = gains
    return hold_gains


def build_hold_tau(active: str, hold_names: Iterable[str], completed_names: Iterable[str]) -> Dict[str, float]:
    completed = set(completed_names)
    hold_tau = {}
    for hold_name in hold_names:
        tau_source = REPLAY_COMPLETED_HOLD_TAU if hold_name in completed else REPLAY_PENDING_HOLD_TAU
        if hold_name in tau_source:
            hold_tau[hold_name] = tau_source[hold_name]
    for hold_name, tau in REPLAY_PENDING_HOLD_TAU_BY_ACTIVE.get(active, {}).items():
        if hold_name in hold_names and hold_name not in completed:
            hold_tau[hold_name] = tau
    return hold_tau


def tune_wrist_active_hold_tau(active: str, hold_targets: Dict[str, float], hold_tau: Dict[str, float]) -> Dict[str, float]:
    api = arm_api()
    low_shoulder_front = (
        "shoulder_front" in hold_targets
        and hold_targets["shoulder_front"] < api.NUDGE_WRIST_ACTIVE_LOW_SHOULDER_FRONT_THRESHOLD
    )
    if not low_shoulder_front:
        return hold_tau
    tuned = dict(hold_tau)
    if "shoulder_front" in hold_targets:
        tuned["shoulder_front"] = api.NUDGE_LOW_SHOULDER_FRONT_HOLD_TAU
    if active in ("shoulder_front", "wrist") and "wrist_side" in hold_targets:
        tuned["wrist_side"] = api.NUDGE_LOW_SHOULDER_WRIST_SIDE_HOLD_TAU
    if active in ("wrist_side", "wrist"):
        for hold_name, tau in api.NUDGE_LOW_SHOULDER_WRIST_HOLD_TAU.items():
            if hold_name in hold_targets:
                tuned[hold_name] = tau
    return tuned


def replay_settle_seconds(delta_deg: float) -> float:
    return min(
        REPLAY_SETTLE_MAX_SECONDS,
        max(REPLAY_SETTLE_BASE_SECONDS, abs(delta_deg) * REPLAY_SETTLE_SECONDS_PER_DEG),
    )


def run_replay_coupled_shoulder_elbow_settle(
    arm,
    pose: Dict[str, float],
    errors: Dict[str, float],
    fallback_kp: float,
    fallback_kd: float,
) -> bool:
    group_names = [joint for joint in REPLAY_COUPLED_SETTLE_GROUP if joint in pose]
    if len(group_names) != len(REPLAY_COUPLED_SETTLE_GROUP):
        return False
    if not any(abs(errors.get(joint, 0.0)) > REPLAY_SETTLE_DEADBAND_DEG for joint in group_names):
        return False

    max_delta_deg = max(abs(errors.get(joint, 0.0)) for joint in group_names)
    seconds = replay_settle_seconds(max_delta_deg)
    targets = {joint: float(pose[joint]) for joint in group_names}
    current = arm.positions(DEFAULT_JOINTS)
    current_wrist = current.get(REPLAY_FINAL_JOINT, float(pose.get(REPLAY_FINAL_JOINT, 0.0)))
    hold_targets = {
        joint: current_wrist if joint == REPLAY_FINAL_JOINT else float(pose[joint])
        for joint in DEFAULT_JOINTS
        if joint not in targets and joint in pose
    }
    hold_gains = build_hold_gains(
        "elbow",
        hold_targets.keys(),
        hold_targets.keys(),
        fallback_kp,
        fallback_kd,
    )
    hold_tau = build_hold_tau("elbow", hold_targets.keys(), hold_targets.keys())
    print(
        "v2.5.3 replay settle coupled shoulder/elbow active",
        ",".join(group_names),
        "errors_deg=",
        json.dumps({joint: errors.get(joint, 0.0) for joint in group_names}, ensure_ascii=False),
        "targets=",
        json.dumps(targets, ensure_ascii=False),
        "seconds=",
        seconds,
        "hold joints:",
        ",".join(hold_targets.keys()),
        flush=True,
    )
    arm.move_targets_with_holds(
        targets,
        seconds_per_step=seconds,
        move_gains=REPLAY_COUPLED_SETTLE_GAINS,
        hold_targets=hold_targets,
        hold_gains=hold_gains,
        fallback_kp=fallback_kp,
        fallback_kd=fallback_kd,
        hold_tau=hold_tau,
        preload_seconds=0.2,
        control_dt=0.01,
        trajectory="smoothstep",
    )
    return True


def run_replay_settle_pass(
    arm,
    pose: Dict[str, float],
    fallback_kp: float,
    fallback_kd: float,
) -> None:
    api = arm_api()
    current = arm.positions(DEFAULT_JOINTS)
    error_names = list(dict.fromkeys(REPLAY_COUPLED_SETTLE_GROUP + REPLAY_SETTLE_ORDER))
    errors = {
        joint: math.degrees(float(pose[joint]) - current[joint])
        for joint in error_names
        if joint in pose
    }
    selected = [joint for joint in REPLAY_SETTLE_ORDER if abs(errors.get(joint, 0.0)) > REPLAY_SETTLE_DEADBAND_DEG]
    print(
        "v2.5.3 replay settle errors_deg=",
        json.dumps(errors, ensure_ascii=False),
        "deadband_deg=",
        REPLAY_SETTLE_DEADBAND_DEG,
        flush=True,
    )
    if not REPLAY_SETTLE_ENABLED:
        print("v2.5.3 replay settle disabled; relying on completed joint holds", flush=True)
        return
    ran_coupled = False
    if REPLAY_COUPLED_SETTLE_ENABLED:
        ran_coupled = run_replay_coupled_shoulder_elbow_settle(arm, pose, errors, fallback_kp, fallback_kd)
    if ran_coupled:
        current = arm.positions(DEFAULT_JOINTS)
        errors = {
            joint: math.degrees(float(pose[joint]) - current[joint])
            for joint in error_names
            if joint in pose
        }
        print(
            "v2.5.3 replay settle errors_after_coupled_deg=",
            json.dumps(errors, ensure_ascii=False),
            flush=True,
        )

    selected = [joint for joint in REPLAY_SETTLE_ORDER if abs(errors.get(joint, 0.0)) > REPLAY_SETTLE_DEADBAND_DEG]
    if not selected:
        if not ran_coupled:
            print("v2.5.3 replay settle skip all joints inside deadband", flush=True)
        return

    for active in selected:
        current = arm.positions(DEFAULT_JOINTS)
        target = float(pose[active])
        delta_deg = math.degrees(target - current[active])
        if abs(delta_deg) <= REPLAY_SETTLE_DEADBAND_DEG:
            print(
                "v2.5.3 replay settle skip",
                active,
                "delta_deg=",
                delta_deg,
                "deadband_deg=",
                REPLAY_SETTLE_DEADBAND_DEG,
                flush=True,
            )
            continue
        gains = REPLAY_SETTLE_GAINS.get(active, active_replay_gains(active, fallback_kp, fallback_kd))
        seconds = replay_settle_seconds(delta_deg)
        hold_targets = {
            joint: current[REPLAY_FINAL_JOINT] if joint == REPLAY_FINAL_JOINT else float(pose[joint])
            for joint in DEFAULT_JOINTS
            if joint != active and joint in pose
        }
        hold_gains = build_hold_gains(active, hold_targets.keys(), hold_targets.keys(), fallback_kp, fallback_kd)
        hold_tau = build_hold_tau(active, hold_targets.keys(), hold_targets.keys())
        active_tau = api.nudge_active_tau_for(active, delta_deg)
        if active == "shoulder_front":
            active_tau = min(active_tau, REPLAY_COMPLETED_HOLD_TAU.get("shoulder_front", active_tau))
        print(
            "v2.5.3 replay settle active",
            active,
            "delta_deg=",
            delta_deg,
            "target=",
            target,
            "kp=",
            gains["kp"],
            "kd=",
            gains["kd"],
            "seconds=",
            seconds,
            "hold joints:",
            ",".join(hold_targets.keys()),
            flush=True,
        )
        arm.move_target_with_holds(
            active,
            target,
            seconds_per_step=seconds,
            kp=gains["kp"],
            kd=gains["kd"],
            hold_targets=hold_targets,
            hold_gains=hold_gains,
            fallback_kp=fallback_kp,
            fallback_kd=fallback_kd,
            hold_tau=hold_tau,
            active_tau=active_tau,
            control_dt=0.01,
            preload_seconds=0.2,
            trajectory="smoothstep",
            linear_blend=0.0,
        )


def run_final_wrist_move(
    arm,
    pose: Dict[str, float],
    fallback_kp: float,
    fallback_kd: float,
    deadband_deg: float,
) -> None:
    active = REPLAY_FINAL_JOINT
    if active not in pose:
        return
    current = arm.positions(DEFAULT_JOINTS)
    target = float(pose[active])
    delta_deg = math.degrees(target - current[active])
    if abs(delta_deg) <= deadband_deg:
        print(
            "v2.5.3 replay final wrist skip",
            "delta_deg=",
            delta_deg,
            "deadband_deg=",
            deadband_deg,
            flush=True,
        )
        return
    gains = active_replay_gains(active, fallback_kp, fallback_kd)
    seconds = replay_seconds(active, delta_deg)
    hold_targets = {
        joint: float(pose[joint])
        for joint in DEFAULT_JOINTS
        if joint != active and joint in pose
    }
    hold_gains = build_hold_gains(active, hold_targets.keys(), hold_targets.keys(), fallback_kp, fallback_kd)
    hold_tau = build_hold_tau(active, hold_targets.keys(), hold_targets.keys())
    api = arm_api()
    low_shoulder_pose = is_low_shoulder_pose(pose)
    if low_shoulder_pose and "wrist_side" in hold_gains:
        hold_gains["wrist_side"] = api.NUDGE_LOW_SHOULDER_GAINS.get("wrist_side", hold_gains["wrist_side"])
    if low_shoulder_pose:
        for hold_name, gains_override in api.NUDGE_LOW_SHOULDER_WRIST_HOLD_GAINS.items():
            if hold_name in hold_gains:
                hold_gains[hold_name] = gains_override
    hold_tau = tune_wrist_active_hold_tau(active, hold_targets, hold_tau)
    if low_shoulder_pose and "wrist_side" in hold_targets:
        hold_tau["wrist_side"] = REPLAY_LOW_SHOULDER_WRIST_SIDE_HOLD_TAU
    if low_shoulder_pose:
        for hold_name, tau in api.NUDGE_LOW_SHOULDER_WRIST_HOLD_TAU.items():
            if hold_name in hold_targets:
                hold_tau[hold_name] = tau
    active_tau = api.nudge_active_tau_for(active, delta_deg)
    print(
        "v2.5.3 replay final wrist active",
        "delta_deg=",
        delta_deg,
        "target=",
        target,
        "kp=",
        gains["kp"],
        "kd=",
        gains["kd"],
        "seconds=",
        seconds,
        "hold joints:",
        ",".join(hold_targets.keys()),
        flush=True,
    )
    print("v2.5.3 replay final wrist hold_tau=", json.dumps(hold_tau, ensure_ascii=False), flush=True)
    if low_shoulder_pose and "wrist_side" in hold_gains:
        print(
            "v2.5.3 replay final wrist low shoulder wrist_side hold gains=",
            json.dumps(hold_gains["wrist_side"], ensure_ascii=False),
            "hold_tau=",
            hold_tau.get("wrist_side"),
            flush=True,
        )
    if low_shoulder_pose:
        final_hold_overrides = {
            hold_name: hold_gains[hold_name]
            for hold_name in api.NUDGE_LOW_SHOULDER_WRIST_HOLD_GAINS
            if hold_name in hold_gains
        }
        if final_hold_overrides:
            print(
                "v2.5.3 replay final wrist low shoulder hold overrides=",
                json.dumps(final_hold_overrides, ensure_ascii=False),
                "hold_tau=",
                json.dumps(
                    {
                        hold_name: hold_tau[hold_name]
                        for hold_name in ("arm_roll", "shoulder_rotate", "shoulder_side")
                        if hold_name in hold_tau
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
    arm.move_target_with_holds(
        active,
        target,
        seconds_per_step=seconds,
        kp=gains["kp"],
        kd=gains["kd"],
        hold_targets=hold_targets,
        hold_gains=hold_gains,
        fallback_kp=fallback_kp,
        fallback_kd=fallback_kd,
        hold_tau=hold_tau,
        active_tau=active_tau,
        control_dt=0.01,
        preload_seconds=0.2,
        trajectory="smoothstep",
        linear_blend=0.0,
    )


def replay_move(name: str, moves_file: str, clearance_file: str, python_bin: str, fallback_kp: float, fallback_kd: float, deadband_deg: float) -> None:
    move_name = normalize_move_name(name)
    moves = load_moves(moves_file).get("moves", {})
    if move_name not in moves:
        raise RuntimeError(f"unknown move: {move_name}")
    pose = moves[move_name].get("pose", {})
    missing = [joint for joint in REPLAY_REQUIRED_JOINTS if joint not in pose]
    if missing:
        raise RuntimeError(f"move {move_name!r} is missing joints: {', '.join(missing)}")

    run_table_clearance(python_bin, clearance_file)

    api = arm_api()
    arm = api.LeftArmV2()
    try:
        print("Serial port is open", flush=True)
        arm.enable(DEFAULT_JOINTS)
        print(
            "v2.5.3 replay move name=",
            move_name,
            "order=",
            ",".join(REPLAY_ORDER),
            "final=",
            REPLAY_FINAL_JOINT,
            flush=True,
        )
        completed_targets: Dict[str, float] = {}
        low_shoulder_pose = is_low_shoulder_pose(pose)
        for active in REPLAY_ORDER:
            current = arm.positions(DEFAULT_JOINTS)
            requested_target = float(pose[active])
            target = apply_target_limits(active, requested_target)
            delta_deg = math.degrees(target - current[active])
            if abs(delta_deg) <= deadband_deg:
                print(
                    "v2.5.3 replay skip",
                    active,
                    "delta_deg=",
                    delta_deg,
                    "deadband_deg=",
                    deadband_deg,
                    flush=True,
                )
                completed_targets[active] = target
                continue
            gains = active_replay_gains(active, fallback_kp, fallback_kd)
            seconds = replay_seconds(active, delta_deg)
            if low_shoulder_pose and active in api.NUDGE_LOW_SHOULDER_GAINS:
                gains = api.NUDGE_LOW_SHOULDER_GAINS[active]
                seconds = max(seconds, api.NUDGE_LOW_SHOULDER_MIN_SECONDS.get(active, seconds))
            hold_targets = {
                joint: completed_targets.get(joint, current[joint])
                for joint in DEFAULT_JOINTS
                if joint != active
            }
            hold_gains = build_hold_gains(
                active,
                hold_targets.keys(),
                completed_targets.keys(),
                fallback_kp,
                fallback_kd,
            )
            hold_tau = build_hold_tau(active, hold_targets.keys(), completed_targets.keys())
            if low_shoulder_pose:
                hold_tau = tune_wrist_active_hold_tau(active, hold_targets, hold_tau)
            active_tau = api.nudge_active_tau_for(active, delta_deg)
            preload_seconds = api.NUDGE_PRELOAD_SECONDS.get(active, 0.2)
            trajectory = api.NUDGE_TRAJECTORY.get(active, "smoothstep")
            linear_blend = api.NUDGE_LINEAR_BLEND.get(active, 0.0)
            print(
                "v2.5.3 replay active",
                active,
                "delta_deg=",
                delta_deg,
                "target=",
                target,
                "kp=",
                gains["kp"],
                "kd=",
                gains["kd"],
                "seconds=",
                seconds,
                "hold joints:",
                ",".join(hold_targets.keys()),
                flush=True,
            )
            if active_tau:
                print("v2.5.3 replay active_tau=", {active: active_tau}, flush=True)
            if low_shoulder_pose and active in api.NUDGE_LOW_SHOULDER_GAINS:
                print(
                    "v2.5.3 replay low shoulder gains active",
                    active,
                    "shoulder_front_target=",
                    pose.get("shoulder_front"),
                    flush=True,
                )
            if completed_targets:
                print("v2.5.3 replay completed hold targets=", json.dumps(completed_targets, ensure_ascii=False), flush=True)
                print(
                    "v2.5.3 replay completed hold gains=",
                    json.dumps(
                        {joint: hold_gains[joint] for joint in completed_targets if joint in hold_gains},
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                print(
                    "v2.5.3 replay hold_tau=",
                    json.dumps(hold_tau, ensure_ascii=False),
                    flush=True,
                )
            compliant_hold_names = (
                api.CLEARANCE_COMPLIANT_HOLDS_BY_ACTIVE.get(active, set())
                | api.NUDGE_COMPLIANT_HOLDS_BY_ACTIVE.get(active, set())
            )
            compliant_holds = {
                joint: hold_gains[joint]
                for joint in compliant_hold_names
                if joint in hold_gains
            }
            if compliant_holds:
                print("v2.5.3 replay compliant hold gains=", json.dumps(compliant_holds, ensure_ascii=False), flush=True)
            arm.move_target_with_holds(
                active,
                target,
                seconds_per_step=seconds,
                kp=gains["kp"],
                kd=gains["kd"],
                hold_targets=hold_targets,
                hold_gains=hold_gains,
                fallback_kp=fallback_kp,
                fallback_kd=fallback_kd,
                hold_tau=hold_tau,
                active_tau=active_tau,
                control_dt=0.01,
                preload_seconds=preload_seconds,
                trajectory=trajectory,
                linear_blend=linear_blend,
            )
            if low_shoulder_pose and active == "wrist_side":
                wrist_side_pos = arm.positions([active])[active]
                wrist_side_delta_deg = math.degrees(target - wrist_side_pos)
                if abs(wrist_side_delta_deg) > api.NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_DEADBAND_DEG:
                    fine = api.NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_GAINS
                    fine_bias_deg = max(
                        -api.NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_MAX_BIAS_DEG,
                        min(api.NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_MAX_BIAS_DEG, wrist_side_delta_deg),
                    )
                    fine_target = target + math.radians(fine_bias_deg)
                    print(
                        "v2.5.3 replay low shoulder wrist_side fine",
                        "delta_deg=",
                        wrist_side_delta_deg,
                        "bias_deg=",
                        fine_bias_deg,
                        "target=",
                        fine_target,
                        "kp=",
                        fine["kp"],
                        "kd=",
                        fine["kd"],
                        "seconds=",
                        fine["seconds"],
                        flush=True,
                    )
                    arm.move_target_with_holds(
                        active,
                        fine_target,
                        seconds_per_step=fine["seconds"],
                        kp=fine["kp"],
                        kd=fine["kd"],
                        hold_targets=hold_targets,
                        hold_gains=hold_gains,
                        fallback_kp=fallback_kp,
                        fallback_kd=fallback_kd,
                        hold_tau=hold_tau,
                        active_tau=active_tau,
                        control_dt=0.01,
                        preload_seconds=0.1,
                        trajectory="smoothstep",
                        linear_blend=0.0,
                    )
                else:
                    print(
                        "v2.5.3 replay low shoulder wrist_side fine skip delta_deg=",
                        wrist_side_delta_deg,
                        "deadband_deg=",
                        api.NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_DEADBAND_DEG,
                        flush=True,
                    )
            completed_targets[active] = target
        run_replay_settle_pass(arm, pose, fallback_kp, fallback_kd)
        run_final_wrist_move(arm, pose, fallback_kp, fallback_kd, deadband_deg)
        arm.print_status(DEFAULT_JOINTS)
    finally:
        arm.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Saved move library for left arm web controller 1.2")
    sub = parser.add_subparsers(dest="cmd", required=True)

    low = sub.add_parser("low-torque")
    low.add_argument("--kp", type=float, default=0.15)
    low.add_argument("--kd", type=float, default=0.35)
    low.add_argument("--control-dt", type=float, default=0.02)
    low.add_argument("--trace-interval", type=float, default=1.0)

    cap = sub.add_parser("capture-move")
    cap.add_argument("--name", required=True)
    cap.add_argument("--note", default="")
    cap.add_argument("--moves-file", default=MOVES_PATH)

    show = sub.add_parser("list-moves")
    show.add_argument("--moves-file", default=MOVES_PATH)

    replay = sub.add_parser("replay-move")
    replay.add_argument("--name", required=True)
    replay.add_argument("--moves-file", default=MOVES_PATH)
    replay.add_argument("--clearance-file", default=TABLE_CLEARANCE_PATH)
    replay.add_argument("--python-bin", default=sys.executable or "python3")
    replay.add_argument("--kp", type=float, default=3.0)
    replay.add_argument("--kd", type=float, default=0.3)
    replay.add_argument("--deadband-deg", type=float, default=0.5)

    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.cmd == "low-torque":
        low_torque(args.kp, args.kd, args.control_dt, args.trace_interval)
    elif args.cmd == "capture-move":
        capture_move(args.name, args.note, args.moves_file)
    elif args.cmd == "list-moves":
        print(json.dumps(list_moves(args.moves_file), ensure_ascii=False, indent=2), flush=True)
    elif args.cmd == "replay-move":
        replay_move(args.name, args.moves_file, args.clearance_file, args.python_bin, args.kp, args.kd, args.deadband_deg)
    else:
        raise RuntimeError(f"unknown command: {args.cmd}")


if __name__ == "__main__":
    main()
