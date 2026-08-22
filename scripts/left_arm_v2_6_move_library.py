#!/usr/bin/env python3
"""Saved move helpers for left_arm_v2_6 web controller v2.6.

This file is intentionally separate from left_arm_v2_6.py so the existing
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

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MOVES_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_v2_6_saved_moves.json")
LEGACY_MOVES_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_v2_5_3_saved_moves.json")
ARM_SCRIPT = os.path.join(SCRIPT_DIR, "left_arm_v2_6.py")
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
REPLAY_STYLE_DEFAULT = "adaptive"
REPLAY_ADAPTIVE_GROUPS = [
    ["shoulder_front", "shoulder_side", "elbow"],
    ["shoulder_rotate", "arm_roll", "wrist_side"],
]
REPLAY_ADAPTIVE_SOFT_GROUP_DELTA_DEG = 18.0
REPLAY_ADAPTIVE_FORCE_SPLIT_DEG = 28.0

REPLAY_COMPLETED_HOLD_GAINS = {
    "shoulder_front": {"kp": 75.0, "kd": 7.0},
    "shoulder_side": {"kp": 65.0, "kd": 4.5},
    "shoulder_rotate": {"kp": 8.0, "kd": 4.0},
    "elbow": {"kp": 78.0, "kd": 7.2},
    "arm_roll": {"kp": 6.0, "kd": 6.0},
    "wrist_side": {"kp": 8.0, "kd": 1.6},
    "wrist": {"kp": 8.0, "kd": 1.4},
}

REPLAY_FINAL_WRIST_SOFT_HOLD_GAINS = {
    "elbow": {"kp": 25.0, "kd": 3.0},
}

REPLAY_PROTECTED_COMPLETED_HOLDS = {
    "shoulder_rotate",
}

REPLAY_COMPLETED_HOLD_TAU = {
    "shoulder_front": 2.4,
    "elbow": 5.1,
    "wrist": 0.55,
}

REPLAY_FINAL_WRIST_SOFT_HOLD_TAU = {
    "elbow": 3.2,
}

REPLAY_ADAPTIVE_GROUP_SOFT_HOLD_GAINS_BY_ACTIVE = {
    "wrist_side": {
        "elbow": {"kp": 25.0, "kd": 3.0},
    },
}

REPLAY_ADAPTIVE_GROUP_SOFT_HOLD_TAU_BY_ACTIVE = {
    "wrist_side": {
        "elbow": 3.2,
    },
}

REPLAY_FINAL_WRIST_FREEZE_HOLD_JOINTS = {
    "elbow",
}
REPLAY_FINAL_WRIST_FREEZE_HOLD_DEADBAND_DEG = 1.2
REPLAY_FINAL_WRIST_FROZEN_HOLD_GAINS = {
    "elbow": {"kp": 25.0, "kd": 3.0},
}
REPLAY_FINAL_WRIST_FROZEN_HOLD_TAU = {
    "elbow": 3.2,
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
REPLAY_ENTRY_TAKEOVER_SECONDS = 0.5
REPLAY_ENTRY_TAKEOVER_CONTROL_DT = 0.01
REPLAY_ENTRY_HOLD_TAU = {
    "shoulder_front": 3.0,
}
REPLAY_SHOULDER_FRONT_DESCENT_TAU = 3.0
REPLAY_SHOULDER_FRONT_DESCENT_END_TAU = 1.0
REPLAY_SHOULDER_FRONT_DESCENT_TAU_RAMP_FRACTION = 0.25
REPLAY_SHOULDER_FRONT_GROUP_PRELOAD_SECONDS = 0.8

REPLAY_FINAL_WRIST_ACTIVE_GAINS = {
    "kp": 22.0,
    "kd": 1.4,
}
REPLAY_FINAL_WRIST_SECONDS = 12.0

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
REPLAY_CORRECTION_ENABLED = False
REPLAY_CORRECTION_CANDIDATES = [
    "shoulder_side",
    "elbow",
]
REPLAY_CORRECTION_TRIGGER_DEADBAND_DEG = 1.5
REPLAY_CORRECTION_MIN_SAMPLES = 3
REPLAY_CORRECTION_MAX_STEP_SCALE = 0.5
REPLAY_CORRECTION_BASE_SECONDS = 3.0
REPLAY_CORRECTION_SECONDS_PER_DEG = 0.8
REPLAY_CORRECTION_MAX_SECONDS = 8.0

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
LOW_SHOULDER_FRONT_THRESHOLD = 1.8
def arm_api():
    import left_arm_v2_6

    return left_arm_v2_6


def now_string() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def load_moves(path: str = MOVES_PATH) -> dict:
    if not os.path.exists(path):
        return {"type": "left_arm_v2_6_saved_moves", "moves": {}}
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


def migrate_moves(source_path: str, dest_path: str, overwrite: bool = False) -> dict:
    if not os.path.exists(source_path):
        raise RuntimeError(f"source moves file does not exist: {source_path}")
    source = load_moves(source_path)
    dest = load_moves(dest_path)
    source_moves = source.get("moves", {})
    dest_moves = dest.setdefault("moves", {})
    migrated = []
    skipped = []
    for name in sorted(source_moves):
        if name in dest_moves and not overwrite:
            skipped.append(name)
            continue
        record = dict(source_moves[name])
        record["name"] = name
        record["type"] = "left_arm_v2_6_saved_move"
        record["migrated_from"] = {
            "path": os.path.abspath(source_path),
            "type": source_moves[name].get("type", source.get("type")),
            "migrated_at": now_string(),
        }
        record.setdefault("replay_requires_table_clearance", True)
        record.setdefault("replay_order", REPLAY_ORDER)
        record.setdefault("replay_final_joint", REPLAY_FINAL_JOINT)
        dest_moves[name] = record
        migrated.append(name)
    dest["type"] = "left_arm_v2_6_saved_moves"
    save_moves(dest, dest_path)
    result = {
        "source": os.path.abspath(source_path),
        "dest": os.path.abspath(dest_path),
        "migrated": migrated,
        "skipped_existing": skipped,
        "overwrite": overwrite,
        "moves": sorted(dest_moves.keys()),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return result


def normalize_move_name(name: str) -> str:
    normalized = " ".join(name.strip().split())
    if not normalized:
        raise ValueError("move name is required")
    if len(normalized) > 80:
        raise ValueError("move name must be 80 characters or fewer")
    return normalized


def resolve_replay_order(record: dict) -> tuple[list[str], str]:
    replay_order = record.get("replay_order", REPLAY_ORDER)
    final_joint = str(record.get("replay_final_joint", REPLAY_FINAL_JOINT))
    if not isinstance(replay_order, list) or not all(isinstance(joint, str) for joint in replay_order):
        raise RuntimeError("move replay_order must be a list of joint names")
    replay_order = list(replay_order)
    unknown = [joint for joint in replay_order + [final_joint] if joint not in DEFAULT_JOINTS]
    if unknown:
        raise RuntimeError("move replay order contains unknown joints: " + ", ".join(unknown))
    if final_joint != REPLAY_FINAL_JOINT:
        raise RuntimeError(f"only {REPLAY_FINAL_JOINT!r} is currently supported as replay_final_joint")
    if final_joint in replay_order:
        raise RuntimeError(f"move replay_final_joint {final_joint!r} must not also be in replay_order")
    missing = [joint for joint in DEFAULT_JOINTS if joint not in set(replay_order + [final_joint])]
    if missing:
        raise RuntimeError("move replay order is missing joints: " + ", ".join(missing))
    return replay_order, final_joint


def resolve_replay_style(sequential: bool = False, coupled: bool = False) -> str:
    if sequential and coupled:
        raise RuntimeError("choose only one replay style: --sequential or --coupled")
    if sequential:
        return "sequential"
    if coupled:
        return "coupled"
    return REPLAY_STYLE_DEFAULT


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
        "type": "left_arm_v2_6_saved_move",
        "created_at": now_string(),
        "note": note,
        "pose": pose,
        "replay_requires_table_clearance": True,
        "replay_order": REPLAY_ORDER,
        "replay_final_joint": REPLAY_FINAL_JOINT,
    }
    save_moves(data, path)
    print("v2.6 capture move " + ("overwritten" if replaced else "saved") + ":", move_name, flush=True)
    print(json.dumps(data["moves"][move_name], ensure_ascii=False, indent=2), flush=True)
    return data["moves"][move_name]


def list_moves(path: str) -> dict:
    data = load_moves(path)
    names = sorted(data.get("moves", {}).keys())
    return {"moves": [{"name": name, **data["moves"][name]} for name in names]}


def describe_replay(name: str, moves_file: str, replay_style: str = REPLAY_STYLE_DEFAULT) -> dict:
    move_name = normalize_move_name(name)
    moves = load_moves(moves_file).get("moves", {})
    if move_name not in moves:
        raise RuntimeError(f"unknown move: {move_name}")
    record = moves[move_name]
    pose = record.get("pose", {})
    missing = [joint for joint in REPLAY_REQUIRED_JOINTS if joint not in pose]
    if missing:
        raise RuntimeError(f"move {move_name!r} is missing joints: {', '.join(missing)}")
    replay_order, final_joint = resolve_replay_order(record)
    completed: list[str] = []
    phases = [
        {
            "phase": "table_clearance",
            "command": [
                sys.executable or "python3",
                "-u",
                ARM_SCRIPT,
                "clearance",
                "--clearance-file",
                TABLE_CLEARANCE_PATH,
                "--deadband-deg",
                "0.5",
                "--max-delta-deg",
                "120",
                "--step-deg",
                "5",
                "--execute",
            ],
        }
    ]
    if replay_style == "adaptive":
        for index, group in enumerate(adaptive_replay_groups(replay_order, {}, deadband_deg=0.0, planning_only=True), start=1):
            phases.append(
                {
                    "phase": "adaptive_non_wrist_replay",
                    "group": index,
                    "active_joints": group,
                    "hold_joints": [final_joint] + [joint for prior in phases if prior.get("phase") == "adaptive_non_wrist_replay" for joint in prior.get("active_joints", [])],
                    "hold_target_source": "current_position_for_final_wrist_then_completed_targets",
                }
            )
    elif replay_style == "coupled":
        phases.append(
            {
                "phase": "coupled_non_wrist_replay",
                "active_joints": replay_order,
                "hold_joints": [final_joint],
                "hold_target_source": "current_position",
            }
        )
    elif replay_style == "sequential":
        for active in replay_order:
            hold_joints = [joint for joint in DEFAULT_JOINTS if joint != active]
            phases.append(
                {
                    "phase": "joint_replay",
                    "active": active,
                    "hold_joints": hold_joints,
                    "completed_hold_targets": list(completed),
                    "pending_hold_targets": [joint for joint in hold_joints if joint not in completed],
                }
            )
            completed.append(active)
    else:
        raise RuntimeError(f"unknown replay style: {replay_style}")
    phases.append(
        {
            "phase": "final_wrist",
            "active": final_joint,
            "hold_joints": [joint for joint in DEFAULT_JOINTS if joint != final_joint],
            "completed_hold_targets": [joint for joint in DEFAULT_JOINTS if joint != final_joint],
        }
    )
    return {
        "name": move_name,
        "moves_file": os.path.abspath(moves_file),
        "requires_table_clearance": bool(record.get("replay_requires_table_clearance", True)),
        "replay_style": replay_style,
        "replay_order": replay_order,
        "replay_final_joint": final_joint,
        "low_shoulder_pose": is_low_shoulder_pose(pose),
        "pose": pose,
        "phases": phases,
    }


def low_torque(kp: float, kd: float, control_dt: float, trace_interval: float) -> None:
    api = arm_api()
    arm = api.LeftArmV2()
    try:
        print("Serial port is open", flush=True)
        arm.enable(DEFAULT_JOINTS)
        print(
            "v2.6 low torque mode active",
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
                print("v2.6 low torque status=", json.dumps(current, ensure_ascii=False), flush=True)
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
    print("v2.6 replay pre-step table clearance command=", " ".join(cmd), flush=True)
    completed = subprocess.run(cmd, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"table clearance failed with returncode {completed.returncode}")


def replay_entry_takeover_hold(arm, low_shoulder_pose: bool) -> Dict[str, float]:
    api = arm_api()
    current = arm.positions(DEFAULT_JOINTS)
    hold_gains = {
        name: api.CLEARANCE_HOLD_GAINS.get(
            name,
            api.CLEARANCE_BASE_HOLD_GAINS.get(name, {"kp": 3.0, "kd": 0.3}),
        )
        for name in DEFAULT_JOINTS
    }
    hold_gains = api.adaptive_hold_gains_for("replay_entry", hold_gains, low_shoulder_pose)
    hold_tau = dict(REPLAY_ENTRY_HOLD_TAU)
    print(
        "v2.6 replay entry takeover hold seconds=",
        REPLAY_ENTRY_TAKEOVER_SECONDS,
        "hold_tau=",
        json.dumps(hold_tau, ensure_ascii=False),
        flush=True,
    )
    start = time.time()
    while time.time() - start < REPLAY_ENTRY_TAKEOVER_SECONDS:
        for name, target in current.items():
            gains = hold_gains.get(name, {})
            arm.ctrl.controlMIT(
                arm.motors[name],
                gains.get("kp", 3.0),
                gains.get("kd", 0.3),
                target,
                0,
                hold_tau.get(name, 0.0),
            )
        time.sleep(REPLAY_ENTRY_TAKEOVER_CONTROL_DT)
    return current


def active_replay_gains(name: str, fallback_kp: float, fallback_kd: float) -> Dict[str, float]:
    api = arm_api()
    if name in REPLAY_ACTIVE_GAINS:
        return REPLAY_ACTIVE_GAINS[name]
    if name == "wrist":
        return dict(REPLAY_FINAL_WRIST_ACTIVE_GAINS)
    if name == "wrist_side":
        return api.PREHOME_CLEARANCE_GAINS.get("wrist_side", api.COUPLED_CLEARANCE_MOVE_GAINS["wrist_side"])
    return api.NUDGE_GAINS.get(name, api.CLEARANCE_MOVE_GAINS.get(name, {"kp": fallback_kp, "kd": fallback_kd}))


def replay_active_tau(name: str, delta_deg: float, low_shoulder_pose: bool) -> float:
    active_tau = arm_api().nudge_active_tau_for(name, delta_deg, low_shoulder_pose)
    if name == "shoulder_front" and delta_deg < 0.0:
        return max(active_tau, REPLAY_SHOULDER_FRONT_DESCENT_TAU)
    return active_tau


def replay_move_tau_ff(name: str, delta_deg: float, low_shoulder_pose: bool) -> object:
    active_tau = arm_api().nudge_active_tau_for(name, delta_deg, low_shoulder_pose)
    if name == "shoulder_front" and delta_deg < 0.0:
        return {
            "start_tau": max(active_tau, REPLAY_SHOULDER_FRONT_DESCENT_TAU),
            "end_tau": max(active_tau, REPLAY_SHOULDER_FRONT_DESCENT_END_TAU),
            "ramp_fraction": REPLAY_SHOULDER_FRONT_DESCENT_TAU_RAMP_FRACTION,
            "ramp": "smoothstep",
        }
    return active_tau


def replay_active_tau_learning_allowed(name: str, delta_deg: float, applied_tau: float) -> bool:
    comparable_tau = (
        float(applied_tau.get("start_tau", applied_tau.get("tau", 0.0)))
        if isinstance(applied_tau, dict)
        else float(applied_tau)
    )
    return not (
        name == "shoulder_front"
        and delta_deg < 0.0
        and comparable_tau >= REPLAY_SHOULDER_FRONT_DESCENT_TAU
    )


def replay_seconds(name: str, delta_deg: float) -> float:
    api = arm_api()
    if name == "wrist_side":
        return api.prehome_clearance_seconds_for(name, delta_deg)
    if name == "wrist":
        base = REPLAY_FINAL_WRIST_SECONDS
        max_seconds = REPLAY_FINAL_WRIST_SECONDS
    else:
        base = max(REPLAY_MIN_SECONDS.get(name, 2.2), api.NUDGE_MIN_SECONDS.get(name, 2.2), 2.2)
        max_seconds = 20.0
    return min(max_seconds, max(base, base * abs(delta_deg) / 5.0))


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
            "v2.6 replay target clamped",
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
    shoulder_front = pose.get("shoulder_front")
    if shoulder_front is None:
        return False
    try:
        threshold = arm_api().NUDGE_WRIST_ACTIVE_LOW_SHOULDER_FRONT_THRESHOLD
    except Exception:
        threshold = LOW_SHOULDER_FRONT_THRESHOLD
    return shoulder_front < threshold


def build_hold_gains(
    active: str,
    hold_names: Iterable[str],
    completed_names: Iterable[str],
    fallback_kp: float,
    fallback_kd: float,
    low_shoulder_pose: bool = False,
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
    return api.adaptive_hold_gains_for(active, hold_gains, low_shoulder_pose)


def print_replay_error_report(label: str, current: Dict[str, float], pose: Dict[str, float]) -> Dict[str, float]:
    errors_deg = {
        joint: math.degrees(float(pose[joint]) - current[joint])
        for joint in DEFAULT_JOINTS
        if joint in pose and joint in current
    }
    print(
        "v2.6 replay",
        label,
        "errors_deg=",
        json.dumps(errors_deg, ensure_ascii=False),
        flush=True,
    )
    return errors_deg


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
    low_rules = api.HOLD_TAU_RULES[api.LOW_SHOULDER_POSTURE]
    for hold_name, tau in low_rules.get("*", {}).items():
        if hold_name in hold_targets:
            tuned[hold_name] = api.clamp_tau(hold_name, tau, api.HOLD_TAU_LIMITS)
    for hold_name, tau in low_rules.get(active, {}).items():
        if hold_name in hold_targets:
            tuned[hold_name] = api.clamp_tau(hold_name, tau, api.HOLD_TAU_LIMITS)
    return tuned


def apply_soft_hold_overrides(
    active: str,
    hold_gains: Dict[str, Dict[str, float]],
    hold_tau: Dict[str, float],
    gain_rules: Dict[str, Dict[str, Dict[str, float]]],
    tau_rules: Dict[str, Dict[str, float]],
    low_shoulder_pose: bool = False,
) -> tuple[Dict[str, Dict[str, float]], Dict[str, float], Dict[str, Dict[str, float]], Dict[str, Dict[str, float]]]:
    updated_gains = dict(hold_gains)
    updated_tau = dict(hold_tau)
    applied_gains: Dict[str, Dict[str, float]] = {}
    applied_tau: Dict[str, Dict[str, float]] = {}
    for hold_name, gain_override in gain_rules.get(active, {}).items():
        if hold_name in updated_gains:
            applied_gains[hold_name] = gain_override
            updated_gains[hold_name] = gain_override
    for hold_name, tau_cap in tau_rules.get(active, {}).items():
        if hold_name in updated_tau:
            if adaptive_hold_tau_record_exists(active, hold_name, low_shoulder_pose):
                applied_tau[hold_name] = {
                    "adaptive_tau": updated_tau[hold_name],
                    "soft_cap_skipped": tau_cap,
                }
                continue
            capped = min(updated_tau[hold_name], tau_cap)
            if capped != updated_tau[hold_name]:
                applied_tau[hold_name] = {"previous_tau": updated_tau[hold_name], "tau": capped}
                updated_tau[hold_name] = capped
    return updated_gains, updated_tau, applied_gains, applied_tau


def adaptive_hold_tau_record_exists(active: str, hold_name: str, low_shoulder_pose: bool) -> bool:
    api = arm_api()
    posture = api.adaptive_posture_key(low_shoulder_pose)
    if posture is None:
        return False
    data = api.load_adaptive_hold_tau()
    record = data.get("rules", {}).get(posture, {}).get(active, {}).get(hold_name)
    return isinstance(record, dict) and bool(record)


def adaptive_hold_tau_ready_for_target_bias(active: str, hold_name: str, low_shoulder_pose: bool) -> bool:
    api = arm_api()
    posture = api.adaptive_posture_key(low_shoulder_pose)
    if posture is None:
        return False
    data = api.load_adaptive_hold_tau()
    record = data.get("rules", {}).get(posture, {}).get(active, {}).get(hold_name)
    if not isinstance(record, dict) or not record:
        return False
    samples = int(record.get("samples", 0))
    state = str(record.get("learning_state", ""))
    return samples >= api.ADAPTIVE_TARGET_BIAS_MIN_HOLD_TAU_SAMPLES and state in ("plateau", "backoff")


def hold_tau_update_waits_for_more_holding(update: object) -> bool:
    if not isinstance(update, dict) or not update:
        return False
    state = str(update.get("learning_state", ""))
    return bool(update.get("refined_best")) or state in ("improved", "recent_improved")


def record_hold_tau_adaptation(
    arm,
    active: str,
    pose: Dict[str, float],
    hold_targets: Dict[str, float],
    hold_tau: Dict[str, float],
    low_shoulder_pose: bool,
    label: str,
    learn_names: Iterable[str] | None = None,
) -> Dict[str, Dict[str, object]]:
    if not low_shoulder_pose or not hold_tau:
        return {}
    allowed = set(learn_names) if learn_names is not None else set(hold_targets)
    measured_names = [name for name in hold_targets if name in pose and name in hold_tau and name in allowed]
    if not measured_names:
        return {}
    measured = arm.positions(measured_names)
    hold_errors_deg = {
        name: math.degrees(float(pose[name]) - measured[name])
        for name in measured
        if name in hold_tau
    }
    updates = arm_api().update_adaptive_hold_tau(
        active,
        hold_errors_deg,
        hold_tau,
        low_shoulder_front=low_shoulder_pose,
        label=label,
    )
    print(
        "v2.6 replay adaptive hold tau observation",
        label,
        "active=",
        active,
        "errors_deg=",
        json.dumps(hold_errors_deg, ensure_ascii=False),
        flush=True,
    )
    if updates:
        print(
            "v2.6 replay adaptive hold tau updates",
            label,
            "active=",
            active,
            json.dumps(updates, ensure_ascii=False),
            flush=True,
        )
    return updates


def record_hold_target_bias_adaptation(
    arm,
    active: str,
    pose: Dict[str, float],
    hold_targets: Dict[str, float],
    low_shoulder_pose: bool,
    label: str,
    learn_names: Iterable[str] | None = None,
) -> None:
    if not low_shoulder_pose:
        return
    allowed = set(learn_names) if learn_names is not None else set(hold_targets)
    measured_names = [name for name in hold_targets if name in pose and name in allowed]
    if not measured_names:
        return
    measured = arm.positions(measured_names)
    hold_errors_deg = {
        name: math.degrees(float(pose[name]) - measured[name])
        for name in measured
    }
    updates = arm_api().update_adaptive_hold_target_bias(
        active,
        hold_errors_deg,
        low_shoulder_front=low_shoulder_pose,
        label=label,
    )
    print(
        "v2.6 replay adaptive hold target bias observation",
        label,
        "active=",
        active,
        "errors_deg=",
        json.dumps(hold_errors_deg, ensure_ascii=False),
        flush=True,
    )
    if updates:
        print(
            "v2.6 replay adaptive hold target bias updates",
            label,
            "active=",
            active,
            json.dumps(updates, ensure_ascii=False),
            flush=True,
        )


def replay_settle_seconds(delta_deg: float) -> float:
    return min(
        REPLAY_SETTLE_MAX_SECONDS,
        max(REPLAY_SETTLE_BASE_SECONDS, abs(delta_deg) * REPLAY_SETTLE_SECONDS_PER_DEG),
    )


def replay_correction_seconds(delta_deg: float) -> float:
    return min(
        REPLAY_CORRECTION_MAX_SECONDS,
        max(REPLAY_CORRECTION_BASE_SECONDS, abs(delta_deg) * REPLAY_CORRECTION_SECONDS_PER_DEG),
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
        is_low_shoulder_pose(pose),
    )
    hold_tau = build_hold_tau("elbow", hold_targets.keys(), hold_targets.keys())
    print(
        "v2.6 replay settle coupled shoulder/elbow active",
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
    label: str = "auto_trim",
) -> Dict[str, float]:
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
        "v2.6 replay settle errors_deg=",
        label,
        json.dumps(errors, ensure_ascii=False),
        "deadband_deg=",
        REPLAY_SETTLE_DEADBAND_DEG,
        flush=True,
    )
    if not REPLAY_SETTLE_ENABLED:
        print("v2.6 replay settle disabled", label, "relying on completed joint holds", flush=True)
        return errors
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
            "v2.6 replay settle errors_after_coupled_deg=",
            label,
            json.dumps(errors, ensure_ascii=False),
            flush=True,
        )

    selected = [joint for joint in REPLAY_SETTLE_ORDER if abs(errors.get(joint, 0.0)) > REPLAY_SETTLE_DEADBAND_DEG]
    if not selected:
        if not ran_coupled:
            print("v2.6 replay settle skip all joints inside deadband", label, flush=True)
        return errors

    for active in selected:
        current = arm.positions(DEFAULT_JOINTS)
        target = float(pose[active])
        delta_deg = math.degrees(target - current[active])
        if abs(delta_deg) <= REPLAY_SETTLE_DEADBAND_DEG:
            print(
                "v2.6 replay settle skip",
                label,
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
        hold_gains = build_hold_gains(
            active,
            hold_targets.keys(),
            hold_targets.keys(),
            fallback_kp,
            fallback_kd,
            is_low_shoulder_pose(pose),
        )
        hold_tau = build_hold_tau(active, hold_targets.keys(), hold_targets.keys())
        active_tau = replay_active_tau(active, delta_deg, is_low_shoulder_pose(pose))
        if active == "shoulder_front":
            active_tau = min(active_tau, REPLAY_COMPLETED_HOLD_TAU.get("shoulder_front", active_tau))
        print(
            "v2.6 replay settle active",
            label,
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
    current = arm.positions(DEFAULT_JOINTS)
    return {
        joint: math.degrees(float(pose[joint]) - current[joint])
        for joint in error_names
        if joint in pose
    }


def adaptive_target_bias_record(joint: str, low_shoulder_pose: bool) -> dict:
    api = arm_api()
    posture = api.adaptive_posture_key(low_shoulder_pose)
    if posture is None:
        return {}
    data = api.load_adaptive_target_bias()
    record = data.get("rules", {}).get(posture, {}).get(joint, {})
    return record if isinstance(record, dict) else {}


def replay_correction_decisions(errors: Dict[str, float], low_shoulder_pose: bool) -> tuple[list[str], dict]:
    decisions = {}
    selected = []
    for joint in REPLAY_CORRECTION_CANDIDATES:
        error_deg = float(errors.get(joint, 0.0))
        record = adaptive_target_bias_record(joint, low_shoulder_pose)
        samples = int(record.get("samples", 0)) if record else 0
        state = str(record.get("learning_state", "")) if record else ""
        step_scale = float(record.get("step_scale", 1.0)) if record else 1.0
        best_error = float(record.get("best_error_deg", 0.0)) if record else 0.0
        reason = "below_error_deadband"
        should_correct = False
        if abs(error_deg) >= REPLAY_CORRECTION_TRIGGER_DEADBAND_DEG:
            if samples < REPLAY_CORRECTION_MIN_SAMPLES:
                reason = "learning_samples_pending"
            elif state not in ("plateau", "backoff"):
                reason = "learning_still_improving"
            elif step_scale > REPLAY_CORRECTION_MAX_STEP_SCALE:
                reason = "learning_step_still_large"
            else:
                reason = "plateaued_residual"
                should_correct = True
        decisions[joint] = {
            "error_deg": error_deg,
            "samples": samples,
            "learning_state": state,
            "step_scale": step_scale,
            "best_error_deg": best_error,
            "decision": reason,
        }
        if should_correct:
            selected.append(joint)
    return selected, decisions


def run_replay_correction_pass(
    arm,
    pose: Dict[str, float],
    before_final_wrist_errors: Dict[str, float],
    fallback_kp: float,
    fallback_kd: float,
    deadband_deg: float,
    low_shoulder_pose: bool,
) -> Dict[str, float]:
    selected, decisions = replay_correction_decisions(before_final_wrist_errors, low_shoulder_pose)
    print(
        "v2.6 replay correction decisions=",
        json.dumps(decisions, ensure_ascii=False),
        "enabled=",
        REPLAY_CORRECTION_ENABLED,
        flush=True,
    )
    if not REPLAY_CORRECTION_ENABLED or not selected:
        print("v2.6 replay correction skipped selected=", ",".join(selected), flush=True)
        return before_final_wrist_errors

    api = arm_api()
    current = arm.positions(DEFAULT_JOINTS)
    targets: Dict[str, float] = {}
    move_gains: Dict[str, Dict[str, float]] = {}
    move_tau_ff: Dict[str, float] = {}
    delta_report: Dict[str, float] = {}
    seconds = 0.0
    for joint in selected:
        requested_target = float(pose[joint])
        target_bias = api.adaptive_target_bias_for(joint, low_shoulder_pose)
        target = apply_target_limits(joint, requested_target + target_bias)
        delta_deg = math.degrees(target - current[joint])
        delta_report[joint] = delta_deg
        if abs(delta_deg) <= deadband_deg:
            print(
                "v2.6 replay correction skip within deadband",
                joint,
                "delta_deg=",
                delta_deg,
                "deadband_deg=",
                deadband_deg,
                flush=True,
            )
            continue
        gains = active_replay_gains(joint, fallback_kp, fallback_kd)
        if low_shoulder_pose and joint in api.NUDGE_LOW_SHOULDER_GAINS:
            gains = api.NUDGE_LOW_SHOULDER_GAINS[joint]
        targets[joint] = target
        move_gains[joint] = gains
        active_tau = replay_active_tau(joint, delta_deg, low_shoulder_pose)
        if active_tau:
            move_tau_ff[joint] = active_tau
        seconds = max(seconds, replay_correction_seconds(delta_deg))

    if not targets:
        print("v2.6 replay correction no active targets after deadband", flush=True)
        return before_final_wrist_errors

    hold_targets = {
        joint: current[joint]
        for joint in DEFAULT_JOINTS
        if joint not in targets
    }
    hold_active = "elbow" if "elbow" in targets else next(iter(targets))
    hold_gains = build_hold_gains(
        hold_active,
        hold_targets.keys(),
        hold_targets.keys(),
        fallback_kp,
        fallback_kd,
        low_shoulder_pose,
    )
    hold_tau = build_hold_tau(hold_active, hold_targets.keys(), hold_targets.keys())
    if low_shoulder_pose:
        hold_tau = tune_wrist_active_hold_tau("wrist_side", hold_targets, hold_tau)
    print(
        "v2.6 replay correction active",
        ",".join(targets.keys()),
        "deltas_deg=",
        json.dumps(delta_report, ensure_ascii=False),
        "targets=",
        json.dumps(targets, ensure_ascii=False),
        "seconds=",
        seconds,
        "hold joints:",
        ",".join(hold_targets.keys()),
        flush=True,
    )
    if move_tau_ff:
        print("v2.6 replay correction active_tau=", json.dumps(move_tau_ff, ensure_ascii=False), flush=True)
    print("v2.6 replay correction hold_tau=", json.dumps(hold_tau, ensure_ascii=False), flush=True)
    arm.move_targets_with_holds(
        targets,
        seconds_per_step=seconds,
        move_gains=move_gains,
        hold_targets=hold_targets,
        hold_gains=hold_gains,
        fallback_kp=fallback_kp,
        fallback_kd=fallback_kd,
        hold_tau=hold_tau,
        move_tau_ff=move_tau_ff,
        control_dt=0.01,
        preload_seconds=0.2,
        trajectory="smoothstep",
        linear_blend=0.0,
    )
    return print_replay_error_report("after_correction", arm.positions(DEFAULT_JOINTS), pose)


def run_final_wrist_move(
    arm,
    pose: Dict[str, float],
    fallback_kp: float,
    fallback_kd: float,
    deadband_deg: float,
    completed_targets: Dict[str, float] | None = None,
) -> None:
    active = REPLAY_FINAL_JOINT
    if active not in pose:
        return
    api = arm_api()
    low_shoulder_pose = is_low_shoulder_pose(pose)
    current = arm.positions(DEFAULT_JOINTS)
    requested_target = float(pose[active])
    target_bias = api.adaptive_target_bias_for(active, low_shoulder_pose)
    target = apply_target_limits(active, requested_target + target_bias)
    if target_bias:
        print(
            "v2.6 replay final wrist adaptive target bias",
            active,
            "bias_deg=",
            math.degrees(target_bias),
            "nominal_target=",
            requested_target,
            "biased_target=",
            requested_target + target_bias,
            flush=True,
        )
    delta_deg = math.degrees(target - current[active])
    if abs(delta_deg) <= deadband_deg:
        print(
            "v2.6 replay final wrist skip",
            "delta_deg=",
            delta_deg,
            "deadband_deg=",
            deadband_deg,
            flush=True,
        )
        return
    gains = active_replay_gains(active, fallback_kp, fallback_kd)
    if active == REPLAY_FINAL_JOINT and gains != REPLAY_FINAL_WRIST_ACTIVE_GAINS:
        print(
            "v2.6 replay final wrist active gains forced",
            "previous=",
            json.dumps(gains, ensure_ascii=False),
            "forced=",
            json.dumps(REPLAY_FINAL_WRIST_ACTIVE_GAINS, ensure_ascii=False),
            flush=True,
        )
        gains = dict(REPLAY_FINAL_WRIST_ACTIVE_GAINS)
    seconds = replay_seconds(active, delta_deg)
    completed_targets = completed_targets or {}
    hold_targets = {
        joint: float(completed_targets.get(joint, pose[joint]))
        for joint in DEFAULT_JOINTS
        if joint != active and joint in pose
    }
    carried_hold_targets = {
        joint: hold_targets[joint]
        for joint in completed_targets
        if joint in hold_targets and abs(hold_targets[joint] - float(pose[joint])) > 1e-6
    }
    frozen_hold_targets: Dict[str, Dict[str, float]] = {}
    for hold_name in REPLAY_FINAL_WRIST_FREEZE_HOLD_JOINTS:
        if hold_name not in hold_targets or hold_name not in current or hold_name not in pose:
            continue
        nominal_error_deg = math.degrees(float(pose[hold_name]) - current[hold_name])
        if abs(nominal_error_deg) <= REPLAY_FINAL_WRIST_FREEZE_HOLD_DEADBAND_DEG:
            previous_target = hold_targets[hold_name]
            hold_targets[hold_name] = current[hold_name]
            frozen_hold_targets[hold_name] = {
                "previous_target": previous_target,
                "frozen_target": current[hold_name],
                "nominal_error_deg": nominal_error_deg,
            }
    hold_gains = build_hold_gains(
        active,
        hold_targets.keys(),
        hold_targets.keys(),
        fallback_kp,
        fallback_kd,
        is_low_shoulder_pose(pose),
    )
    hold_tau = build_hold_tau(active, hold_targets.keys(), hold_targets.keys())
    hold_tau = tune_wrist_active_hold_tau(active, hold_targets, hold_tau)
    if low_shoulder_pose and "wrist_side" in hold_targets:
        hold_tau["wrist_side"] = REPLAY_LOW_SHOULDER_WRIST_SIDE_HOLD_TAU
    if low_shoulder_pose:
        for hold_name, tau in api.NUDGE_LOW_SHOULDER_WRIST_HOLD_TAU.items():
            if hold_name in hold_targets:
                hold_tau[hold_name] = tau
        hold_tau = api.adaptive_hold_tau_for(active, hold_targets, hold_tau, low_shoulder_pose)
        hold_target_ready_targets = {
            hold_name: hold_target
            for hold_name, hold_target in hold_targets.items()
            if adaptive_hold_tau_ready_for_target_bias(active, hold_name, low_shoulder_pose)
        }
        hold_target_pending = sorted(set(hold_targets) - set(hold_target_ready_targets))
        if hold_target_pending:
            print(
                "v2.6 replay final wrist hold target bias pending hold-tau improvement",
                json.dumps(hold_target_pending, ensure_ascii=False),
                flush=True,
            )
        hold_target_offsets = api.adaptive_hold_target_bias_for(active, hold_target_ready_targets, low_shoulder_pose)
        if hold_target_offsets:
            for hold_name, offset in hold_target_offsets.items():
                hold_targets[hold_name] += offset
            print(
                "v2.6 replay final wrist adaptive hold target bias=",
                json.dumps({name: math.degrees(offset) for name, offset in hold_target_offsets.items()}, ensure_ascii=False),
                "hold_targets=",
                json.dumps({name: hold_targets[name] for name in hold_target_offsets}, ensure_ascii=False),
                flush=True,
            )
    soft_hold_gains = {
        hold_name: gains
        for hold_name, gains in REPLAY_FINAL_WRIST_SOFT_HOLD_GAINS.items()
        if hold_name in hold_gains
    }
    for hold_name, hold_gain_override in soft_hold_gains.items():
        hold_gains[hold_name] = hold_gain_override
    soft_hold_tau = {}
    for hold_name, tau in REPLAY_FINAL_WRIST_SOFT_HOLD_TAU.items():
        if hold_name in hold_tau:
            if adaptive_hold_tau_record_exists(active, hold_name, low_shoulder_pose):
                soft_hold_tau[hold_name] = {
                    "adaptive_tau": hold_tau[hold_name],
                    "soft_cap_skipped": tau,
                }
                continue
            capped = min(hold_tau[hold_name], tau)
            if capped != hold_tau[hold_name]:
                soft_hold_tau[hold_name] = {"previous_tau": hold_tau[hold_name], "tau": capped}
                hold_tau[hold_name] = capped
    frozen_hold_gains = {}
    frozen_hold_tau = {}
    for hold_name in frozen_hold_targets:
        if hold_name in REPLAY_FINAL_WRIST_FROZEN_HOLD_GAINS and hold_name in hold_gains:
            frozen_hold_gains[hold_name] = REPLAY_FINAL_WRIST_FROZEN_HOLD_GAINS[hold_name]
            hold_gains[hold_name] = REPLAY_FINAL_WRIST_FROZEN_HOLD_GAINS[hold_name]
        if hold_name in REPLAY_FINAL_WRIST_FROZEN_HOLD_TAU and hold_name in hold_tau:
            capped = min(hold_tau[hold_name], REPLAY_FINAL_WRIST_FROZEN_HOLD_TAU[hold_name])
            if capped != hold_tau[hold_name]:
                frozen_hold_tau[hold_name] = {"previous_tau": hold_tau[hold_name], "tau": capped}
                hold_tau[hold_name] = capped
    if carried_hold_targets:
        print(
            "v2.6 replay final wrist carried hold targets=",
            json.dumps(carried_hold_targets, ensure_ascii=False),
            flush=True,
        )
    if frozen_hold_targets:
        print(
            "v2.6 replay final wrist frozen hold targets=",
            json.dumps(frozen_hold_targets, ensure_ascii=False),
            flush=True,
        )
    if soft_hold_gains or soft_hold_tau:
        print(
            "v2.6 replay final wrist soft hold overrides gains=",
            json.dumps(soft_hold_gains, ensure_ascii=False),
            "tau=",
            json.dumps(soft_hold_tau, ensure_ascii=False),
            flush=True,
        )
    if frozen_hold_gains or frozen_hold_tau:
        print(
            "v2.6 replay final wrist frozen hold overrides gains=",
            json.dumps(frozen_hold_gains, ensure_ascii=False),
            "tau=",
            json.dumps(frozen_hold_tau, ensure_ascii=False),
            flush=True,
        )
    active_tau = getattr(
        api,
        "WRIST_ACTIVE_REPLAY_ACTIVE_TAU",
        replay_active_tau(active, delta_deg, is_low_shoulder_pose(pose)),
    )
    print(
        "v2.6 replay final wrist active",
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
    print("v2.6 replay final wrist hold_tau=", json.dumps(hold_tau, ensure_ascii=False), flush=True)
    if low_shoulder_pose and "wrist_side" in hold_gains:
        print(
            "v2.6 replay final wrist low shoulder wrist_side hold gains=",
            json.dumps(hold_gains["wrist_side"], ensure_ascii=False),
            "hold_tau=",
            hold_tau.get("wrist_side"),
            flush=True,
        )
    if low_shoulder_pose:
        final_hold_overrides = {
            hold_name: hold_gains[hold_name]
            for hold_name in (
                set(api.POSTURE_HOLD_GAIN_RULES[api.LOW_SHOULDER_POSTURE].get("*", {}))
                | set(api.POSTURE_HOLD_GAIN_RULES[api.LOW_SHOULDER_POSTURE].get(active, {}))
            )
            if hold_name in hold_gains
        }
        if final_hold_overrides:
            print(
                "v2.6 replay final wrist low shoulder adaptive hold gains=",
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
    trajectory = getattr(api, "WRIST_ACTIVE_REPLAY_TRAJECTORY", "smoothstep")
    linear_blend = getattr(api, "WRIST_ACTIVE_REPLAY_LINEAR_BLEND", 0.0)
    active_velocity_ff = bool(getattr(api, "WRIST_ACTIVE_REPLAY_VELOCITY_FF", False))
    if trajectory != "smoothstep" or linear_blend or active_velocity_ff:
        print(
            "v2.6 replay final wrist trajectory=",
            trajectory,
            "linear_blend=",
            linear_blend,
            "active_velocity_ff=",
            active_velocity_ff,
            "active_tau=",
            active_tau,
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
        active_velocity_ff=active_velocity_ff,
        control_dt=0.01,
        preload_seconds=0.2,
        trajectory=trajectory,
        linear_blend=linear_blend,
    )
    soft_final_learn_names = {
        hold_name
        for hold_name in hold_targets
        if hold_name in REPLAY_FINAL_WRIST_SOFT_HOLD_GAINS
    }
    hold_tau_updates = record_hold_tau_adaptation(
        arm,
        active,
        pose,
        hold_targets,
        hold_tau,
        low_shoulder_pose,
        label="replay-final-wrist",
        learn_names=hold_targets.keys(),
    )
    target_bias_final_learn_names = [
        hold_name
        for hold_name in soft_final_learn_names
        if not hold_tau_update_waits_for_more_holding(hold_tau_updates.get(hold_name))
    ]
    pending_final_target_bias_names = sorted(set(soft_final_learn_names) - set(target_bias_final_learn_names))
    if pending_final_target_bias_names:
        print(
            "v2.6 replay adaptive hold target bias waiting for hold-tau learning",
            "replay-final-wrist-soft-hold",
            "active=",
            active,
            json.dumps(pending_final_target_bias_names, ensure_ascii=False),
            flush=True,
        )
    if target_bias_final_learn_names:
        record_hold_target_bias_adaptation(
            arm,
            active,
            pose,
            hold_targets,
            low_shoulder_pose,
            label="replay-final-wrist-soft-hold",
            learn_names=target_bias_final_learn_names,
        )


def biased_replay_target(active: str, pose: Dict[str, float], low_shoulder_pose: bool) -> tuple[float, float, float]:
    api = arm_api()
    requested_target = float(pose[active])
    target_bias = api.adaptive_target_bias_for(active, low_shoulder_pose)
    biased_target = requested_target + target_bias
    target = apply_target_limits(active, biased_target)
    return target, requested_target, target_bias


def adaptive_replay_groups(
    replay_order: list[str],
    deltas_deg: Dict[str, float],
    deadband_deg: float,
    planning_only: bool = False,
) -> list[list[str]]:
    active_set = set(replay_order)
    if planning_only:
        active = active_set
    else:
        active = {
            joint
            for joint in replay_order
            if abs(float(deltas_deg.get(joint, 0.0))) > deadband_deg
        }
    if not active:
        return []

    max_delta = max((abs(float(deltas_deg.get(joint, 0.0))) for joint in active), default=0.0)
    total_delta = sum(abs(float(deltas_deg.get(joint, 0.0))) for joint in active)
    if not planning_only and max_delta <= REPLAY_ADAPTIVE_SOFT_GROUP_DELTA_DEG and total_delta <= REPLAY_ADAPTIVE_FORCE_SPLIT_DEG:
        return [[joint for joint in replay_order if joint in active]]

    grouped: list[list[str]] = []
    used: set[str] = set()
    for group in REPLAY_ADAPTIVE_GROUPS:
        planned = [joint for joint in group if joint in active_set and joint in active]
        if planned:
            grouped.append(planned)
            used.update(planned)
    remaining = [joint for joint in replay_order if joint in active and joint not in used]
    if remaining:
        grouped.append(remaining)
    return grouped


def run_adaptive_replay_group(
    arm,
    pose: Dict[str, float],
    group: list[str],
    completed_targets: Dict[str, float],
    fallback_kp: float,
    fallback_kd: float,
    deadband_deg: float,
    low_shoulder_pose: bool,
    label: str,
) -> Dict[str, float]:
    api = arm_api()
    current = arm.positions(DEFAULT_JOINTS)
    targets: Dict[str, float] = {}
    move_gains: Dict[str, Dict[str, float]] = {}
    move_tau_ff: Dict[str, float] = {}
    delta_report: Dict[str, float] = {}
    seconds = 0.0
    for active in group:
        target, requested_target, target_bias = biased_replay_target(active, pose, low_shoulder_pose)
        if target_bias:
            print(
                "v2.6 replay adaptive group target bias",
                active,
                "bias_deg=",
                math.degrees(target_bias),
                "nominal_target=",
                requested_target,
                "biased_target=",
                requested_target + target_bias,
                flush=True,
            )
        delta_deg = math.degrees(target - current[active])
        delta_report[active] = delta_deg
        if abs(delta_deg) <= deadband_deg:
            print(
                "v2.6 replay adaptive group skip",
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
        joint_seconds = replay_seconds(active, delta_deg)
        if low_shoulder_pose and active in api.NUDGE_LOW_SHOULDER_GAINS:
            gains = api.NUDGE_LOW_SHOULDER_GAINS[active]
            joint_seconds = max(joint_seconds, api.NUDGE_LOW_SHOULDER_MIN_SECONDS.get(active, joint_seconds))
        targets[active] = target
        move_gains[active] = gains
        active_tau = replay_move_tau_ff(active, delta_deg, low_shoulder_pose)
        if active_tau:
            move_tau_ff[active] = active_tau
        seconds = max(seconds, joint_seconds)

    if not targets:
        print(
            "v2.6 replay adaptive group skip all within deadband",
            label,
            "deltas_deg=",
            json.dumps(delta_report, ensure_ascii=False),
            flush=True,
        )
        return completed_targets

    hold_targets: Dict[str, float] = {REPLAY_FINAL_JOINT: current[REPLAY_FINAL_JOINT]}
    hold_targets.update(
        {
            joint: target
            for joint, target in completed_targets.items()
            if joint not in targets
        }
    )
    hold_gains = build_hold_gains(
        group[-1],
        hold_targets.keys(),
        completed_targets.keys(),
        fallback_kp,
        fallback_kd,
        low_shoulder_pose,
    )
    hold_tau = build_hold_tau(group[-1], hold_targets.keys(), completed_targets.keys())
    if low_shoulder_pose:
        hold_tau = tune_wrist_active_hold_tau(group[-1], hold_targets, hold_tau)
        completed_hold_targets = {
            joint: hold_targets[joint]
            for joint in completed_targets
            if joint in hold_targets
        }
        hold_target_ready_targets = {
            hold_name: hold_target
            for hold_name, hold_target in completed_hold_targets.items()
            if adaptive_hold_tau_ready_for_target_bias(group[-1], hold_name, low_shoulder_pose)
        }
        hold_target_pending = sorted(set(completed_hold_targets) - set(hold_target_ready_targets))
        if hold_target_pending:
            print(
                "v2.6 replay adaptive group hold target bias pending hold-tau improvement",
                group[-1],
                json.dumps(hold_target_pending, ensure_ascii=False),
                flush=True,
            )
        hold_target_offsets = api.adaptive_hold_target_bias_for(group[-1], hold_target_ready_targets, low_shoulder_pose)
        if hold_target_offsets:
            for hold_name, offset in hold_target_offsets.items():
                hold_targets[hold_name] += offset
            print(
                "v2.6 replay adaptive group hold target bias=",
                json.dumps({name: math.degrees(offset) for name, offset in hold_target_offsets.items()}, ensure_ascii=False),
                "hold_targets=",
                json.dumps({name: hold_targets[name] for name in hold_target_offsets}, ensure_ascii=False),
                flush=True,
            )
        adaptive_completed_tau = api.adaptive_hold_tau_for(
            group[-1],
            completed_hold_targets,
            hold_tau,
            low_shoulder_pose,
        )
        hold_tau.update(
            {
                joint: adaptive_completed_tau[joint]
                for joint in completed_hold_targets
                if joint in adaptive_completed_tau
            }
        )
    hold_gains, hold_tau, soft_group_gains, soft_group_tau = apply_soft_hold_overrides(
        group[-1],
        hold_gains,
        hold_tau,
        REPLAY_ADAPTIVE_GROUP_SOFT_HOLD_GAINS_BY_ACTIVE,
        REPLAY_ADAPTIVE_GROUP_SOFT_HOLD_TAU_BY_ACTIVE,
        low_shoulder_pose,
    )
    print(
        "v2.6 replay adaptive group active",
        label,
        ",".join(targets.keys()),
        "deltas_deg=",
        json.dumps(delta_report, ensure_ascii=False),
        "targets=",
        json.dumps(targets, ensure_ascii=False),
        "seconds=",
        seconds,
        "hold joints:",
        ",".join(hold_targets.keys()),
        flush=True,
    )
    if move_tau_ff:
        print("v2.6 replay adaptive group active_tau=", json.dumps(move_tau_ff, ensure_ascii=False), flush=True)
    if completed_targets:
        print("v2.6 replay adaptive group completed hold targets=", json.dumps(completed_targets, ensure_ascii=False), flush=True)
        print("v2.6 replay adaptive group hold_tau=", json.dumps(hold_tau, ensure_ascii=False), flush=True)
    if soft_group_gains or soft_group_tau:
        print(
            "v2.6 replay adaptive group soft hold overrides gains=",
            json.dumps(soft_group_gains, ensure_ascii=False),
            "tau=",
            json.dumps(soft_group_tau, ensure_ascii=False),
            flush=True,
        )
    preload_seconds = 0.2
    if "shoulder_front" in targets:
        preload_seconds = max(preload_seconds, REPLAY_SHOULDER_FRONT_GROUP_PRELOAD_SECONDS)
        print(
            "v2.6 replay adaptive group shoulder_front preload_seconds=",
            preload_seconds,
            flush=True,
        )
    arm.move_targets_with_holds(
        targets,
        seconds_per_step=seconds,
        move_gains=move_gains,
        hold_targets=hold_targets,
        hold_gains=hold_gains,
        fallback_kp=fallback_kp,
        fallback_kd=fallback_kd,
        hold_tau=hold_tau,
        move_tau_ff=move_tau_ff,
        control_dt=0.01,
        preload_seconds=preload_seconds,
        trajectory="smoothstep",
        linear_blend=0.0,
    )
    after_positions = arm.positions(list(targets.keys()))
    active_tau_updates = {}
    for active, target in targets.items():
        error_deg = math.degrees(target - after_positions[active])
        applied_tau = move_tau_ff.get(active, 0.0)
        if not replay_active_tau_learning_allowed(active, delta_report.get(active, 0.0), applied_tau):
            continue
        updates = api.update_adaptive_active_tau(
            active,
            delta_report.get(active, 0.0),
            error_deg,
            applied_tau,
            low_shoulder_front=low_shoulder_pose,
            label="replay-adaptive-group",
        )
        active_tau_updates.update(updates)
    if active_tau_updates:
        print(
            "v2.6 replay adaptive group active tau updates",
            json.dumps(active_tau_updates, ensure_ascii=False),
            flush=True,
        )
    completed_targets.update(targets)
    soft_learn_names = {
        hold_name
        for hold_name in completed_targets
        if hold_name in REPLAY_ADAPTIVE_GROUP_SOFT_HOLD_GAINS_BY_ACTIVE.get(group[-1], {})
    }
    hold_tau_updates = record_hold_tau_adaptation(
        arm,
        group[-1],
        pose,
        hold_targets,
        hold_tau,
        low_shoulder_pose,
        label="replay-adaptive-group",
        learn_names=completed_targets.keys(),
    )
    target_bias_learn_names = [
        hold_name
        for hold_name in soft_learn_names
        if not hold_tau_update_waits_for_more_holding(hold_tau_updates.get(hold_name))
    ]
    pending_target_bias_names = sorted(set(soft_learn_names) - set(target_bias_learn_names))
    if pending_target_bias_names:
        print(
            "v2.6 replay adaptive hold target bias waiting for hold-tau learning",
            "replay-adaptive-group-soft-hold",
            "active=",
            group[-1],
            json.dumps(pending_target_bias_names, ensure_ascii=False),
            flush=True,
        )
    if target_bias_learn_names:
        record_hold_target_bias_adaptation(
            arm,
            group[-1],
            pose,
            hold_targets,
            low_shoulder_pose,
            label="replay-adaptive-group-soft-hold",
            learn_names=target_bias_learn_names,
        )
    return completed_targets


def run_adaptive_non_wrist_replay(
    arm,
    pose: Dict[str, float],
    replay_order: list[str],
    fallback_kp: float,
    fallback_kd: float,
    deadband_deg: float,
    low_shoulder_pose: bool,
) -> Dict[str, float]:
    current = arm.positions(DEFAULT_JOINTS)
    planned_targets = {
        joint: biased_replay_target(joint, pose, low_shoulder_pose)[0]
        for joint in replay_order
    }
    deltas_deg = {
        joint: math.degrees(planned_targets[joint] - current[joint])
        for joint in replay_order
    }
    groups = adaptive_replay_groups(replay_order, deltas_deg, deadband_deg)
    print(
        "v2.6 replay adaptive plan",
        json.dumps(
            {
                "deltas_deg": deltas_deg,
                "groups": groups,
                "soft_group_delta_deg": REPLAY_ADAPTIVE_SOFT_GROUP_DELTA_DEG,
                "force_split_deg": REPLAY_ADAPTIVE_FORCE_SPLIT_DEG,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    completed_targets: Dict[str, float] = {}
    if not groups:
        print("v2.6 replay adaptive: all non-wrist joints inside deadband", flush=True)
        return completed_targets
    for index, group in enumerate(groups, start=1):
        completed_targets = run_adaptive_replay_group(
            arm,
            pose,
            group,
            completed_targets,
            fallback_kp,
            fallback_kd,
            deadband_deg,
            low_shoulder_pose,
            label=f"group-{index}",
        )
    return completed_targets


def run_coupled_non_wrist_replay(
    arm,
    pose: Dict[str, float],
    replay_order: list[str],
    fallback_kp: float,
    fallback_kd: float,
    deadband_deg: float,
    low_shoulder_pose: bool,
) -> None:
    api = arm_api()
    current = arm.positions(DEFAULT_JOINTS)
    targets: Dict[str, float] = {}
    move_gains: Dict[str, Dict[str, float]] = {}
    move_tau_ff: Dict[str, float] = {}
    delta_report: Dict[str, float] = {}
    seconds = 0.0
    for active in replay_order:
        requested_target = float(pose[active])
        target_bias = api.adaptive_target_bias_for(active, low_shoulder_pose)
        biased_target = requested_target + target_bias
        if target_bias:
            print(
                "v2.6 replay coupled adaptive target bias",
                active,
                "bias_deg=",
                math.degrees(target_bias),
                "nominal_target=",
                requested_target,
                "biased_target=",
                biased_target,
                flush=True,
            )
        target = apply_target_limits(active, biased_target)
        delta_deg = math.degrees(target - current[active])
        delta_report[active] = delta_deg
        if abs(delta_deg) <= deadband_deg:
            print(
                "v2.6 replay coupled skip within deadband",
                active,
                "delta_deg=",
                delta_deg,
                "deadband_deg=",
                deadband_deg,
                flush=True,
            )
            continue
        gains = active_replay_gains(active, fallback_kp, fallback_kd)
        joint_seconds = replay_seconds(active, delta_deg)
        if low_shoulder_pose and active in api.NUDGE_LOW_SHOULDER_GAINS:
            gains = api.NUDGE_LOW_SHOULDER_GAINS[active]
            joint_seconds = max(joint_seconds, api.NUDGE_LOW_SHOULDER_MIN_SECONDS.get(active, joint_seconds))
        targets[active] = target
        move_gains[active] = gains
        active_tau = replay_active_tau(active, delta_deg, low_shoulder_pose)
        if active_tau:
            move_tau_ff[active] = active_tau
        seconds = max(seconds, joint_seconds)

    if not targets:
        print(
            "v2.6 replay coupled non-wrist skip all joints within deadband",
            "deadband_deg=",
            deadband_deg,
            "deltas_deg=",
            json.dumps(delta_report, ensure_ascii=False),
            flush=True,
        )
        return

    hold_targets = {REPLAY_FINAL_JOINT: current[REPLAY_FINAL_JOINT]}
    hold_gains = build_hold_gains(
        "wrist_side",
        hold_targets.keys(),
        [],
        fallback_kp,
        fallback_kd,
        low_shoulder_pose,
    )
    hold_tau = build_hold_tau("wrist_side", hold_targets.keys(), [])
    if low_shoulder_pose:
        hold_tau = tune_wrist_active_hold_tau("wrist_side", hold_targets, hold_tau)
    print(
        "v2.6 replay coupled non-wrist active",
        ",".join(targets.keys()),
        "deltas_deg=",
        json.dumps(delta_report, ensure_ascii=False),
        "targets=",
        json.dumps(targets, ensure_ascii=False),
        "seconds=",
        seconds,
        "hold joints:",
        ",".join(hold_targets.keys()),
        flush=True,
    )
    if move_tau_ff:
        print("v2.6 replay coupled active_tau=", json.dumps(move_tau_ff, ensure_ascii=False), flush=True)
    if low_shoulder_pose:
        print(
            "v2.6 replay coupled low shoulder gains=",
            json.dumps(move_gains, ensure_ascii=False),
            flush=True,
        )
    arm.move_targets_with_holds(
        targets,
        seconds_per_step=seconds,
        move_gains=move_gains,
        hold_targets=hold_targets,
        hold_gains=hold_gains,
        fallback_kp=fallback_kp,
        fallback_kd=fallback_kd,
        hold_tau=hold_tau,
        move_tau_ff=move_tau_ff,
        control_dt=0.01,
        preload_seconds=0.2,
        trajectory="smoothstep",
        linear_blend=0.0,
    )


def replay_move(
    name: str,
    moves_file: str,
    clearance_file: str,
    python_bin: str,
    fallback_kp: float,
    fallback_kd: float,
    deadband_deg: float,
    replay_style: str = REPLAY_STYLE_DEFAULT,
) -> None:
    move_name = normalize_move_name(name)
    moves = load_moves(moves_file).get("moves", {})
    if move_name not in moves:
        raise RuntimeError(f"unknown move: {move_name}")
    record = moves[move_name]
    pose = record.get("pose", {})
    missing = [joint for joint in REPLAY_REQUIRED_JOINTS if joint not in pose]
    if missing:
        raise RuntimeError(f"move {move_name!r} is missing joints: {', '.join(missing)}")
    replay_order, final_joint = resolve_replay_order(record)

    run_table_clearance(python_bin, clearance_file)

    api = arm_api()
    arm = api.LeftArmV2()
    try:
        print("Serial port is open", flush=True)
        arm.enable(DEFAULT_JOINTS)
        print(
            "v2.6 replay move name=",
            move_name,
            "order=",
            ",".join(replay_order),
            "final=",
            final_joint,
            "replay_style=",
            replay_style,
            flush=True,
        )
        print("v2.6 replay target pose=", json.dumps(pose, ensure_ascii=False), flush=True)
        completed_targets: Dict[str, float] = {}
        low_shoulder_pose = is_low_shoulder_pose(pose)
        print(
            "v2.6 replay shoulder_front handoff config=",
            json.dumps(
                {
                    "entry_takeover_seconds": REPLAY_ENTRY_TAKEOVER_SECONDS,
                    "entry_hold_tau": REPLAY_ENTRY_HOLD_TAU.get("shoulder_front"),
                    "descent_tau": REPLAY_SHOULDER_FRONT_DESCENT_TAU,
                    "group_preload_seconds": REPLAY_SHOULDER_FRONT_GROUP_PRELOAD_SECONDS,
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        replay_entry_takeover_hold(arm, low_shoulder_pose)
        print_replay_error_report("after_clearance", arm.positions(DEFAULT_JOINTS), pose)
        if replay_style == "adaptive":
            completed_targets = run_adaptive_non_wrist_replay(
                arm,
                pose,
                replay_order,
                fallback_kp,
                fallback_kd,
                deadband_deg,
                low_shoulder_pose,
            )
        elif replay_style == "coupled":
            run_coupled_non_wrist_replay(
                arm,
                pose,
                replay_order,
                fallback_kp,
                fallback_kd,
                deadband_deg,
                low_shoulder_pose,
            )
        elif replay_style == "sequential":
            for active in replay_order:
                current = arm.positions(DEFAULT_JOINTS)
                target, requested_target, target_bias = biased_replay_target(active, pose, low_shoulder_pose)
                if target_bias:
                    print(
                        "v2.6 replay adaptive target bias",
                        active,
                        "bias_deg=",
                        math.degrees(target_bias),
                        "nominal_target=",
                        requested_target,
                        "biased_target=",
                        requested_target + target_bias,
                        flush=True,
                    )
                delta_deg = math.degrees(target - current[active])
                if abs(delta_deg) <= deadband_deg:
                    print(
                        "v2.6 replay skip",
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
                    low_shoulder_pose,
                )
                hold_tau = build_hold_tau(active, hold_targets.keys(), completed_targets.keys())
                if low_shoulder_pose:
                    hold_tau = tune_wrist_active_hold_tau(active, hold_targets, hold_tau)
                    completed_hold_targets = {
                        joint: hold_targets[joint]
                        for joint in completed_targets
                        if joint in hold_targets
                    }
                    adaptive_completed_tau = api.adaptive_hold_tau_for(
                        active,
                        completed_hold_targets,
                        hold_tau,
                        low_shoulder_pose,
                    )
                    hold_tau.update(
                        {
                            joint: adaptive_completed_tau[joint]
                            for joint in completed_hold_targets
                            if joint in adaptive_completed_tau
                        }
                    )
                active_tau = replay_active_tau(active, delta_deg, low_shoulder_pose)
                preload_seconds = api.NUDGE_PRELOAD_SECONDS.get(active, 0.2)
                trajectory = api.NUDGE_TRAJECTORY.get(active, "smoothstep")
                linear_blend = api.NUDGE_LINEAR_BLEND.get(active, 0.0)
                print(
                    "v2.6 replay active",
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
                    print("v2.6 replay active_tau=", {active: active_tau}, flush=True)
                if low_shoulder_pose and active in api.NUDGE_LOW_SHOULDER_GAINS:
                    print(
                        "v2.6 replay low shoulder gains active",
                        active,
                        "shoulder_front_target=",
                        pose.get("shoulder_front"),
                        flush=True,
                    )
                low_shoulder_hold_overrides = {
                    joint: hold_gains[joint]
                    for joint in (
                        set(api.POSTURE_HOLD_GAIN_RULES[api.LOW_SHOULDER_POSTURE].get("*", {}))
                        | set(api.POSTURE_HOLD_GAIN_RULES[api.LOW_SHOULDER_POSTURE].get(active, {}))
                    )
                    if joint in hold_gains
                }
                if low_shoulder_pose and low_shoulder_hold_overrides:
                    print(
                        "v2.6 replay low shoulder adaptive hold gains=",
                        json.dumps(low_shoulder_hold_overrides, ensure_ascii=False),
                        flush=True,
                    )
                if completed_targets:
                    print("v2.6 replay completed hold targets=", json.dumps(completed_targets, ensure_ascii=False), flush=True)
                    print(
                        "v2.6 replay completed hold gains=",
                        json.dumps(
                            {joint: hold_gains[joint] for joint in completed_targets if joint in hold_gains},
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
                    print(
                        "v2.6 replay hold_tau=",
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
                    print("v2.6 replay compliant hold gains=", json.dumps(compliant_holds, ensure_ascii=False), flush=True)
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
                record_hold_tau_adaptation(
                    arm,
                    active,
                    pose,
                    hold_targets,
                    hold_tau,
                    low_shoulder_pose,
                    label="replay-main",
                    learn_names=completed_targets.keys(),
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
                            "v2.6 replay low shoulder wrist_side fine",
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
                        record_hold_tau_adaptation(
                            arm,
                            active,
                            pose,
                            hold_targets,
                            hold_tau,
                            low_shoulder_pose,
                            label="replay-wrist-side-fine",
                            learn_names=completed_targets.keys(),
                        )
                    else:
                        print(
                            "v2.6 replay low shoulder wrist_side fine skip delta_deg=",
                            wrist_side_delta_deg,
                            "deadband_deg=",
                            api.NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_DEADBAND_DEG,
                            flush=True,
                        )
                completed_targets[active] = target
        else:
            raise RuntimeError(f"unknown replay style: {replay_style}")
        before_final_wrist_errors = run_replay_settle_pass(arm, pose, fallback_kp, fallback_kd, label="before_final_wrist")
        before_final_wrist_errors = run_replay_correction_pass(
            arm,
            pose,
            before_final_wrist_errors,
            fallback_kp,
            fallback_kd,
            deadband_deg,
            low_shoulder_pose,
        )
        run_final_wrist_move(arm, pose, fallback_kp, fallback_kd, deadband_deg, completed_targets)
        final_positions = arm.positions(DEFAULT_JOINTS)
        final_errors = print_replay_error_report("final", final_positions, pose)
        final_wrist_induced_errors = {
            joint: {
                "before_final_wrist": before_final_wrist_errors[joint],
                "final": error,
            }
            for joint, error in final_errors.items()
            if joint in before_final_wrist_errors
            and abs(before_final_wrist_errors[joint]) < api.ADAPTIVE_TARGET_BIAS_ERROR_DEADBAND_DEG
            and abs(error) >= api.ADAPTIVE_TARGET_BIAS_ERROR_DEADBAND_DEG
        }
        if final_wrist_induced_errors:
            print(
                "v2.6 replay adaptive target bias skipped final-wrist induced errors",
                json.dumps(final_wrist_induced_errors, ensure_ascii=False),
                flush=True,
            )
            hold_target_updates = api.update_adaptive_hold_target_bias(
                REPLAY_FINAL_JOINT,
                {joint: values["final"] for joint, values in final_wrist_induced_errors.items()},
                low_shoulder_front=low_shoulder_pose,
                label="replay-final-wrist-induced",
            )
            if hold_target_updates:
                print(
                    "v2.6 replay adaptive hold target bias updates",
                    json.dumps(hold_target_updates, ensure_ascii=False),
                    flush=True,
                )
        target_bias_errors = {
            joint: error
            for joint, error in final_errors.items()
            if joint not in final_wrist_induced_errors
        }
        target_bias_updates = api.update_adaptive_target_bias(
            target_bias_errors,
            low_shoulder_front=low_shoulder_pose,
            label="replay-final",
            require_ready=False,
        )
        if target_bias_updates:
            print(
                "v2.6 replay adaptive target bias updates",
                json.dumps(target_bias_updates, ensure_ascii=False),
                flush=True,
            )
        arm.print_status(DEFAULT_JOINTS)
    finally:
        arm.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Saved move library for left arm web controller v2.6")
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

    describe = sub.add_parser("describe-replay")
    describe.add_argument("--name", required=True)
    describe.add_argument("--moves-file", default=MOVES_PATH)
    describe.add_argument("--sequential", action="store_true", help="describe the legacy one-joint-at-a-time replay path")
    describe.add_argument("--coupled", action="store_true", help="describe the all-non-wrist coupled replay path")

    migrate = sub.add_parser("migrate-moves")
    migrate.add_argument("--source-file", default=LEGACY_MOVES_PATH)
    migrate.add_argument("--moves-file", default=MOVES_PATH)
    migrate.add_argument("--overwrite", action="store_true")

    replay = sub.add_parser("replay-move")
    replay.add_argument("--name", required=True)
    replay.add_argument("--moves-file", default=MOVES_PATH)
    replay.add_argument("--clearance-file", default=TABLE_CLEARANCE_PATH)
    replay.add_argument("--python-bin", default=sys.executable or "python3")
    replay.add_argument("--kp", type=float, default=3.0)
    replay.add_argument("--kd", type=float, default=0.3)
    replay.add_argument("--deadband-deg", type=float, default=0.5)
    replay.add_argument("--sequential", action="store_true", help="use the legacy one-joint-at-a-time replay path")
    replay.add_argument("--coupled", action="store_true", help="use the all-non-wrist coupled replay path")

    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.cmd == "low-torque":
        low_torque(args.kp, args.kd, args.control_dt, args.trace_interval)
    elif args.cmd == "capture-move":
        capture_move(args.name, args.note, args.moves_file)
    elif args.cmd == "list-moves":
        print(json.dumps(list_moves(args.moves_file), ensure_ascii=False, indent=2), flush=True)
    elif args.cmd == "describe-replay":
        replay_style = resolve_replay_style(args.sequential, args.coupled)
        print(
            json.dumps(
                describe_replay(args.name, args.moves_file, replay_style=replay_style),
                ensure_ascii=False,
                indent=2,
            ),
            flush=True,
        )
    elif args.cmd == "migrate-moves":
        migrate_moves(args.source_file, args.moves_file, args.overwrite)
    elif args.cmd == "replay-move":
        replay_style = resolve_replay_style(args.sequential, args.coupled)
        replay_move(
            args.name,
            args.moves_file,
            args.clearance_file,
            args.python_bin,
            args.kp,
            args.kd,
            args.deadband_deg,
            replay_style=replay_style,
        )
    else:
        raise RuntimeError(f"unknown command: {args.cmd}")


if __name__ == "__main__":
    main()
