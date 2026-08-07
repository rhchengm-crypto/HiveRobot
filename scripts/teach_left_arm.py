#!/usr/bin/env python3
"""Manual teach workflow for the HiveRobot left arm.

Typical safe workflow:

1. table-clearance: move to the trained clearance pose.
2. nudge: adjust one joint from the current motor position.
3. record: save the final successful pose.
4. home: return to home without writing a teach record.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import sys
import time
from typing import Dict, Iterable, List, Optional

from left_arm_controller import LeftArmController


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POSES_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_teach_poses.jsonl")
TEACH_HOME_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_teach_home.json")
TABLE_CLEARANCE_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_table_clearance.json")
HOME_FINAL_DEADBAND_DEG = 5.0
HOME_FINAL_JOINT_DEADBAND_DEG = {
    "shoulder_front": 8.0,
    "arm_roll": 10.0,
}
DEFAULT_REPLAY_WRIST_PREUP_DEG = 70.0
DEFAULT_TEACH_JOINTS = [
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
    "elbow",
    "arm_roll",
    "wrist_side",
    "wrist",
]
DEFAULT_REPLAY_ORDER = [
    "shoulder_front",
    "elbow",
    "wrist_side",
    "shoulder_side",
    "shoulder_rotate",
    "arm_roll",
    "wrist",
]
DEFAULT_HOME_ORDER = [
    "wrist",
    "wrist_side",
    "elbow",
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
    "arm_roll",
]
DEFAULT_STOW_ORDER = [
    "wrist",
    "wrist_side",
    "elbow",
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
    "arm_roll",
]


def parse_joints(value: str) -> List[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def ensure_pose_dir(path: str) -> None:
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)


def read_pose_records(path: str) -> List[dict]:
    if not os.path.exists(path):
        return []
    records = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def append_pose_record(path: str, record: dict) -> None:
    ensure_pose_dir(path)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")


def find_pose(path: str, name: str) -> dict:
    for record in reversed(read_pose_records(path)):
        if record.get("name") == name:
            return record
    raise RuntimeError(f"teach pose not found: {name}")


def load_teach_home(path: str) -> Dict[str, float]:
    if not os.path.exists(path):
        raise RuntimeError(
            "teach home file not found: "
            + path
            + "\nRun low-torque, manually place the new mechanical home, then run capture-home."
        )
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    joints = payload.get("joints")
    if not isinstance(joints, dict):
        raise RuntimeError("invalid teach home file, missing joints: " + path)
    print("loaded teach home:", path)
    return {name: float(pos) for name, pos in joints.items()}


def load_named_joint_pose(path: str, label: str) -> dict:
    if not os.path.exists(path):
        raise RuntimeError(f"{label} file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    joints = payload.get("joints")
    if not isinstance(joints, dict):
        raise RuntimeError(f"invalid {label} file, missing joints: {path}")
    print(f"loaded {label}:", path)
    return payload


def validate_joints(arm: LeftArmController, joints: Iterable[str]) -> List[str]:
    valid = set(arm.specs) | {"claw"}
    names = list(joints)
    unknown = sorted(set(names) - valid)
    if unknown:
        raise ValueError("unknown teach joints: " + ", ".join(unknown))
    return names


def joint_status(arm: LeftArmController, joints: Iterable[str]) -> Dict[str, dict]:
    names = validate_joints(arm, joints)
    status = arm.read_status()
    return {name: status[name] for name in names}


def joint_positions(arm: LeftArmController, joints: Iterable[str]) -> Dict[str, float]:
    return {name: data["pos"] for name, data in joint_status(arm, joints).items()}


def check_teach_target(
    arm: LeftArmController,
    home: Dict[str, float],
    name: str,
    target: float,
    strict_limits: bool,
) -> None:
    spec = arm.specs[name]
    offset = target - home[name]
    if spec.min_offset <= offset <= spec.max_offset:
        return
    message = (
        f"{name} taught target outside old controller offset limits: "
        f"offset={offset:.4f}, limit=[{spec.min_offset:.4f}, {spec.max_offset:.4f}]"
    )
    if strict_limits:
        raise ValueError(message)
    print("teach warning:", message)


def low_torque(
    arm: LeftArmController,
    joints: Iterable[str],
    seconds: float,
    kp: float,
    kd: float,
) -> None:
    names = validate_joints(arm, joints)
    print("low torque joints:", ", ".join(names))
    print("seconds=", seconds, "kp=", kp, "kd=", kd)
    end = time.time() + seconds
    while time.time() < end:
        for name in names:
            if name == "claw":
                arm.ctrl.refresh_motor_status(arm.claw)
                time.sleep(0.005)
                arm.ctrl.recv()
                pos = float(arm.claw.getPosition())
                arm.ctrl.controlMIT(arm.claw, kp, kd, pos, 0, 0)
            else:
                motor = arm.motors[name]
                arm.ctrl.refresh_motor_status(motor)
                time.sleep(0.005)
                arm.ctrl.recv()
                pos = float(motor.getPosition())
                arm.ctrl.controlMIT(motor, kp, kd, pos, 0, 0)
        time.sleep(0.03)
    print(json.dumps(joint_status(arm, names), ensure_ascii=False, indent=2))


def record_pose(
    arm: LeftArmController,
    path: str,
    name: str,
    joints: Iterable[str],
    note: str,
) -> dict:
    names = validate_joints(arm, joints)
    record = {
        "type": "teach_pose",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "name": name,
        "note": note,
        "joint_order": names,
        "joints": joint_positions(arm, names),
    }
    append_pose_record(path, record)
    print("recorded teach pose:", name)
    print("path:", path)
    print(json.dumps(record, ensure_ascii=False, indent=2))
    return record


def copy_pose_joint_from_home(
    poses_path: str,
    home_path: str,
    name: str,
    joint: str,
    note: str,
) -> dict:
    record = dict(find_pose(poses_path, name))
    record["joints"] = dict(record["joints"])
    home = load_teach_home(home_path)
    if joint not in record["joints"]:
        raise RuntimeError(f"pose {name} does not contain joint: {joint}")
    if joint not in home:
        raise RuntimeError(f"teach home does not contain joint: {joint}")
    old_value = float(record["joints"][joint])
    new_value = float(home[joint])
    record["created_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    record["note"] = note or (
        f"corrected {joint} from {old_value:.6f} to teach home {new_value:.6f}"
    )
    record["joints"][joint] = new_value
    record["corrected_from"] = {
        "name": name,
        "joint": joint,
        "old_value": old_value,
        "new_value": new_value,
        "source_home": home_path,
    }
    append_pose_record(poses_path, record)
    print("appended corrected teach pose:", name)
    print("path:", poses_path)
    print(json.dumps(record, ensure_ascii=False, indent=2))
    return record


def offset_pose_joint(
    poses_path: str,
    name: str,
    joint: str,
    delta_deg: float,
    note: str,
) -> dict:
    record = dict(find_pose(poses_path, name))
    record["joints"] = dict(record["joints"])
    if joint not in record["joints"]:
        raise RuntimeError(f"pose {name} does not contain joint: {joint}")
    old_value = float(record["joints"][joint])
    delta_rad = math.radians(delta_deg)
    new_value = old_value + delta_rad
    record["created_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    record["note"] = note or (
        f"offset {joint} by {delta_deg:.3f} deg from {old_value:.6f} to {new_value:.6f}"
    )
    record["joints"][joint] = new_value
    record["corrected_from"] = {
        "name": name,
        "joint": joint,
        "old_value": old_value,
        "delta_deg": delta_deg,
        "delta_rad": delta_rad,
        "new_value": new_value,
    }
    append_pose_record(poses_path, record)
    print("appended offset teach pose:", name)
    print("path:", poses_path)
    print(json.dumps(record, ensure_ascii=False, indent=2))
    return record


def nudge_joint(
    arm: LeftArmController,
    home: Dict[str, float],
    joint: str,
    delta_deg: float,
    seconds: float,
    strict_limits: bool,
) -> None:
    validate_joints(arm, [joint])
    if joint == "claw":
        raise ValueError("use claw-specific commands; nudge does not move claw")
    targets = arm.read_position_targets()
    start = targets[joint]
    target = start + math.radians(delta_deg)
    targets[joint] = target
    check_teach_target(arm, home, joint, target, strict_limits)
    print(
        "nudge joint:",
        joint,
        "start=",
        start,
        "delta_deg=",
        delta_deg,
        "target=",
        target,
        "seconds=",
        seconds,
    )
    arm.move_single_joint(joint, target, seconds=seconds)
    arm.print_status()


def save_table_clearance_from_pose(
    poses_path: str,
    table_clearance_path: str,
    name: str,
    note: str,
) -> dict:
    source = find_pose(poses_path, name)
    record = {
        "type": "table_clearance_pose",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "name": name,
        "note": note or f"promoted from teach pose {name}",
        "joint_order": source.get("joint_order") or DEFAULT_TEACH_JOINTS,
        "joints": dict(source["joints"]),
        "source_pose": {
            "name": name,
            "created_at": source.get("created_at"),
            "note": source.get("note", ""),
        },
    }
    ensure_pose_dir(table_clearance_path)
    with open(table_clearance_path, "w", encoding="utf-8") as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print("saved table clearance pose:", table_clearance_path)
    print(json.dumps(record, ensure_ascii=False, indent=2))
    return record


def capture_teach_home(
    arm: LeftArmController,
    path: str,
    joints: Iterable[str],
    note: str,
) -> dict:
    names = validate_joints(arm, joints)
    names = [name for name in names if name != "claw"]
    record = {
        "type": "teach_home",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": note,
        "joint_order": names,
        "joints": joint_positions(arm, names),
    }
    ensure_pose_dir(path)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print("captured teach home")
    print("path:", path)
    print("this is not a teach pose record; replay will use it as the new home baseline")
    print(json.dumps(record, ensure_ascii=False, indent=2))
    return record


def capture_claw_home(arm: LeftArmController) -> float:
    return arm.capture_claw_home()


def enable_home_soft(arm: LeftArmController) -> None:
    arm.enable_all(
        ff_scale_overrides={"shoulder_front": 0.0},
        enable_hold_scale=0.08,
        enable_ff_scale=0.0,
        startup_hold_seconds=0.10,
    )


def replay_pose(
    arm: LeftArmController,
    home: Dict[str, float],
    record: dict,
    seconds: float,
    strict_limits: bool,
    order: Iterable[str],
    final_only: bool,
    wrist_preup_deg: float,
    skip_wrist_preup: bool,
    close_claw: bool,
) -> None:
    print("replay pose:", record["name"])
    if final_only:
        move_stored_pose_final_only(
            arm,
            home,
            record,
            seconds=seconds,
            strict_limits=strict_limits,
            label="replay",
        )
    else:
        if not skip_wrist_preup:
            move_wrist_preup(
                arm,
                home,
                wrist_preup_deg=wrist_preup_deg,
                seconds=max(1.0, min(seconds, 2.0)),
                strict_limits=strict_limits,
            )
        move_stored_pose_one_by_one(
            arm,
            home,
            record,
            order=order,
            seconds=seconds,
            strict_limits=strict_limits,
            label="replay",
        )
    if close_claw:
        print("replay step: close claw until pressure/contact")
        arm.close_claw_pressure()


def move_wrist_preup(
    arm: LeftArmController,
    home: Dict[str, float],
    wrist_preup_deg: float,
    seconds: float,
    strict_limits: bool,
) -> None:
    target = home["wrist"] - math.radians(wrist_preup_deg)
    check_teach_target(arm, home, "wrist", target, strict_limits)
    targets = arm.read_position_targets()
    targets["wrist"] = target
    print(
        "replay wrist preup:",
        "wrist up offset deg=",
        wrist_preup_deg,
        "home rad=",
        home["wrist"],
        "target rad=",
        target,
    )
    arm.move_pose(targets, seconds=seconds, active="wrist")


def move_stored_pose_final_only(
    arm: LeftArmController,
    home: Dict[str, float],
    record: dict,
    seconds: float,
    strict_limits: bool,
    label: str,
) -> None:
    targets = arm.read_position_targets()
    stored_order = record.get("joint_order") or DEFAULT_TEACH_JOINTS
    active_joints = [
        name for name in stored_order if name in record["joints"] and name != "claw"
    ]
    for name in record["joints"]:
        if name != "claw" and name not in active_joints:
            active_joints.append(name)

    print(label, "final-only move joints:", ", ".join(active_joints))
    for name in active_joints:
        targets[name] = float(record["joints"][name])
        check_teach_target(arm, home, name, targets[name], strict_limits)
    print(label, "final targets:")
    print(json.dumps({name: targets[name] for name in active_joints}, ensure_ascii=False, indent=2))
    arm.move_pose(targets, seconds=seconds, active=tuple(active_joints))
    arm.print_status()


def move_stored_pose_one_by_one(
    arm: LeftArmController,
    home: Dict[str, float],
    record: dict,
    order: Iterable[str],
    seconds: float,
    strict_limits: bool,
    label: str,
    ff_scale_overrides: Optional[Dict[str, float]] = None,
) -> None:
    targets = arm.read_position_targets()
    ordered_joints = validate_joints(arm, order)
    ordered_joints = [
        name for name in ordered_joints if name in record["joints"] and name != "claw"
    ]
    for name in record["joints"]:
        if name != "claw" and name not in ordered_joints:
            ordered_joints.append(name)

    print(label, "one-by-one move joints:", ", ".join(ordered_joints))
    for name in ordered_joints:
        target = float(record["joints"][name])
        check_teach_target(arm, home, name, target, strict_limits)
        targets[name] = target
        print(label, "joint:", name, "target=", target)
        arm.move_pose(
            targets,
            seconds=max(1.0, seconds),
            active=name,
            ff_scale_overrides=ff_scale_overrides,
        )

    arm.print_status()


def print_session_help() -> None:
    print(
        "\n".join(
            [
                "session commands:",
                "  help",
                "  status",
                "  hold",
                "  table-clearance [seconds]",
                "  home [seconds]",
                "  stow [seconds]",
                "  replay NAME [seconds]",
                "  nudge JOINT DEG [seconds]",
                "  low-torque [seconds] [kp] [kd]",
                "  record NAME [note...]",
                "  capture-home [note...]",
                "  set-table-clearance NAME [note...]",
                "  quit",
            ]
        )
    )


def read_stdin_line_nonblocking(timeout: float) -> str:
    if os.name == "nt":
        return ""
    import select

    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return ""
    return sys.stdin.readline().strip()


def teach_session(
    arm: LeftArmController,
    poses_file: str,
    home_file: str,
    table_clearance_file: str,
    hold_hz: float,
    hold_scale: float,
) -> None:
    print("teach session started; controller stays alive and continuously holds.")
    print_session_help()
    hold_targets = arm.read_position_targets()
    dt = 1.0 / max(1.0, hold_hz)

    while True:
        arm.command_targets(hold_targets, hold_scale=hold_scale)
        line = read_stdin_line_nonblocking(dt)
        if not line:
            continue
        try:
            parts = shlex.split(line)
        except ValueError as exc:
            print("invalid command:", exc)
            continue
        if not parts:
            continue

        cmd = parts[0]
        try:
            if cmd in ("quit", "exit"):
                print("leaving teach session")
                return
            if cmd == "help":
                print_session_help()
            elif cmd == "status":
                arm.print_status()
            elif cmd == "hold":
                hold_targets = arm.read_position_targets()
                print("holding current pose")
            elif cmd == "table-clearance":
                seconds = float(parts[1]) if len(parts) > 1 else 5.0
                home = load_teach_home(home_file)
                record = load_named_joint_pose(table_clearance_file, "table clearance")
                move_stored_pose_final_only(
                    arm,
                    home,
                    record,
                    seconds=seconds,
                    strict_limits=False,
                    label="table clearance",
                )
                hold_targets = arm.read_position_targets()
            elif cmd == "home":
                seconds = float(parts[1]) if len(parts) > 1 else 6.0
                home = load_teach_home(home_file)
                record = load_named_joint_pose(table_clearance_file, "table clearance")
                home_via_table_clearance(
                    arm,
                    home,
                    record,
                    order=DEFAULT_HOME_ORDER,
                    seconds=seconds,
                    open_claw=True,
                    strict_limits=False,
                )
                hold_targets = arm.read_position_targets()
            elif cmd == "stow":
                seconds = float(parts[1]) if len(parts) > 1 else 6.0
                home = load_teach_home(home_file)
                home_final_only(
                    arm,
                    home,
                    order=DEFAULT_STOW_ORDER,
                    seconds=seconds,
                    open_claw=True,
                    strict_limits=False,
                )
                hold_targets = arm.read_position_targets()
            elif cmd == "replay":
                if len(parts) < 2:
                    print("usage: replay NAME [seconds]")
                    continue
                seconds = float(parts[2]) if len(parts) > 2 else 5.0
                home = load_teach_home(home_file)
                record = find_pose(poses_file, parts[1])
                replay_pose(
                    arm,
                    home,
                    record,
                    seconds=seconds,
                    strict_limits=False,
                    order=DEFAULT_REPLAY_ORDER,
                    final_only=False,
                    wrist_preup_deg=DEFAULT_REPLAY_WRIST_PREUP_DEG,
                    skip_wrist_preup=False,
                    close_claw=False,
                )
                hold_targets = arm.read_position_targets()
            elif cmd == "nudge":
                if len(parts) < 3:
                    print("usage: nudge JOINT DEG [seconds]")
                    continue
                seconds = float(parts[3]) if len(parts) > 3 else 1.0
                home = load_teach_home(home_file)
                nudge_joint(
                    arm,
                    home,
                    joint=parts[1],
                    delta_deg=float(parts[2]),
                    seconds=seconds,
                    strict_limits=False,
                )
                hold_targets = arm.read_position_targets()
            elif cmd == "low-torque":
                seconds = float(parts[1]) if len(parts) > 1 else 30.0
                kp = float(parts[2]) if len(parts) > 2 else 8.0
                kd = float(parts[3]) if len(parts) > 3 else 0.6
                low_torque(arm, DEFAULT_TEACH_JOINTS, seconds=seconds, kp=kp, kd=kd)
                hold_targets = arm.read_position_targets()
                print("low-torque ended; session is holding the new current pose")
            elif cmd == "record":
                if len(parts) < 2:
                    print("usage: record NAME [note...]")
                    continue
                note = " ".join(parts[2:])
                record_pose(
                    arm,
                    poses_file,
                    name=parts[1],
                    joints=DEFAULT_TEACH_JOINTS,
                    note=note,
                )
                hold_targets = arm.read_position_targets()
            elif cmd == "capture-home":
                note = " ".join(parts[1:])
                capture_teach_home(
                    arm,
                    home_file,
                    joints=DEFAULT_TEACH_JOINTS,
                    note=note,
                )
                hold_targets = arm.read_position_targets()
            elif cmd == "set-table-clearance":
                if len(parts) < 2:
                    print("usage: set-table-clearance NAME [note...]")
                    continue
                save_table_clearance_from_pose(
                    poses_file,
                    table_clearance_file,
                    name=parts[1],
                    note=" ".join(parts[2:]),
                )
            else:
                print("unknown session command:", cmd)
                print_session_help()
        except Exception as exc:
            print("session command failed:", exc)
            hold_targets = arm.read_position_targets()


def home_one_joint_at_a_time(
    arm: LeftArmController,
    home: Dict[str, float],
    order: Iterable[str],
    seconds: float,
    open_claw: bool,
    strict_limits: bool,
) -> None:
    targets = arm.read_position_targets()
    ordered_joints = validate_joints(arm, order)
    ordered_joints = [name for name in ordered_joints if name != "claw"]
    print("teach home joint order:", ", ".join(ordered_joints))

    for name in ordered_joints:
        targets[name] = home[name]
        check_teach_target(arm, home, name, targets[name], strict_limits)
        print("teach home joint:", name, "target=", targets[name])
        arm.move_pose(targets, seconds=seconds, active=name)

    arm.print_status()
    print("home completed; no teach record written")


def home_final_only(
    arm: LeftArmController,
    home: Dict[str, float],
    order: Iterable[str],
    seconds: float,
    open_claw: bool,
    strict_limits: bool,
) -> None:
    targets = arm.read_position_targets()
    ordered_joints = validate_joints(arm, order)
    active_joints = []
    skipped_joints = []
    for name in ordered_joints:
        if name == "claw":
            continue
        current = targets[name]
        target = home[name]
        deadband_deg = HOME_FINAL_JOINT_DEADBAND_DEG.get(
            name,
            HOME_FINAL_DEADBAND_DEG,
        )
        deadband_rad = math.radians(deadband_deg)
        if abs(target - current) <= deadband_rad:
            skipped_joints.append(name)
            continue
        targets[name] = target
        check_teach_target(arm, home, name, targets[name], strict_limits)
        active_joints.append(name)
        print(
            "teach home active delta:",
            name,
            "current=",
            current,
            "target=",
            target,
            "delta_deg=",
            math.degrees(target - current),
            "deadband_deg=",
            deadband_deg,
        )
    print("teach home final-only joints:", ", ".join(active_joints) or "(none)")
    if skipped_joints:
        print(
            "teach home skipped near-home joints:",
            ", ".join(skipped_joints),
            "default_deadband_deg=",
            HOME_FINAL_DEADBAND_DEG,
            "joint_deadbands_deg=",
            HOME_FINAL_JOINT_DEADBAND_DEG,
        )
    if not active_joints:
        if open_claw:
            enable_home_soft(arm)
            print("teach home step 0: open claw first")
            arm.open_claw_safe()
        arm.print_status()
        print("home completed; no teach record written")
        return
    enable_home_soft(arm)
    if open_claw:
        print("teach home step 0: open claw first")
        arm.open_claw_safe()
    print("teach home final targets:")
    print(json.dumps({name: targets[name] for name in active_joints}, ensure_ascii=False, indent=2))

    print("teach home direct interpolation to final home")
    arm.move_pose(targets, seconds=seconds, active=tuple(active_joints))
    arm.print_status()
    print("home completed; no teach record written")


def home_via_table_clearance(
    arm: LeftArmController,
    home: Dict[str, float],
    table_clearance_record: dict,
    order: Iterable[str],
    seconds: float,
    open_claw: bool,
    strict_limits: bool,
) -> None:
    enable_home_soft(arm)
    if open_claw:
        print("teach home step 0: open claw before retreat to table clearance")
        arm.open_claw_safe()

    print("teach home stage 1: retreat to table clearance")
    move_stored_pose_one_by_one(
        arm,
        home,
        table_clearance_record,
        order=order,
        seconds=max(2.0, seconds),
        strict_limits=strict_limits,
        label="table clearance before home",
        ff_scale_overrides={"shoulder_front": 0.0},
    )
    print("teach home stage 2: move from table clearance to home")
    home_final_only(
        arm,
        home,
        order=order,
        seconds=seconds,
        open_claw=False,
        strict_limits=strict_limits,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual teach workflow for left arm")
    sub = parser.add_subparsers(dest="cmd", required=True)

    common_joints = ",".join(DEFAULT_TEACH_JOINTS)

    status = sub.add_parser("status")
    status.add_argument("--joints", default=common_joints)

    low = sub.add_parser("low-torque")
    low.add_argument("--joints", default=common_joints)
    low.add_argument("--seconds", type=float, default=30.0)
    low.add_argument("--kp", type=float, default=8.0)
    low.add_argument("--kd", type=float, default=0.6)

    rec = sub.add_parser("record")
    rec.add_argument("--name", required=True)
    rec.add_argument("--note", default="")
    rec.add_argument("--joints", default=common_joints)
    rec.add_argument("--poses-file", default=POSES_PATH)

    cap_home = sub.add_parser("capture-home")
    cap_home.add_argument("--note", default="")
    cap_home.add_argument("--joints", default=common_joints)
    cap_home.add_argument("--home-file", default=TEACH_HOME_PATH)

    sub.add_parser("capture-claw-home")

    home = sub.add_parser("home")
    home.add_argument("--seconds", type=float, default=4.0)
    home.add_argument("--home-file", default=TEACH_HOME_PATH)
    home.add_argument("--table-clearance-file", default=TABLE_CLEARANCE_PATH)
    home.add_argument(
        "--joints",
        default=",".join(DEFAULT_HOME_ORDER),
        help="comma-separated one-joint-at-a-time home order",
    )
    home.add_argument("--no-open-claw", action="store_true")
    home.add_argument("--direct-home", action="store_true")
    home.add_argument("--via-table-clearance", action="store_true")
    home.add_argument("--strict-limits", action="store_true")

    stow = sub.add_parser("stow")
    stow.add_argument("--seconds", type=float, default=6.0)
    stow.add_argument("--home-file", default=TEACH_HOME_PATH)
    stow.add_argument(
        "--joints",
        default=",".join(DEFAULT_STOW_ORDER),
        help="comma-separated final-home joint set, including shoulder_front",
    )
    stow.add_argument("--no-open-claw", action="store_true")
    stow.add_argument("--strict-limits", action="store_true")

    replay = sub.add_parser("replay")
    replay.add_argument("--name", required=True)
    replay.add_argument("--seconds", type=float, default=5.0)
    replay.add_argument("--poses-file", default=POSES_PATH)
    replay.add_argument("--home-file", default=TEACH_HOME_PATH)
    replay.add_argument(
        "--joints",
        default=",".join(DEFAULT_REPLAY_ORDER),
        help="comma-separated one-joint-at-a-time replay order",
    )
    replay.add_argument("--final-only", action="store_true")
    replay.add_argument("--wrist-preup-deg", type=float, default=DEFAULT_REPLAY_WRIST_PREUP_DEG)
    replay.add_argument("--no-wrist-preup", action="store_true")
    replay.add_argument("--close-claw", action="store_true")
    replay.add_argument("--strict-limits", action="store_true")

    listing = sub.add_parser("list")
    listing.add_argument("--poses-file", default=POSES_PATH)

    show = sub.add_parser("show")
    show.add_argument("--name", required=True)
    show.add_argument("--poses-file", default=POSES_PATH)

    fix = sub.add_parser("copy-home-joint")
    fix.add_argument("--name", required=True)
    fix.add_argument("--joint", required=True)
    fix.add_argument("--poses-file", default=POSES_PATH)
    fix.add_argument("--home-file", default=TEACH_HOME_PATH)
    fix.add_argument("--note", default="")

    offset = sub.add_parser("offset-joint")
    offset.add_argument("--name", required=True)
    offset.add_argument("--joint", required=True)
    offset.add_argument("--deg", type=float, required=True)
    offset.add_argument("--poses-file", default=POSES_PATH)
    offset.add_argument("--note", default="")

    nudge = sub.add_parser("nudge")
    nudge.add_argument("--joint", required=True)
    nudge.add_argument("--deg", type=float, required=True)
    nudge.add_argument("--seconds", type=float, default=1.0)
    nudge.add_argument("--home-file", default=TEACH_HOME_PATH)
    nudge.add_argument("--strict-limits", action="store_true")

    promote_clearance = sub.add_parser("set-table-clearance")
    promote_clearance.add_argument("--name", required=True)
    promote_clearance.add_argument("--poses-file", default=POSES_PATH)
    promote_clearance.add_argument("--table-clearance-file", default=TABLE_CLEARANCE_PATH)
    promote_clearance.add_argument("--note", default="")

    table_clearance = sub.add_parser("table-clearance")
    table_clearance.add_argument("--seconds", type=float, default=5.0)
    table_clearance.add_argument("--home-file", default=TEACH_HOME_PATH)
    table_clearance.add_argument("--table-clearance-file", default=TABLE_CLEARANCE_PATH)
    table_clearance.add_argument("--strict-limits", action="store_true")

    session = sub.add_parser("session")
    session.add_argument("--poses-file", default=POSES_PATH)
    session.add_argument("--home-file", default=TEACH_HOME_PATH)
    session.add_argument("--table-clearance-file", default=TABLE_CLEARANCE_PATH)
    session.add_argument("--hold-hz", type=float, default=50.0)
    session.add_argument("--hold-scale", type=float, default=1.25)

    args = parser.parse_args()

    if args.cmd == "list":
        records = read_pose_records(args.poses_file)
        for record in records:
            print(
                record.get("created_at", ""),
                record.get("name", ""),
                record.get("note", ""),
            )
        print("count:", len(records))
        return
    if args.cmd == "show":
        record = find_pose(args.poses_file, args.name)
        print(json.dumps(record, ensure_ascii=False, indent=2))
        return
    if args.cmd == "copy-home-joint":
        copy_pose_joint_from_home(
            args.poses_file,
            args.home_file,
            name=args.name,
            joint=args.joint,
            note=args.note,
        )
        return
    if args.cmd == "offset-joint":
        offset_pose_joint(
            args.poses_file,
            name=args.name,
            joint=args.joint,
            delta_deg=args.deg,
            note=args.note,
        )
        return
    if args.cmd == "set-table-clearance":
        save_table_clearance_from_pose(
            args.poses_file,
            args.table_clearance_file,
            name=args.name,
            note=args.note,
        )
        return

    arm = LeftArmController()
    try:
        print("Serial port is open")
        if args.cmd == "nudge":
            arm.enable_named([args.joint])
        elif args.cmd == "low-torque":
            arm.enable_named(parse_joints(args.joints))
        elif args.cmd in ("record", "capture-home", "capture-claw-home", "status"):
            arm.refresh_all()
        elif args.cmd == "home":
            arm.refresh_all()
        else:
            arm.enable_all()

        if args.cmd == "status":
            print(json.dumps(joint_status(arm, parse_joints(args.joints)), ensure_ascii=False, indent=2))
        elif args.cmd == "low-torque":
            low_torque(
                arm,
                parse_joints(args.joints),
                seconds=args.seconds,
                kp=args.kp,
                kd=args.kd,
            )
        elif args.cmd == "record":
            record_pose(
                arm,
                args.poses_file,
                name=args.name,
                joints=parse_joints(args.joints),
                note=args.note,
            )
        elif args.cmd == "capture-home":
            capture_teach_home(
                arm,
                args.home_file,
                joints=parse_joints(args.joints),
                note=args.note,
            )
        elif args.cmd == "capture-claw-home":
            capture_claw_home(arm)
        elif args.cmd == "nudge":
            home_pose = load_teach_home(args.home_file)
            nudge_joint(
                arm,
                home_pose,
                joint=args.joint,
                delta_deg=args.deg,
                seconds=args.seconds,
                strict_limits=args.strict_limits,
            )
        elif args.cmd == "home":
            home_pose = load_teach_home(args.home_file)
            if args.via_table_clearance and args.direct_home:
                raise ValueError("choose only one: --direct-home or --via-table-clearance")
            if args.via_table_clearance:
                table_clearance_pose = load_named_joint_pose(
                    args.table_clearance_file,
                    "table clearance",
                )
                home_via_table_clearance(
                    arm,
                    home_pose,
                    table_clearance_pose,
                    order=parse_joints(args.joints),
                    seconds=args.seconds,
                    open_claw=not args.no_open_claw,
                    strict_limits=args.strict_limits,
                )
            else:
                home_final_only(
                    arm,
                    home_pose,
                    order=parse_joints(args.joints),
                    seconds=args.seconds,
                    open_claw=not args.no_open_claw,
                    strict_limits=args.strict_limits,
                )
        elif args.cmd == "stow":
            home_pose = load_teach_home(args.home_file)
            home_final_only(
                arm,
                home_pose,
                order=parse_joints(args.joints),
                seconds=args.seconds,
                open_claw=not args.no_open_claw,
                strict_limits=args.strict_limits,
            )
        elif args.cmd == "replay":
            home_pose = load_teach_home(args.home_file)
            record = find_pose(args.poses_file, args.name)
            replay_pose(
                arm,
                home_pose,
                record,
                seconds=args.seconds,
                strict_limits=args.strict_limits,
                order=parse_joints(args.joints),
                final_only=args.final_only,
                wrist_preup_deg=args.wrist_preup_deg,
                skip_wrist_preup=args.no_wrist_preup,
                close_claw=args.close_claw,
            )
        elif args.cmd == "table-clearance":
            home_pose = load_teach_home(args.home_file)
            record = load_named_joint_pose(args.table_clearance_file, "table clearance")
            move_stored_pose_final_only(
                arm,
                home_pose,
                record,
                seconds=args.seconds,
                strict_limits=args.strict_limits,
                label="table clearance",
            )
        elif args.cmd == "session":
            teach_session(
                arm,
                poses_file=args.poses_file,
                home_file=args.home_file,
                table_clearance_file=args.table_clearance_file,
                hold_hz=args.hold_hz,
                hold_scale=args.hold_scale,
            )
    finally:
        arm.close()


if __name__ == "__main__":
    main()
