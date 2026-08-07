#!/usr/bin/env python3
"""Clean left-arm bring-up controller.

This v2 controller intentionally does not import or reuse the old
left_arm_controller / teach_left_arm motion stack. It is for post-reassembly
bring-up: read status, low-gain holds, capture a new home, small nudges, and
low-gain home moves.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List

import serial
from DM_CAN import DM_Motor_Type, Motor, MotorControl


SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 921600
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HOME_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_v2_home.json")


@dataclass(frozen=True)
class JointSpec:
    name: str
    motor_type: DM_Motor_Type
    slave_id: int
    master_id: int


JOINTS = [
    JointSpec("shoulder_front", DM_Motor_Type.DM4340, 0x1E, 0x0E),
    JointSpec("shoulder_side", DM_Motor_Type.DM4340, 0x1F, 0x0F),
    JointSpec("shoulder_rotate", DM_Motor_Type.DM4340, 0x20, 0x10),
    JointSpec("elbow", DM_Motor_Type.DM4340, 0x21, 0x11),
    JointSpec("arm_roll", DM_Motor_Type.DM4340, 0x22, 0x12),
    JointSpec("wrist_side", DM_Motor_Type.DM4310, 0x2A, 0x1A),
    JointSpec("wrist", DM_Motor_Type.DM4310, 0x29, 0x19),
]

DEFAULT_JOINTS = [spec.name for spec in JOINTS]
DEFAULT_HOME_ORDER = [
    "wrist",
    "wrist_side",
    "arm_roll",
    "elbow",
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
]

OLD_HOME_ORDER = [
    "wrist",
    "wrist_side",
    "elbow",
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
    "arm_roll",
]

HOME_GAINS = {
    "wrist": {"kp": 22.0, "kd": 1.4, "seconds": 5.0},
    "wrist_side": {"kp": 22.0, "kd": 1.4, "seconds": 5.0},
    "elbow": {"kp": 70.0, "kd": 3.0, "seconds": 6.0},
    "arm_roll": {"kp": 45.0, "kd": 2.5, "seconds": 5.0},
    "shoulder_front": {"kp": 90.0, "kd": 4.0, "seconds": 6.0},
    "shoulder_side": {"kp": 70.0, "kd": 3.0, "seconds": 6.0},
    "shoulder_rotate": {"kp": 60.0, "kd": 3.0, "seconds": 6.0},
}

NUDGE_GAINS = {
    "wrist": {"kp": 18.0, "kd": 1.2},
    "wrist_side": {"kp": 18.0, "kd": 1.2},
    "arm_roll": {"kp": 45.0, "kd": 2.5},
    "elbow": {"kp": 70.0, "kd": 3.0},
    "shoulder_front": {"kp": 80.0, "kd": 3.5},
    "shoulder_side": {"kp": 70.0, "kd": 3.0},
    "shoulder_rotate": {"kp": 60.0, "kd": 3.0},
}


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def parse_joints(value: str) -> List[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


class LeftArmV2:
    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD) -> None:
        self.serial = serial.Serial(port, baud, timeout=1.0)
        self.ctrl = MotorControl(self.serial)
        self.specs = {spec.name: spec for spec in JOINTS}
        self.motors: Dict[str, Motor] = {}
        for spec in JOINTS:
            motor = Motor(spec.motor_type, spec.slave_id, spec.master_id)
            self.ctrl.addMotor(motor)
            self.motors[spec.name] = motor

    def close(self) -> None:
        self.serial.close()

    def validate(self, names: Iterable[str]) -> List[str]:
        names = list(names)
        unknown = sorted(set(names) - set(self.specs))
        if unknown:
            raise ValueError("unknown joints: " + ", ".join(unknown))
        return names

    def enable(self, names: Iterable[str]) -> None:
        for name in self.validate(names):
            print("enable", name)
            self.ctrl.enable(self.motors[name])
            time.sleep(0.05)

    def read_status(self, names: Iterable[str]) -> Dict[str, Dict[str, float]]:
        names = self.validate(names)
        for name in names:
            self.ctrl.refresh_motor_status(self.motors[name])
        time.sleep(0.02)
        self.ctrl.recv()
        return {
            name: {
                "pos": float(self.motors[name].getPosition()),
                "vel": float(self.motors[name].getVelocity()),
                "tau": float(self.motors[name].getTorque()),
            }
            for name in names
        }

    def positions(self, names: Iterable[str]) -> Dict[str, float]:
        return {name: data["pos"] for name, data in self.read_status(names).items()}

    def print_status(self, names: Iterable[str]) -> None:
        print(json.dumps(self.read_status(names), ensure_ascii=False, indent=2))

    def hold_positions(
        self,
        targets: Dict[str, float],
        seconds: float,
        kp: float,
        kd: float,
    ) -> None:
        names = self.validate(targets.keys())
        end = time.time() + seconds
        while time.time() < end:
            for name in names:
                self.ctrl.controlMIT(self.motors[name], kp, kd, targets[name], 0, 0)
            time.sleep(0.02)

    def hold_current(
        self,
        names: Iterable[str],
        seconds: float,
        kp: float,
        kd: float,
    ) -> None:
        targets = self.positions(names)
        self.hold_positions(targets, seconds=seconds, kp=kp, kd=kd)

    def move_targets(
        self,
        targets: Dict[str, float],
        seconds: float,
        kp: float,
        kd: float,
    ) -> None:
        names = self.validate(targets.keys())
        starts = self.positions(names)
        self.hold_positions(starts, seconds=0.2, kp=kp, kd=kd)
        start_time = time.time()
        while time.time() - start_time < seconds:
            t = (time.time() - start_time) / max(seconds, 1e-6)
            s = smoothstep(t)
            for name in names:
                target = starts[name] * (1.0 - s) + targets[name] * s
                self.ctrl.controlMIT(self.motors[name], kp, kd, target, 0, 0)
            time.sleep(0.02)
        self.hold_positions(targets, seconds=0.5, kp=kp, kd=kd)

    def move_targets_auto_gains(
        self,
        targets: Dict[str, float],
        seconds: float,
        gains: Dict[str, Dict[str, float]],
        fallback_kp: float,
        fallback_kd: float,
    ) -> None:
        names = self.validate(targets.keys())
        starts = self.positions(names)
        start_time = time.time()
        while time.time() - start_time < 0.2:
            for name in names:
                joint_gains = gains.get(name, {})
                kp = joint_gains.get("kp", fallback_kp)
                kd = joint_gains.get("kd", fallback_kd)
                self.ctrl.controlMIT(self.motors[name], kp, kd, starts[name], 0, 0)
            time.sleep(0.02)

        start_time = time.time()
        while time.time() - start_time < seconds:
            t = (time.time() - start_time) / max(seconds, 1e-6)
            s = smoothstep(t)
            for name in names:
                joint_gains = gains.get(name, {})
                kp = joint_gains.get("kp", fallback_kp)
                kd = joint_gains.get("kd", fallback_kd)
                target = starts[name] * (1.0 - s) + targets[name] * s
                self.ctrl.controlMIT(self.motors[name], kp, kd, target, 0, 0)
            time.sleep(0.02)

        start_time = time.time()
        while time.time() - start_time < 0.5:
            for name in names:
                joint_gains = gains.get(name, {})
                kp = joint_gains.get("kp", fallback_kp)
                kd = joint_gains.get("kd", fallback_kd)
                self.ctrl.controlMIT(self.motors[name], kp, kd, targets[name], 0, 0)
            time.sleep(0.02)


def load_home(path: str = HOME_PATH) -> Dict[str, float]:
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    joints = payload.get("joints")
    if not isinstance(joints, dict):
        raise RuntimeError("invalid v2 home file: " + path)
    return {name: float(value) for name, value in joints.items()}


def save_home(home: Dict[str, float], note: str, path: str = HOME_PATH) -> None:
    payload = {
        "type": "left_arm_v2_home",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": note,
        "joint_order": list(home.keys()),
        "joints": home,
    }
    ensure_parent(path)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print("saved v2 home:", path)
    print(json.dumps(payload, ensure_ascii=False, indent=2))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Clean v2 left-arm bring-up controller")
    sub = parser.add_subparsers(dest="cmd", required=True)

    common = ",".join(DEFAULT_JOINTS)

    status = sub.add_parser("status")
    status.add_argument("--joints", default=common)

    hold = sub.add_parser("hold")
    hold.add_argument("--joints", default=common)
    hold.add_argument("--seconds", type=float, default=10.0)
    hold.add_argument("--kp", type=float, default=2.0)
    hold.add_argument("--kd", type=float, default=0.2)

    cap = sub.add_parser("capture-home")
    cap.add_argument("--joints", default=common)
    cap.add_argument("--note", default="")
    cap.add_argument("--home-file", default=HOME_PATH)

    show = sub.add_parser("show-home")
    show.add_argument("--home-file", default=HOME_PATH)

    nudge = sub.add_parser("nudge")
    nudge.add_argument("--joint", required=True)
    nudge.add_argument("--deg", type=float, required=True)
    nudge.add_argument("--seconds", type=float, default=2.0)
    nudge.add_argument("--kp", type=float, default=3.0)
    nudge.add_argument("--kd", type=float, default=0.3)
    nudge.add_argument("--max-deg", type=float, default=5.0)
    nudge.add_argument(
        "--auto-gains",
        action="store_true",
        help="Use verified per-joint nudge gains instead of uniform --kp/--kd.",
    )

    home = sub.add_parser("home")
    home.add_argument("--joints", default=",".join(DEFAULT_HOME_ORDER))
    home.add_argument("--home-file", default=HOME_PATH)
    home.add_argument("--seconds", type=float, default=5.0)
    home.add_argument("--kp", type=float, default=3.0)
    home.add_argument("--kd", type=float, default=0.3)
    home.add_argument("--deadband-deg", type=float, default=3.0)
    home.add_argument("--max-delta-deg", type=float, default=20.0)
    home.add_argument(
        "--auto-gains",
        dest="auto_gains",
        action="store_true",
        default=True,
        help="Use verified per-joint home gains. This is the default.",
    )
    home.add_argument(
        "--no-auto-gains",
        dest="auto_gains",
        action="store_false",
        help="Use the uniform --kp/--kd/--seconds values instead.",
    )
    home.add_argument("--execute", action="store_true")

    return parser


def main() -> None:
    args = build_parser().parse_args()

    if args.cmd == "show-home":
        print(json.dumps(load_home(args.home_file), ensure_ascii=False, indent=2))
        return

    arm = LeftArmV2()
    try:
        print("Serial port is open")

        if args.cmd == "status":
            arm.print_status(parse_joints(args.joints))
            return

        if args.cmd == "hold":
            joints = parse_joints(args.joints)
            arm.enable(joints)
            arm.hold_current(joints, seconds=args.seconds, kp=args.kp, kd=args.kd)
            arm.print_status(joints)
            return

        if args.cmd == "capture-home":
            joints = parse_joints(args.joints)
            home = arm.positions(joints)
            save_home(home, note=args.note, path=args.home_file)
            return

        if args.cmd == "nudge":
            if abs(args.deg) > args.max_deg:
                raise ValueError(f"nudge deg exceeds --max-deg {args.max_deg}")
            joints = parse_joints(args.joint)
            arm.validate(joints)
            arm.enable(joints)
            starts = arm.positions(joints)
            targets = {name: starts[name] + math.radians(args.deg) for name in joints}
            print(
                "v2 nudge joints:",
                ", ".join(joints),
                "delta_deg=",
                args.deg,
                "kp=",
                args.kp,
                "kd=",
                args.kd,
            )
            print(json.dumps({"start": starts, "target": targets}, ensure_ascii=False, indent=2))
            if args.auto_gains:
                for name in joints:
                    gains = NUDGE_GAINS.get(name, {"kp": args.kp, "kd": args.kd})
                    print(
                        "v2 nudge auto gains",
                        name,
                        "kp=",
                        gains["kp"],
                        "kd=",
                        gains["kd"],
                        "seconds=",
                        args.seconds,
                    )
                arm.move_targets_auto_gains(
                    targets,
                    seconds=args.seconds,
                    gains=NUDGE_GAINS,
                    fallback_kp=args.kp,
                    fallback_kd=args.kd,
                )
            else:
                arm.move_targets(targets, seconds=args.seconds, kp=args.kp, kd=args.kd)
            arm.print_status(joints)
            return

        if args.cmd == "home":
            home = load_home(args.home_file)
            ordered = [name for name in parse_joints(args.joints) if name in home]
            current = arm.positions(ordered)
            targets: Dict[str, float] = {}
            for name in ordered:
                delta_deg = math.degrees(home[name] - current[name])
                if abs(delta_deg) <= args.deadband_deg:
                    print("v2 home skip", name, "delta_deg=", delta_deg)
                    continue
                if abs(delta_deg) > args.max_delta_deg:
                    raise RuntimeError(
                        f"{name} home delta {delta_deg:.2f} deg exceeds --max-delta-deg {args.max_delta_deg}; "
                        "recapture home or move in smaller steps."
                    )
                targets[name] = home[name]
                print("v2 home target", name, "delta_deg=", delta_deg, "target=", home[name])

            if not targets:
                print("v2 home: all selected joints inside deadband; no motor enabled")
                arm.print_status(ordered)
                return

            if not args.execute:
                print("dry run only. Add --execute to move.")
                return

            arm.enable(targets.keys())
            for name in ordered:
                if name not in targets:
                    continue
                if args.auto_gains:
                    gains = HOME_GAINS.get(name, {"kp": args.kp, "kd": args.kd, "seconds": args.seconds})
                    kp = gains["kp"]
                    kd = gains["kd"]
                    seconds = gains["seconds"]
                    print("v2 home auto gains", name, "kp=", kp, "kd=", kd, "seconds=", seconds)
                else:
                    kp = args.kp
                    kd = args.kd
                    seconds = args.seconds
                arm.move_targets({name: targets[name]}, seconds=seconds, kp=kp, kd=kd)
            arm.print_status(ordered)
            return

    finally:
        arm.close()


if __name__ == "__main__":
    main()
