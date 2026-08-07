#!/usr/bin/env python3
"""Temporary maintenance home command for shoulder_front and shoulder_side only.

Use this only while the joints below shoulder_side are removed. It enables and
moves only shoulder_front and shoulder_side, using left_arm_home.json as the
home source. It intentionally does not touch shoulder_rotate, elbow, arm_roll,
wrist, or claw.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict

import serial
from DM_CAN import DM_Motor_Type, Motor, MotorControl


SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 921600
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, "left_arm_home.json")


@dataclass(frozen=True)
class JointSpec:
    name: str
    motor_type: DM_Motor_Type
    slave_id: int
    master_id: int
    kp_hold: float
    kd_hold: float
    kp_move: float
    kd_move: float
    min_offset: float
    max_offset: float


JOINTS = [
    JointSpec(
        name="shoulder_front",
        motor_type=DM_Motor_Type.DM4340,
        slave_id=0x1E,
        master_id=0x0E,
        kp_hold=260,
        kd_hold=5.0,
        kp_move=360,
        kd_move=5.5,
        min_offset=-math.radians(95),
        max_offset=math.radians(35),
    ),
    JointSpec(
        name="shoulder_side",
        motor_type=DM_Motor_Type.DM4340,
        slave_id=0x1F,
        master_id=0x0F,
        kp_hold=300,
        kd_hold=5.0,
        kp_move=380,
        kd_move=5.5,
        min_offset=-math.radians(30),
        max_offset=math.radians(95),
    ),
]


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


class FrontSideHome:
    def __init__(self, port: str, baud: int) -> None:
        self.serial = serial.Serial(port, baud, timeout=1.0)
        self.ctrl = MotorControl(self.serial)
        self.motors: Dict[str, Motor] = {}
        for spec in JOINTS:
            motor = Motor(spec.motor_type, spec.slave_id, spec.master_id)
            self.ctrl.addMotor(motor)
            self.motors[spec.name] = motor

    def close(self) -> None:
        try:
            self.serial.close()
        except Exception:
            pass

    def enable(self) -> None:
        for spec in JOINTS:
            print("enable", spec.name)
            self.ctrl.enable(self.motors[spec.name])
            time.sleep(0.15)

    def recv_once(self) -> None:
        time.sleep(0.03)
        self.ctrl.recv()

    def refresh(self) -> None:
        for motor in self.motors.values():
            self.ctrl.refresh_motor_status(motor)
        self.recv_once()

    def read_positions(self) -> Dict[str, float]:
        self.refresh()
        return {name: float(motor.getPosition()) for name, motor in self.motors.items()}

    def command(self, targets: Dict[str, float], active: str) -> None:
        for spec in JOINTS:
            motor = self.motors[spec.name]
            target = targets[spec.name]
            if spec.name == active:
                kp, kd = spec.kp_move, spec.kd_move
            else:
                kp, kd = spec.kp_hold, spec.kd_hold
            self.ctrl.controlMIT(motor, kp, kd, target, 0, 0)

    def move_one(
        self,
        start_targets: Dict[str, float],
        final_targets: Dict[str, float],
        active: str,
        seconds: float,
    ) -> None:
        start_time = time.time()
        while time.time() - start_time < seconds:
            a = (time.time() - start_time) / seconds
            s = smoothstep(a)
            step = {}
            for spec in JOINTS:
                name = spec.name
                step[name] = start_targets[name] * (1.0 - s) + final_targets[name] * s
            self.command(step, active=active)
            time.sleep(0.01)

    def hold(self, targets: Dict[str, float], seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            self.command(targets, active="")
            time.sleep(0.01)


def load_front_side_home(path: str) -> Dict[str, float]:
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    home = payload["home"]
    return {spec.name: float(home[spec.name]) for spec in JOINTS}


def main() -> None:
    parser = argparse.ArgumentParser(description="Temporary front/side home only")
    parser.add_argument("--port", default=SERIAL_PORT)
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--home-file", default=CONFIG_PATH)
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually move shoulder_front and shoulder_side",
    )
    args = parser.parse_args()

    home = load_front_side_home(args.home_file)
    arm = FrontSideHome(args.port, args.baud)
    try:
        arm.enable()
        current = arm.read_positions()
        print("current:", current)
        print("front/side home:", home)
        if not args.execute:
            print("dry run only. add --execute to move front and side home.")
            return

        step1 = dict(current)
        step1["shoulder_front"] = home["shoulder_front"]
        print("step 1: shoulder_front home")
        arm.move_one(current, step1, active="shoulder_front", seconds=args.seconds)

        step2 = dict(step1)
        step2["shoulder_side"] = home["shoulder_side"]
        print("step 2: shoulder_side home")
        arm.move_one(step1, step2, active="shoulder_side", seconds=args.seconds)

        arm.hold(step2, 0.8)
        print("done:", arm.read_positions())
    finally:
        arm.close()


if __name__ == "__main__":
    main()
