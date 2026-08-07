#!/usr/bin/env python3
"""Temporary debug tool for the new left wrist-side joint.

This script only talks to the new wrist_side motor:

- slave_id 0x2A
- master_id 0x1A
- motor type DM4310

Use it before adding the joint to the full arm controller. It is intentionally
small and does not enable or command any other arm joint.
"""

from __future__ import annotations

import argparse
import math
import time

import serial
from DM_CAN import DM_Motor_Type, Motor, MotorControl


SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 921600

WRIST_SIDE_SLAVE_ID = 0x2A
WRIST_SIDE_MASTER_ID = 0x1A


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


class WristSideDebug:
    def __init__(self, port: str, baud: int) -> None:
        self.serial = serial.Serial(port, baud, timeout=1.0)
        self.ctrl = MotorControl(self.serial)
        self.motor = Motor(
            DM_Motor_Type.DM4310,
            WRIST_SIDE_SLAVE_ID,
            WRIST_SIDE_MASTER_ID,
        )
        self.ctrl.addMotor(self.motor)

    def close(self) -> None:
        try:
            self.serial.close()
        except Exception:
            pass

    def enable(self) -> None:
        print(
            "enable wrist_side",
            "slave=0x%02X" % WRIST_SIDE_SLAVE_ID,
            "master=0x%02X" % WRIST_SIDE_MASTER_ID,
        )
        self.ctrl.enable(self.motor)
        time.sleep(0.2)

    def refresh(self) -> None:
        self.ctrl.refresh_motor_status(self.motor)
        time.sleep(0.03)
        self.ctrl.recv()

    def status(self) -> dict:
        self.refresh()
        return {
            "pos": float(self.motor.getPosition()),
            "vel": float(self.motor.getVelocity()),
            "tau": float(self.motor.getTorque()),
        }

    def print_status(self, label: str = "status") -> None:
        data = self.status()
        print(
            label,
            "pos=", data["pos"],
            "vel=", data["vel"],
            "tau=", data["tau"],
        )

    def command(self, target: float, kp: float, kd: float) -> None:
        self.ctrl.controlMIT(self.motor, kp, kd, target, 0, 0)

    def hold_current(self, seconds: float, kp: float, kd: float) -> None:
        pos = self.status()["pos"]
        print("hold current pos", pos, "seconds", seconds, "kp", kp, "kd", kd)
        end = time.time() + seconds
        while time.time() < end:
            self.command(pos, kp, kd)
            time.sleep(0.01)
        self.print_status("after hold")

    def move_relative(self, delta_deg: float, seconds: float, kp: float, kd: float) -> None:
        start = self.status()["pos"]
        delta = math.radians(delta_deg)
        target = start + delta
        print(
            "move relative",
            "start=", start,
            "delta_deg=", delta_deg,
            "target=", target,
            "seconds=", seconds,
        )
        t0 = time.time()
        while time.time() - t0 < seconds:
            a = (time.time() - t0) / seconds
            s = smoothstep(a)
            self.command(start * (1.0 - s) + target * s, kp, kd)
            time.sleep(0.01)
        end = time.time() + 0.5
        while time.time() < end:
            self.command(target, kp, kd)
            time.sleep(0.01)
        self.print_status("after move")

    def low_torque(self, seconds: float, kp: float, kd: float) -> None:
        print("low torque mode seconds", seconds, "kp", kp, "kd", kd)
        end = time.time() + seconds
        while time.time() < end:
            pos = self.status()["pos"]
            self.command(pos, kp, kd)
            time.sleep(0.03)
        self.print_status("after low torque")


def main() -> None:
    parser = argparse.ArgumentParser(description="Debug only the new wrist_side joint")
    parser.add_argument("cmd", choices=["status", "hold", "left", "right", "low-torque"])
    parser.add_argument("--port", default=SERIAL_PORT)
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--deg", type=float, default=3.0)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--kp", type=float, default=18.0)
    parser.add_argument("--kd", type=float, default=0.8)
    args = parser.parse_args()

    arm = WristSideDebug(args.port, args.baud)
    try:
        arm.enable()
        arm.print_status("initial")
        if args.cmd == "status":
            return
        if args.cmd == "hold":
            arm.hold_current(args.seconds, args.kp, args.kd)
        elif args.cmd == "left":
            # Installed joint sign is inverted: negative motor delta moves the claw left.
            arm.move_relative(-abs(args.deg), args.seconds, args.kp, args.kd)
        elif args.cmd == "right":
            arm.move_relative(abs(args.deg), args.seconds, args.kp, args.kd)
        elif args.cmd == "low-torque":
            arm.low_torque(args.seconds, kp=min(args.kp, 4.0), kd=min(args.kd, 0.3))
    finally:
        arm.close()


if __name__ == "__main__":
    main()
