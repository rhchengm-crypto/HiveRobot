#!/usr/bin/env python3
"""
Formal left arm control scaffold for the HiveRobot Damiao arm.

Copy this file to ~/hive_robot/DM_Control_Python/ on the Orin and run it
beside DM_CAN.py.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

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
    JointSpec(
        name="shoulder_rotate",
        motor_type=DM_Motor_Type.DM4340,
        slave_id=0x20,
        master_id=0x10,
        kp_hold=180,
        kd_hold=4.0,
        kp_move=260,
        kd_move=5.0,
        min_offset=-math.radians(95),
        max_offset=math.radians(95),
    ),
    JointSpec(
        name="elbow",
        motor_type=DM_Motor_Type.DM4340,
        slave_id=0x21,
        master_id=0x11,
        kp_hold=180,
        kd_hold=4.0,
        kp_move=330,
        kd_move=5.0,
        min_offset=-math.radians(15),
        max_offset=math.radians(100),
    ),
    JointSpec(
        name="arm_roll",
        motor_type=DM_Motor_Type.DM4340,
        slave_id=0x22,
        master_id=0x12,
        kp_hold=180,
        kd_hold=4.0,
        kp_move=300,
        kd_move=5.0,
        min_offset=-math.radians(95),
        max_offset=math.radians(95),
    ),
    JointSpec(
        name="wrist",
        motor_type=DM_Motor_Type.DM4310,
        slave_id=0x29,
        master_id=0x19,
        kp_hold=35,
        kd_hold=1.2,
        kp_move=55,
        kd_move=1.8,
        min_offset=-math.radians(90),
        max_offset=math.radians(90),
    ),
]


# Current measured claw constants.
CLAW_SLAVE_ID = 0x28
CLAW_MASTER_ID = 0x18
CLAW_Q_OPEN = -3.88056
CLAW_OPEN_TOL = 0.08
CLAW_CLOSE_OFFSET = 14.0
CLAW_CONTACT_TAU_THRESHOLD = 0.30
CLAW_STALL_TAU_THRESHOLD = 0.24
CLAW_STALL_VEL_THRESHOLD = 0.08
CLAW_BACKOFF = 0.04


# Post-shortening table clearance measurements.
READY_OFFSETS = {
    "shoulder_front": -math.radians(80),  # arm body clears table
    "shoulder_side": 0.0,
    "shoulder_rotate": 0.0,
    "elbow": 0.0,
    "arm_roll": 0.0,
    "wrist": 0.0,
}

PREGRASP_OFFSETS = {
    "shoulder_front": -math.radians(90),  # wrist can point down for tabletop grasp
    "shoulder_side": 0.0,
    "shoulder_rotate": 0.0,
    "elbow": math.radians(42),
    "arm_roll": 0.0,
    "wrist": 0.0,
}


# Coarse tabletop IK constants. This pass maps the horizontal target distance
# to shoulder_front, and arm_y to shoulder_side. It is intentionally simple
# until the full link geometry is measured.
ARM_Z_TABLE_GRASP_CM = 42.0
ARM_Z_TABLE_CLEAR_CM = 55.0
ARM_Z_HIGH_APPROACH_CM = 70.0

TARGET_DISTANCE_NEAR_CM = 35.0
TARGET_DISTANCE_FAR_CM = 55.0
SHOULDER_FRONT_NEAR = -math.radians(50)
SHOULDER_FRONT_FAR = -math.radians(90)

SHOULDER_FRONT_TABLE_GRASP = -math.radians(90)
SHOULDER_FRONT_TABLE_CLEAR = -math.radians(80)
SHOULDER_FRONT_HIGH_APPROACH = -math.radians(60)

# Empirical first pass for front/back random placement. The current successful
# tabletop grasp baseline was around arm_x=8.8 cm. Keep this deliberately
# small until more random-position samples are measured.
ARM_X_BASELINE_CM = 8.8
ARM_X_CM_PER_DEG = 1.0
ARM_X_FRONT_LIMIT_DEG = 5.0
ARM_X_ELBOW_LIMIT_DEG = 0.0
ARM_X_ELBOW_GAIN = 0.0
ARM_X_WRIST_CM_PER_DEG = 0.45
ARM_X_WRIST_LIMIT_DEG = 8.0

ARM_Y_CENTER_CM = 40.6
# Vision targets that land left/right of center are currently converted with a
# conservative gain. 1.2 cm/deg was still too inward, while 1.5 cm/deg was
# slightly too outward in the first RGB-refined random-position tests.
ARM_Y_CM_PER_DEG = 1.4
ARM_Y_NEAR_INWARD_BIAS_DEG = -2.0
ARM_Y_FAR_INWARD_BIAS_DEG = -10.0
ARM_X_ADAPT_NEAR_CM = 8.0
ARM_X_ADAPT_FAR_CM = 12.0
SHOULDER_SIDE_TEST_LIMIT_DEG = 25.0

WRIST_DOWN_TEST_DEG = -50.0
WRIST_NEAR_EXTRA_DOWN_BIAS_DEG = 12.0
WRIST_FAR_EXTRA_DOWN_BIAS_DEG = 22.0
WRIST_FAR_X_UP_START_CM = 14.0
WRIST_FAR_X_UP_FULL_CM = 18.0
WRIST_FAR_X_UP_MAX_DEG = 10.0
WRIST_LOW_Z_START_CM = 40.0
WRIST_LOW_Z_EXTRA_MAX_DEG = 18.0
WRIST_LOW_Z_EXTRA_DEG_PER_CM = 1.8
ELBOW_LOW_Z_START_CM = 40.0
ELBOW_LOW_Z_UP_MAX_DEG = 10.0
ELBOW_LOW_Z_UP_DEG_PER_CM = 1.2
MID_LOW_Z_MIN_CM = 34.0
MID_LOW_Z_MAX_CM = 40.0
MID_LOW_FRONT_UP_DEG = 3.0
MID_LOW_ELBOW_DOWN_DEG = 4.0
MID_LOW_WRIST_DOWN_DEG = 2.0
ELBOW_NEAR_EXTRA_DOWN_BIAS_DEG = 0.0
ELBOW_FAR_EXTRA_DOWN_BIAS_DEG = 9.0
LIFT_SHOULDER_FRONT_DEG = 0.0
LIFT_ELBOW_DEG = 22.0
LIFT_WRIST_UP_DEG = 10.0



# Safe staged approach for tabletop targets. Do not move every joint at once.
# The first movement must retract shoulder_front backward before the elbow
# folds forward, otherwise the wrist can sweep into the table.
TABLE_CLEARANCE_OFFSETS = {
    "shoulder_front": math.radians(30),
    "shoulder_side": 0.0,
    "shoulder_rotate": 0.0,
    "elbow": 0.0,
    "arm_roll": 0.0,
    "wrist": 0.0,
}

ELBOW_SAFE_OFFSETS = {
    "shoulder_front": math.radians(30),
    "shoulder_side": 0.0,
    "shoulder_rotate": 0.0,
    "elbow": math.radians(100),
    "arm_roll": 0.0,
    "wrist": 0.0,
}

def smoothstep(a: float) -> float:
    a = max(0.0, min(1.0, a))
    return 0.5 - 0.5 * math.cos(math.pi * a)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def lerp(a: float, b: float, t: float) -> float:
    return a * (1.0 - t) + b * t


class LeftArmController:
    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD) -> None:
        self.serial = serial.Serial(port, baud, timeout=1.0)
        self.ctrl = MotorControl(self.serial)
        self.specs: Dict[str, JointSpec] = {spec.name: spec for spec in JOINTS}
        self.motors: Dict[str, Motor] = {}

        for spec in JOINTS:
            motor = Motor(spec.motor_type, spec.slave_id, spec.master_id)
            self.ctrl.addMotor(motor)
            self.motors[spec.name] = motor

        self.claw = Motor(DM_Motor_Type.DM4310, CLAW_SLAVE_ID, CLAW_MASTER_ID)
        self.ctrl.addMotor(self.claw)

    def enable_all(self) -> None:
        for name in self.specs:
            print("enable", name)
            self.ctrl.enable(self.motors[name])
            time.sleep(0.12)

        print("enable claw")
        self.ctrl.enable(self.claw)
        time.sleep(0.12)

    def refresh_all(self) -> None:
        for motor in self.motors.values():
            self.ctrl.refresh_motor_status(motor)
        self.ctrl.refresh_motor_status(self.claw)
        time.sleep(0.03)
        self.ctrl.recv()

    def read_status(self) -> Dict[str, Dict[str, float]]:
        self.refresh_all()
        status: Dict[str, Dict[str, float]] = {}
        for name, motor in self.motors.items():
            status[name] = {
                "pos": float(motor.getPosition()),
                "vel": float(motor.getVelocity()),
                "tau": float(motor.getTorque()),
            }

        status["claw"] = {
            "pos": float(self.claw.getPosition()),
            "vel": float(self.claw.getVelocity()),
            "tau": float(self.claw.getTorque()),
        }
        return status

    def print_status(self) -> None:
        status = self.read_status()
        for name, data in status.items():
            print(
                name,
                "pos=", data["pos"],
                "vel=", data["vel"],
                "tau=", data["tau"],
            )

    def capture_home(self, path: str = CONFIG_PATH) -> Dict[str, float]:
        status = self.read_status()
        home = {name: status[name]["pos"] for name in self.specs}
        payload = {
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "home": home,
            "notes": "Captured from current physical arm pose.",
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        print("saved home:", path)
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return home

    def load_home(self, path: str = CONFIG_PATH) -> Dict[str, float]:
        with open(path, "r", encoding="utf-8") as f:
            payload = json.load(f)
        home = payload["home"]
        missing = sorted(set(self.specs) - set(home))
        if missing:
            raise RuntimeError("home file missing joints: " + ", ".join(missing))
        return {name: float(home[name]) for name in self.specs}

    def targets_from_offsets(
        self, home: Dict[str, float], offsets: Dict[str, float]
    ) -> Dict[str, float]:
        targets = {}
        for name, spec in self.specs.items():
            offset = offsets.get(name, 0.0)
            safe_offset = clamp(offset, spec.min_offset, spec.max_offset)
            if abs(safe_offset - offset) > 1e-9:
                print("limit clamp", name, "requested", offset, "used", safe_offset)
            targets[name] = home[name] + safe_offset
        return targets

    def check_targets(self, home: Dict[str, float], targets: Dict[str, float]) -> None:
        eps = 1e-6
        for name, target in targets.items():
            spec = self.specs[name]
            offset = target - home[name]
            if offset < spec.min_offset - eps or offset > spec.max_offset + eps:
                raise ValueError(
                    f"{name} target outside safe limits: "
                    f"offset={offset:.4f}, "
                    f"limit=[{spec.min_offset:.4f}, {spec.max_offset:.4f}]"
                )

    def command_targets(
        self,
        targets: Dict[str, float],
        active: Optional[object] = None,
        hold_scale: float = 1.0,
    ) -> None:
        if active is None:
            active_names = set()
        elif isinstance(active, str):
            active_names = {active}
        else:
            active_names = set(active)

        for name, motor in self.motors.items():
            spec = self.specs[name]
            target = targets[name]
            if name in active_names:
                kp, kd = spec.kp_move, spec.kd_move
            else:
                kp, kd = spec.kp_hold * hold_scale, spec.kd_hold
            self.ctrl.controlMIT(motor, kp, kd, target, 0, 0)

    def hold_pose(self, targets: Dict[str, float], seconds: float) -> None:
        start = time.time()
        while time.time() - start < seconds:
            self.command_targets(targets)
            time.sleep(0.01)

    def move_pose(
        self,
        targets: Dict[str, float],
        seconds: float = 4.0,
        active: Optional[str] = None,
    ) -> None:
        current = self.read_status()
        starts = {name: current[name]["pos"] for name in self.specs}

        start_time = time.time()
        while time.time() - start_time < seconds:
            a = (time.time() - start_time) / seconds
            s = smoothstep(a)
            step_targets = {}
            for name in self.specs:
                step_targets[name] = starts[name] * (1.0 - s) + targets[name] * s
            self.command_targets(step_targets, active=active)
            time.sleep(0.01)

        self.hold_pose(targets, 0.8)

    def move_pose_with_claw_hold(
        self,
        targets: Dict[str, float],
        claw_hold_pos: float,
        seconds: float = 3.0,
        active: Optional[object] = None,
    ) -> None:
        current = self.read_status()
        starts = {name: current[name]["pos"] for name in self.specs}

        start_time = time.time()
        while time.time() - start_time < seconds:
            a = (time.time() - start_time) / seconds
            s = smoothstep(a)
            step_targets = {}
            for name in self.specs:
                step_targets[name] = starts[name] * (1.0 - s) + targets[name] * s
            self.command_targets(step_targets, active=active)
            self.ctrl.controlMIT(self.claw, 18, 0.8, claw_hold_pos, 0, 0)
            time.sleep(0.01)

        start_hold = time.time()
        while time.time() - start_hold < 1.0:
            self.command_targets(targets, active=active)
            self.ctrl.controlMIT(self.claw, 18, 0.8, claw_hold_pos, 0, 0)
            time.sleep(0.01)

    def move_joint_slow(
        self,
        home: Dict[str, float],
        joint: str,
        target: float,
        seconds: float = 4.0,
    ) -> None:
        if joint not in self.specs:
            raise ValueError("unknown joint: " + joint)
        targets = self.read_position_targets()
        targets[joint] = target
        self.check_targets(home, targets)
        self.move_pose(targets, seconds=seconds, active=joint)

    def read_position_targets(self) -> Dict[str, float]:
        status = self.read_status()
        return {name: status[name]["pos"] for name in self.specs}

    def go_home(self, home: Dict[str, float], seconds: float = 4.0) -> None:
        self.check_targets(home, home)
        self.move_home_safe(home, seconds=seconds)

    def move_home_safe(self, home: Dict[str, float], seconds: float = 4.0) -> None:
        print("safe home step 0: open claw first")
        self.open_claw_safe()

        current = self.read_position_targets()
        wrist_home_first = dict(current)
        wrist_home_first["wrist"] = home["wrist"]
        self.check_targets(home, wrist_home_first)

        print("safe home step 1: wrist back home first")
        self.move_pose(
            wrist_home_first,
            seconds=max(2.0, seconds * 0.6),
            active="wrist",
        )

        side_home_first = dict(wrist_home_first)
        side_home_first["shoulder_side"] = home["shoulder_side"]
        self.check_targets(home, side_home_first)

        print("safe home step 2: shoulder_side back home")
        self.move_pose(
            side_home_first,
            seconds=max(3.0, seconds * 0.8),
            active="shoulder_side",
        )

        elbow_safe_next = dict(side_home_first)
        elbow_safe_next["elbow"] = home["elbow"] + ELBOW_SAFE_OFFSETS["elbow"]
        self.check_targets(home, elbow_safe_next)

        print("safe home step 3: elbow back to safe angle")
        self.move_pose(
            elbow_safe_next,
            seconds=max(3.0, seconds * 0.8),
            active="elbow",
        )

        shoulder_home_next = dict(elbow_safe_next)
        shoulder_home_next["shoulder_front"] = (
            home["shoulder_front"] + TABLE_CLEARANCE_OFFSETS["shoulder_front"]
        )
        self.check_targets(home, shoulder_home_next)

        print("safe home step 4: shoulder_front back to safe angle while elbow stays safe")
        self.move_pose(
            shoulder_home_next,
            seconds=max(3.5, seconds),
            active="shoulder_front",
        )

        print("safe home step 5: elbow back home")
        elbow_home_next = dict(shoulder_home_next)
        elbow_home_next["elbow"] = home["elbow"]
        self.check_targets(home, elbow_home_next)
        self.move_pose(
            elbow_home_next,
            seconds=max(3.0, seconds * 0.8),
            active="elbow",
        )

        print("safe home step 6: shoulder_front back home")
        self.move_pose(
            home,
            seconds=max(3.5, seconds),
            active="shoulder_front",
        )

    def go_ready(self, home: Dict[str, float], seconds: float = 4.0) -> Dict[str, float]:
        targets = self.targets_from_offsets(home, READY_OFFSETS)
        self.check_targets(home, targets)
        self.move_pose(targets, seconds=seconds, active="shoulder_front")
        return targets

    def go_pregrasp(
        self, home: Dict[str, float], seconds: float = 4.0
    ) -> Dict[str, float]:
        targets = self.targets_from_offsets(home, PREGRASP_OFFSETS)
        self.check_targets(home, targets)
        self.move_pregrasp_safe(home, final_targets=targets, seconds=seconds)
        return targets

    def move_pregrasp_safe(
        self,
        home: Dict[str, float],
        final_targets: Optional[Dict[str, float]] = None,
        seconds: float = 4.0,
        skip_claw: bool = False,
    ) -> Dict[str, float]:
        if final_targets is None:
            final_targets = self.targets_from_offsets(home, PREGRASP_OFFSETS)
        self.check_targets(home, final_targets)

        table_clearance = self.targets_from_offsets(home, TABLE_CLEARANCE_OFFSETS)
        elbow_safe = self.targets_from_offsets(home, ELBOW_SAFE_OFFSETS)
        shoulder_grasp = dict(elbow_safe)
        shoulder_grasp["shoulder_front"] = final_targets["shoulder_front"]
        self.check_targets(home, table_clearance)
        self.check_targets(home, elbow_safe)
        self.check_targets(home, shoulder_grasp)

        print("safe path step 1: shoulder_front backward only")
        self.move_pose(
            table_clearance,
            seconds=max(3.5, seconds),
            active="shoulder_front",
        )

        print("safe path step 2: elbow forward only; other joints hold")
        self.move_pose(
            elbow_safe,
            seconds=max(3.0, seconds * 0.8),
            active="elbow",
        )

        print("safe path step 3: shoulder_front to planned grasp angle; elbow holds")
        self.move_pose(
            shoulder_grasp,
            seconds=max(4.0, seconds),
            active="shoulder_front",
        )

        elbow_planned = dict(shoulder_grasp)
        elbow_planned["elbow"] = final_targets["elbow"]
        self.check_targets(home, elbow_planned)

        print("safe path step 4: elbow to planned grasp angle; shoulder holds")
        self.move_pose(
            elbow_planned,
            seconds=max(3.0, seconds * 0.8),
            active="elbow",
        )

        side_planned = dict(elbow_planned)
        side_planned["shoulder_side"] = final_targets["shoulder_side"]
        self.check_targets(home, side_planned)

        print("safe path step 5: shoulder_side to planned lateral angle")
        self.move_pose(
            side_planned,
            seconds=max(3.0, seconds * 0.8),
            active="shoulder_side",
        )

        wrist_planned = dict(side_planned)
        wrist_planned["wrist"] = final_targets.get(
            "wrist",
            home["wrist"] + math.radians(WRIST_DOWN_TEST_DEG),
        )
        self.check_targets(home, wrist_planned)

        print("safe path step 6: wrist down test angle")
        self.move_pose(
            wrist_planned,
            seconds=max(2.5, seconds * 0.6),
            active="wrist",
        )

        if skip_claw:
            print("safe path stopped after six motions. Claw close is skipped.")
            return wrist_planned

        print("safe path step 7: claw close until pressure/contact")
        claw_hold_pos = self.close_claw_pressure()

        lift_planned = dict(wrist_planned)
        lift_planned["shoulder_front"] = (
            lift_planned["shoulder_front"] + math.radians(LIFT_SHOULDER_FRONT_DEG)
        )
        lift_planned["elbow"] = lift_planned["elbow"] + math.radians(LIFT_ELBOW_DEG)
        lift_planned["wrist"] = lift_planned["wrist"] + math.radians(LIFT_WRIST_UP_DEG)
        self.check_targets(home, lift_planned)

        print("safe path step 8: lift grasped object about 5cm")
        self.move_pose_with_claw_hold(
            lift_planned,
            claw_hold_pos=claw_hold_pos,
            seconds=3.0,
            active=("shoulder_front", "elbow", "wrist"),
        )

        print("safe path stopped after eight motions. Object should be lifted.")
        return lift_planned

    def open_claw_safe(self) -> None:
        status = self.read_status()["claw"]
        pos = status["pos"]
        err = pos - CLAW_Q_OPEN
        print("claw current", status)
        print("claw Q_OPEN", CLAW_Q_OPEN, "err", err)
        if abs(err) <= CLAW_OPEN_TOL:
            print("claw already open")
            return

        start_pos = pos
        seconds = 4.0
        start_time = time.time()
        while time.time() - start_time < seconds:
            a = (time.time() - start_time) / seconds
            s = smoothstep(a)
            target = start_pos * (1.0 - s) + CLAW_Q_OPEN * s
            self.ctrl.controlMIT(self.claw, 22, 0.8, target, 0, 0)
            time.sleep(0.01)

        for _ in range(100):
            self.ctrl.controlMIT(self.claw, 14, 0.7, CLAW_Q_OPEN, 0, 0)
            time.sleep(0.01)

        print("claw opened", self.read_status()["claw"])

    def close_claw_pressure(self) -> float:
        status = self.read_status()["claw"]
        start_pos = status["pos"]
        print("claw pressure close start", status)
        print("claw pressure close mode: keep closing until contact or Ctrl+C")

        contact_pos: Optional[float] = None
        close_rate = 0.7  # rad/s target advance; slow enough for pressure stop.
        start_time = time.time()
        target = start_pos
        last_print = 0.0

        try:
            while True:
                elapsed = time.time() - start_time
                target = start_pos + close_rate * elapsed
                self.ctrl.controlMIT(self.claw, 26, 0.9, target, 0, 0)

                now = time.time()
                if now - last_print > 0.35:
                    self.ctrl.refresh_motor_status(self.claw)
                    time.sleep(0.01)
                    self.ctrl.recv()
                    pos = float(self.claw.getPosition())
                    vel = float(self.claw.getVelocity())
                    tau = float(self.claw.getTorque())
                    print("claw closing", "pos=", pos, "vel=", vel, "tau=", tau, "target=", target)
                    last_print = now

                    tau_contact = tau >= CLAW_CONTACT_TAU_THRESHOLD
                    stall_contact = (
                        tau >= CLAW_STALL_TAU_THRESHOLD
                        and abs(vel) <= CLAW_STALL_VEL_THRESHOLD
                        and target - pos > 0.08
                    )
                    if tau_contact or stall_contact:
                        contact_pos = pos
                        print("claw contact detected", "pos=", pos, "vel=", vel, "tau=", tau)
                        break

                time.sleep(0.01)
        except KeyboardInterrupt:
            self.ctrl.refresh_motor_status(self.claw)
            time.sleep(0.02)
            self.ctrl.recv()
            contact_pos = float(self.claw.getPosition())
            print("claw close interrupted, hold current pos", contact_pos)

        hold_pos = contact_pos - CLAW_BACKOFF
        print("claw backoff hold pos", hold_pos)
        for _ in range(120):
            self.ctrl.controlMIT(self.claw, 18, 0.8, hold_pos, 0, 0)
            time.sleep(0.01)

        print("claw final", self.read_status()["claw"])
        return hold_pos

    def target_from_arm_xyz(
        self,
        home: Dict[str, float],
        arm_x_cm: float,
        arm_y_cm: float,
        arm_z_cm: float,
    ) -> Dict[str, float]:
        """Map arm-space target to a conservative coarse pre-grasp pose.

        This first pass only uses arm_z to choose shoulder_front. Lateral and
        reach components are intentionally not executed until the vertical
        approach path is proven safe on the physical arm.
        """
        print("received arm target cm:", arm_x_cm, arm_y_cm, arm_z_cm)
        offsets = dict(PREGRASP_OFFSETS)
        horizontal_distance_cm = math.sqrt(arm_x_cm * arm_x_cm + arm_y_cm * arm_y_cm)

        if horizontal_distance_cm <= TARGET_DISTANCE_NEAR_CM:
            shoulder_front = SHOULDER_FRONT_NEAR
            zone = "near_distance_clamped"
        elif horizontal_distance_cm >= TARGET_DISTANCE_FAR_CM:
            shoulder_front = SHOULDER_FRONT_FAR
            zone = "far_distance_clamped"
        else:
            t = (horizontal_distance_cm - TARGET_DISTANCE_NEAR_CM) / (
                TARGET_DISTANCE_FAR_CM - TARGET_DISTANCE_NEAR_CM
            )
            shoulder_front = lerp(
                SHOULDER_FRONT_NEAR,
                SHOULDER_FRONT_FAR,
                t,
            )
            zone = "distance_interp"

        if arm_z_cm > ARM_Z_TABLE_CLEAR_CM:
            if arm_z_cm <= ARM_Z_HIGH_APPROACH_CM:
                t = (arm_z_cm - ARM_Z_TABLE_CLEAR_CM) / (
                    ARM_Z_HIGH_APPROACH_CM - ARM_Z_TABLE_CLEAR_CM
                )
                z_limited_front = lerp(
                    SHOULDER_FRONT_TABLE_CLEAR,
                    SHOULDER_FRONT_HIGH_APPROACH,
                    t,
                )
                shoulder_front = max(shoulder_front, z_limited_front)
                zone += "+z_high_limited"
            else:
                shoulder_front = max(shoulder_front, SHOULDER_FRONT_HIGH_APPROACH)
                zone += "+z_high_clamped"

        reach_delta_deg = clamp(
            -(arm_x_cm - ARM_X_BASELINE_CM) / ARM_X_CM_PER_DEG,
            -ARM_X_FRONT_LIMIT_DEG,
            ARM_X_FRONT_LIMIT_DEG,
        )
        mid_low_z = MID_LOW_Z_MIN_CM <= arm_z_cm < MID_LOW_Z_MAX_CM
        if mid_low_z:
            reach_delta_deg -= MID_LOW_FRONT_UP_DEG
        shoulder_front += math.radians(reach_delta_deg)
        offsets["shoulder_front"] = shoulder_front

        arm_x_adapt_t = clamp(
            (arm_x_cm - ARM_X_ADAPT_NEAR_CM) / (ARM_X_ADAPT_FAR_CM - ARM_X_ADAPT_NEAR_CM),
            0.0,
            1.0,
        )
        side_inward_bias_deg = lerp(
            ARM_Y_NEAR_INWARD_BIAS_DEG,
            ARM_Y_FAR_INWARD_BIAS_DEG,
            arm_x_adapt_t,
        )
        elbow_extra_down_bias_deg = lerp(
            ELBOW_NEAR_EXTRA_DOWN_BIAS_DEG,
            ELBOW_FAR_EXTRA_DOWN_BIAS_DEG,
            arm_x_adapt_t,
        )
        wrist_extra_down_bias_deg = lerp(
            WRIST_NEAR_EXTRA_DOWN_BIAS_DEG,
            WRIST_FAR_EXTRA_DOWN_BIAS_DEG,
            arm_x_adapt_t,
        )

        elbow_reach_delta_deg = clamp(
            reach_delta_deg * ARM_X_ELBOW_GAIN,
            -ARM_X_ELBOW_LIMIT_DEG,
            ARM_X_ELBOW_LIMIT_DEG,
        )
        elbow_up_deg = 0.0
        if arm_z_cm < ELBOW_LOW_Z_START_CM:
            elbow_up_deg = min(
                ELBOW_LOW_Z_UP_MAX_DEG,
                (ELBOW_LOW_Z_START_CM - arm_z_cm) * ELBOW_LOW_Z_UP_DEG_PER_CM,
            )
        elbow_mid_low_down_deg = MID_LOW_ELBOW_DOWN_DEG if mid_low_z else 0.0
        offsets["elbow"] = PREGRASP_OFFSETS["elbow"] + math.radians(
            elbow_reach_delta_deg
            + elbow_up_deg
            - elbow_mid_low_down_deg
            - elbow_extra_down_bias_deg
        )

        wrist_down_deg = WRIST_DOWN_TEST_DEG
        if arm_z_cm < WRIST_LOW_Z_START_CM:
            extra_wrist_down = min(
                WRIST_LOW_Z_EXTRA_MAX_DEG,
                (WRIST_LOW_Z_START_CM - arm_z_cm) * WRIST_LOW_Z_EXTRA_DEG_PER_CM,
            )
            wrist_down_deg -= extra_wrist_down
        wrist_x_delta_deg = clamp(
            (arm_x_cm - ARM_X_BASELINE_CM) / ARM_X_WRIST_CM_PER_DEG,
            -ARM_X_WRIST_LIMIT_DEG,
            ARM_X_WRIST_LIMIT_DEG,
        )
        wrist_down_deg += wrist_x_delta_deg
        if mid_low_z:
            wrist_down_deg -= MID_LOW_WRIST_DOWN_DEG
        wrist_down_deg -= wrist_extra_down_bias_deg
        wrist_far_x_up_t = clamp(
            (arm_x_cm - WRIST_FAR_X_UP_START_CM) / (WRIST_FAR_X_UP_FULL_CM - WRIST_FAR_X_UP_START_CM),
            0.0,
            1.0,
        )
        wrist_far_x_up_deg = wrist_far_x_up_t * WRIST_FAR_X_UP_MAX_DEG
        wrist_down_deg += wrist_far_x_up_deg
        offsets["wrist"] = math.radians(wrist_down_deg)

        side_deg = (arm_y_cm - ARM_Y_CENTER_CM) / ARM_Y_CM_PER_DEG
        side_deg += side_inward_bias_deg
        side_deg = clamp(
            side_deg,
            -SHOULDER_SIDE_TEST_LIMIT_DEG,
            SHOULDER_SIDE_TEST_LIMIT_DEG,
        )
        offsets["shoulder_side"] = math.radians(side_deg)

        print(
            "coarse IK:",
            "zone=", zone,
            "horizontal_distance_cm=", horizontal_distance_cm,
            "shoulder_front_offset_rad=", shoulder_front,
            "shoulder_front_offset_deg=", math.degrees(shoulder_front),
        )
        print(
            "coarse wrist:",
            "arm_z_cm=", arm_z_cm,
            "wrist_x_delta_deg=", wrist_x_delta_deg,
            "wrist_extra_down_bias_deg=", wrist_extra_down_bias_deg,
            "wrist_far_x_up_deg=", wrist_far_x_up_deg,
            "wrist_down_offset_deg=", wrist_down_deg,
        )
        print(
            "coarse reach:",
            "arm_x_baseline_cm=", ARM_X_BASELINE_CM,
            "arm_x_offset_cm=", arm_x_cm - ARM_X_BASELINE_CM,
            "shoulder_front_reach_delta_deg=", reach_delta_deg,
            "elbow_reach_delta_deg=", elbow_reach_delta_deg,
            "elbow_low_z_up_deg=", elbow_up_deg,
            "elbow_mid_low_down_deg=", elbow_mid_low_down_deg,
            "elbow_extra_down_bias_deg=", elbow_extra_down_bias_deg,
        )
        print(
            "coarse lateral:",
            "arm_y_center_cm=", ARM_Y_CENTER_CM,
            "arm_x_adapt_t=", arm_x_adapt_t,
            "arm_y_inward_bias_deg=", side_inward_bias_deg,
            "shoulder_side_offset_deg=", side_deg,
            "limited_to_deg=+/-", SHOULDER_SIDE_TEST_LIMIT_DEG,
        )
        print("arm_x controls wrist plus small reach, arm_z controls wrist/height, arm_y controls shoulder_side.")
        return self.targets_from_offsets(home, offsets)

    def close(self) -> None:
        try:
            self.serial.close()
        except Exception:
            pass


def print_limits() -> None:
    for spec in JOINTS:
        print(
            spec.name,
            "slave=0x%02X" % spec.slave_id,
            "master=0x%02X" % spec.master_id,
            "min_offset=%.3f" % spec.min_offset,
            "max_offset=%.3f" % spec.max_offset,
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="HiveRobot left arm controller")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status")
    sub.add_parser("limits")
    sub.add_parser("capture-home")
    sub.add_parser("home")
    sub.add_parser("ready")
    sub.add_parser("pregrasp")
    sub.add_parser("open-claw")

    move_joint = sub.add_parser("move-joint")
    move_joint.add_argument("--joint", required=True)
    move_joint.add_argument("--target", type=float, required=True)
    move_joint.add_argument("--seconds", type=float, default=4.0)

    target = sub.add_parser("target")
    target.add_argument("--x", type=float, required=True, help="arm_x in cm")
    target.add_argument("--y", type=float, required=True, help="arm_y in cm")
    target.add_argument("--z", type=float, required=True, help="arm_z in cm")
    target.add_argument("--execute", action="store_true")
    target.add_argument("--skip-claw", action="store_true")
    target.add_argument("--front-bias-deg", type=float, default=0.0)
    target.add_argument("--side-bias-deg", type=float, default=0.0)
    target.add_argument("--elbow-bias-deg", type=float, default=0.0)
    target.add_argument("--wrist-bias-deg", type=float, default=0.0)
    target.add_argument("--seconds", type=float, default=4.0)

    return parser


def main() -> None:
    args = build_parser().parse_args()

    if args.cmd == "limits":
        print_limits()
        return

    arm = LeftArmController()
    try:
        print("Serial port is open")
        arm.enable_all()

        if args.cmd == "status":
            arm.print_status()
            return

        if args.cmd == "capture-home":
            arm.capture_home()
            return

        home = arm.load_home()

        if args.cmd == "home":
            arm.go_home(home)
            arm.print_status()
        elif args.cmd == "ready":
            arm.go_ready(home)
            arm.print_status()
        elif args.cmd == "pregrasp":
            arm.go_pregrasp(home)
            arm.print_status()
        elif args.cmd == "open-claw":
            arm.open_claw_safe()
        elif args.cmd == "move-joint":
            arm.move_joint_slow(home, args.joint, args.target, args.seconds)
            arm.print_status()
        elif args.cmd == "target":
            targets = arm.target_from_arm_xyz(home, args.x, args.y, args.z)
            bias_map = {
                "shoulder_front": args.front_bias_deg,
                "shoulder_side": args.side_bias_deg,
                "elbow": args.elbow_bias_deg,
                "wrist": args.wrist_bias_deg,
            }
            active_biases = {
                name: deg for name, deg in bias_map.items() if abs(deg) > 1e-9
            }
            if active_biases:
                print("manual target biases deg:", active_biases)
                for name, deg in active_biases.items():
                    targets[name] += math.radians(deg)
                arm.check_targets(home, targets)
            print("planned targets:")
            print(json.dumps(targets, ensure_ascii=False, indent=2))
            if args.execute:
                arm.move_pregrasp_safe(
                    home,
                    final_targets=targets,
                    seconds=args.seconds,
                    skip_claw=args.skip_claw,
                )
                arm.print_status()
            else:
                print("dry run only. Add --execute to move.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
