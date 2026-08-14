#!/usr/bin/env python3
"""Clean left-arm bring-up controller v2.5.3.

This v2 controller intentionally does not import or reuse the old
left_arm_controller / teach_left_arm motion stack. It is for post-reassembly
bring-up: read status, low-gain holds, capture a new home, small nudges, and
low-gain home moves.

v2.5.3 keeps the stable v2.3 clearance/home motion layout and starts tuning
the shoulder_front hold feedforward to reduce sag during later phases.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple

import serial
from DM_CAN import DM_Motor_Type, Motor, MotorControl


SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 921600
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HOME_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_v2_home.json")
TABLE_CLEARANCE_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_v2_table_clearance.json")
CLAW_HOME_PATH = os.path.join(SCRIPT_DIR, "data", "left_arm_v2_claw_home.json")


@dataclass(frozen=True)
class JointSpec:
    name: str
    motor_type: DM_Motor_Type
    slave_id: int
    master_id: int


JOINTS = [
    JointSpec("shoulder_front", DM_Motor_Type.DM4340, 0x1E, 0x0E),
    JointSpec("shoulder_side", DM_Motor_Type.DM4340, 0x1F, 0x0F),
    JointSpec("shoulder_rotate", DM_Motor_Type.DM4310, 0x20, 0x10),
    JointSpec("elbow", DM_Motor_Type.DM4340, 0x21, 0x11),
    JointSpec("arm_roll", DM_Motor_Type.DM4310, 0x22, 0x12),
    JointSpec("wrist_side", DM_Motor_Type.DM4310, 0x2A, 0x1A),
    JointSpec("wrist", DM_Motor_Type.DM4310, 0x29, 0x19),
]
CLAW_SPEC = JointSpec("claw", DM_Motor_Type.DM4310, 0x28, 0x18)
ALL_MOTOR_SPECS = JOINTS + [CLAW_SPEC]

DEFAULT_JOINTS = [spec.name for spec in JOINTS]
DEFAULT_HOME_ORDER = [
    "elbow",
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
    "wrist",
    "wrist_side",
    "arm_roll",
]

DEFAULT_CLEARANCE_ORDER = [
    "shoulder_front",
    "shoulder_side",
    "elbow",
    "shoulder_rotate",
    "arm_roll",
    "wrist_side",
    "wrist",
]

PREHOME_CLEARANCE_ORDER = [
    "wrist",
    "wrist_side",
    "arm_roll",
    "elbow",
    "shoulder_rotate",
    "shoulder_side",
    "shoulder_front",
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
    "wrist_side": {"kp": 16.0, "kd": 2.2, "seconds": 6.0},
    "elbow": {"kp": 70.0, "kd": 3.0, "seconds": 6.0},
    "arm_roll": {"kp": 18.0, "kd": 6.0, "seconds": 12.0},
    "shoulder_front": {"kp": 90.0, "kd": 4.0, "seconds": 6.0},
    "shoulder_side": {"kp": 70.0, "kd": 3.0, "seconds": 6.0},
    "shoulder_rotate": {"kp": 24.0, "kd": 4.5, "seconds": 8.0},
}

HOME_HOLD_GAINS = {
    "wrist": {"kp": 18.0, "kd": 1.4},
    "wrist_side": {"kp": 18.0, "kd": 1.4},
    "arm_roll": {"kp": 32.0, "kd": 3.0},
    "elbow": {"kp": 65.0, "kd": 4.5},
    "shoulder_front": {"kp": 75.0, "kd": 5.0},
    "shoulder_side": {"kp": 65.0, "kd": 4.5},
    "shoulder_rotate": {"kp": 24.0, "kd": 4.0},
}

HOME_BASE_HOLD_GAINS = {
    "wrist": {"kp": 6.0, "kd": 0.8},
    "wrist_side": {"kp": 6.0, "kd": 0.8},
    "arm_roll": {"kp": 10.0, "kd": 2.0},
    "elbow": {"kp": 20.0, "kd": 3.0},
    "shoulder_front": {"kp": 25.0, "kd": 4.0},
    "shoulder_side": {"kp": 16.0, "kd": 2.5},
    "shoulder_rotate": {"kp": 10.0, "kd": 2.5},
}

HOME_JOINT_DEADBANDS_DEG = {
    "arm_roll": 3.0,
    "shoulder_rotate": 3.0,
}

HOME_COMPLIANT_HOLD_GAINS = {
    "arm_roll": {"kp": 4.0, "kd": 6.0},
    "shoulder_rotate": {"kp": 6.0, "kd": 5.0},
}

HOME_COMPLIANT_HOLDS_BY_ACTIVE = {
    "wrist": {"arm_roll", "shoulder_rotate"},
    "wrist_side": {"arm_roll", "shoulder_rotate"},
    "shoulder_rotate": {"arm_roll"},
}

COUPLED_HOME_MOVE_GAINS = {
    "elbow": {"kp": 65.0, "kd": 6.5},
    "shoulder_front": {"kp": 75.0, "kd": 7.0},
    "shoulder_side": {"kp": 65.0, "kd": 4.5},
    "arm_roll": {"kp": 18.0, "kd": 6.0},
    "wrist_side": {"kp": 16.0, "kd": 2.2},
    "wrist": {"kp": 22.0, "kd": 1.4},
}

COUPLED_HOME_MAX_SECONDS = 16.0
COUPLED_HOME_CONTROL_DT = 0.01
COUPLED_HOME_TRAJECTORY = "blend_smootherstep"
COUPLED_HOME_LINEAR_BLEND = 0.35
COUPLED_HOME_COMMAND_BIAS_DEG = {
    "shoulder_side": 1.0,
}
COUPLED_HOME_PROGRESS_WINDOWS = {
    "arm_roll": (0.65, 1.0),
    "wrist": (0.35, 1.0),
}
COUPLED_HOME_PRE_WINDOW_GAINS = {
    "wrist": {"kp": 6.0, "kd": 0.8},
}

CLEARANCE_JOINT_DEADBANDS_DEG = {
    "shoulder_rotate": 180.0,
}

CLEARANCE_MOVE_GAINS = {
    "wrist": {"kp": 24.0, "kd": 4.4},
    "wrist_side": {"kp": 22.0, "kd": 1.4},
    "arm_roll": {"kp": 50.0, "kd": 3.0},
    "elbow": {"kp": 95.0, "kd": 4.0},
    "shoulder_front": {"kp": 110.0, "kd": 5.0},
    "shoulder_side": {"kp": 80.0, "kd": 3.5},
    "shoulder_rotate": {"kp": 24.0, "kd": 4.5},
}

CLEARANCE_MOVE_SECONDS = {
    "shoulder_rotate": 8.0,
    "wrist": 24.0,
}

CLEARANCE_CONTINUOUS_JOINTS = {
    "wrist",
}

CLEARANCE_VELOCITY_FF_JOINTS = {
    "wrist",
}

COUPLED_CLEARANCE_MOVE_GAINS = {
    "shoulder_front": {"kp": 124.0, "kd": 4.8},
    "shoulder_side": {"kp": 60.0, "kd": 5.0},
    "elbow": {"kp": 65.0, "kd": 6.5},
    "arm_roll": {"kp": 18.0, "kd": 6.0},
    "wrist_side": {"kp": 16.0, "kd": 2.2},
    "wrist": {"kp": 22.0, "kd": 1.4},
}

COUPLED_CLEARANCE_MAX_SECONDS = 12.0
COUPLED_CLEARANCE_SETTLE_SECONDS = 0.5
COUPLED_CLEARANCE_CONTROL_DT = 0.01
COUPLED_CLEARANCE_TRAJECTORY = "blend_smootherstep"
COUPLED_CLEARANCE_LINEAR_BLEND = 0.35
COUPLED_CLEARANCE_PROGRESS_WINDOWS = {
    "arm_roll": (0.35, 1.0),
    "wrist": (0.0, 1.0),
}

COUPLED_CLEARANCE_PRE_WINDOW_GAINS = {
    "wrist": {"kp": 6.0, "kd": 0.8},
}

COUPLED_CLEARANCE_VELOCITY_FF_JOINTS = {
    "shoulder_front",
}

COUPLED_CLEARANCE_MOVE_TAU_FF = {
    "shoulder_front": 1.0,
}

CLEARANCE_HOLD_GAINS = {
    "wrist": {"kp": 18.0, "kd": 1.4},
    "wrist_side": {"kp": 18.0, "kd": 1.4},
    "arm_roll": {"kp": 26.0, "kd": 5.0},
    "elbow": {"kp": 92.0, "kd": 6.2},
    "shoulder_front": {"kp": 125.0, "kd": 6.5},
    "shoulder_side": {"kp": 70.0, "kd": 5.0},
    "shoulder_rotate": {"kp": 45.0, "kd": 4.5},
}

CLEARANCE_COMPLIANT_HOLD_GAINS = {
    "arm_roll": {"kp": 6.0, "kd": 6.0},
    "wrist_side": {"kp": 8.0, "kd": 1.6},
    "shoulder_rotate": {"kp": 8.0, "kd": 4.0},
}

CLEARANCE_COMPLIANT_HOLDS_BY_ACTIVE = {
    "wrist_side": {"arm_roll"},
    "wrist": {"arm_roll", "wrist_side", "shoulder_rotate"},
}

CLEARANCE_JOINT_HOLD_TAU = {
    "shoulder_front": 3.0,
}

NUDGE_HOLD_TAU = {
    **CLEARANCE_JOINT_HOLD_TAU,
    "shoulder_front": 3.8,
    "elbow": 2.2,
    "wrist": 0.55,
}

NUDGE_WRIST_ACTIVE_LOW_SHOULDER_FRONT_THRESHOLD = 1.8
NUDGE_LOW_SHOULDER_FRONT_HOLD_TAU = 1.2
NUDGE_LOW_SHOULDER_WRIST_SIDE_HOLD_TAU = -0.35
NUDGE_LOW_SHOULDER_GAINS = {
    "wrist_side": {"kp": 24.0, "kd": 2.4},
}
NUDGE_LOW_SHOULDER_MIN_SECONDS = {
    "wrist_side": 4.5,
}
NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_DEADBAND_DEG = 0.8
NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_MAX_BIAS_DEG = 1.2
NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_GAINS = {
    "kp": 24.0,
    "kd": 2.8,
    "seconds": 2.0,
}
NUDGE_LOW_SHOULDER_WRIST_HOLD_GAINS = {
    "shoulder_rotate": {"kp": 24.0, "kd": 4.5},
    "shoulder_side": {"kp": 65.0, "kd": 4.5},
    "arm_roll": {"kp": 12.0, "kd": 7.0},
}
NUDGE_LOW_SHOULDER_WRIST_HOLD_TAU = {
    "arm_roll": -0.55,
    "shoulder_rotate": -0.8,
    "shoulder_side": 0.7,
}
NUDGE_LOW_SHOULDER_ARM_ROLL_ACTIVE_TAU = -0.65
NUDGE_LOW_SHOULDER_ARM_ROLL_HOLD_TAU = {
    "elbow": 5.1,
    "shoulder_rotate": -0.8,
    "wrist_side": -0.25,
}

NUDGE_ACTIVE_TAU = {
    "shoulder_front": 2.4,
    "wrist": 0.55,
}

NUDGE_PRELOAD_SECONDS = {
    "shoulder_front": 0.8,
}

NUDGE_TRAJECTORY = {
    "shoulder_front": "blend_smootherstep",
}

NUDGE_LINEAR_BLEND = {
    "shoulder_front": 0.35,
}

NUDGE_STEP_DEG = {}


NUDGE_MIN_SECONDS = {
    "shoulder_front": 2.2,
    "arm_roll": 1.2,
}

PREHOME_CLEARANCE_GAINS = {
    "wrist_side": {"kp": 16.0, "kd": 2.2},
    "arm_roll": {"kp": 12.0, "kd": 7.0},
}

PREHOME_CLEARANCE_BASE_SECONDS = {
    "wrist_side": 6.0,
    "arm_roll": 8.0,
}

PREHOME_CLEARANCE_SECONDS_PER_DEG = {
    "wrist_side": 6.0 / 5.0,
    "arm_roll": 8.0 / 10.0,
}

PREHOME_CLEARANCE_MAX_SECONDS = {
    "wrist_side": 20.0,
    "arm_roll": 20.0,
}

NUDGE_COMPLIANT_HOLD_GAINS = {
    "arm_roll": {"kp": 4.0, "kd": 6.0},
    "shoulder_rotate": {"kp": 6.0, "kd": 5.0},
    "wrist_side": {"kp": 6.0, "kd": 1.2},
    "wrist": {"kp": 8.0, "kd": 1.4},
}

NUDGE_COMPLIANT_HOLD_GAINS_BY_ACTIVE = {
    "shoulder_front": {
        "arm_roll": {"kp": 2.5, "kd": 7.0},
        "shoulder_rotate": {"kp": 4.0, "kd": 6.0},
        "wrist_side": {"kp": 4.0, "kd": 1.6},
        "wrist": {"kp": 6.0, "kd": 2.0},
    },
    "shoulder_side": {
        "arm_roll": {"kp": 2.5, "kd": 7.0},
        "shoulder_rotate": {"kp": 4.0, "kd": 6.0},
        "wrist_side": {"kp": 4.0, "kd": 1.6},
        "wrist": {"kp": 6.0, "kd": 2.0},
    },
    "elbow": {
        "arm_roll": {"kp": 2.5, "kd": 7.0},
        "shoulder_rotate": {"kp": 4.0, "kd": 6.0},
        "wrist_side": {"kp": 4.0, "kd": 1.6},
        "wrist": {"kp": 6.0, "kd": 2.0},
    },
    "arm_roll": {
        "shoulder_rotate": {"kp": 4.0, "kd": 6.0},
        "wrist_side": {"kp": 4.0, "kd": 1.6},
        "wrist": {"kp": 6.0, "kd": 2.0},
    },
    "wrist": {
        "arm_roll": {"kp": 8.0, "kd": 6.0},
    },
}

NUDGE_COMPLIANT_HOLDS_BY_ACTIVE = {
    "shoulder_front": {"arm_roll", "shoulder_rotate", "wrist_side", "wrist"},
    "shoulder_side": {"arm_roll", "shoulder_rotate", "wrist_side", "wrist"},
    "elbow": {"arm_roll", "shoulder_rotate", "wrist_side", "wrist"},
    "arm_roll": {"shoulder_rotate", "wrist_side", "wrist"},
    "shoulder_rotate": {"arm_roll", "wrist_side", "wrist"},
    "wrist": {"arm_roll"},
}

NUDGE_LOW_SHOULDER_COMPLIANT_HOLD_GAINS_BY_ACTIVE = {
    "shoulder_front": {
        "arm_roll": {"kp": 12.0, "kd": 7.0},
        "shoulder_rotate": {"kp": 12.0, "kd": 5.0},
        "wrist_side": {"kp": 8.0, "kd": 1.6},
    },
    "arm_roll": {
        "shoulder_rotate": {"kp": 12.0, "kd": 5.0},
        "wrist_side": {"kp": 8.0, "kd": 1.6},
        "wrist": {"kp": 6.0, "kd": 2.0},
    },
    "wrist": NUDGE_LOW_SHOULDER_WRIST_HOLD_GAINS,
}

NUDGE_COUPLED_ACTIVE_RATIOS = {}

NUDGE_COUPLED_ACTIVE_GAINS = {}

NUDGE_TARGET_LIMITS = {}


CLEARANCE_WRIST_FINE_DEADBAND_DEG = 1.0
CLEARANCE_WRIST_FINE_MAX_BIAS_DEG = 1.5
CLEARANCE_WRIST_FINE_SETTLE_SECONDS = 1.0
CLEARANCE_WRIST_FINE_ACTIVE_TAU = 0.55
CLEARANCE_WRIST_FINE_CONTROL_DT = 0.01
CLEARANCE_WRIST_FINE_GAINS = {
    "kp": 28.0,
    "kd": 5.0,
    "seconds": 4.0,
}
COUPLED_CLEARANCE_WRIST_FINE_GAINS = {
    "kp": 26.0,
    "kd": 4.7,
    "seconds": 8.0,
}

CLEARANCE_BASE_HOLD_GAINS = {
    "wrist": {"kp": 6.0, "kd": 0.8},
    "wrist_side": {"kp": 6.0, "kd": 0.8},
    "arm_roll": {"kp": 12.0, "kd": 2.0},
    "elbow": {"kp": 25.0, "kd": 3.0},
    "shoulder_front": {"kp": 30.0, "kd": 4.0},
    "shoulder_side": {"kp": 18.0, "kd": 2.5},
    "shoulder_rotate": {"kp": 12.0, "kd": 2.5},
}

NUDGE_GAINS = {
    "wrist": {"kp": 18.0, "kd": 1.2},
    "wrist_side": {"kp": 18.0, "kd": 1.2},
    "arm_roll": {"kp": 12.0, "kd": 7.0},
    "elbow": {"kp": 70.0, "kd": 3.0},
    "shoulder_front": {"kp": 140.0, "kd": 5.2},
    "shoulder_side": {"kp": 70.0, "kd": 3.0},
    "shoulder_rotate": {"kp": 60.0, "kd": 3.0},
}

CLAW_OPEN_TOL = 0.08
CLAW_CLOSE_OFFSET = 6.0
CLAW_MOVE_SECONDS = 4.0
CLAW_CLOSE_SECONDS = 8.0
CLAW_KP_MOVE = 28.0
CLAW_KD_MOVE = 1.0
CLAW_KP_OPEN = 22.0
CLAW_KD_OPEN = 0.8
CLAW_KP_HOLD = 14.0
CLAW_KD_HOLD = 0.7
CLAW_TAU_THRESHOLD = 0.30
CLAW_STALL_TAU_THRESHOLD = 0.24
CLAW_VEL_STALL_THRESHOLD = 0.08
CLAW_CONFIRM_COUNT_NEEDED = 5
CLAW_BACKOFF = 0.04
CLAW_HOLD_TRACE_INTERVAL = 1.0


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def smootherstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def smootherstep_derivative(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return 30.0 * t * t * (1.0 - t) * (1.0 - t)


def blend_smootherstep(t: float, linear_blend: float = 0.15) -> float:
    t = max(0.0, min(1.0, t))
    linear_blend = max(0.0, min(1.0, linear_blend))
    return linear_blend * t + (1.0 - linear_blend) * smootherstep(t)


def blend_smootherstep_derivative(t: float, linear_blend: float = 0.15) -> float:
    t = max(0.0, min(1.0, t))
    linear_blend = max(0.0, min(1.0, linear_blend))
    return linear_blend + (1.0 - linear_blend) * smootherstep_derivative(t)


def cosine_smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return 0.5 - 0.5 * math.cos(math.pi * t)


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
        self.specs = {spec.name: spec for spec in ALL_MOTOR_SPECS}
        self.motors: Dict[str, Motor] = {}
        self.enabled: set[str] = set()
        for spec in ALL_MOTOR_SPECS:
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
            if name in self.enabled:
                print("enable skip", name)
                continue
            print("enable", name)
            self.ctrl.enable(self.motors[name])
            self.enabled.add(name)
            time.sleep(0.05)

    def disable(self, names: Iterable[str]) -> None:
        disable_fn = (
            getattr(self.ctrl, "disable", None)
            or getattr(self.ctrl, "disableMotor", None)
            or getattr(self.ctrl, "disable_motor", None)
        )
        if disable_fn is None:
            raise RuntimeError("MotorControl does not expose disable/disableMotor/disable_motor; cannot restart motors safely.")
        for name in self.validate(names):
            print("disable", name)
            disable_fn(self.motors[name])
            self.enabled.discard(name)
            time.sleep(0.05)

    def restart_motors(
        self,
        names: Iterable[str],
        off_seconds: float,
        hold_seconds: float,
        hold_kp: float,
        hold_kd: float,
    ) -> None:
        names = self.validate(names)
        print("v2.5.3 restart before:")
        self.print_status(names)
        self.disable(names)
        time.sleep(off_seconds)
        self.enable(names)
        time.sleep(0.1)
        if hold_seconds > 0:
            targets = self.positions(names)
            print("v2.5.3 restart hold current positions seconds=", hold_seconds, "kp=", hold_kp, "kd=", hold_kd)
            self.hold_positions(targets, seconds=hold_seconds, kp=hold_kp, kd=hold_kd)
        print("v2.5.3 restart after:")
        self.print_status(names)

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

    def hold_positions_with_gains(
        self,
        targets: Dict[str, float],
        seconds: float,
        gains: Dict[str, Dict[str, float]],
        fallback_kp: float,
        fallback_kd: float,
        hold_tau: Optional[Dict[str, float]] = None,
    ) -> None:
        names = self.validate(targets.keys())
        hold_tau = hold_tau or {}
        end = time.time() + seconds
        while time.time() < end:
            for name in names:
                joint_gains = gains.get(name, {})
                kp = joint_gains.get("kp", fallback_kp)
                kd = joint_gains.get("kd", fallback_kd)
                self.ctrl.controlMIT(self.motors[name], kp, kd, targets[name], 0, hold_tau.get(name, 0.0))
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

    def claw_status(self) -> Dict[str, float]:
        return self.read_status(["claw"])["claw"]

    def claw_hold(self, target: float, seconds: float, kp: float = CLAW_KP_HOLD, kd: float = CLAW_KD_HOLD) -> None:
        start = time.time()
        while time.time() - start < seconds:
            self.ctrl.controlMIT(self.motors["claw"], kp, kd, target, 0, 0)
            time.sleep(0.01)

    def claw_hold_forever(self, target: float, kp: float = CLAW_KP_HOLD, kd: float = CLAW_KD_HOLD) -> None:
        print("claw holding at:", target, "Ctrl+C to stop", flush=True)
        next_trace = time.time()
        while True:
            self.ctrl.controlMIT(self.motors["claw"], kp, kd, target, 0, 0)
            now = time.time()
            if now >= next_trace:
                status = self.claw_status()
                print(
                    "claw holding status=",
                    json.dumps({"target": target, "kp": kp, "kd": kd, **status}, ensure_ascii=False),
                    flush=True,
                )
                next_trace = now + CLAW_HOLD_TRACE_INTERVAL
            time.sleep(0.01)

    def claw_move_to(
        self,
        target: float,
        seconds: float,
        kp: float = CLAW_KP_MOVE,
        kd: float = CLAW_KD_MOVE,
        label: str = "claw move",
    ) -> Dict[str, float]:
        start_pos = self.claw_status()["pos"]
        print(label, "from:", start_pos, "to:", target, "kp=", kp, "kd=", kd, "seconds=", seconds)
        start = time.time()
        while time.time() - start < seconds:
            a = (time.time() - start) / max(seconds, 1e-6)
            s = cosine_smoothstep(a)
            pos_target = start_pos * (1.0 - s) + target * s
            self.ctrl.controlMIT(self.motors["claw"], kp, kd, pos_target, 0, 0)
            time.sleep(0.01)
        self.claw_hold(target, 0.8)
        status = self.claw_status()
        print(label, "target:", target, "pos:", status["pos"], "vel:", status["vel"], "tau:", status["tau"])
        return status

    def claw_home(self, home_pos: float) -> Dict[str, float]:
        self.enable(["claw"])
        return self.claw_move_to(home_pos, CLAW_MOVE_SECONDS, kp=CLAW_KP_OPEN, kd=CLAW_KD_OPEN, label="claw home")

    def claw_close_until_pressure(
        self,
        home_pos: float,
        hold: bool = True,
        close_offset: float = CLAW_CLOSE_OFFSET,
    ) -> Dict[str, object]:
        self.enable(["claw"])
        q_close = home_pos + close_offset
        print("claw close pressure stop")
        print("claw home:", home_pos)
        print("claw close target:", q_close)
        contact = False
        contact_pos = home_pos
        confirm_count = 0
        last_status = self.claw_status()
        start = time.time()
        while time.time() - start < CLAW_CLOSE_SECONDS:
            elapsed = time.time() - start
            a = elapsed / max(CLAW_CLOSE_SECONDS, 1e-6)
            s = cosine_smoothstep(a)
            target = home_pos * (1.0 - s) + q_close * s
            self.ctrl.controlMIT(self.motors["claw"], CLAW_KP_MOVE, CLAW_KD_MOVE, target, 0, 0)
            time.sleep(0.01)
            last_status = self.claw_status()
            pos = last_status["pos"]
            vel = last_status["vel"]
            tau = last_status["tau"]
            if elapsed > 0.5:
                tau_hit = tau > CLAW_TAU_THRESHOLD
                stall_hit = (
                    abs(vel) < CLAW_VEL_STALL_THRESHOLD
                    and abs(target - pos) > 0.15
                    and tau > CLAW_STALL_TAU_THRESHOLD
                )
                if tau_hit or stall_hit:
                    confirm_count += 1
                else:
                    confirm_count = 0
                if confirm_count >= CLAW_CONFIRM_COUNT_NEEDED:
                    contact = True
                    contact_pos = pos
                    print("CLAW CONTACT detected")
                    print("pos:", pos, "vel:", vel, "tau:", tau, "target:", target)
                    break
            if int(elapsed * 10) % 20 == 0:
                print("claw closing pos:", pos, "vel:", vel, "tau:", tau, "target:", target)
        if contact:
            hold_pos = contact_pos - CLAW_BACKOFF
            result = {"contact": True, "contact_pos": contact_pos, "hold_pos": hold_pos, "status": last_status}
        else:
            hold_pos = last_status["pos"]
            result = {"contact": False, "contact_pos": None, "hold_pos": hold_pos, "status": last_status}
            print("claw no contact detected")
            print("final close pos:", last_status["pos"], "vel:", last_status["vel"], "tau:", last_status["tau"])
        print("claw pressure result:", json.dumps(result, ensure_ascii=False, indent=2))
        if hold:
            self.claw_hold_forever(float(hold_pos))
        else:
            self.claw_hold(float(hold_pos), 1.0)
        return result

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

    def move_target_with_holds(
        self,
        name: str,
        target: float,
        seconds_per_step: float,
        kp: float,
        kd: float,
        hold_targets: Dict[str, float],
        hold_gains: Dict[str, Dict[str, float]],
        fallback_kp: float,
        fallback_kd: float,
        hold_tau: Optional[Dict[str, float]] = None,
        active_velocity_ff: bool = False,
        active_tau: float = 0.0,
        step_deg: float = 0.0,
        control_dt: float = 0.02,
        preload_seconds: float = 0.2,
        trajectory: str = "smoothstep",
        linear_blend: float = 0.0,
        trace_names: Optional[List[str]] = None,
        trace_interval: float = 0.5,
    ) -> None:
        self.validate([name] + list(hold_targets.keys()))
        start = self.positions([name])[name]
        delta = target - start
        if step_deg > 0:
            steps = int(math.ceil(abs(math.degrees(delta)) / step_deg))
        else:
            steps = 1
        steps = max(1, steps)

        hold_tau = hold_tau or {}
        trace_names = trace_names or []
        all_targets = dict(hold_targets)
        all_targets[name] = start
        start_time = time.time()
        while time.time() - start_time < preload_seconds:
            for hold_name, hold_target in all_targets.items():
                if hold_name == name:
                    joint_kp = kp
                    joint_kd = kd
                    joint_tau = active_tau
                else:
                    joint_gains = hold_gains.get(hold_name, {})
                    joint_kp = joint_gains.get("kp", fallback_kp)
                    joint_kd = joint_gains.get("kd", fallback_kd)
                    joint_tau = hold_tau.get(hold_name, 0.0)
                self.ctrl.controlMIT(self.motors[hold_name], joint_kp, joint_kd, hold_target, 0, joint_tau)
            time.sleep(control_dt)

        previous = start
        for step in range(1, steps + 1):
            intermediate = start + delta * (step / steps)
            print("v2 continuous step", name, f"{step}/{steps}", "target=", intermediate)
            step_start = time.time()
            next_trace_time = step_start
            while time.time() - step_start < seconds_per_step:
                now = time.time()
                t = (now - step_start) / max(seconds_per_step, 1e-6)
                if active_velocity_ff and steps == 1:
                    s = smootherstep(t)
                    active_velocity = delta * smootherstep_derivative(t) / max(seconds_per_step, 1e-6)
                elif trajectory == "blend_smootherstep":
                    s = blend_smootherstep(t, linear_blend)
                    active_velocity = 0.0
                else:
                    s = smoothstep(t)
                    active_velocity = 0.0
                active_target = previous * (1.0 - s) + intermediate * s
                self.ctrl.controlMIT(self.motors[name], kp, kd, active_target, active_velocity, active_tau)
                for hold_name, hold_target in hold_targets.items():
                    joint_gains = hold_gains.get(hold_name, {})
                    joint_kp = joint_gains.get("kp", fallback_kp)
                    joint_kd = joint_gains.get("kd", fallback_kd)
                    self.ctrl.controlMIT(
                        self.motors[hold_name],
                        joint_kp,
                        joint_kd,
                        hold_target,
                        0,
                        hold_tau.get(hold_name, 0.0),
                    )
                if trace_names and now >= next_trace_time:
                    print(
                        "v2 continuous trace",
                        name,
                        f"{step}/{steps}",
                        "t=",
                        round(now - step_start, 3),
                        json.dumps(self.read_status(trace_names), ensure_ascii=False),
                    )
                    next_trace_time = now + trace_interval
                time.sleep(control_dt)
            previous = intermediate

        final_targets = dict(hold_targets)
        final_targets[name] = target
        start_time = time.time()
        while time.time() - start_time < 0.5:
            for hold_name, hold_target in final_targets.items():
                if hold_name == name:
                    joint_kp = kp
                    joint_kd = kd
                    joint_tau = active_tau
                else:
                    joint_gains = hold_gains.get(hold_name, {})
                    joint_kp = joint_gains.get("kp", fallback_kp)
                    joint_kd = joint_gains.get("kd", fallback_kd)
                    joint_tau = hold_tau.get(hold_name, 0.0)
                self.ctrl.controlMIT(self.motors[hold_name], joint_kp, joint_kd, hold_target, 0, joint_tau)
            time.sleep(control_dt)

    def move_targets_with_holds(
        self,
        targets: Dict[str, float],
        seconds_per_step: float,
        move_gains: Dict[str, Dict[str, float]],
        hold_targets: Dict[str, float],
        hold_gains: Dict[str, Dict[str, float]],
        fallback_kp: float,
        fallback_kd: float,
        hold_tau: Optional[Dict[str, float]] = None,
        step_deg: float = 0.0,
        preload_seconds: float = 0.2,
        progress_windows: Optional[Dict[str, Tuple[float, float]]] = None,
        pre_window_gains: Optional[Dict[str, Dict[str, float]]] = None,
        control_dt: float = 0.02,
        velocity_ff_joints: Optional[Set[str]] = None,
        move_tau_ff: Optional[Dict[str, float]] = None,
        trajectory: str = "smoothstep",
        linear_blend: float = 0.0,
    ) -> None:
        active_names = self.validate(targets.keys())
        self.validate(list(hold_targets.keys()))
        starts = self.positions(active_names)
        hold_tau = hold_tau or {}
        progress_windows = progress_windows or {}
        pre_window_gains = pre_window_gains or {}
        velocity_ff_joints = velocity_ff_joints or set()
        move_tau_ff = move_tau_ff or {}
        if trajectory == "linear":
            trajectory_fn = lambda value: max(0.0, min(1.0, value))
            trajectory_derivative_fn = lambda value: 1.0
        elif trajectory == "blend_smootherstep":
            trajectory_fn = lambda value: blend_smootherstep(value, linear_blend)
            trajectory_derivative_fn = lambda value: blend_smootherstep_derivative(value, linear_blend)
        elif trajectory == "smootherstep":
            trajectory_fn = smootherstep
            trajectory_derivative_fn = smootherstep_derivative
        else:
            trajectory_fn = smoothstep
            trajectory_derivative_fn = lambda value: 6.0 * value * (1.0 - value)

        if step_deg > 0:
            steps = max(
                int(math.ceil(abs(math.degrees(targets[name] - starts[name])) / step_deg))
                for name in active_names
            )
        else:
            steps = 1
        steps = max(1, steps)

        preload_targets = dict(hold_targets)
        preload_targets.update(starts)
        start_time = time.time()
        while time.time() - start_time < preload_seconds:
            for name, target in preload_targets.items():
                if name in starts:
                    gains = move_gains.get(name, {})
                    tau = 0.0
                else:
                    gains = hold_gains.get(name, {})
                    tau = hold_tau.get(name, 0.0)
                kp = gains.get("kp", fallback_kp)
                kd = gains.get("kd", fallback_kd)
                self.ctrl.controlMIT(self.motors[name], kp, kd, target, 0, tau)
            time.sleep(0.02)

        previous = dict(starts)
        for step in range(1, steps + 1):
            intermediates = {
                name: starts[name] + (targets[name] - starts[name]) * (step / steps)
                for name in active_names
            }
            print(
                "v2.2 continuous group step",
                ",".join(active_names),
                f"{step}/{steps}",
                "targets=",
                json.dumps(intermediates, ensure_ascii=False),
            )
            step_start = time.time()
            while time.time() - step_start < seconds_per_step:
                t = (time.time() - step_start) / max(seconds_per_step, 1e-6)
                s = trajectory_fn(t)
                ds_dt = trajectory_derivative_fn(t) / max(seconds_per_step, 1e-6)
                for name in active_names:
                    gains = move_gains.get(name, {})
                    active_velocity = 0.0
                    if name in progress_windows:
                        window_start, window_end = progress_windows[name]
                        local_s = (s - window_start) / max(window_end - window_start, 1e-6)
                        local_s = smoothstep(local_s)
                        if s < window_start:
                            gains = pre_window_gains.get(name, gains)
                    else:
                        local_s = s
                        if name in velocity_ff_joints:
                            active_velocity = (intermediates[name] - previous[name]) * ds_dt
                    kp = gains.get("kp", fallback_kp)
                    kd = gains.get("kd", fallback_kd)
                    target = previous[name] * (1.0 - local_s) + intermediates[name] * local_s
                    active_tau = move_tau_ff.get(name, 0.0)
                    self.ctrl.controlMIT(self.motors[name], kp, kd, target, active_velocity, active_tau)
                for hold_name, hold_target in hold_targets.items():
                    gains = hold_gains.get(hold_name, {})
                    kp = gains.get("kp", fallback_kp)
                    kd = gains.get("kd", fallback_kd)
                    self.ctrl.controlMIT(self.motors[hold_name], kp, kd, hold_target, 0, hold_tau.get(hold_name, 0.0))
                time.sleep(control_dt)
            previous = intermediates

        final_targets = dict(hold_targets)
        final_targets.update(targets)
        start_time = time.time()
        while time.time() - start_time < 0.5:
            for name, target in final_targets.items():
                if name in targets:
                    gains = move_gains.get(name, {})
                    tau = move_tau_ff.get(name, 0.0)
                else:
                    gains = hold_gains.get(name, {})
                    tau = hold_tau.get(name, 0.0)
                kp = gains.get("kp", fallback_kp)
                kd = gains.get("kd", fallback_kd)
                self.ctrl.controlMIT(self.motors[name], kp, kd, target, 0, tau)
            time.sleep(0.02)


def load_home(path: str = HOME_PATH) -> Dict[str, float]:
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    joints = payload.get("joints")
    if not isinstance(joints, dict):
        raise RuntimeError("invalid v2 home file: " + path)
    return {name: float(value) for name, value in joints.items()}


def load_pose(path: str) -> Dict[str, float]:
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    joints = payload.get("joints")
    if not isinstance(joints, dict):
        raise RuntimeError("invalid v2 pose file: " + path)
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


def save_pose(pose_type: str, pose: Dict[str, float], note: str, path: str) -> None:
    payload = {
        "type": pose_type,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": note,
        "joint_order": list(pose.keys()),
        "joints": pose,
    }
    ensure_parent(path)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print("saved v2 pose:", path)
    print(json.dumps(payload, ensure_ascii=False, indent=2))


def clearance_hold_plan_for(
    active: str,
    ordered: List[str],
    current: Dict[str, float],
    completed_targets: Dict[str, float],
) -> Tuple[Dict[str, float], Dict[str, Dict[str, float]]]:
    strong_hold_names = set(completed_targets.keys())
    compliant_hold_names = set(CLEARANCE_COMPLIANT_HOLDS_BY_ACTIVE.get(active, set()))
    strong_hold_names.difference_update(compliant_hold_names)

    if active == "shoulder_front":
        strong_hold_names.update(["elbow"])
    elif active == "shoulder_side":
        strong_hold_names.update(["shoulder_front", "elbow"])
    elif active == "elbow":
        strong_hold_names.update(["shoulder_front", "shoulder_side"])
    elif active in ("arm_roll", "wrist_side", "wrist"):
        strong_hold_names.update(["shoulder_front", "shoulder_side", "elbow"])

    hold_targets = {}
    hold_gains = {}
    for hold_name in ordered:
        if hold_name == active:
            continue
        hold_targets[hold_name] = completed_targets.get(hold_name, current[hold_name])
        if hold_name in compliant_hold_names:
            hold_gains[hold_name] = CLEARANCE_COMPLIANT_HOLD_GAINS.get(
                hold_name,
                CLEARANCE_BASE_HOLD_GAINS[hold_name],
            )
        elif hold_name in strong_hold_names:
            hold_gains[hold_name] = CLEARANCE_HOLD_GAINS.get(hold_name, CLEARANCE_BASE_HOLD_GAINS[hold_name])
        else:
            hold_gains[hold_name] = CLEARANCE_BASE_HOLD_GAINS[hold_name]
    return hold_targets, hold_gains


def home_hold_plan_for(
    active: str,
    ordered: List[str],
    current: Dict[str, float],
    completed_targets: Dict[str, float],
) -> Tuple[Dict[str, float], Dict[str, Dict[str, float]]]:
    strong_hold_names = set(completed_targets.keys())
    compliant_hold_names = set(HOME_COMPLIANT_HOLDS_BY_ACTIVE.get(active, set()))
    strong_hold_names.difference_update(compliant_hold_names)

    if active == "elbow":
        strong_hold_names.update(["shoulder_front", "shoulder_side"])
    elif active == "shoulder_front":
        strong_hold_names.update(["elbow", "shoulder_side"])
    elif active in ("wrist", "wrist_side", "arm_roll"):
        strong_hold_names.update(["elbow", "shoulder_front"])

    hold_targets = {}
    hold_gains = {}
    for hold_name in ordered:
        if hold_name == active:
            continue
        hold_targets[hold_name] = completed_targets.get(hold_name, current[hold_name])
        if hold_name in compliant_hold_names:
            hold_gains[hold_name] = HOME_COMPLIANT_HOLD_GAINS.get(
                hold_name,
                HOME_BASE_HOLD_GAINS[hold_name],
            )
        elif hold_name in strong_hold_names:
            hold_gains[hold_name] = HOME_HOLD_GAINS.get(hold_name, HOME_BASE_HOLD_GAINS[hold_name])
        else:
            hold_gains[hold_name] = HOME_BASE_HOLD_GAINS[hold_name]
    return hold_targets, hold_gains


def prehome_clearance_gains_for(name: str, fallback_kp: float, fallback_kd: float) -> Dict[str, float]:
    if name in PREHOME_CLEARANCE_GAINS:
        return PREHOME_CLEARANCE_GAINS[name]
    return NUDGE_GAINS.get(name, CLEARANCE_MOVE_GAINS.get(name, {"kp": fallback_kp, "kd": fallback_kd}))


def prehome_clearance_seconds_for(name: str, delta_deg: float) -> float:
    default_base = 6.0 if name == "wrist" else 2.2
    base = PREHOME_CLEARANCE_BASE_SECONDS.get(name, NUDGE_MIN_SECONDS.get(name, default_base))
    seconds_per_deg = PREHOME_CLEARANCE_SECONDS_PER_DEG.get(name)
    if seconds_per_deg is not None:
        base = max(base, abs(delta_deg) * seconds_per_deg)
    return min(PREHOME_CLEARANCE_MAX_SECONDS.get(name, 20.0), base)


def prehome_clearance_hold_gains_for(
    active: str,
    hold_names: Iterable[str],
    fallback_kp: float,
    fallback_kd: float,
) -> Dict[str, Dict[str, float]]:
    hold_gains = {
        hold_name: CLEARANCE_HOLD_GAINS.get(
            hold_name,
            CLEARANCE_BASE_HOLD_GAINS.get(hold_name, {"kp": fallback_kp, "kd": fallback_kd}),
        )
        for hold_name in hold_names
    }
    for compliant_name in CLEARANCE_COMPLIANT_HOLDS_BY_ACTIVE.get(active, set()):
        if compliant_name in hold_gains:
            hold_gains[compliant_name] = CLEARANCE_COMPLIANT_HOLD_GAINS[compliant_name]
    nudge_compliant_gains = NUDGE_COMPLIANT_HOLD_GAINS_BY_ACTIVE.get(active, NUDGE_COMPLIANT_HOLD_GAINS)
    for compliant_name in NUDGE_COMPLIANT_HOLDS_BY_ACTIVE.get(active, set()):
        if compliant_name in hold_gains:
            hold_gains[compliant_name] = nudge_compliant_gains[compliant_name]
    return hold_gains


def nudge_hold_tau_for(active: str, hold_targets: Dict[str, float], low_shoulder_front: bool = False) -> Dict[str, float]:
    hold_tau = dict(NUDGE_HOLD_TAU)
    if low_shoulder_front and "shoulder_front" in hold_targets:
        hold_tau["shoulder_front"] = NUDGE_LOW_SHOULDER_FRONT_HOLD_TAU
    if low_shoulder_front and active in ("shoulder_front", "wrist") and "wrist_side" in hold_targets:
        hold_tau["wrist_side"] = NUDGE_LOW_SHOULDER_WRIST_SIDE_HOLD_TAU
    if low_shoulder_front and active == "shoulder_front":
        for hold_name, tau in NUDGE_LOW_SHOULDER_WRIST_HOLD_TAU.items():
            if hold_name in hold_targets:
                hold_tau[hold_name] = tau
    if low_shoulder_front and active in ("wrist_side", "wrist"):
        for hold_name, tau in NUDGE_LOW_SHOULDER_WRIST_HOLD_TAU.items():
            if hold_name in hold_targets:
                hold_tau[hold_name] = tau
    if low_shoulder_front and active == "arm_roll":
        for hold_name, tau in NUDGE_LOW_SHOULDER_ARM_ROLL_HOLD_TAU.items():
            if hold_name in hold_targets:
                hold_tau[hold_name] = tau
    return hold_tau


def nudge_active_tau_for(name: str, delta_deg: float) -> float:
    active_tau = NUDGE_ACTIVE_TAU.get(name, 0.0)
    if name == "shoulder_front" and delta_deg < 0:
        return 0.0
    return active_tau


def is_low_shoulder_front(current: Dict[str, float]) -> bool:
    shoulder_front = current.get("shoulder_front")
    return (
        shoulder_front is not None
        and shoulder_front < NUDGE_WRIST_ACTIVE_LOW_SHOULDER_FRONT_THRESHOLD
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Clean v2 left-arm bring-up controller")
    sub = parser.add_subparsers(dest="cmd", required=True)

    common = ",".join(DEFAULT_JOINTS)

    status = sub.add_parser("status")
    status.add_argument("--joints", default=common)

    restart = sub.add_parser("restart", help="Disable and re-enable selected motors after shaking.")
    restart.add_argument("--joints", default=common)
    restart.add_argument("--off-seconds", type=float, default=0.8)
    restart.add_argument("--hold-seconds", type=float, default=1.0)
    restart.add_argument("--hold-kp", type=float, default=2.0)
    restart.add_argument("--hold-kd", type=float, default=0.4)
    restart.add_argument("--execute", action="store_true")

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

    cap_clearance = sub.add_parser("capture-clearance")
    cap_clearance.add_argument("--joints", default=common)
    cap_clearance.add_argument("--note", default="")
    cap_clearance.add_argument("--clearance-file", default=TABLE_CLEARANCE_PATH)

    show_clearance = sub.add_parser("show-clearance")
    show_clearance.add_argument("--clearance-file", default=TABLE_CLEARANCE_PATH)

    cap_claw = sub.add_parser("capture-claw-home")
    cap_claw.add_argument("--note", default="")
    cap_claw.add_argument("--claw-home-file", default=CLAW_HOME_PATH)

    show_claw = sub.add_parser("show-claw-home")
    show_claw.add_argument("--claw-home-file", default=CLAW_HOME_PATH)

    claw_home = sub.add_parser("claw-home")
    claw_home.add_argument("--claw-home-file", default=CLAW_HOME_PATH)
    claw_home.add_argument("--execute", action="store_true")

    claw_close = sub.add_parser("claw-close")
    claw_close.add_argument("--claw-home-file", default=CLAW_HOME_PATH)
    claw_close.add_argument("--close-offset", type=float, default=CLAW_CLOSE_OFFSET)
    claw_close.add_argument("--no-hold", action="store_true")
    claw_close.add_argument("--execute", action="store_true")

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

    nudge_hold = sub.add_parser("nudge-hold")
    nudge_hold.add_argument("--joint", required=True)
    nudge_hold.add_argument("--deg", type=float, required=True)
    nudge_hold.add_argument("--seconds", type=float, default=2.0)
    nudge_hold.add_argument("--kp", type=float, default=3.0)
    nudge_hold.add_argument("--kd", type=float, default=0.3)
    nudge_hold.add_argument("--max-deg", type=float, default=5.0)
    nudge_hold.add_argument("--hold-joints", default=common)
    nudge_hold.add_argument("--auto-gains", action="store_true")

    home = sub.add_parser("home")
    home.add_argument("--joints", default=",".join(DEFAULT_HOME_ORDER))
    home.add_argument("--home-file", default=HOME_PATH)
    home.add_argument("--clearance-file", default=TABLE_CLEARANCE_PATH)
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
    home.add_argument(
        "--prehome-clearance",
        action="store_true",
        help="Before home, return joints to table clearance one by one in the tested reverse-chain order.",
    )

    clearance = sub.add_parser("clearance")
    clearance.add_argument("--joints", default=",".join(DEFAULT_CLEARANCE_ORDER))
    clearance.add_argument("--clearance-file", default=TABLE_CLEARANCE_PATH)
    clearance.add_argument("--seconds", type=float, default=5.0)
    clearance.add_argument("--kp", type=float, default=3.0)
    clearance.add_argument("--kd", type=float, default=0.3)
    clearance.add_argument("--deadband-deg", type=float, default=0.5)
    clearance.add_argument("--max-delta-deg", type=float, default=20.0)
    clearance.add_argument(
        "--step-deg",
        type=float,
        default=0.0,
        help="Split each clearance joint move into steps no larger than this many degrees. Use for large home-to-clearance moves.",
    )
    clearance.add_argument(
        "--parallel",
        action="store_true",
        help="Move selected clearance joints together on one continuous trajectory.",
    )
    clearance.add_argument(
        "--no-coupled-shoulder-group",
        dest="coupled_shoulder_group",
        action="store_false",
        default=True,
        help="Disable v2.2 coupled shoulder_front/shoulder_side/elbow clearance move and use sequential v2 behavior.",
    )
    clearance.add_argument(
        "--auto-gains",
        dest="auto_gains",
        action="store_true",
        default=True,
        help="Use verified per-joint home gains. This is the default.",
    )
    clearance.add_argument(
        "--no-auto-gains",
        dest="auto_gains",
        action="store_false",
        help="Use the uniform --kp/--kd/--seconds values instead.",
    )
    clearance.add_argument("--execute", action="store_true")

    return parser


def main() -> None:
    args = build_parser().parse_args()

    if args.cmd == "show-home":
        print(json.dumps(load_home(args.home_file), ensure_ascii=False, indent=2))
        return

    if args.cmd == "show-clearance":
        print(json.dumps(load_pose(args.clearance_file), ensure_ascii=False, indent=2))
        return

    if args.cmd == "show-claw-home":
        print(json.dumps(load_pose(args.claw_home_file), ensure_ascii=False, indent=2))
        return

    arm = LeftArmV2()
    try:
        print("Serial port is open")

        if args.cmd == "status":
            arm.print_status(parse_joints(args.joints))
            return

        if args.cmd == "restart":
            joints = parse_joints(args.joints)
            arm.validate(joints)
            if not args.execute:
                print("dry run only. Add --execute to disable and re-enable motors.")
                arm.print_status(joints)
                return
            arm.restart_motors(
                joints,
                off_seconds=args.off_seconds,
                hold_seconds=args.hold_seconds,
                hold_kp=args.hold_kp,
                hold_kd=args.hold_kd,
            )
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

        if args.cmd == "capture-clearance":
            joints = parse_joints(args.joints)
            pose = arm.positions(joints)
            save_pose("left_arm_v2_table_clearance", pose, note=args.note, path=args.clearance_file)
            return

        if args.cmd == "capture-claw-home":
            pose = {"claw": arm.claw_status()["pos"]}
            save_pose("left_arm_v2_claw_home", pose, note=args.note, path=args.claw_home_file)
            return

        if args.cmd == "claw-home":
            home = load_pose(args.claw_home_file)
            if "claw" not in home:
                raise RuntimeError("invalid claw home file: missing claw")
            if not args.execute:
                print("dry run only. Add --execute to move claw home.")
                print(json.dumps({"target": {"claw": home["claw"]}, "status": arm.claw_status()}, ensure_ascii=False, indent=2))
                return
            status = arm.claw_home(home["claw"])
            print(json.dumps({"target": {"claw": home["claw"]}, "status": status}, ensure_ascii=False, indent=2))
            return

        if args.cmd == "claw-close":
            home = load_pose(args.claw_home_file)
            if "claw" not in home:
                raise RuntimeError("invalid claw home file: missing claw")
            if not args.execute:
                print("dry run only. Add --execute to close claw.")
                print(
                    json.dumps(
                        {
                            "home": home["claw"],
                            "close_target": home["claw"] + args.close_offset,
                            "status": arm.claw_status(),
                        },
                        ensure_ascii=False,
                        indent=2,
                    )
                )
                return
            arm.claw_close_until_pressure(home["claw"], hold=not args.no_hold, close_offset=args.close_offset)
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

        if args.cmd == "nudge-hold":
            if abs(args.deg) > args.max_deg:
                raise ValueError(f"nudge deg exceeds --max-deg {args.max_deg}")
            joints = parse_joints(args.joint)
            if len(joints) != 1:
                raise ValueError("nudge-hold expects exactly one --joint")
            name = joints[0]
            hold_names = [joint for joint in parse_joints(args.hold_joints) if joint != name]
            arm.validate([name] + hold_names)
            all_names = [name] + hold_names
            arm.enable(all_names)
            current = arm.positions(all_names)
            requested_target = current[name] + math.radians(args.deg)
            target = requested_target
            target_limits = NUDGE_TARGET_LIMITS.get(name)
            if target_limits:
                min_target = target_limits.get("min")
                max_target = target_limits.get("max")
                if min_target is not None and target < min_target:
                    target = min_target
                if max_target is not None and target > max_target:
                    target = max_target
                if target != requested_target:
                    print(
                        "v2.5.3 nudge-hold target clamped",
                        name,
                        "requested=",
                        requested_target,
                        "target=",
                        target,
                        "limits=",
                        json.dumps(target_limits, ensure_ascii=False),
                    )
            low_shoulder_front = is_low_shoulder_front(current)
            if args.auto_gains:
                if low_shoulder_front and name in NUDGE_LOW_SHOULDER_GAINS:
                    active_gains = NUDGE_LOW_SHOULDER_GAINS[name]
                elif name == "wrist":
                    active_gains = COUPLED_CLEARANCE_WRIST_FINE_GAINS
                elif name == "wrist_side":
                    active_gains = CLEARANCE_HOLD_GAINS["wrist_side"]
                else:
                    active_gains = NUDGE_GAINS.get(name, {"kp": args.kp, "kd": args.kd})
                kp = active_gains["kp"]
                kd = active_gains["kd"]
                if name == "wrist":
                    args.seconds = max(args.seconds, COUPLED_CLEARANCE_WRIST_FINE_GAINS["seconds"])
                if low_shoulder_front and name in NUDGE_LOW_SHOULDER_MIN_SECONDS:
                    args.seconds = max(args.seconds, NUDGE_LOW_SHOULDER_MIN_SECONDS[name])
                if name in NUDGE_MIN_SECONDS:
                    args.seconds = max(args.seconds, NUDGE_MIN_SECONDS[name])
            else:
                kp = args.kp
                kd = args.kd
            hold_targets = {joint: current[joint] for joint in hold_names}
            hold_gains = {
                joint: CLEARANCE_HOLD_GAINS.get(
                    joint,
                    CLEARANCE_BASE_HOLD_GAINS.get(joint, NUDGE_GAINS.get(joint, {"kp": args.kp, "kd": args.kd})),
                )
                for joint in hold_names
            }
            for compliant_name in CLEARANCE_COMPLIANT_HOLDS_BY_ACTIVE.get(name, set()):
                if compliant_name in hold_gains:
                    hold_gains[compliant_name] = CLEARANCE_COMPLIANT_HOLD_GAINS[compliant_name]
            nudge_compliant_gains = NUDGE_COMPLIANT_HOLD_GAINS_BY_ACTIVE.get(name, NUDGE_COMPLIANT_HOLD_GAINS)
            if is_low_shoulder_front(current):
                nudge_compliant_gains = {
                    **nudge_compliant_gains,
                    **NUDGE_LOW_SHOULDER_COMPLIANT_HOLD_GAINS_BY_ACTIVE.get(name, {}),
                }
            for compliant_name in NUDGE_COMPLIANT_HOLDS_BY_ACTIVE.get(name, set()):
                if compliant_name in hold_gains:
                    hold_gains[compliant_name] = nudge_compliant_gains[compliant_name]
            coupled_targets = {name: target}
            coupled_requested_targets = {name: requested_target}
            coupled_gains = {name: {"kp": kp, "kd": kd}}
            coupled_tau = {}
            for coupled_name, ratio in NUDGE_COUPLED_ACTIVE_RATIOS.get(name, {}).items():
                if coupled_name not in hold_targets:
                    continue
                coupled_requested = current[coupled_name] + math.radians(args.deg * ratio)
                coupled_targets[coupled_name] = coupled_requested
                coupled_requested_targets[coupled_name] = coupled_requested
                coupled_gains[coupled_name] = NUDGE_COUPLED_ACTIVE_GAINS.get(name, {}).get(
                    coupled_name,
                    COUPLED_CLEARANCE_MOVE_GAINS.get(
                        coupled_name,
                        NUDGE_GAINS.get(coupled_name, {"kp": args.kp, "kd": args.kd}),
                    ),
                )
                if coupled_name in NUDGE_HOLD_TAU and ratio > 0:
                    coupled_tau[coupled_name] = NUDGE_HOLD_TAU[coupled_name]
                hold_targets.pop(coupled_name, None)
                hold_gains.pop(coupled_name, None)
            hold_tau = nudge_hold_tau_for(name, hold_targets, low_shoulder_front)
            if name in NUDGE_ACTIVE_TAU:
                coupled_tau[name] = nudge_active_tau_for(name, args.deg)
            print(
                "v2.5.3 nudge-hold active",
                name,
                "delta_deg=",
                args.deg,
                "kp=",
                kp,
                "kd=",
                kd,
                "seconds=",
                args.seconds,
                "hold joints:",
                ",".join(hold_names),
            )
            print(
                "v2.5.3 nudge-hold hold_tau=",
                json.dumps({joint: hold_tau[joint] for joint in hold_targets if joint in hold_tau}, ensure_ascii=False),
                "control_dt=",
                0.01,
            )
            if hold_tau.get("shoulder_front") != NUDGE_HOLD_TAU.get("shoulder_front"):
                print(
                    "v2.5.3 nudge-hold adjusted shoulder_front hold_tau=",
                    hold_tau["shoulder_front"],
                    "shoulder_front_pos=",
                    hold_targets.get("shoulder_front"),
                )
            active_tau = nudge_active_tau_for(name, args.deg)
            if low_shoulder_front and name == "arm_roll":
                active_tau = NUDGE_LOW_SHOULDER_ARM_ROLL_ACTIVE_TAU
            if active_tau:
                print("v2.5.3 nudge-hold active_tau=", {name: active_tau})
            if name in NUDGE_LOW_SHOULDER_GAINS and is_low_shoulder_front(current):
                print(
                    "v2.5.3 nudge-hold low shoulder gains active",
                    name,
                    "shoulder_front_pos=",
                    current.get("shoulder_front"),
                )
            low_shoulder_hold_overrides = NUDGE_LOW_SHOULDER_COMPLIANT_HOLD_GAINS_BY_ACTIVE.get(name, {})
            if low_shoulder_hold_overrides and is_low_shoulder_front(current):
                print(
                    "v2.5.3 nudge-hold low shoulder compliant hold overrides=",
                    json.dumps(low_shoulder_hold_overrides, ensure_ascii=False),
                )
            preload_seconds = NUDGE_PRELOAD_SECONDS.get(name, 0.2)
            if preload_seconds != 0.2:
                print("v2.5.3 nudge-hold preload_seconds=", preload_seconds)
            trajectory = NUDGE_TRAJECTORY.get(name, "smoothstep")
            linear_blend = NUDGE_LINEAR_BLEND.get(name, 0.0)
            if trajectory != "smoothstep":
                print("v2.5.3 nudge-hold trajectory=", trajectory, "linear_blend=", linear_blend)
            compliant_holds = {
                joint: hold_gains[joint]
                for joint in NUDGE_COMPLIANT_HOLDS_BY_ACTIVE.get(name, set())
                if joint in hold_gains
            }
            if compliant_holds:
                print("v2.5.3 nudge-hold compliant hold gains=", json.dumps(compliant_holds, ensure_ascii=False))
            step_deg = NUDGE_STEP_DEG.get(name, 0.0)
            if step_deg > 0:
                print("v2.5.3 nudge-hold step_deg=", step_deg)
            if len(coupled_targets) > 1:
                print(
                    "v2.5.3 nudge-hold coupled active targets=",
                    json.dumps(coupled_targets, ensure_ascii=False),
                    "gains=",
                    json.dumps(coupled_gains, ensure_ascii=False),
                    "move_tau_ff=",
                    json.dumps(coupled_tau, ensure_ascii=False),
                )
            print(
                json.dumps(
                    {
                        "start": current,
                        "requested_target": coupled_requested_targets,
                        "target": coupled_targets,
                        "hold_targets": hold_targets,
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
            if len(coupled_targets) > 1:
                arm.move_targets_with_holds(
                    coupled_targets,
                    seconds_per_step=args.seconds,
                    move_gains=coupled_gains,
                    hold_targets=hold_targets,
                    hold_gains=hold_gains,
                    fallback_kp=args.kp,
                    fallback_kd=args.kd,
                    hold_tau=hold_tau,
                    preload_seconds=preload_seconds,
                    control_dt=0.01,
                    move_tau_ff=coupled_tau,
                    trajectory=trajectory,
                    linear_blend=linear_blend,
                )
                arm.print_status(all_names)
                return
            arm.move_target_with_holds(
                name,
                target,
                seconds_per_step=args.seconds,
                kp=kp,
                kd=kd,
                hold_targets=hold_targets,
                hold_gains=hold_gains,
                fallback_kp=args.kp,
                fallback_kd=args.kd,
                hold_tau=hold_tau,
                active_tau=active_tau,
                step_deg=step_deg,
                control_dt=0.01,
                preload_seconds=preload_seconds,
                trajectory=trajectory,
                linear_blend=linear_blend,
            )
            if name == "wrist_side" and is_low_shoulder_front(current):
                wrist_side_pos = arm.positions([name])[name]
                wrist_side_delta_deg = math.degrees(target - wrist_side_pos)
                if abs(wrist_side_delta_deg) > NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_DEADBAND_DEG:
                    fine = NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_GAINS
                    fine_bias_deg = max(
                        -NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_MAX_BIAS_DEG,
                        min(NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_MAX_BIAS_DEG, wrist_side_delta_deg),
                    )
                    fine_target = target + math.radians(fine_bias_deg)
                    print(
                        "v2.5.3 nudge-hold low shoulder wrist_side fine",
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
                    )
                    arm.move_target_with_holds(
                        name,
                        fine_target,
                        seconds_per_step=fine["seconds"],
                        kp=fine["kp"],
                        kd=fine["kd"],
                        hold_targets=hold_targets,
                        hold_gains=hold_gains,
                        fallback_kp=args.kp,
                        fallback_kd=args.kd,
                        hold_tau=hold_tau,
                        active_tau=active_tau,
                        step_deg=0.0,
                        control_dt=0.01,
                        preload_seconds=0.1,
                        trajectory="smoothstep",
                        linear_blend=0.0,
                    )
                else:
                    print(
                        "v2.5.3 nudge-hold low shoulder wrist_side fine skip delta_deg=",
                        wrist_side_delta_deg,
                        "deadband_deg=",
                        NUDGE_LOW_SHOULDER_WRIST_SIDE_FINE_DEADBAND_DEG,
                    )
            arm.print_status(all_names)
            return

        if args.cmd == "home":
            home = load_home(args.home_file)
            ordered = [name for name in parse_joints(args.joints) if name in home]
            current = arm.positions(ordered)
            targets: Dict[str, float] = {}
            for name in ordered:
                delta_deg = math.degrees(home[name] - current[name])
                deadband_deg = max(args.deadband_deg, HOME_JOINT_DEADBANDS_DEG.get(name, args.deadband_deg))
                if abs(delta_deg) <= deadband_deg:
                    print("v2 home skip", name, "delta_deg=", delta_deg, "deadband_deg=", deadband_deg)
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

            if args.prehome_clearance:
                clearance = load_pose(args.clearance_file)
                prehome_order = [name for name in PREHOME_CLEARANCE_ORDER if name in clearance and name in DEFAULT_JOINTS]
                print("v2.5.3 pre-home clearance sequential order=", ",".join(prehome_order))
                arm.enable(DEFAULT_JOINTS)
                for prehome_name in prehome_order:
                    prehome_current = arm.positions(DEFAULT_JOINTS)
                    prehome_target = clearance[prehome_name]
                    delta_deg = math.degrees(prehome_target - prehome_current[prehome_name])
                    deadband_deg = max(0.5, CLEARANCE_JOINT_DEADBANDS_DEG.get(prehome_name, 0.5))
                    if abs(delta_deg) <= deadband_deg:
                        print(
                            "v2.5.3 pre-home clearance skip",
                            prehome_name,
                            "delta_deg=",
                            delta_deg,
                            "deadband_deg=",
                            deadband_deg,
                        )
                        continue
                    if abs(delta_deg) > args.max_delta_deg:
                        raise RuntimeError(
                            f"{prehome_name} pre-home clearance delta {delta_deg:.2f} deg exceeds "
                            f"--max-delta-deg {args.max_delta_deg}; move in smaller steps."
                        )
                    active_gains = prehome_clearance_gains_for(prehome_name, args.kp, args.kd)
                    seconds = prehome_clearance_seconds_for(prehome_name, delta_deg)
                    hold_targets = {
                        hold_name: prehome_current[hold_name]
                        for hold_name in DEFAULT_JOINTS
                        if hold_name != prehome_name
                    }
                    hold_gains = prehome_clearance_hold_gains_for(
                        prehome_name,
                        hold_targets.keys(),
                        args.kp,
                        args.kd,
                    )
                    hold_tau = nudge_hold_tau_for(
                        prehome_name,
                        hold_targets,
                        is_low_shoulder_front(prehome_current),
                    )
                    active_tau = nudge_active_tau_for(prehome_name, delta_deg)
                    preload_seconds = NUDGE_PRELOAD_SECONDS.get(prehome_name, 0.2)
                    trajectory = NUDGE_TRAJECTORY.get(prehome_name, "smoothstep")
                    linear_blend = NUDGE_LINEAR_BLEND.get(prehome_name, 0.0)
                    print(
                        "v2.5.3 pre-home clearance active",
                        prehome_name,
                        "delta_deg=",
                        delta_deg,
                        "target=",
                        prehome_target,
                        "kp=",
                        active_gains["kp"],
                        "kd=",
                        active_gains["kd"],
                        "seconds=",
                        seconds,
                        "hold joints:",
                        ",".join(hold_targets.keys()),
                    )
                    if active_tau:
                        print("v2.5.3 pre-home clearance active_tau=", {prehome_name: active_tau})
                    print(
                        "v2.5.3 pre-home clearance hold_tau=",
                        json.dumps({joint: hold_tau[joint] for joint in hold_targets if joint in hold_tau}, ensure_ascii=False),
                    )
                    compliant_hold_names = (
                        CLEARANCE_COMPLIANT_HOLDS_BY_ACTIVE.get(prehome_name, set())
                        | NUDGE_COMPLIANT_HOLDS_BY_ACTIVE.get(prehome_name, set())
                    )
                    compliant_holds = {
                        joint: hold_gains[joint]
                        for joint in compliant_hold_names
                        if joint in hold_gains
                    }
                    if compliant_holds:
                        print(
                            "v2.5.3 pre-home clearance compliant hold gains=",
                            json.dumps(compliant_holds, ensure_ascii=False),
                        )
                    arm.move_target_with_holds(
                        prehome_name,
                        prehome_target,
                        seconds_per_step=seconds,
                        kp=active_gains["kp"],
                        kd=active_gains["kd"],
                        hold_targets=hold_targets,
                        hold_gains=hold_gains,
                        fallback_kp=args.kp,
                        fallback_kd=args.kd,
                        hold_tau=hold_tau,
                        active_tau=active_tau,
                        control_dt=0.01,
                        preload_seconds=preload_seconds,
                        trajectory=trajectory,
                        linear_blend=linear_blend,
                    )
                current = arm.positions(ordered)

            completed_targets: Dict[str, float] = {}
            for name in ordered:
                if name not in targets:
                    continue
                if name in completed_targets:
                    continue
                if name == "elbow" and "shoulder_front" in targets:
                    group_names = ["elbow", "shoulder_front"]
                    if "shoulder_side" in ordered and "shoulder_side" in home:
                        group_names.append("shoulder_side")
                    if "arm_roll" in targets:
                        group_names.append("arm_roll")
                    if "wrist_side" in targets:
                        group_names.append("wrist_side")
                    if "wrist" in targets:
                        group_names.append("wrist")
                    group_targets = {
                        group_name: targets.get(group_name, home[group_name])
                        for group_name in group_names
                    }
                    command_targets = dict(group_targets)
                    command_bias = {
                        group_name: math.radians(COUPLED_HOME_COMMAND_BIAS_DEG[group_name])
                        for group_name in group_names
                        if group_name in COUPLED_HOME_COMMAND_BIAS_DEG
                    }
                    for group_name, bias_rad in command_bias.items():
                        command_targets[group_name] += bias_rad
                    raw_seconds = max(HOME_GAINS.get(group_name, {"seconds": args.seconds})["seconds"] for group_name in group_names)
                    coupled_seconds = min(raw_seconds * 4.0, COUPLED_HOME_MAX_SECONDS)
                    hold_targets = {}
                    hold_gains = {}
                    for hold_name in ordered:
                        if hold_name in group_names:
                            continue
                        hold_targets[hold_name] = current[hold_name]
                        hold_gains[hold_name] = HOME_BASE_HOLD_GAINS.get(hold_name, {"kp": args.kp, "kd": args.kd})
                    move_gains = {
                        group_name: COUPLED_HOME_MOVE_GAINS.get(group_name, HOME_GAINS.get(group_name, {"kp": args.kp, "kd": args.kd}))
                        for group_name in group_names
                    }
                    print(
                        "v2.5.3 home coupled shoulder/roll/wrist active",
                        ",".join(group_names),
                        "hold joints:",
                        ", ".join(hold_targets.keys()),
                    )
                    for group_name in group_names:
                        gains = move_gains[group_name]
                        print("v2.5.3 home coupled shoulder/roll/wrist gains", group_name, "kp=", gains["kp"], "kd=", gains["kd"], "seconds=", coupled_seconds)
                    print("v2.5.3 home coupled shoulder/roll/wrist progress windows=", json.dumps(COUPLED_HOME_PROGRESS_WINDOWS, ensure_ascii=False))
                    if COUPLED_HOME_PRE_WINDOW_GAINS:
                        print("v2.5.3 home coupled shoulder/roll/wrist pre-window gains=", json.dumps(COUPLED_HOME_PRE_WINDOW_GAINS, ensure_ascii=False))
                    if command_bias:
                        print(
                            "v2.5.3 home coupled shoulder/roll/wrist command bias deg=",
                            json.dumps({name: COUPLED_HOME_COMMAND_BIAS_DEG[name] for name in command_bias}, ensure_ascii=False),
                        )
                    print("v2.5.3 home coupled shoulder/roll/wrist control_dt=", COUPLED_HOME_CONTROL_DT)
                    print("v2.5.3 home coupled shoulder/roll/wrist trajectory=", COUPLED_HOME_TRAJECTORY)
                    if COUPLED_HOME_TRAJECTORY == "blend_smootherstep":
                        print("v2.5.3 home coupled shoulder/roll/wrist linear_blend=", COUPLED_HOME_LINEAR_BLEND)
                    arm.enable(group_names + list(hold_targets.keys()))
                    arm.move_targets_with_holds(
                        command_targets,
                        seconds_per_step=coupled_seconds,
                        move_gains=move_gains,
                        hold_targets=hold_targets,
                        hold_gains=hold_gains,
                        fallback_kp=args.kp,
                        fallback_kd=args.kd,
                        step_deg=0.0,
                        progress_windows=COUPLED_HOME_PROGRESS_WINDOWS,
                        pre_window_gains=COUPLED_HOME_PRE_WINDOW_GAINS,
                        control_dt=COUPLED_HOME_CONTROL_DT,
                        trajectory=COUPLED_HOME_TRAJECTORY,
                        linear_blend=COUPLED_HOME_LINEAR_BLEND,
                    )
                    reached = arm.positions(group_names)
                    print("v2.5.3 home coupled shoulder/roll/wrist reached=", json.dumps(reached, ensure_ascii=False))
                    completed_targets.update(command_targets)
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
                hold_targets, hold_gains = home_hold_plan_for(name, ordered, current, completed_targets)
                print("v2 home active", name, "hold joints:", ", ".join(hold_targets.keys()))
                arm.enable([name] + list(hold_targets.keys()))
                arm.move_target_with_holds(
                    name,
                    targets[name],
                    seconds_per_step=seconds,
                    kp=kp,
                    kd=kd,
                    hold_targets=hold_targets,
                    hold_gains=hold_gains,
                    fallback_kp=args.kp,
                    fallback_kd=args.kd,
                    step_deg=0.0,
                )
                completed_targets[name] = targets[name]
            arm.print_status(ordered)
            return

        if args.cmd == "clearance":
            clearance = load_pose(args.clearance_file)
            ordered = [name for name in parse_joints(args.joints) if name in clearance]
            current = arm.positions(ordered)
            targets: Dict[str, float] = {}
            deltas_deg: Dict[str, float] = {}
            for name in ordered:
                delta_deg = math.degrees(clearance[name] - current[name])
                deadband_deg = max(args.deadband_deg, CLEARANCE_JOINT_DEADBANDS_DEG.get(name, args.deadband_deg))
                if abs(delta_deg) <= deadband_deg:
                    print("v2 clearance skip", name, "delta_deg=", delta_deg, "deadband_deg=", deadband_deg)
                    continue
                if abs(delta_deg) > args.max_delta_deg:
                    raise RuntimeError(
                        f"{name} clearance delta {delta_deg:.2f} deg exceeds --max-delta-deg {args.max_delta_deg}; "
                        "move in smaller steps or recapture clearance."
                    )
                targets[name] = clearance[name]
                deltas_deg[name] = delta_deg
                print("v2 clearance target", name, "delta_deg=", delta_deg, "target=", clearance[name])
                if name in CLEARANCE_CONTINUOUS_JOINTS:
                    print("v2 clearance planned continuous", name, "steps= 1")
                elif args.step_deg > 0:
                    effective_step_deg = args.step_deg
                    steps = int(math.ceil(abs(delta_deg) / effective_step_deg))
                    print("v2 clearance planned steps", name, "steps=", steps, "step_deg<=", effective_step_deg)

            if not targets:
                print("v2 clearance: all selected joints inside deadband; no motor enabled")
                arm.print_status(ordered)
                return

            if not args.execute:
                print("dry run only. Add --execute to move.")
                return

            if args.parallel:
                arm.enable(ordered)
                if args.step_deg > 0:
                    max_steps = max(int(math.ceil(abs(delta) / args.step_deg)) for delta in deltas_deg.values())
                else:
                    max_steps = 1
                total_seconds = args.seconds * max(1, max_steps)
                if args.auto_gains:
                    for name in ordered:
                        if name not in targets:
                            continue
                        gains = CLEARANCE_MOVE_GAINS.get(name, {"kp": args.kp, "kd": args.kd})
                        print(
                            "v2 clearance parallel auto gains",
                            name,
                            "kp=",
                            gains["kp"],
                            "kd=",
                            gains["kd"],
                        )
                    print("v2 clearance parallel seconds=", total_seconds, "steps=", max_steps)
                    arm.move_targets_auto_gains(
                        targets,
                        seconds=total_seconds,
                        gains=CLEARANCE_MOVE_GAINS,
                        fallback_kp=args.kp,
                        fallback_kd=args.kd,
                    )
                else:
                    print("v2 clearance parallel uniform gains kp=", args.kp, "kd=", args.kd, "seconds=", total_seconds)
                    arm.move_targets(targets, seconds=total_seconds, kp=args.kp, kd=args.kd)
                arm.print_status(ordered)
                return

            completed_targets: Dict[str, float] = {}
            for name in ordered:
                if name not in targets:
                    continue
                if name in completed_targets:
                    continue
                if (
                    args.coupled_shoulder_group
                    and name == "shoulder_front"
                    and "shoulder_side" in targets
                    and "elbow" in targets
                ):
                    group_names = ["shoulder_front", "shoulder_side", "elbow"]
                    if "arm_roll" in targets:
                        group_names.append("arm_roll")
                    if "wrist_side" in targets:
                        group_names.append("wrist_side")
                    if "wrist" in targets:
                        group_names.append("wrist")
                    group_targets = {group_name: targets[group_name] for group_name in group_names}
                    seconds = max(HOME_GAINS.get(group_name, {"seconds": args.seconds})["seconds"] for group_name in group_names)
                    if args.step_deg > 0:
                        coupled_steps = max(
                            int(math.ceil(abs(math.degrees(group_targets[group_name] - current[group_name])) / args.step_deg))
                            for group_name in group_names
                        )
                    else:
                        coupled_steps = 1
                    raw_coupled_seconds = seconds * max(1, coupled_steps)
                    coupled_seconds = min(raw_coupled_seconds, COUPLED_CLEARANCE_MAX_SECONDS)
                    hold_targets, hold_gains = clearance_hold_plan_for(
                        "shoulder_front",
                        ordered,
                        current,
                        completed_targets,
                    )
                    for group_name in group_names:
                        hold_targets.pop(group_name, None)
                        hold_gains.pop(group_name, None)
                    move_gains = {
                        group_name: COUPLED_CLEARANCE_MOVE_GAINS.get(group_name, CLEARANCE_MOVE_GAINS.get(group_name, {"kp": args.kp, "kd": args.kd}))
                        for group_name in group_names
                    }
                    print(
                        "v2.5.3 clearance coupled shoulder/roll/wrist active",
                        ",".join(group_names),
                        "hold joints:",
                        ", ".join(hold_targets.keys()),
                    )
                    for group_name in group_names:
                        gains = move_gains[group_name]
                        print("v2.5.3 clearance coupled shoulder/roll/wrist gains", group_name, "kp=", gains["kp"], "kd=", gains["kd"], "seconds=", coupled_seconds)
                    print(
                        "v2.5.3 clearance coupled shoulder/roll/wrist continuous interpolation steps_removed=",
                        coupled_steps,
                        "raw_seconds=",
                        raw_coupled_seconds,
                        "used_seconds=",
                        coupled_seconds,
                    )
                    print("v2.5.3 clearance coupled shoulder/roll/wrist progress windows=", json.dumps(COUPLED_CLEARANCE_PROGRESS_WINDOWS, ensure_ascii=False))
                    print("v2.5.3 clearance coupled shoulder/roll/wrist pre-window gains=", json.dumps(COUPLED_CLEARANCE_PRE_WINDOW_GAINS, ensure_ascii=False))
                    print("v2.5.3 clearance coupled shoulder/roll/wrist velocity_ff_joints=", ",".join(sorted(COUPLED_CLEARANCE_VELOCITY_FF_JOINTS)))
                    print("v2.5.3 clearance coupled shoulder/roll/wrist move_tau_ff=", json.dumps(COUPLED_CLEARANCE_MOVE_TAU_FF, ensure_ascii=False))
                    print("v2.5.3 clearance coupled shoulder/roll/wrist control_dt=", COUPLED_CLEARANCE_CONTROL_DT)
                    print("v2.5.3 clearance coupled shoulder/roll/wrist trajectory=", COUPLED_CLEARANCE_TRAJECTORY)
                    if COUPLED_CLEARANCE_TRAJECTORY == "blend_smootherstep":
                        print("v2.5.3 clearance coupled shoulder/roll/wrist linear_blend=", COUPLED_CLEARANCE_LINEAR_BLEND)
                    arm.enable(group_names + list(hold_targets.keys()))
                    arm.move_targets_with_holds(
                        group_targets,
                        seconds_per_step=coupled_seconds,
                        move_gains=move_gains,
                        hold_targets=hold_targets,
                        hold_gains=hold_gains,
                        fallback_kp=args.kp,
                        fallback_kd=args.kd,
                        hold_tau=CLEARANCE_JOINT_HOLD_TAU,
                        step_deg=0.0,
                        progress_windows=COUPLED_CLEARANCE_PROGRESS_WINDOWS,
                        pre_window_gains=COUPLED_CLEARANCE_PRE_WINDOW_GAINS,
                        control_dt=COUPLED_CLEARANCE_CONTROL_DT,
                        velocity_ff_joints=COUPLED_CLEARANCE_VELOCITY_FF_JOINTS,
                        move_tau_ff=COUPLED_CLEARANCE_MOVE_TAU_FF,
                        trajectory=COUPLED_CLEARANCE_TRAJECTORY,
                        linear_blend=COUPLED_CLEARANCE_LINEAR_BLEND,
                    )
                    time.sleep(COUPLED_CLEARANCE_SETTLE_SECONDS)
                    reached = arm.positions(group_names)
                    print("v2.5.3 clearance coupled shoulder/roll/wrist reached=", json.dumps(reached, ensure_ascii=False))
                    if "wrist" in group_names:
                        wrist_delta_deg = math.degrees(group_targets["wrist"] - reached["wrist"])
                        if abs(wrist_delta_deg) > CLEARANCE_WRIST_FINE_DEADBAND_DEG:
                            fine = COUPLED_CLEARANCE_WRIST_FINE_GAINS
                            fine_bias_deg = max(
                                -CLEARANCE_WRIST_FINE_MAX_BIAS_DEG,
                                min(CLEARANCE_WRIST_FINE_MAX_BIAS_DEG, wrist_delta_deg),
                            )
                            fine_target = group_targets["wrist"] + math.radians(fine_bias_deg)
                            fine_hold_targets = dict(hold_targets)
                            fine_hold_targets.update(
                                {
                                    group_name: group_targets[group_name]
                                    for group_name in group_names
                                    if group_name != "wrist"
                                }
                            )
                            fine_hold_gains = dict(hold_gains)
                            for group_name in group_names:
                                if group_name != "wrist":
                                    fine_hold_gains[group_name] = CLEARANCE_HOLD_GAINS.get(
                                        group_name,
                                        CLEARANCE_BASE_HOLD_GAINS[group_name],
                                    )
                            for compliant_name in CLEARANCE_COMPLIANT_HOLDS_BY_ACTIVE.get("wrist", set()):
                                if compliant_name in fine_hold_gains:
                                    fine_hold_gains[compliant_name] = CLEARANCE_COMPLIANT_HOLD_GAINS[compliant_name]
                            print("v2.5.3 clearance coupled wrist fine hold gains=", json.dumps(fine_hold_gains, ensure_ascii=False))
                            if CLEARANCE_WRIST_FINE_SETTLE_SECONDS > 0:
                                settle_targets = dict(fine_hold_targets)
                                settle_targets["wrist"] = group_targets["wrist"]
                                settle_gains = dict(fine_hold_gains)
                                settle_gains["wrist"] = CLEARANCE_HOLD_GAINS["wrist"]
                                print("v2.5.3 clearance coupled wrist settle before fine seconds=", CLEARANCE_WRIST_FINE_SETTLE_SECONDS)
                                arm.hold_positions_with_gains(
                                    settle_targets,
                                    seconds=CLEARANCE_WRIST_FINE_SETTLE_SECONDS,
                                    gains=settle_gains,
                                    fallback_kp=args.kp,
                                    fallback_kd=args.kd,
                                    hold_tau=CLEARANCE_JOINT_HOLD_TAU,
                                )
                            print(
                                "v2.5.3 clearance coupled wrist fine target delta_deg=",
                                wrist_delta_deg,
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
                            )
                            arm.move_target_with_holds(
                                "wrist",
                                fine_target,
                                seconds_per_step=fine["seconds"],
                                kp=fine["kp"],
                                kd=fine["kd"],
                                hold_targets=fine_hold_targets,
                                hold_gains=fine_hold_gains,
                                fallback_kp=args.kp,
                                fallback_kd=args.kd,
                                hold_tau=CLEARANCE_JOINT_HOLD_TAU,
                                active_tau=CLEARANCE_WRIST_FINE_ACTIVE_TAU,
                                control_dt=CLEARANCE_WRIST_FINE_CONTROL_DT,
                                active_velocity_ff=False,
                                step_deg=0.0,
                            )
                        else:
                            print(
                                "v2.5.3 clearance coupled wrist fine skip delta_deg=",
                                wrist_delta_deg,
                                "deadband_deg=",
                                CLEARANCE_WRIST_FINE_DEADBAND_DEG,
                            )
                    completed_targets.update(group_targets)
                    continue
                if args.auto_gains:
                    gains = CLEARANCE_MOVE_GAINS.get(name, {"kp": args.kp, "kd": args.kd})
                    kp = gains["kp"]
                    kd = gains["kd"]
                    seconds = CLEARANCE_MOVE_SECONDS.get(
                        name,
                        HOME_GAINS.get(name, {"seconds": args.seconds})["seconds"],
                    )
                    print("v2 clearance auto gains", name, "kp=", kp, "kd=", kd, "seconds=", seconds)
                else:
                    kp = args.kp
                    kd = args.kd
                    seconds = args.seconds
                hold_targets, hold_gains = clearance_hold_plan_for(name, ordered, current, completed_targets)
                print("v2 clearance active", name, "hold joints:", ", ".join(hold_targets.keys()))
                arm.enable([name] + list(hold_targets.keys()))
                arm.move_target_with_holds(
                    name,
                    targets[name],
                    seconds_per_step=seconds,
                    kp=kp,
                    kd=kd,
                    hold_targets=hold_targets,
                    hold_gains=hold_gains,
                    fallback_kp=args.kp,
                    fallback_kd=args.kd,
                    hold_tau=CLEARANCE_JOINT_HOLD_TAU,
                    active_velocity_ff=name in CLEARANCE_VELOCITY_FF_JOINTS,
                    step_deg=0.0 if name in CLEARANCE_CONTINUOUS_JOINTS else args.step_deg,
                )
                if name == "wrist":
                    if CLEARANCE_WRIST_FINE_SETTLE_SECONDS > 0:
                        settle_targets = dict(hold_targets)
                        settle_targets[name] = targets[name]
                        settle_gains = dict(hold_gains)
                        settle_gains[name] = CLEARANCE_HOLD_GAINS[name]
                        print("v2.5.3 clearance wrist settle before fine seconds=", CLEARANCE_WRIST_FINE_SETTLE_SECONDS)
                        arm.hold_positions_with_gains(
                            settle_targets,
                            seconds=CLEARANCE_WRIST_FINE_SETTLE_SECONDS,
                            gains=settle_gains,
                            fallback_kp=args.kp,
                            fallback_kd=args.kd,
                            hold_tau=CLEARANCE_JOINT_HOLD_TAU,
                        )
                    wrist_pos = arm.positions([name])[name]
                    wrist_delta_deg = math.degrees(targets[name] - wrist_pos)
                    if abs(wrist_delta_deg) > CLEARANCE_WRIST_FINE_DEADBAND_DEG:
                        fine = CLEARANCE_WRIST_FINE_GAINS
                        fine_bias_deg = max(
                            -CLEARANCE_WRIST_FINE_MAX_BIAS_DEG,
                            min(CLEARANCE_WRIST_FINE_MAX_BIAS_DEG, wrist_delta_deg),
                        )
                        fine_target = targets[name] + math.radians(fine_bias_deg)
                        print(
                            "v2.5.3 clearance wrist fine target delta_deg=",
                            wrist_delta_deg,
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
                        )
                        arm.move_target_with_holds(
                            name,
                            fine_target,
                            seconds_per_step=fine["seconds"],
                            kp=fine["kp"],
                            kd=fine["kd"],
                            hold_targets=hold_targets,
                            hold_gains=hold_gains,
                            fallback_kp=args.kp,
                            fallback_kd=args.kd,
                            hold_tau=CLEARANCE_JOINT_HOLD_TAU,
                            active_tau=CLEARANCE_WRIST_FINE_ACTIVE_TAU,
                            control_dt=CLEARANCE_WRIST_FINE_CONTROL_DT,
                            active_velocity_ff=False,
                            step_deg=0.0,
                        )
                    else:
                        print(
                            "v2.5.3 clearance wrist fine skip delta_deg=",
                            wrist_delta_deg,
                            "deadband_deg=",
                            CLEARANCE_WRIST_FINE_DEADBAND_DEG,
                        )
                completed_targets[name] = targets[name]
            arm.print_status(ordered)
            return

    finally:
        arm.close()


if __name__ == "__main__":
    main()
