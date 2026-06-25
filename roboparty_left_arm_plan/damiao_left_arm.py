import json
import math
import os
import time
from dataclasses import dataclass

import serial

from DM_CAN import DM_Motor_Type, Motor, MotorControl


def parse_int(value):
    if isinstance(value, int):
        return value
    return int(str(value), 0)


@dataclass
class JointSpec:
    name: str
    model: str
    master_id: int
    slave_id: int
    installed: bool
    home_rad: float
    min_rad: float
    max_rad: float
    kp_hold: float
    kd_hold: float
    kp_move: float
    kd_move: float


class LeftArm:
    def __init__(self, config_path=None, include_uninstalled=False):
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "left_arm_config.json")

        with open(config_path, "r", encoding="utf-8") as f:
            self.config = json.load(f)

        self.specs = []
        for item in self.config["joints"]:
            if not include_uninstalled and not item.get("installed", False):
                continue

            self.specs.append(
                JointSpec(
                    name=item["name"],
                    model=item["model"],
                    master_id=parse_int(item["master_id"]),
                    slave_id=parse_int(item["slave_id"]),
                    installed=bool(item.get("installed", False)),
                    home_rad=float(item["home_rad"]),
                    min_rad=float(item["min_rad"]),
                    max_rad=float(item["max_rad"]),
                    kp_hold=float(item["kp_hold"]),
                    kd_hold=float(item["kd_hold"]),
                    kp_move=float(item["kp_move"]),
                    kd_move=float(item["kd_move"]),
                )
            )

        self.serial_device = None
        self.ctrl = None
        self.motors = {}

    def connect(self, port=None, baudrate=None, timeout=1.0):
        port = port or self.config.get("serial_port", "/dev/ttyACM0")
        baudrate = int(baudrate or self.config.get("baudrate", 921600))
        self.serial_device = serial.Serial(port, baudrate, timeout=timeout)
        self.ctrl = MotorControl(self.serial_device)

        for spec in self.specs:
            motor_type = getattr(DM_Motor_Type, spec.model)
            motor = Motor(motor_type, spec.slave_id, spec.master_id)
            self.ctrl.addMotor(motor)
            self.motors[spec.name] = motor

        print("Serial port is open")

    @property
    def period(self):
        return 1.0 / float(self.config.get("control_hz", 100))

    def enable_all(self, delay=0.12):
        for spec in self.specs:
            self.ctrl.enable(self.motors[spec.name])
            time.sleep(delay)

    def disable_all(self, delay=0.08):
        for spec in self.specs:
            self.ctrl.disable(self.motors[spec.name])
            time.sleep(delay)

    def refresh(self):
        for spec in self.specs:
            self.ctrl.refresh_motor_status(self.motors[spec.name])
        time.sleep(0.02)
        self.ctrl.recv()

    def state(self):
        self.refresh()
        out = {}
        for spec in self.specs:
            motor = self.motors[spec.name]
            out[spec.name] = {
                "q": float(motor.getPosition()),
                "dq": float(motor.getVelocity()),
                "tau": float(motor.getTorque()),
            }
        return out

    def home_targets(self):
        return {spec.name: spec.home_rad for spec in self.specs}

    def clamp_targets(self, targets):
        clamped = {}
        for spec in self.specs:
            target = float(targets.get(spec.name, spec.home_rad))
            clamped[spec.name] = min(max(target, spec.min_rad), spec.max_rad)
        return clamped

    def command_targets(self, targets, moving_name=None, kp_scale=1.0):
        targets = self.clamp_targets(targets)
        for spec in self.specs:
            if moving_name == spec.name:
                kp = spec.kp_move * kp_scale
                kd = spec.kd_move
            else:
                kp = spec.kp_hold * kp_scale
                kd = spec.kd_hold

            self.ctrl.controlMIT(self.motors[spec.name], kp, kd, targets[spec.name], 0.0, 0.0)

    def hold(self, targets=None, seconds=1.0, kp_scale=1.0):
        targets = targets or self.home_targets()
        end_time = time.time() + seconds
        while time.time() < end_time:
            self.command_targets(targets, kp_scale=kp_scale)
            time.sleep(self.period)

    def settle(self, targets=None, tol=0.015, stable_time=0.8, timeout=12.0, kp_scale=1.3):
        targets = self.clamp_targets(targets or self.home_targets())
        start = time.time()
        stable_start = None
        last_errors = {}

        while time.time() - start < timeout:
            self.command_targets(targets, kp_scale=kp_scale)
            time.sleep(self.period)
            current = self.state()

            max_err = 0.0
            last_errors = {}
            for spec in self.specs:
                err = abs(current[spec.name]["q"] - targets[spec.name])
                last_errors[spec.name] = err
                max_err = max(max_err, err)

            if max_err <= tol:
                if stable_start is None:
                    stable_start = time.time()
                if time.time() - stable_start >= stable_time:
                    print("settled, max_err=", max_err)
                    return True, last_errors
            else:
                stable_start = None

        print("settle timeout")
        for name, err in last_errors.items():
            print(name, "err=", err)
        return False, last_errors

    def cosine_sine_target(self, base, active_name, amp, freq, elapsed):
        offset = amp * math.sin(2.0 * math.pi * freq * elapsed)
        targets = dict(base)
        targets[active_name] = base[active_name] + offset
        return targets, offset

    def ramp_down_hold(self, targets=None):
        targets = targets or self.home_targets()
        for kp_scale, seconds in [(1.0, 0.8), (0.7, 0.8), (0.45, 0.8)]:
            self.hold(targets, seconds=seconds, kp_scale=kp_scale)

