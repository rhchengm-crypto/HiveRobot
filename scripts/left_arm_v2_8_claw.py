#!/usr/bin/env python3
"""Guarded v2.8 claw closing shared by standalone and placement flows."""
from __future__ import annotations

import json
import math
import time


CONTROL_DT = 0.04
CLOSE_RATE_RAD_S = 0.45
MAX_TARGET_LEAD_RAD = 0.10
MOVE_KP = 14.0
MOVE_KD = 1.2
CONTACT_TAU = 0.24
STALL_TAU = 0.18
STALL_VELOCITY = 0.08
STALL_ERROR_RAD = 0.06
CONTACT_CONFIRM_SAMPLES = 4
MIN_CONTACT_SECONDS = 0.50
TRACE_INTERVAL = 0.50
TIMEOUT_MARGIN_SECONDS = 4.0


def cached_motor_status(motor):
    """Return feedback already received by controlMIT without another serial read."""
    return {
        "pos": float(motor.getPosition()),
        "vel": float(motor.getVelocity()),
        "tau": float(motor.getTorque()),
    }


def resisting_torque(tau: float, direction: float) -> float:
    """Torque opposing the requested closing direction, as a positive value."""
    return max(0.0, -math.copysign(1.0, direction) * float(tau))


def lead_limited_target(position: float, planned: float, goal: float, direction: float) -> float:
    """Keep the command close to actual feedback so tracking error cannot run away."""
    proposed = position + direction * MAX_TARGET_LEAD_RAD
    return min(proposed, planned, goal) if direction > 0 else max(proposed, planned, goal)


def guarded_pressure_close(
    arm,
    close_offset: float,
    *,
    before_claw_command=None,
    hold_seconds: float = 0.8,
    hold_after: bool = False,
    hold_callback=None,
    label: str = "v2.8 claw",
):
    """Close from the live position with bounded tracking error and pressure stop.

    ``controlMIT`` receives and decodes the motor response. Reading the Motor
    getters afterwards avoids the second request/recv cycle used by v2.6.
    """
    motor = arm.motors["claw"]
    initial = arm.claw_status()
    start_pos = float(initial["pos"])
    direction = 1.0 if close_offset >= 0 else -1.0
    travel = abs(float(close_offset))
    goal = start_pos + direction * travel
    timeout = travel / max(CLOSE_RATE_RAD_S, 1e-6) + TIMEOUT_MARGIN_SECONDS
    print(label, "guarded pressure stop", flush=True)
    print(label, "live start:", start_pos, "travel:", direction * travel, "limit:", goal, flush=True)
    print(
        label,
        "control rate_rad_s=", CLOSE_RATE_RAD_S,
        "max_target_lead_rad=", MAX_TARGET_LEAD_RAD,
        "kp=", MOVE_KP,
        "kd=", MOVE_KD,
        flush=True,
    )

    started = time.time()
    next_trace = started
    confirm_count = 0
    contact = False
    contact_pos = None
    last_status = dict(initial)
    while time.time() - started < timeout:
        if before_claw_command is not None:
            before_claw_command()
        elapsed_before_command = time.time() - started
        scheduled_travel = min(travel, CLOSE_RATE_RAD_S * elapsed_before_command)
        planned = start_pos + direction * scheduled_travel
        target = lead_limited_target(float(last_status["pos"]), planned, goal, direction)
        try:
            arm.ctrl.controlMIT(motor, MOVE_KP, MOVE_KD, target, 0, 0)
        except Exception as exc:
            raise RuntimeError(
                "v2.8 claw serial feedback failed; the close command has stopped. "
                "Check that only one arm-control service is using the serial port and "
                "that the USB/CAN adapter did not reset under motor load."
            ) from exc
        time.sleep(CONTROL_DT)
        last_status = cached_motor_status(motor)
        elapsed = time.time() - started
        opposition = resisting_torque(last_status["tau"], direction)
        error = abs(target - last_status["pos"])

        if elapsed >= MIN_CONTACT_SECONDS:
            tau_hit = opposition >= CONTACT_TAU
            stall_hit = (
                opposition >= STALL_TAU
                and abs(last_status["vel"]) <= STALL_VELOCITY
                and error >= STALL_ERROR_RAD
            )
            confirm_count = confirm_count + 1 if (tau_hit or stall_hit) else 0
            if confirm_count >= CONTACT_CONFIRM_SAMPLES:
                contact = True
                contact_pos = float(last_status["pos"])
                print(
                    label,
                    "CONTACT pos=", contact_pos,
                    "vel=", last_status["vel"],
                    "tau=", last_status["tau"],
                    "resisting_tau=", opposition,
                    "target=", target,
                    flush=True,
                )
                break

        now = time.time()
        if now >= next_trace:
            print(
                label,
                "closing status=",
                json.dumps({
                    **last_status,
                    "target": target,
                    "target_error": target - last_status["pos"],
                    "resisting_tau": opposition,
                }, ensure_ascii=False),
                flush=True,
            )
            next_trace = now + TRACE_INTERVAL

        reached_limit = (
            last_status["pos"] >= goal - 0.02 if direction > 0
            else last_status["pos"] <= goal + 0.02
        )
        if reached_limit:
            break

    backoff = direction * 0.04
    hold_pos = (contact_pos - backoff) if contact else float(last_status["pos"])
    result = {
        "contact": contact,
        "contact_pos": contact_pos,
        "hold_pos": hold_pos,
        "live_start_pos": start_pos,
        "travel_limit": goal,
        "status": last_status,
    }
    print(label, "pressure result=", json.dumps(result, ensure_ascii=False, indent=2), flush=True)

    def command_hold():
        if hold_callback is not None:
            hold_callback(hold_pos)
        else:
            arm.ctrl.controlMIT(motor, 14.0, 0.7, hold_pos, 0, 0)

    end = time.time() + hold_seconds
    while time.time() < end:
        command_hold()
        time.sleep(CONTROL_DT)

    if not contact:
        raise RuntimeError(
            "v2.8 claw close stopped safely: pressure contact was not detected; "
            "the controller did not continue increasing tracking error"
        )
    if hold_after:
        print(label, "holding at:", hold_pos, "Ctrl+C or Claw Home to stop", flush=True)
        while True:
            command_hold()
            time.sleep(CONTROL_DT)
    return result
