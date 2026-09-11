#!/usr/bin/env python3
"""v2.8 left-arm command entry point.

The v2.6 controller remains unchanged. For a Home command that uses pre-home
clearance, this wrapper guarantees that every selected joint participates in
the formal Home phase, then performs one bounded correction pass using fresh
joint positions.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path
from typing import Iterable, List


SCRIPT_DIR = Path(__file__).resolve().parent
CLEARANCE_BIAS_PATH = SCRIPT_DIR / "data" / "left_arm_v2_8_clearance_bias.json"
CLEARANCE_TOLERANCE_DEG = 0.5
HOME_CAPTURED_TRANSITION_MARGIN_DEG = 5.0
CLEARANCE_COMMAND_MARGIN_DEG = 5.0


def option_value(argv: List[str], option: str, default: str) -> str:
    try:
        return argv[argv.index(option) + 1]
    except (ValueError, IndexError):
        return default


def replace_option(argv: List[str], option: str, value: str) -> List[str]:
    result = list(argv)
    try:
        index = result.index(option)
    except ValueError:
        result.extend([option, value])
    else:
        if index + 1 >= len(result):
            raise ValueError(f"{option} requires a value")
        result[index + 1] = value
    return result


def remove_flag(argv: Iterable[str], flag: str) -> List[str]:
    return [value for value in argv if value != flag]


def is_fixed_prehome(argv: List[str]) -> bool:
    return bool(argv and argv[0] == "home" and "--prehome-clearance" in argv)


def clearance_validation_joints(legacy, selected: Iterable[str]) -> List[str]:
    return [
        name for name in selected
        if float(legacy.CLEARANCE_JOINT_DEADBANDS_DEG.get(name, 0.0)) < 180.0
    ]


def clearance_errors_deg(target, current, joints: Iterable[str]):
    return {
        name: math.degrees(float(target[name]) - float(current[name]))
        for name in joints if name in target and name in current
    }


def captured_home_transition_limit(requested_limit_deg: float,
                                   transition_deltas_deg: Iterable[float]) -> float:
    largest = max((abs(float(value)) for value in transition_deltas_deg), default=0.0)
    if largest <= requested_limit_deg:
        return float(requested_limit_deg)
    if largest <= requested_limit_deg + HOME_CAPTURED_TRANSITION_MARGIN_DEG:
        return largest + 0.1
    return float(requested_limit_deg)


def run_trained_clearance(original: List[str], legacy) -> None:
    from left_arm_v2_8_move_library import (
        LocalTargetBias,
        final_blocking_joint_errors,
        rollback_rejected_placement1_learning,
    )

    parsed = legacy.build_parser().parse_args(original)
    expanded = replace_option(
        original,
        "--max-delta-deg",
        str(parsed.max_delta_deg + CLEARANCE_COMMAND_MARGIN_DEG),
    )
    nominal = legacy.load_pose(parsed.clearance_file)
    selected = [name for name in legacy.parse_joints(parsed.joints) if name in nominal]
    validation_joints = clearance_validation_joints(legacy, selected)
    local = LocalTargetBias(
        CLEARANCE_BIAS_PATH,
        nominal,
        "clearance:" + Path(parsed.clearance_file).name,
    )
    # A failed payload carry must not poison the shared clearance target used
    # by the next pre-replay clearance command.
    rollback_rejected_placement1_learning(local)
    biased = dict(nominal)
    applied_bias_deg = {}
    for name in validation_joints:
        offset = local.hold_bias_rad("clearance", name)
        if offset:
            biased[name] += offset
            applied_bias_deg[name] = math.degrees(offset)

    original_load_pose = legacy.load_pose
    original_print_status = legacy.LeftArmV2.print_status
    final_positions = {}
    clearance_path = os.path.abspath(parsed.clearance_file)

    def load_biased_pose(path):
        if os.path.abspath(path) == clearance_path:
            return dict(biased)
        return original_load_pose(path)

    def capture_status(arm, joints):
        final_positions.update(arm.positions(joints))
        return original_print_status(arm, joints)

    print("v2.8 clearance local correction=", json.dumps({
        "clearance_file": clearance_path,
        "anchor_id": local.anchor_id,
        "new_anchor": local.created,
        "nominal_target_rad": {name: nominal[name] for name in selected},
        "applied_bias_deg": applied_bias_deg,
        "validation_joints": validation_joints,
        "requested_max_delta_deg": parsed.max_delta_deg,
        "effective_max_delta_deg": parsed.max_delta_deg + CLEARANCE_COMMAND_MARGIN_DEG,
    }, ensure_ascii=False), flush=True)
    legacy.load_pose = load_biased_pose
    legacy.LeftArmV2.print_status = capture_status
    try:
        sys.argv = [sys.argv[0], *expanded]
        legacy.main()
    finally:
        legacy.load_pose = original_load_pose
        legacy.LeftArmV2.print_status = original_print_status

    if not parsed.execute or not final_positions:
        return
    errors = clearance_errors_deg(nominal, final_positions, validation_joints)
    updates = local.update_hold_bias("clearance", errors, label="clearance-final")
    validation = local.record_move_validation(errors)
    print("v2.8 clearance final verification=", json.dumps(validation, ensure_ascii=False), flush=True)
    if updates:
        print("v2.8 clearance local learning update=", json.dumps(updates, ensure_ascii=False), flush=True)
    blockers = final_blocking_joint_errors(errors, CLEARANCE_TOLERANCE_DEG)
    if blockers:
        raise RuntimeError(
            "v2.8 clearance training incomplete: joints exceed 0.5deg; learned data was saved; "
            "run clearance again before Home or Replay: " + json.dumps(blockers, ensure_ascii=False)
        )


def run_guarded_claw_close(original: List[str], legacy) -> None:
    from left_arm_v2_8_claw import guarded_pressure_close

    parsed = legacy.build_parser().parse_args(original)
    home = legacy.load_pose(parsed.claw_home_file)
    if "claw" not in home:
        raise RuntimeError("invalid claw home file: missing claw")
    if not parsed.execute:
        legacy.main()
        return
    arm = legacy.LeftArmV2()
    try:
        print("Serial port is open", flush=True)
        arm.enable(["claw"])
        guarded_pressure_close(
            arm,
            parsed.close_offset,
            hold_after=not parsed.no_hold,
            label="v2.8 standalone claw",
        )
    finally:
        arm.close()


def main() -> None:
    import left_arm_v2_6 as legacy

    original = list(sys.argv[1:])
    if original and original[0] == "claw-close":
        run_guarded_claw_close(original, legacy)
        return
    if original and original[0] == "clearance":
        run_trained_clearance(original, legacy)
        return
    if not is_fixed_prehome(original):
        legacy.main()
        return

    parsed = legacy.build_parser().parse_args(original)
    home = legacy.load_home(parsed.home_file)
    clearance = legacy.load_pose(parsed.clearance_file)
    selected = [name for name in legacy.parse_joints(parsed.joints) if name in home]
    transition_deltas = {
        name: abs(legacy.math.degrees(home[name] - clearance[name]))
        for name in selected
        if name in clearance
    }
    effective_max_delta = captured_home_transition_limit(
        parsed.max_delta_deg,
        transition_deltas.values(),
    )
    unsafe = {
        name: delta for name, delta in transition_deltas.items()
        if delta > effective_max_delta
    }
    if unsafe:
        report = ", ".join(f"{name}={delta:.2f}deg" for name, delta in unsafe.items())
        raise RuntimeError(
            "v2.8 Home refused: clearance-to-home delta exceeds --max-delta-deg: " + report
        )

    expanded = replace_option(original, "--max-delta-deg", str(effective_max_delta))
    if effective_max_delta > parsed.max_delta_deg:
        print(
            "v2.8 Home captured-pose limit margin: requested_deg=",
            parsed.max_delta_deg,
            "effective_deg=",
            effective_max_delta,
            "largest_transition_deg=",
            max(transition_deltas.values(), default=0.0),
            flush=True,
        )

    # v2.6 determines its target set before clearance. Disable all Home
    # deadbands only for this first pass so joints displaced by clearance
    # cannot disappear from the later formal Home phase.
    legacy.HOME_JOINT_DEADBANDS_DEG = {name: -1.0 for name in selected}
    forced = replace_option(expanded, "--deadband-deg", "-1.0")
    print(
        "v2.8 Home pre-clearance fix: formal Home includes all selected joints=",
        ",".join(selected),
        flush=True,
    )
    sys.argv = [sys.argv[0], *forced]
    legacy.main()

    if not parsed.execute:
        return

    # Re-open the controller once and let the normal deadbands select only
    # residual errors. This is a single bounded pass, not an endless loop.
    original_deadband = option_value(original, "--deadband-deg", str(parsed.deadband_deg))
    correction = remove_flag(expanded, "--prehome-clearance")
    correction = replace_option(correction, "--deadband-deg", original_deadband)
    legacy.HOME_JOINT_DEADBANDS_DEG = {
        "arm_roll": 3.0,
        "shoulder_rotate": 3.0,
    }
    print(
        "v2.8 Home verification correction pass: recomputing from fresh joint positions",
        flush=True,
    )
    sys.argv = [sys.argv[0], *correction]
    legacy.main()


if __name__ == "__main__":
    main()
