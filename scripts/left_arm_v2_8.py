#!/usr/bin/env python3
"""v2.8 left-arm command entry point.

The v2.6 controller remains unchanged. For a Home command that uses pre-home
clearance, this wrapper guarantees that every selected joint participates in
the formal Home phase, then performs bounded correction passes using fresh
joint positions.
"""
from __future__ import annotations

import json
import math
import os
import copy
import sys
import time
from pathlib import Path
from typing import Iterable, List


SCRIPT_DIR = Path(__file__).resolve().parent
CLEARANCE_BIAS_PATH = SCRIPT_DIR / "data" / "left_arm_v2_8_clearance_bias.json"
CLEARANCE_TOLERANCE_DEG = 0.5
HOME_CAPTURED_TRANSITION_MARGIN_DEG = 5.0
CLEARANCE_COMMAND_MARGIN_DEG = 5.0
CLEARANCE_FINE_JOINTS = ("wrist_side",)
CLEARANCE_FINE_MAX_ERROR_DEG = 5.0
CLEARANCE_FINE_MAX_BIAS_DEG = 1.5
CLEARANCE_FINE_SECONDS = 6.0
CLEARANCE_FINE_MAX_ATTEMPTS = 2
CLEARANCE_FINE_ACTIVE_TAU_LIMIT = 1.0
CLEARANCE_FINE_GAINS = {
    "wrist_side": {"kp": 24.0, "kd": 3.0},
}


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


def ensure_clearance_best_snapshot(local):
    snapshots = local.anchor.setdefault("best_snapshots", {})
    if "clearance" in snapshots:
        return False
    snapshots["clearance"] = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "hold_bias": copy.deepcopy(local.anchor.get("hold_bias", {}).get("clearance", {})),
        "source": "active shared Clearance parameters before transactional verification",
    }
    local.anchor["updated_at"] = snapshots["clearance"]["created_at"]
    local._save()
    return True


def fine_correct_clearance_wrist_side(arm, nominal, validation_joints, legacy):
    current_all = arm.positions(legacy.DEFAULT_JOINTS)
    current = {joint: current_all[joint] for joint in validation_joints}
    errors = clearance_errors_deg(nominal, current, validation_joints)
    corrected = []
    for name in CLEARANCE_FINE_JOINTS:
        for attempt in range(1, CLEARANCE_FINE_MAX_ATTEMPTS + 1):
            # Refresh position and load before every pass. The second pass is
            # only used to remove small under-travel left by the first pass.
            start_status = arm.read_status(legacy.DEFAULT_JOINTS)
            current_all = {
                joint: start_status[joint]["pos"] for joint in legacy.DEFAULT_JOINTS
            }
            current = {joint: current_all[joint] for joint in validation_joints}
            errors = clearance_errors_deg(nominal, current, validation_joints)
            error_deg = errors.get(name, 0.0)
            if abs(error_deg) <= CLEARANCE_TOLERANCE_DEG + 0.011:
                break
            if abs(error_deg) > CLEARANCE_FINE_MAX_ERROR_DEG:
                print(
                    "v2.8 Clearance residual fine skip unsafe error=",
                    json.dumps({name: error_deg, "attempt": attempt}, ensure_ascii=False),
                    flush=True,
                )
                break
            if attempt == 1:
                bias_deg = max(
                    -CLEARANCE_FINE_MAX_BIAS_DEG,
                    min(CLEARANCE_FINE_MAX_BIAS_DEG, error_deg),
                )
                target = float(nominal[name]) + math.radians(bias_deg)
                active_tau = 0.0
                strategy = "position_residual"
            else:
                # Once the first pass has settled, its reported motor torque is
                # the best available estimate of the load needed at this pose.
                # Feed it forward while commanding the nominal position so the
                # joint no longer needs a standing position error for support.
                bias_deg = 0.0
                target = float(nominal[name])
                active_tau = max(
                    -CLEARANCE_FINE_ACTIVE_TAU_LIMIT,
                    min(CLEARANCE_FINE_ACTIVE_TAU_LIMIT, float(start_status[name]["tau"])),
                )
                strategy = "measured_load_feedforward"
            hold_targets = {
                joint: current_all[joint]
                for joint in legacy.DEFAULT_JOINTS if joint != name
            }
            hold_gains = {
                joint: legacy.CLEARANCE_HOLD_GAINS.get(
                    joint, legacy.CLEARANCE_BASE_HOLD_GAINS[joint]
                )
                for joint in hold_targets
            }
            # Preserve the exact load observed immediately before each pass.
            hold_tau = {joint: start_status[joint]["tau"] for joint in hold_targets}
            gains = CLEARANCE_FINE_GAINS[name]
            print("v2.8 Clearance residual fine=", json.dumps({
                "joint": name,
                "attempt": attempt,
                "max_attempts": CLEARANCE_FINE_MAX_ATTEMPTS,
                "nominal_error_deg": error_deg,
                "bias_deg": bias_deg,
                "target_rad": target,
                "strategy": strategy,
                "active_tau": active_tau,
                "seconds": CLEARANCE_FINE_SECONDS,
                "kp": gains["kp"],
                "kd": gains["kd"],
                "hold_targets_rad": hold_targets,
                "inherited_hold_tau": hold_tau,
            }, ensure_ascii=False), flush=True)
            arm.move_target_with_holds(
                name,
                target,
                seconds_per_step=CLEARANCE_FINE_SECONDS,
                kp=gains["kp"],
                kd=gains["kd"],
                hold_targets=hold_targets,
                hold_gains=hold_gains,
                fallback_kp=3.0,
                fallback_kd=0.3,
                hold_tau=hold_tau,
                active_tau=active_tau,
                control_dt=legacy.COUPLED_CLEARANCE_CONTROL_DT,
                active_velocity_ff=False,
                step_deg=0.0,
            )
            current_all = arm.positions(legacy.DEFAULT_JOINTS)
            current = {joint: current_all[joint] for joint in validation_joints}
            errors = clearance_errors_deg(nominal, current, validation_joints)
            corrected.append({
                "joint": name,
                "attempt": attempt,
                "final_error_deg": errors[name],
            })
    if corrected:
        print("v2.8 Clearance residual fine result=", json.dumps(corrected, ensure_ascii=False), flush=True)
    return current


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
        recover_placement1_shared_clearance_contamination,
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
    recovered = recover_placement1_shared_clearance_contamination(local)
    if recovered:
        print(
            "v2.8 Clearance history restored only; no motor controller was opened and no motion was issued.",
            flush=True,
        )
        return
    snapshot_created = ensure_clearance_best_snapshot(local)
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
        if parsed.execute:
            final_positions.update(
                fine_correct_clearance_wrist_side(arm, nominal, validation_joints, legacy)
            )
        else:
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
        "best_snapshot_created": snapshot_created,
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
    validation = local.record_move_validation(errors)
    print("v2.8 clearance final verification=", json.dumps(validation, ensure_ascii=False), flush=True)
    blockers = final_blocking_joint_errors(errors, CLEARANCE_TOLERANCE_DEG)
    if blockers:
        raise RuntimeError(
            "v2.8 clearance verification failed: joints exceed 0.5deg; active learned biases "
            "were not changed and the best snapshot was retained: " + json.dumps(blockers, ensure_ascii=False)
        )
    updates = local.update_hold_bias("clearance", errors, label="clearance-validated")
    if updates:
        print("v2.8 clearance validated learning update=", json.dumps(updates, ensure_ascii=False), flush=True)


def restore_clearance_history(original: List[str], legacy) -> None:
    from left_arm_v2_8_move_library import (
        LocalTargetBias,
        recover_placement1_shared_clearance_contamination,
    )

    clearance_file = option_value(original, "--clearance-file", legacy.TABLE_CLEARANCE_PATH)
    nominal = legacy.load_pose(clearance_file)
    local = LocalTargetBias(
        CLEARANCE_BIAS_PATH,
        nominal,
        "clearance:" + Path(clearance_file).name,
    )
    recovered = recover_placement1_shared_clearance_contamination(local, force=True)
    local.anchor.pop("best_snapshots", None)
    ensure_clearance_best_snapshot(local)
    backup_path = Path(str(CLEARANCE_BIAS_PATH) + ".pre_placement1_restored.json")
    if not backup_path.exists():
        backup_path.write_text(
            json.dumps(local.data, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print("v2.8 explicit Clearance history restore=", json.dumps({
        "ok": True,
        "clearance_file": os.path.abspath(clearance_file),
        "anchor_id": local.anchor_id,
        "restored": recovered,
        "motor_controller_opened": False,
        "motion_issued": False,
        "immutable_backup_path": str(backup_path),
    }, ensure_ascii=False), flush=True)


def main() -> None:
    import left_arm_v2_6 as legacy

    original = list(sys.argv[1:])
    if original and original[0] == "restore-clearance-history":
        restore_clearance_history(original, legacy)
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
