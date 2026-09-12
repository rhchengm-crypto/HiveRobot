#!/usr/bin/env python3
"""v2.8 saved-move entry point with pose-local target correction.

All motion remains in the v2.6 implementation. During replay this wrapper
adds a pose-local target bias on top of the existing shared v2.6 bias.
Non-replay commands are delegated unchanged.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Dict, Iterable, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
V28_MOVE_BUILD = "v2.8-white-bishop-placement-v1"
LOCAL_BIAS_PATH = SCRIPT_DIR / "data" / "left_arm_v2_8_local_target_bias.json"
PLACEMENT1_CLEARANCE_BIAS_PATH = SCRIPT_DIR / "data" / "left_arm_v2_8_placement1_clearance_bias.json"
JOINTS = (
    "shoulder_front", "shoulder_side", "shoulder_rotate", "elbow",
    "arm_roll", "wrist_side", "wrist",
)
MATCH_RMS_DEG = 10.0
MATCH_MAX_DEG = 20.0
ERROR_DEADBAND_DEG = 0.5
# Joint positions in the observed DM feedback advance in roughly
# 0.0218566-degree counts.  A requested 0.5-degree gate can therefore land on
# either side of the floating-point boundary even though both readings denote
# the same achievable encoder bin.  Allow half a count only when deciding
# whether a joint blocks the next motion; keep the nominal training target at
# 0.5 degrees.
ENCODER_HALF_COUNT_MARGIN_DEG = 0.011
IMPROVEMENT_DEADBAND_DEG = 0.15
STEP_SCALE = 0.5
MAX_STEP_DEG = 0.75
MIN_STEP_SCALE = 0.2
BIAS_LIMIT_DEG = 5.0
PRE_WRIST_TOLERANCE_DEG = 0.5
HOLD_BIAS_STEP_SCALE = 0.35
HOLD_BIAS_MAX_STEP_DEG = 0.40
HOLD_BIAS_MIN_STEP_SCALE = 0.25
HOLD_BIAS_LIMIT_DEG = 3.0
PLACEMENT1_HOLD_BIAS_LIMIT_DEG = 5.0
PLACEMENT1_MOVE_NAME = "bishop01"
WHITE_BISHOP_PLACE_MOVE_NAME = "white_bishop_place"
PLACEMENT1_MAX_LEARNABLE_ERROR_DEG = 5.0
PLACEMENT1_TERMINAL_SETTLE_SECONDS = 1.5
PLACEMENT1_SHARED_CLEARANCE_RECOVERY_ID = "restore-pre-placement1-clearance-20260911-v2"
PLACEMENT1_WRIST_SIDE_WORSENING_RECOVERY_ID = "restore-placement1-wrist-side-20260911-2043-v1"
PLACEMENT1_WRIST_SIDE_RETRY_RECOVERY_ID = "avoid-rejected-placement1-wrist-side-retry-20260911-v1"
WRIST_SIDE_ACTIVE_TAU_MIGRATION_ID = "replay-wrist-side-active-tau-20260911-v1"
PLACEMENT1_CONTAMINATED_CLEARANCE_ANCHOR = "8b20a284d5592a0c"
# Values printed immediately before the first Placement1 run.  Placement1
# subsequently wrote arm_roll, wrist_side and wrist into this shared anchor.
PLACEMENT1_PRECONTAMINATION_CLEARANCE_BIASES_DEG = {
    "arm_roll": 0.3671909697798855,
    "wrist_side": 0.6578945784035709,
    "wrist": -0.820746824571414,
}


def command_holds(arm, targets, gains, hold_tau, fallback_kp=3.0, fallback_kd=0.3):
    for name, target in targets.items():
        joint_gains = gains.get(name, {})
        arm.ctrl.controlMIT(
            arm.motors[name],
            joint_gains.get("kp", fallback_kp),
            joint_gains.get("kd", fallback_kd),
            target,
            0,
            hold_tau.get(name, 0.0),
        )


def close_claw_while_holding_arm(arm, arm_targets, api, arm_gains=None, arm_tau=None):
    """Run the v2.6 pressure-stop close while continuously holding the arm."""
    claw_home = api.load_pose(api.CLAW_HOME_PATH)
    if "claw" not in claw_home:
        raise RuntimeError("invalid claw home file: missing claw")
    arm.enable([*arm_targets.keys(), "claw"])
    arm_gains = arm_gains or {
        name: api.CLEARANCE_HOLD_GAINS.get(name, api.CLEARANCE_BASE_HOLD_GAINS[name])
        for name in arm_targets
    }
    arm_tau = arm_tau or {name: 0.0 for name in arm_targets}
    home_pos = float(claw_home["claw"])
    q_close = home_pos + api.CLAW_CLOSE_OFFSET
    print("v2.8 placement1 claw close pressure stop with arm holds", flush=True)
    contact = False
    contact_pos = home_pos
    confirm_count = 0
    last_status = arm.claw_status()
    started = time.time()
    while time.time() - started < api.CLAW_CLOSE_SECONDS:
        elapsed = time.time() - started
        progress = elapsed / max(api.CLAW_CLOSE_SECONDS, 1e-6)
        target = home_pos * (1.0 - api.cosine_smoothstep(progress)) + q_close * api.cosine_smoothstep(progress)
        command_holds(arm, arm_targets, arm_gains, arm_tau)
        arm.ctrl.controlMIT(arm.motors["claw"], api.CLAW_KP_MOVE, api.CLAW_KD_MOVE, target, 0, 0)
        time.sleep(0.01)
        last_status = arm.claw_status()
        if elapsed > 0.5:
            tau_hit = last_status["tau"] > api.CLAW_TAU_THRESHOLD
            stall_hit = (
                abs(last_status["vel"]) < api.CLAW_VEL_STALL_THRESHOLD
                and abs(target - last_status["pos"]) > 0.15
                and last_status["tau"] > api.CLAW_STALL_TAU_THRESHOLD
            )
            confirm_count = confirm_count + 1 if (tau_hit or stall_hit) else 0
            if confirm_count >= api.CLAW_CONFIRM_COUNT_NEEDED:
                contact = True
                contact_pos = last_status["pos"]
                break
    hold_pos = contact_pos - api.CLAW_BACKOFF if contact else last_status["pos"]
    result = {
        "contact": contact,
        "contact_pos": contact_pos if contact else None,
        "hold_pos": hold_pos,
        "status": last_status,
    }
    print("v2.8 placement1 claw pressure result=", json.dumps(result, ensure_ascii=False), flush=True)
    hold_targets = dict(arm_targets)
    hold_targets["claw"] = hold_pos
    hold_gains = dict(arm_gains)
    hold_gains["claw"] = {"kp": api.CLAW_KP_HOLD, "kd": api.CLAW_KD_HOLD}
    hold_tau = dict(arm_tau)
    hold_tau["claw"] = 0.0
    end = time.time() + 0.8
    while time.time() < end:
        command_holds(arm, hold_targets, hold_gains, hold_tau)
        time.sleep(0.01)
    if not contact:
        raise RuntimeError(
            "v2.8 placement1 stopped after claw close: pressure contact was not detected; "
            "clearance carry was not started"
        )
    return hold_pos


def rollback_rejected_placement1_learning(local):
    validation = local.anchor.get("move_validation", {}).get(local.move_name, {})
    errors = validation.get("errors_deg", {}) if isinstance(validation, dict) else {}
    if not errors or max(abs(float(value)) for value in errors.values()) <= PLACEMENT1_MAX_LEARNABLE_ERROR_DEG:
        return {}
    rules = local.anchor.get("hold_bias", {}).get("clearance", {})
    rolled_back = {}
    for joint, record in rules.items():
        if not isinstance(record, dict) or record.get("label") != "placement1-clearance-final":
            continue
        previous = float(record.get("previous_bias_deg", record.get("bias_deg", 0.0)))
        rolled_back[joint] = {"rejected_bias_deg": record.get("bias_deg"), "restored_bias_deg": previous}
        record["bias_deg"] = previous
        record["samples"] = max(0, int(record.get("samples", 1)) - 1)
        record["label"] = "placement1-rejected-rollback"
        record["rolled_back_at"] = _now()
    if rolled_back:
        local.anchor["updated_at"] = _now()
        local._save()
        print("v2.8 placement1 rejected learning rollback=", json.dumps(rolled_back, ensure_ascii=False), flush=True)
    return rolled_back


def recover_placement1_shared_clearance_contamination(local, force=False):
    """Restore the shared clearance anchor once, using pre-Placement1 evidence.

    The first Placement1 implementation reused the ordinary clearance learner.
    Its failed payload carry therefore changed three values that were already
    trained.  The exact pre-run values are available in the 14:48 execution
    log; bind the repair to that pose hash so no other clearance is touched.
    """
    if local.anchor_id != PLACEMENT1_CONTAMINATED_CLEARANCE_ANCHOR:
        return {}
    migrations = local.anchor.setdefault("migrations", {})
    if migrations.get(PLACEMENT1_SHARED_CLEARANCE_RECOVERY_ID) and not force:
        return {}
    rules = local.anchor.setdefault("hold_bias", {}).setdefault("clearance", {})
    recovered = {}
    for joint, restored in PLACEMENT1_PRECONTAMINATION_CLEARANCE_BIASES_DEG.items():
        previous = rules.get(joint, {})
        contaminated = (
            float(previous.get("bias_deg", 0.0))
            if isinstance(previous, dict) else float(previous)
        )
        rules[joint] = {
            "bias_deg": restored,
            "previous_bias_deg": contaminated,
            "delta_bias_deg": restored - contaminated,
            "last_error_deg": 0.0,
            "step_scale": 1.0,
            "learning_state": "restored_pre_placement1",
            "samples": int(previous.get("samples", 0)) if isinstance(previous, dict) else 0,
            "updated_at": _now(),
            "label": "placement1-shared-clearance-recovery",
        }
        recovered[joint] = {
            "contaminated_bias_deg": contaminated,
            "restored_bias_deg": restored,
        }
    migrations[PLACEMENT1_SHARED_CLEARANCE_RECOVERY_ID] = {
        "applied_at": _now(),
        "source": "pre-Placement1 execution log 2026-09-11 14:48",
        "recovered": recovered,
    }
    local.anchor["updated_at"] = _now()
    local._save()
    print(
        "v2.8 shared clearance Placement1 contamination recovery=",
        json.dumps(recovered, ensure_ascii=False),
        flush=True,
    )
    return recovered


def run_placement1_on_arm(arm, clearance_file: str, fallback_kp: float, fallback_kd: float,
                          carry_hold=None, clearance_handoff=None):
    """Carry the gripped bishop to learned clearance without releasing holds."""
    import left_arm_v2_6 as api
    from left_arm_v2_8 import CLEARANCE_BIAS_PATH, clearance_errors_deg

    nominal = api.load_pose(clearance_file)
    selected = [name for name in api.DEFAULT_CLEARANCE_ORDER if name in nominal]
    # Placement1 must reproduce the complete captured Clearance pose. Ordinary
    # Table Clearance may intentionally skip shoulder_rotate, but preserving
    # bishop01's shoulder_rotate here leaves the arm about 44 degrees away.
    validation_joints = list(selected)
    shared = LocalTargetBias(
        CLEARANCE_BIAS_PATH,
        nominal,
        "clearance:" + Path(clearance_file).name,
    )
    rollback_rejected_placement1_learning(shared)
    recovered_shared = recover_placement1_shared_clearance_contamination(shared)
    if recovered_shared:
        raise RuntimeError(
            "v2.8 restored the pre-Placement1 shared Clearance history without moving; "
            "run the requested action again only after reviewing the restored values"
        )
    local = LocalTargetBias(
        PLACEMENT1_CLEARANCE_BIAS_PATH,
        nominal,
        "placement1-clearance:" + Path(clearance_file).name,
    )
    local.recover_known_placement1_wrist_side_worsening()
    local.avoid_repeating_rejected_placement1_wrist_side_step()
    biased = dict(nominal)
    applied_bias_deg = {}
    shared_bias_deg = {}
    placement_bias_deg = {}
    for name in validation_joints:
        shared_offset = shared.hold_bias_rad("clearance", name)
        placement_offset = local.hold_bias_rad(
            "clearance", name, limit_deg=PLACEMENT1_HOLD_BIAS_LIMIT_DEG
        )
        offset = shared_offset + placement_offset
        if offset:
            biased[name] += offset
            applied_bias_deg[name] = math.degrees(offset)
        if shared_offset:
            shared_bias_deg[name] = math.degrees(shared_offset)
        if placement_offset:
            placement_bias_deg[name] = math.degrees(placement_offset)

    carry_hold = carry_hold or {}
    # Continue the exact final-wrist control law into claw closing. Replacing
    # these targets with measured positions would erase the PD position error
    # that is currently producing the wrist's load-supporting torque.
    arm_targets = dict(
        carry_hold.get("hold_targets") or arm.positions(api.DEFAULT_JOINTS)
    )
    print("v2.8 placement1 seamless takeover=", json.dumps({
        "source_move": PLACEMENT1_MOVE_NAME,
        "arm_hold_targets_rad": arm_targets,
        "clearance_file": os.path.abspath(clearance_file),
        "applied_bias_deg": applied_bias_deg,
        "shared_clearance_bias_deg_read_only": shared_bias_deg,
        "placement1_bias_deg": placement_bias_deg,
        "placement1_bias_file": str(PLACEMENT1_CLEARANCE_BIAS_PATH),
        "carry_hold_source": (
            "final_wrist_controller" if carry_hold.get("hold_targets") else "measured_fallback"
        ),
    }, ensure_ascii=False), flush=True)
    carry_gains = carry_hold.get("hold_gains", {})
    carry_tau = carry_hold.get("hold_tau", {})
    claw_hold_pos = close_claw_while_holding_arm(
        arm, arm_targets, api, arm_gains=carry_gains, arm_tau=carry_tau
    )

    # Retract wrist to Clearance before moving the rest of the arm so the
    # gripper and carried piece rise out of the occupied board volume first.
    wrist_hold_target = float(arm_targets["wrist"])
    wrist_hold_tau = float(carry_tau.get("wrist", 0.0))
    wrist_move_gains = api.COUPLED_CLEARANCE_MOVE_GAINS.get(
        "wrist", api.CLEARANCE_MOVE_GAINS["wrist"]
    )
    wrist_hold_gains = dict(wrist_move_gains)
    pre_wrist_holds = {
        name: target for name, target in arm_targets.items() if name != "wrist"
    }
    pre_wrist_holds["claw"] = claw_hold_pos
    pre_wrist_gains = {
        name: carry_gains.get(
            name,
            api.CLEARANCE_HOLD_GAINS.get(name, api.CLEARANCE_BASE_HOLD_GAINS[name]),
        )
        for name in pre_wrist_holds if name != "claw"
    }
    pre_wrist_gains["claw"] = {"kp": api.CLAW_KP_HOLD, "kd": api.CLAW_KD_HOLD}
    pre_wrist_tau = {
        name: float(carry_tau.get(name, 0.0))
        for name in pre_wrist_holds if name != "claw"
    }
    pre_wrist_tau["claw"] = 0.0
    print("v2.8 placement1 wrist-first retract=", json.dumps({
        "nominal_clearance_rad": nominal["wrist"],
        "learned_command_rad": biased["wrist"],
        "start_controller_target_rad": wrist_hold_target,
        "held_joints": list(pre_wrist_holds),
    }, ensure_ascii=False), flush=True)
    arm.enable(["wrist", *pre_wrist_holds.keys()])
    arm.move_target_with_holds(
        "wrist",
        biased["wrist"],
        seconds_per_step=api.COUPLED_CLEARANCE_MAX_SECONDS,
        kp=wrist_move_gains["kp"],
        kd=wrist_move_gains["kd"],
        hold_targets=pre_wrist_holds,
        hold_gains=pre_wrist_gains,
        fallback_kp=fallback_kp,
        fallback_kd=fallback_kd,
        hold_tau=pre_wrist_tau,
        active_tau=0.0,
        control_dt=api.COUPLED_CLEARANCE_CONTROL_DT,
        active_velocity_ff=False,
        step_deg=0.0,
        trajectory=api.COUPLED_CLEARANCE_TRAJECTORY,
        linear_blend=api.COUPLED_CLEARANCE_LINEAR_BLEND,
    )
    wrist_hold_target = float(biased["wrist"])

    wrist_fine_results = []
    for attempt in range(1, 3):
        status = arm.read_status(api.DEFAULT_JOINTS)
        wrist_error_deg = math.degrees(float(nominal["wrist"]) - float(status["wrist"]["pos"]))
        if abs(wrist_error_deg) <= ERROR_DEADBAND_DEG + ENCODER_HALF_COUNT_MARGIN_DEG:
            break
        if abs(wrist_error_deg) > PLACEMENT1_MAX_LEARNABLE_ERROR_DEG:
            print("v2.8 placement1 wrist-first fine skip gross error=", wrist_error_deg, flush=True)
            break
        fine = api.COUPLED_CLEARANCE_WRIST_FINE_GAINS
        if attempt == 1:
            fine_bias_deg = max(
                -api.CLEARANCE_WRIST_FINE_MAX_BIAS_DEG,
                min(api.CLEARANCE_WRIST_FINE_MAX_BIAS_DEG, wrist_error_deg),
            )
            fine_target = float(nominal["wrist"]) + math.radians(fine_bias_deg)
            active_tau = api.CLEARANCE_WRIST_FINE_ACTIVE_TAU
            strategy = "position_residual"
        else:
            fine_bias_deg = 0.0
            fine_target = float(nominal["wrist"])
            measured_tau = float(status["wrist"]["tau"])
            limits = getattr(api, "ACTIVE_TAU_LIMITS", {}).get("wrist", (0.35, 0.7))
            active_tau = max(float(limits[0]), min(float(limits[1]), measured_tau))
            strategy = "measured_load_feedforward"
        print("v2.8 placement1 wrist-first fine=", json.dumps({
            "attempt": attempt,
            "error_deg": wrist_error_deg,
            "bias_deg": fine_bias_deg,
            "target_rad": fine_target,
            "active_tau": active_tau,
            "strategy": strategy,
        }, ensure_ascii=False), flush=True)
        arm.move_target_with_holds(
            "wrist",
            fine_target,
            seconds_per_step=fine["seconds"],
            kp=fine["kp"],
            kd=fine["kd"],
            hold_targets=pre_wrist_holds,
            hold_gains=pre_wrist_gains,
            fallback_kp=fallback_kp,
            fallback_kd=fallback_kd,
            hold_tau=pre_wrist_tau,
            active_tau=active_tau,
            control_dt=api.CLEARANCE_WRIST_FINE_CONTROL_DT,
            active_velocity_ff=False,
            step_deg=0.0,
        )
        wrist_hold_target = fine_target
        wrist_hold_tau = active_tau
        wrist_hold_gains = {"kp": fine["kp"], "kd": fine["kd"]}
        reached_wrist = arm.positions(["wrist"])["wrist"]
        wrist_fine_results.append({
            "attempt": attempt,
            "error_deg": math.degrees(float(nominal["wrist"]) - reached_wrist),
        })
    if wrist_fine_results:
        print("v2.8 placement1 wrist-first fine result=", json.dumps(wrist_fine_results), flush=True)

    wrist_actual = arm.positions(["wrist"])["wrist"]
    wrist_error_deg = math.degrees(float(nominal["wrist"]) - wrist_actual)
    if abs(wrist_error_deg) > ERROR_DEADBAND_DEG + ENCODER_HALF_COUNT_MARGIN_DEG:
        local.update_hold_bias(
            "clearance", {"wrist": wrist_error_deg}, label="placement1-wrist-first"
        )
        if abs(wrist_error_deg) > PLACEMENT1_MAX_LEARNABLE_ERROR_DEG:
            raise RuntimeError(
                "v2.8 placement1 stopped after wrist-first retract: gross wrist error exceeds "
                f"{PLACEMENT1_MAX_LEARNABLE_ERROR_DEG}deg safety limit; other arm joints were not moved: "
                + json.dumps({"wrist": wrist_error_deg})
            )
        print("v2.8 placement1 wrist residual accepted for coupled follow-up=", json.dumps({
            "status": "proceeding",
            "blocks_followup": False,
            "error_deg": wrist_error_deg,
            "learning_saved": True,
        }), flush=True)

    move_targets = {
        name: biased[name] for name in validation_joints if name != "wrist"
    }
    current = arm.positions(api.DEFAULT_JOINTS)
    deltas = {name: math.degrees(move_targets[name] - current[name]) for name in move_targets}
    unsafe = {name: value for name, value in deltas.items() if abs(value) > 125.0}
    if unsafe:
        raise RuntimeError("v2.8 placement1 clearance delta exceeds 125deg: " + json.dumps(unsafe, ensure_ascii=False))
    coupled_steps = max((int(math.ceil(abs(value) / 5.0)) for value in deltas.values()), default=1)
    raw_seconds = max(
        api.HOME_GAINS.get(name, {"seconds": 6.0})["seconds"] for name in move_targets
    ) * max(1, coupled_steps)
    seconds = min(raw_seconds, api.COUPLED_CLEARANCE_MAX_SECONDS)
    hold_targets = {"wrist": wrist_hold_target}
    hold_targets["claw"] = claw_hold_pos
    hold_gains = {
        name: api.CLEARANCE_BASE_HOLD_GAINS[name]
        for name in hold_targets if name != "claw"
    }
    hold_gains["wrist"] = wrist_hold_gains
    hold_gains["claw"] = {"kp": api.CLAW_KP_HOLD, "kd": api.CLAW_KD_HOLD}
    hold_tau = {
        name: api.CLEARANCE_JOINT_HOLD_TAU.get(name, 0.0)
        for name in [*move_targets.keys(), *hold_targets.keys()]
    }
    hold_tau["wrist"] = wrist_hold_tau
    hold_tau["claw"] = 0.0
    move_gains = {
        name: api.COUPLED_CLEARANCE_MOVE_GAINS.get(name, api.CLEARANCE_MOVE_GAINS[name])
        for name in move_targets
    }
    move_tau_ff = {}
    for name in move_targets:
        start_tau = float(carry_tau.get(name, 0.0))
        configured = api.COUPLED_CLEARANCE_MOVE_TAU_FF.get(name, 0.0)
        end_tau = float(configured) if not isinstance(configured, dict) else float(configured.get("end_tau", configured.get("tau", 0.0)))
        if abs(start_tau - end_tau) > 1e-9:
            move_tau_ff[name] = {
                "start_tau": start_tau,
                "end_tau": end_tau,
                "ramp_fraction": 0.35,
                "ramp": "smoothstep",
            }
        elif end_tau:
            move_tau_ff[name] = end_tau
    print("v2.8 placement1 coupled clearance=", json.dumps({
        "targets_rad": move_targets,
        "deltas_deg": deltas,
        "seconds": seconds,
        "hold_joints": list(hold_targets),
        "inherited_start_tau": {name: carry_tau.get(name, 0.0) for name in move_targets},
    }, ensure_ascii=False), flush=True)
    arm.enable([*move_targets.keys(), *hold_targets.keys()])
    arm.move_targets_with_holds(
        move_targets,
        seconds_per_step=seconds,
        move_gains=move_gains,
        hold_targets=hold_targets,
        hold_gains=hold_gains,
        fallback_kp=fallback_kp,
        fallback_kd=fallback_kd,
        hold_tau=hold_tau,
        step_deg=0.0,
        preload_seconds=0.4,
        progress_windows=api.COUPLED_CLEARANCE_PROGRESS_WINDOWS,
        pre_window_gains=api.COUPLED_CLEARANCE_PRE_WINDOW_GAINS,
        control_dt=api.COUPLED_CLEARANCE_CONTROL_DT,
        velocity_ff_joints=api.COUPLED_CLEARANCE_VELOCITY_FF_JOINTS,
        move_tau_ff=move_tau_ff,
        trajectory=api.COUPLED_CLEARANCE_TRAJECTORY,
        linear_blend=api.COUPLED_CLEARANCE_LINEAR_BLEND,
    )
    # Keep the exact controller used at the end of the trajectory while the
    # arm settles.  The old handoff changed several gains at once and changed
    # shoulder_front feed-forward torque from 1.0 to 3.0 Nm.  When a joint was
    # still lagging its target, that discontinuity produced the observed
    # end-of-motion kick and oscillation.
    final_hold_targets = dict(move_targets)
    final_hold_targets.update(hold_targets)
    final_hold_gains = {name: dict(move_gains[name]) for name in move_targets}
    final_hold_gains.update(hold_gains)
    final_hold_tau = dict(hold_tau)
    terminal_move_tau = {}
    for name in move_targets:
        config = move_tau_ff.get(name, 0.0)
        if isinstance(config, dict):
            terminal_tau = float(config.get("end_tau", config.get("tau", 0.0)))
        else:
            terminal_tau = float(config)
        final_hold_tau[name] = terminal_tau
        terminal_move_tau[name] = terminal_tau
    settle_seconds = max(
        PLACEMENT1_TERMINAL_SETTLE_SECONDS,
        float(api.COUPLED_CLEARANCE_SETTLE_SECONDS),
    )
    print("v2.8 placement1 smooth terminal settle=", json.dumps({
        "seconds": settle_seconds,
        "gains": {name: final_hold_gains[name] for name in move_targets},
        "move_tau": terminal_move_tau,
        "reason": "preserve_end_of_trajectory_controller",
    }, ensure_ascii=False), flush=True)
    arm.hold_positions_with_gains(
        final_hold_targets,
        seconds=settle_seconds,
        gains=final_hold_gains,
        fallback_kp=fallback_kp,
        fallback_kd=fallback_kd,
        hold_tau=final_hold_tau,
    )
    final_positions = arm.positions(validation_joints)
    errors = clearance_errors_deg(nominal, final_positions, validation_joints)
    comparison = {
        name: {
            "nominal_clearance_rad": float(nominal[name]),
            "learned_command_rad": float(
                wrist_hold_target if name == "wrist" else move_targets[name]
            ),
            "actual_rad": float(final_positions[name]),
            "error_deg": float(errors[name]),
        }
        for name in validation_joints
    }
    print(
        "v2.8 placement1 final clearance comparison=",
        json.dumps(comparison, ensure_ascii=False),
        flush=True,
    )
    if clearance_handoff is not None:
        clearance_handoff.clear()
        clearance_handoff.update({
            "hold_targets": final_hold_targets,
            "hold_gains": final_hold_gains,
            "hold_tau": final_hold_tau,
            "claw_hold_pos": claw_hold_pos,
        })
    gross_errors = {
        name: value for name, value in errors.items()
        if abs(value) > PLACEMENT1_MAX_LEARNABLE_ERROR_DEG
    }
    if gross_errors:
        validation = local.record_move_validation(errors)
        print("v2.8 placement1 clearance verification=", json.dumps(validation, ensure_ascii=False), flush=True)
        print("v2.8 placement1 gross clearance residual accepted for follow-up=", json.dumps({
            "status": "proceeding",
            "blocks_followup": False,
            "gross_errors_deg": gross_errors,
            "learning_changed": False,
        }, ensure_ascii=False), flush=True)
        return {
            "status": "training incomplete",
            "tolerance_deg": ERROR_DEADBAND_DEG,
            "errors_deg": errors,
            "blocking_errors_deg": gross_errors,
        }
    validation = local.record_move_validation(errors)
    print("v2.8 placement1 clearance verification=", json.dumps(validation, ensure_ascii=False), flush=True)
    updates = local.update_hold_bias(
        "clearance", errors, label="placement1-clearance-final", backoff_on_worse=True,
        bias_limit_deg=PLACEMENT1_HOLD_BIAS_LIMIT_DEG,
    )
    if updates:
        print("v2.8 placement1 clearance learning update=", json.dumps(updates, ensure_ascii=False), flush=True)
    blockers = final_blocking_joint_errors(errors, ERROR_DEADBAND_DEG)
    if blockers:
        print("v2.8 placement1 clearance residual accepted for follow-up=", json.dumps({
            "status": "proceeding",
            "blocks_followup": False,
            "blocking_errors_deg": blockers,
            "learning_saved": True,
        }, ensure_ascii=False), flush=True)
        return {
            "status": "training incomplete",
            "tolerance_deg": ERROR_DEADBAND_DEG,
            "errors_deg": errors,
            "blocking_errors_deg": blockers,
        }
    return {
        "status": "training complete",
        "tolerance_deg": ERROR_DEADBAND_DEG,
        "errors_deg": errors,
        "blocking_errors_deg": {},
    }


def run_white_bishop_place_on_arm(arm, moves_file: str, fallback_kp: float,
                                  fallback_kd: float, deadband_deg: float,
                                  api, legacy, local, claw_hold):
    """Continue from Placement1 Clearance into the trained placement pose."""
    moves = legacy.load_moves(moves_file).get("moves", {})
    move_name = legacy.normalize_move_name(WHITE_BISHOP_PLACE_MOVE_NAME)
    if move_name not in moves:
        raise RuntimeError(f"White Bishop Placement requires saved move {move_name!r}")
    record = moves[move_name]
    pose = record.get("pose", {})
    missing = [joint for joint in legacy.REPLAY_REQUIRED_JOINTS if joint not in pose]
    if missing:
        raise RuntimeError(
            f"move {move_name!r} is missing joints: {', '.join(missing)}"
        )
    replay_order, final_joint = legacy.resolve_replay_order(record)
    low_shoulder_pose = legacy.is_low_shoulder_pose(pose)
    claw_target = float(claw_hold["claw_hold_pos"])

    print("v2.8 White Bishop Placement follow-up=", json.dumps({
        "move": move_name,
        "starts_after": "placement1-clearance",
        "same_motor_session": True,
        "claw_hold_pos": claw_target,
        "replay_order": replay_order,
        "final_joint": final_joint,
        "local_anchor_id": local.anchor_id,
        "local_samples": {
            name: value.get("samples", 0)
            for name, value in local.anchor.get("joint_bias", {}).items()
        },
    }, ensure_ascii=False), flush=True)
    arm.enable([*api.DEFAULT_JOINTS, "claw"])

    # Take over the Clearance endpoint without dropping the piece.  The
    # ordinary replay entry helper only commands the seven arm joints.
    current = arm.positions(api.DEFAULT_JOINTS)
    entry_gains = {
        name: api.CLEARANCE_HOLD_GAINS.get(
            name,
            api.CLEARANCE_BASE_HOLD_GAINS.get(name, {"kp": 3.0, "kd": 0.3}),
        )
        for name in api.DEFAULT_JOINTS
    }
    entry_gains = api.adaptive_hold_gains_for(
        "replay_entry", entry_gains, low_shoulder_pose
    )
    entry_tau = dict(legacy.REPLAY_ENTRY_HOLD_TAU)
    started = time.time()
    while time.time() - started < legacy.REPLAY_ENTRY_TAKEOVER_SECONDS:
        command_holds(arm, current, entry_gains, entry_tau, fallback_kp, fallback_kd)
        arm.ctrl.controlMIT(
            arm.motors["claw"], api.CLAW_KP_HOLD, api.CLAW_KD_HOLD,
            claw_target, 0, 0.0,
        )
        time.sleep(legacy.REPLAY_ENTRY_TAKEOVER_CONTROL_DT)

    # Every lower-level replay movement receives the claw as an additional
    # hold joint.  This preserves the trained arm controller while preventing
    # the pressure-stop grasp from being dropped between groups.
    current_single = api.LeftArmV2.move_target_with_holds
    current_group = api.LeftArmV2.move_targets_with_holds

    def add_claw_hold(call_kwargs):
        updated = dict(call_kwargs)
        hold_targets = dict(updated.get("hold_targets", {}))
        hold_targets["claw"] = claw_target
        updated["hold_targets"] = hold_targets
        hold_gains = dict(updated.get("hold_gains", {}))
        hold_gains["claw"] = {"kp": api.CLAW_KP_HOLD, "kd": api.CLAW_KD_HOLD}
        updated["hold_gains"] = hold_gains
        hold_tau = dict(updated.get("hold_tau", {}))
        hold_tau["claw"] = 0.0
        updated["hold_tau"] = hold_tau
        return updated

    def single_with_claw(self, name, target, *call_args, **call_kwargs):
        return current_single(
            self, name, target, *call_args, **add_claw_hold(call_kwargs)
        )

    def group_with_claw(self, targets, *call_args, **call_kwargs):
        return current_group(
            self, targets, *call_args, **add_claw_hold(call_kwargs)
        )

    api.LeftArmV2.move_target_with_holds = single_with_claw
    api.LeftArmV2.move_targets_with_holds = group_with_claw
    try:
        print(
            "v2.8 White Bishop Placement target pose=",
            json.dumps(pose, ensure_ascii=False),
            flush=True,
        )
        legacy.print_replay_error_report(
            "white_bishop_place_after_clearance",
            arm.positions(api.DEFAULT_JOINTS),
            pose,
        )
        completed_targets = legacy.run_adaptive_non_wrist_replay(
            arm, pose, replay_order, fallback_kp, fallback_kd,
            deadband_deg, low_shoulder_pose,
        )
        before_final = legacy.run_replay_settle_pass(
            arm, pose, fallback_kp, fallback_kd,
            label="white_bishop_place_before_final_wrist",
        )
        before_final = legacy.run_replay_correction_pass(
            arm, pose, before_final, fallback_kp, fallback_kd,
            deadband_deg, low_shoulder_pose,
        )
        legacy.run_final_wrist_move(
            arm, pose, fallback_kp, fallback_kd,
            deadband_deg, completed_targets,
        )
        final_positions = arm.positions(api.DEFAULT_JOINTS)
        final_errors = {
            joint: math.degrees(float(pose[joint]) - float(final_positions[joint]))
            for joint in api.DEFAULT_JOINTS
        }
        print(
            "v2.8 White Bishop Placement final errors_deg=",
            json.dumps(final_errors, ensure_ascii=False),
            flush=True,
        )
        local.update(final_errors, label="white-bishop-placement-final")
        validation = local.record_move_validation(final_errors)
        print(
            "v2.8 White Bishop Placement final training status=",
            json.dumps({
                "status": (
                    "training complete"
                    if not validation["blocking_errors_deg"]
                    else "training incomplete"
                ),
                "tolerance_deg": ERROR_DEADBAND_DEG,
                "errors_deg": final_errors,
                "blocking_errors_deg": validation["blocking_errors_deg"],
            }, ensure_ascii=False),
            flush=True,
        )
        if validation["blocking_errors_deg"]:
            raise RuntimeError(
                "White Bishop Placement training incomplete; learned data was saved: "
                + json.dumps(validation["blocking_errors_deg"], ensure_ascii=False)
            )
        return validation
    finally:
        api.LeftArmV2.move_target_with_holds = current_single
        api.LeftArmV2.move_targets_with_holds = current_group


def _now() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def _pose(pose: Dict[str, float]) -> Dict[str, float]:
    missing = [name for name in JOINTS if name not in pose]
    if missing:
        raise ValueError("pose is missing joints: " + ", ".join(missing))
    result = {name: float(pose[name]) for name in JOINTS}
    if not all(math.isfinite(value) for value in result.values()):
        raise ValueError("pose contains a non-finite joint value")
    return result


def pose_id(pose: Dict[str, float]) -> str:
    normalized = _pose(pose)
    payload = json.dumps(normalized, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


def pose_distance_deg(a: Dict[str, float], b: Dict[str, float]) -> tuple[float, float]:
    aa, bb = _pose(a), _pose(b)
    errors = [abs(math.degrees(aa[name] - bb[name])) for name in JOINTS]
    return math.sqrt(sum(value * value for value in errors) / len(errors)), max(errors)


def blocking_joint_errors(errors_deg: Dict[str, float], tolerance_deg: float = PRE_WRIST_TOLERANCE_DEG) -> Dict[str, float]:
    effective_tolerance = tolerance_deg + ENCODER_HALF_COUNT_MARGIN_DEG
    return {
        joint: float(errors_deg[joint])
        for joint in JOINTS
        if joint != "wrist"
        and joint in errors_deg
        and abs(float(errors_deg[joint])) > effective_tolerance
    }


def final_blocking_joint_errors(errors_deg: Dict[str, float],
                                tolerance_deg: float = ERROR_DEADBAND_DEG) -> Dict[str, float]:
    effective_tolerance = tolerance_deg + ENCODER_HALF_COUNT_MARGIN_DEG
    return {
        joint: float(errors_deg[joint])
        for joint in JOINTS
        if joint in errors_deg and abs(float(errors_deg[joint])) > effective_tolerance
    }


def replay_move_tau_with_wrist_side_support(original, legacy, name: str,
                                             delta_deg: float, low_shoulder_pose: bool):
    configured = original(name, delta_deg, low_shoulder_pose)
    if name == "wrist_side" and low_shoulder_pose and not configured:
        return float(legacy.REPLAY_LOW_SHOULDER_WRIST_SIDE_HOLD_TAU)
    return configured


def verify_pre_wrist_or_learn(arm, pose, local,
                               label: str = "before_final_wrist") -> Dict[str, float]:
    current = arm.positions(list(JOINTS))
    errors = {
        joint: math.degrees(float(pose[joint]) - current[joint])
        for joint in JOINTS
        if joint != "wrist" and joint in pose
    }
    blockers = blocking_joint_errors(errors)
    print("v2.8 pre-wrist verification=", json.dumps({
        "tolerance_deg": PRE_WRIST_TOLERANCE_DEG,
        "encoder_half_count_margin_deg": ENCODER_HALF_COUNT_MARGIN_DEG,
        "effective_tolerance_deg": PRE_WRIST_TOLERANCE_DEG + ENCODER_HALF_COUNT_MARGIN_DEG,
        "blocking_errors_deg": blockers,
        "extra_correction_motion": False,
    }, ensure_ascii=False), flush=True)
    if blockers:
        updates = local.update(blockers, label="pre-wrist-gate")
        print("v2.8 pre-wrist local learning update=", json.dumps(updates, ensure_ascii=False), flush=True)
        raise RuntimeError(
            "v2.8 replay stopped before wrist: non-wrist joints exceed "
            f"{PRE_WRIST_TOLERANCE_DEG:.1f}deg; local residuals saved for the next smooth replay; "
            "no extra correction motion was issued: " + json.dumps(blockers, ensure_ascii=False)
        )
    return errors


class LocalTargetBias:
    def __init__(self, path: Path, target_pose: Dict[str, float], move_name: str):
        self.path = Path(path)
        self.target_pose = _pose(target_pose)
        self.move_name = str(move_name)
        self.data = self._load()
        self.anchor_id, self.created, self.distance = self._select_anchor()
        self.repair_legacy_backoff_records()

    def _load(self) -> dict:
        if not self.path.exists():
            return {"schema": 1, "type": "left_arm_v2_8_local_target_bias", "anchors": {}}
        with self.path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
        if data.get("schema") != 1 or not isinstance(data.get("anchors"), dict):
            raise RuntimeError(f"invalid v2.8 local target bias file: {self.path}")
        return data

    def _select_anchor(self) -> tuple[str, bool, tuple[float, float]]:
        candidates = []
        for anchor_id, anchor in self.data["anchors"].items():
            try:
                rms, maximum = pose_distance_deg(self.target_pose, anchor["target_pose_rad"])
            except (KeyError, TypeError, ValueError):
                continue
            if rms <= MATCH_RMS_DEG and maximum <= MATCH_MAX_DEG:
                candidates.append((rms, maximum, anchor_id))
        if candidates:
            rms, maximum, anchor_id = min(candidates)
            anchor = self.data["anchors"][anchor_id]
            names = anchor.setdefault("move_names", [])
            if self.move_name not in names:
                names.append(self.move_name)
                self._save()
            return anchor_id, False, (rms, maximum)
        anchor_id = pose_id(self.target_pose)
        self.data["anchors"][anchor_id] = {
            "target_pose_rad": self.target_pose,
            "move_names": [self.move_name],
            "joint_bias": {},
            "created_at": _now(),
            "updated_at": _now(),
        }
        self._save()
        return anchor_id, True, (0.0, 0.0)

    @property
    def anchor(self) -> dict:
        return self.data["anchors"][self.anchor_id]

    def repair_legacy_backoff_records(self) -> Dict[str, dict]:
        """Restore best points from records written by the old backoff rule."""
        repaired = {}
        rules = self.anchor.get("joint_bias", {})
        for joint, record in rules.items():
            if not isinstance(record, dict) or record.get("learning_state") != "backoff":
                continue
            bias = float(record.get("bias_deg", 0.0))
            best = float(record.get("best_bias_deg", bias))
            logged_previous = float(record.get("previous_bias_deg", bias))
            # Old backoff logs used the restored best point as
            # ``previous_bias_deg`` and then stepped away from it again.
            if abs(bias - best) <= 1e-12 or abs(logged_previous - best) > 1e-12:
                continue
            record["bias_deg"] = best
            record["previous_bias_deg"] = bias
            record["delta_bias_deg"] = best - bias
            record["learning_state"] = "legacy_backoff_restored"
            record["restored_at"] = _now()
            repaired[joint] = {
                "contaminated_bias_deg": bias,
                "restored_best_bias_deg": best,
            }
        if repaired:
            self.anchor["updated_at"] = _now()
            self._save()
            print(
                "v2.8 legacy local backoff recovery=",
                json.dumps(repaired, ensure_ascii=False),
                flush=True,
            )
        return repaired

    def reset_wrist_side_bias_for_active_tau(self) -> Dict[str, float]:
        """Reset position learning once when replay gains load feed-forward."""
        migrations = self.anchor.setdefault("migrations", {})
        if migrations.get(WRIST_SIDE_ACTIVE_TAU_MIGRATION_ID):
            return {}
        record = self.anchor.get("joint_bias", {}).get("wrist_side")
        previous = float(record.get("bias_deg", 0.0)) if isinstance(record, dict) else 0.0
        if isinstance(record, dict):
            record["bias_deg"] = 0.0
            record["previous_bias_deg"] = previous
            record["delta_bias_deg"] = -previous
            record["best_bias_deg"] = 0.0
            record["learning_state"] = "active_tau_baseline_reset"
            record["reset_at"] = _now()
        migrations[WRIST_SIDE_ACTIVE_TAU_MIGRATION_ID] = {
            "applied_at": _now(),
            "previous_bias_deg": previous,
            "new_bias_deg": 0.0,
        }
        self.anchor["updated_at"] = _now()
        self._save()
        result = {"previous_bias_deg": previous, "new_bias_deg": 0.0}
        print("v2.8 wrist_side active-tau baseline reset=", json.dumps(result), flush=True)
        return result

    def bias_rad(self, joint: str) -> float:
        record = self.anchor.get("joint_bias", {}).get(joint, {})
        value = record.get("bias_deg", 0.0) if isinstance(record, dict) else 0.0
        return math.radians(max(-BIAS_LIMIT_DEG, min(BIAS_LIMIT_DEG, float(value))))

    def hold_bias_rad(self, active: str, joint: str,
                      limit_deg: float = HOLD_BIAS_LIMIT_DEG) -> float:
        active_rules = self.anchor.get("hold_bias", {}).get(active, {})
        record = active_rules.get(joint, {}) if isinstance(active_rules, dict) else {}
        value = record.get("bias_deg", 0.0) if isinstance(record, dict) else 0.0
        limit = abs(float(limit_deg))
        return math.radians(max(-limit, min(limit, float(value))))

    def recover_known_placement1_wrist_side_worsening(self) -> Dict[str, float]:
        """Undo the 20:43 wrist_side step that moved opposite its command."""
        migrations = self.anchor.setdefault("migrations", {})
        if migrations.get(PLACEMENT1_WRIST_SIDE_WORSENING_RECOVERY_ID):
            return {}
        rules = self.anchor.get("hold_bias", {}).get("clearance", {})
        record = rules.get("wrist_side") if isinstance(rules, dict) else None
        if not isinstance(record, dict):
            return {}
        current = float(record.get("bias_deg", 0.0))
        previous = float(record.get("previous_bias_deg", current))
        error = float(record.get("last_error_deg", 0.0))
        if not (
            abs(current - 1.2) < 0.05
            and abs(previous - 0.8) < 0.05
            and abs(error - 3.650109441515467) < 0.10
        ):
            return {}
        record.update({
            "bias_deg": previous,
            "previous_bias_deg": current,
            "delta_bias_deg": previous - current,
            "last_error_deg": 3.4533974320981606,
            "best_error_deg": 3.4533974320981606,
            "best_bias_deg": previous,
            "rejected_bias_deg": current,
            "step_scale": 0.5,
            "learning_state": "worsening_backoff_recovery",
            "updated_at": _now(),
        })
        migrations[PLACEMENT1_WRIST_SIDE_WORSENING_RECOVERY_ID] = {
            "applied_at": _now(),
            "rejected_bias_deg": current,
            "restored_bias_deg": previous,
        }
        self.anchor["updated_at"] = _now()
        self._save()
        result = {"rejected_bias_deg": current, "restored_bias_deg": previous}
        print("v2.8 placement1 wrist_side worsening recovery=", json.dumps(result), flush=True)
        return result

    def avoid_repeating_rejected_placement1_wrist_side_step(self) -> Dict[str, float]:
        """Prevent a plateau reading from immediately retrying a rejected full step."""
        migrations = self.anchor.setdefault("migrations", {})
        if migrations.get(PLACEMENT1_WRIST_SIDE_RETRY_RECOVERY_ID):
            return {}
        recovered = migrations.get(PLACEMENT1_WRIST_SIDE_WORSENING_RECOVERY_ID)
        if not isinstance(recovered, dict):
            return {}
        rules = self.anchor.get("hold_bias", {}).get("clearance", {})
        record = rules.get("wrist_side") if isinstance(rules, dict) else None
        if not isinstance(record, dict):
            return {}
        rejected = float(recovered.get("rejected_bias_deg", 1.2))
        best = float(record.get("best_bias_deg", recovered.get("restored_bias_deg", 0.8)))
        current = float(record.get("bias_deg", 0.0))
        previous = float(record.get("previous_bias_deg", current))
        if not (abs(current - rejected) < 0.05 and abs(previous - best) < 0.05):
            return {}
        record.update({
            "bias_deg": best,
            "previous_bias_deg": current,
            "delta_bias_deg": best - current,
            "rejected_bias_deg": rejected,
            "step_scale": max(HOLD_BIAS_MIN_STEP_SCALE, float(record.get("step_scale", 1.0)) * 0.5),
            "learning_state": "rejected_step_memory_backoff",
            "updated_at": _now(),
        })
        migrations[PLACEMENT1_WRIST_SIDE_RETRY_RECOVERY_ID] = {
            "applied_at": _now(),
            "rejected_bias_deg": rejected,
            "restored_bias_deg": best,
            "step_scale": record["step_scale"],
        }
        self.anchor["updated_at"] = _now()
        self._save()
        result = {
            "rejected_bias_deg": rejected,
            "restored_bias_deg": best,
            "step_scale": record["step_scale"],
        }
        print("v2.8 placement1 rejected wrist_side retry recovery=", json.dumps(result), flush=True)
        return result

    def update_hold_bias(self, active: str, errors_deg: Dict[str, float], label: str = "",
                         backoff_on_worse: bool = False,
                         bias_limit_deg: float = HOLD_BIAS_LIMIT_DEG) -> Dict[str, dict]:
        """Learn pose-local hold offsets without stalling on repeated equal errors.

        The inherited v2.6 learner restores its best value when two readings are
        equal. Static wrist-induced errors can therefore remain unchanged for
        many replays.  This v2.8 integral step keeps moving in the measured
        error direction, halves its scale after a sign reversal, and remains
        bounded.  The offset is applied only while ``active`` is moving.
        """
        rules = self.anchor.setdefault("hold_bias", {}).setdefault(str(active), {})
        updates = {}
        for joint, raw_error in errors_deg.items():
            if joint not in JOINTS or joint == active:
                continue
            error = float(raw_error)
            if not math.isfinite(error) or abs(error) < ERROR_DEADBAND_DEG:
                continue
            previous = rules.get(joint, {})
            current = float(previous.get("bias_deg", 0.0)) if isinstance(previous, dict) else 0.0
            previous_error = float(previous.get("last_error_deg", error)) if isinstance(previous, dict) else error
            scale = float(previous.get("step_scale", 1.0)) if isinstance(previous, dict) else 1.0
            reversed_direction = bool(previous) and error * previous_error < 0.0
            if reversed_direction:
                scale = max(HOLD_BIAS_MIN_STEP_SCALE, scale * 0.5)
            best_error = abs(float(previous.get("best_error_deg", previous_error))) if isinstance(previous, dict) else abs(error)
            best_bias = float(
                previous.get("best_bias_deg", previous.get("previous_bias_deg", current))
            ) if isinstance(previous, dict) else current
            improved = abs(error) < best_error - IMPROVEMENT_DEADBAND_DEG
            worse = bool(previous) and abs(error) > abs(previous_error) + IMPROVEMENT_DEADBAND_DEG
            if improved:
                best_error, best_bias = abs(error), current
            if backoff_on_worse and worse:
                new_bias = best_bias
                scale = max(HOLD_BIAS_MIN_STEP_SCALE, scale * 0.5)
                state = "backoff"
            else:
                delta = max(
                    -HOLD_BIAS_MAX_STEP_DEG * scale,
                    min(HOLD_BIAS_MAX_STEP_DEG * scale, error * HOLD_BIAS_STEP_SCALE * scale),
                )
                limit = abs(float(bias_limit_deg))
                new_bias = max(-limit, min(limit, current + delta))
                state = "direction_reversal" if reversed_direction else "improved" if improved else "integrating"
            if new_bias == current:
                continue
            record = {
                "bias_deg": new_bias,
                "previous_bias_deg": current,
                "delta_bias_deg": new_bias - current,
                "last_error_deg": error,
                "best_error_deg": best_error,
                "best_bias_deg": best_bias,
                "step_scale": scale,
                "learning_state": state,
                "samples": int(previous.get("samples", 0)) + 1 if isinstance(previous, dict) else 1,
                "updated_at": _now(),
            }
            if label:
                record["label"] = label
            rules[joint] = record
            updates[joint] = record
        if updates:
            self.anchor["updated_at"] = _now()
            self._save()
        return updates

    def record_move_validation(self, errors_deg: Dict[str, float]) -> Dict[str, object]:
        blockers = final_blocking_joint_errors(errors_deg)
        record = {
            "status": "validated" if not blockers else "training",
            "tolerance_deg": ERROR_DEADBAND_DEG,
            "encoder_half_count_margin_deg": ENCODER_HALF_COUNT_MARGIN_DEG,
            "effective_tolerance_deg": ERROR_DEADBAND_DEG + ENCODER_HALF_COUNT_MARGIN_DEG,
            "errors_deg": {joint: float(errors_deg[joint]) for joint in JOINTS if joint in errors_deg},
            "blocking_errors_deg": blockers,
            "checked_at": _now(),
        }
        if not blockers:
            record["validated_at"] = record["checked_at"]
        self.anchor.setdefault("move_validation", {})[self.move_name] = record
        self.anchor["updated_at"] = _now()
        self._save()
        return record

    def update(self, errors_deg: Dict[str, float], label: str = "") -> Dict[str, dict]:
        rules = self.anchor.setdefault("joint_bias", {})
        updates = {}
        for joint, raw_error in errors_deg.items():
            if joint not in JOINTS:
                continue
            error = float(raw_error)
            if not math.isfinite(error) or abs(error) < ERROR_DEADBAND_DEG:
                continue
            previous = rules.get(joint, {})
            current = float(previous.get("bias_deg", 0.0)) if isinstance(previous, dict) else 0.0
            applied_bias = current
            previous_error = abs(float(previous.get("last_error_deg", error))) if previous else None
            best_error = abs(float(previous.get("best_error_deg", error))) if previous else abs(error)
            best_bias = float(previous.get("best_bias_deg", current)) if previous else current
            scale = float(previous.get("step_scale", 1.0)) if previous else 1.0
            improved = abs(error) < best_error - IMPROVEMENT_DEADBAND_DEG
            worse = previous_error is not None and abs(error) > previous_error + IMPROVEMENT_DEADBAND_DEG
            if improved:
                best_error, best_bias = abs(error), current
            elif worse:
                # This error was produced by the currently applied bias. Go
                # back to the proven best bias only; do not use the same bad
                # observation to launch another step in its error direction.
                current = best_bias
                scale = max(MIN_STEP_SCALE, scale * 0.5)
            elif previous_error is not None:
                scale = max(MIN_STEP_SCALE, scale * 0.8)
            if worse:
                new_bias = current
            else:
                delta = max(
                    -MAX_STEP_DEG * scale,
                    min(MAX_STEP_DEG * scale, error * STEP_SCALE * scale),
                )
                new_bias = max(-BIAS_LIMIT_DEG, min(BIAS_LIMIT_DEG, current + delta))
            if new_bias == applied_bias:
                continue
            record = {
                "bias_deg": new_bias,
                "previous_bias_deg": applied_bias,
                "delta_bias_deg": new_bias - applied_bias,
                "last_error_deg": error,
                "best_error_deg": best_error,
                "best_bias_deg": best_bias,
                "step_scale": scale,
                "learning_state": "improved" if improved else "backoff" if worse else "plateau",
                "samples": int(previous.get("samples", 0)) + 1 if previous else 1,
                "updated_at": _now(),
            }
            if label:
                record["label"] = label
            rules[joint] = record
            updates[joint] = record
        if updates:
            self.anchor["updated_at"] = _now()
            self._save()
        return updates

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.data["updated_at"] = _now()
        fd, temp_name = tempfile.mkstemp(prefix=self.path.name + ".", suffix=".tmp", dir=str(self.path.parent))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(self.data, stream, ensure_ascii=False, indent=2, sort_keys=True)
                stream.write("\n")
            os.replace(temp_name, self.path)
        finally:
            if os.path.exists(temp_name):
                os.unlink(temp_name)


def replay_with_local_bias(args) -> None:
    import left_arm_v2_6 as api
    import left_arm_v2_6_move_library as legacy

    move_name = legacy.normalize_move_name(args.name)
    moves = legacy.load_moves(args.moves_file).get("moves", {})
    if move_name not in moves:
        raise RuntimeError(f"unknown move: {move_name}")
    source_local = LocalTargetBias(
        LOCAL_BIAS_PATH, moves[move_name].get("pose", {}), move_name
    )
    source_local.reset_wrist_side_bias_for_active_tau()
    active_local = [source_local]
    original_get = api.adaptive_target_bias_for
    original_update = api.update_adaptive_target_bias
    original_active_tau_update = api.update_adaptive_active_tau
    original_hold_tau_update = api.update_adaptive_hold_tau
    original_hold_bias_for = api.adaptive_hold_target_bias_for
    original_hold_bias_update = api.update_adaptive_hold_target_bias
    original_hold_bias_ready = legacy.adaptive_hold_tau_ready_for_target_bias
    original_arm_script = legacy.ARM_SCRIPT
    original_replay_move_tau_ff = legacy.replay_move_tau_ff
    original_api_deadband = api.ADAPTIVE_TARGET_BIAS_ERROR_DEADBAND_DEG
    original_settle = legacy.run_replay_settle_pass
    original_error_report = legacy.print_replay_error_report
    original_close = api.LeftArmV2.close
    original_move_target_with_holds = api.LeftArmV2.move_target_with_holds
    final_errors: Dict[str, float] = {}
    placement1_ran = False
    placement1_summary = {}
    white_bishop_placement_summary = {}
    carry_hold = {}

    def combined_bias(joint: str, low_shoulder_front: bool = False,
                      path: str = api.ADAPTIVE_TARGET_BIAS_PATH) -> float:
        return original_get(joint, low_shoulder_front, path) + active_local[0].bias_rad(joint)

    def update_local(final_errors_deg: Dict[str, float], low_shoulder_front: bool = False,
                     path: str = api.ADAPTIVE_TARGET_BIAS_PATH, label: str = "",
                     force_joints: Optional[Iterable[str]] = None,
                     require_ready: bool = True) -> Dict[str, dict]:
        # The v2.6 shared correction is the inherited baseline. v2.8 learns
        # only the residual, preventing a distant pose from rewriting it.
        return active_local[0].update(
            final_errors_deg, label=label or "v2.8-pose-local"
        )

    def update_active_tau_05(name, delta_deg, active_error_deg, applied_tau,
                             low_shoulder_front=False, path=api.ADAPTIVE_ACTIVE_TAU_PATH,
                             label="", error_deadband_deg=ERROR_DEADBAND_DEG):
        return original_active_tau_update(
            name, delta_deg, active_error_deg, applied_tau,
            low_shoulder_front=low_shoulder_front,
            path=path,
            label=label,
            error_deadband_deg=ERROR_DEADBAND_DEG,
        )

    def update_hold_tau_05(active, hold_errors_deg, hold_tau,
                           low_shoulder_front=False, path=api.ADAPTIVE_HOLD_TAU_PATH,
                           label="", error_deadband_deg=ERROR_DEADBAND_DEG):
        return original_hold_tau_update(
            active, hold_errors_deg, hold_tau,
            low_shoulder_front=low_shoulder_front,
            path=path,
            label=label,
            error_deadband_deg=ERROR_DEADBAND_DEG,
        )

    def combined_hold_bias(active, hold_targets, low_shoulder_front=False,
                           path=api.ADAPTIVE_HOLD_TARGET_BIAS_PATH):
        inherited = original_hold_bias_for(
            active, hold_targets,
            low_shoulder_front=low_shoulder_front,
            path=path,
        )
        combined = {}
        for joint in hold_targets:
            # A v2.6 bias remains gated by its original tau-readiness rule.
            # A pose-local v2.8 offset is already based on the final measured
            # wrist-induced residual and may be used without waiting forever
            # on a stale v2.6 "improved" state.
            if original_hold_bias_ready(active, joint, low_shoulder_front) and joint in inherited:
                combined[joint] = inherited[joint]
            local_offset = active_local[0].hold_bias_rad(active, joint)
            if local_offset:
                combined[joint] = combined.get(joint, 0.0) + local_offset
        return combined

    def v28_hold_bias_ready(active, hold_name, low_shoulder_pose):
        return (
            original_hold_bias_ready(active, hold_name, low_shoulder_pose)
            or abs(active_local[0].hold_bias_rad(active, hold_name)) > 1e-12
        )

    def update_local_hold_bias(active, induced_errors_deg, low_shoulder_front=False,
                               path=api.ADAPTIVE_HOLD_TARGET_BIAS_PATH, label=""):
        # Preserve the inherited v2.6 file as a fixed baseline. Equal signed
        # residuals continue integrating in the pose-local v2.8 record.
        # replay_move reports the elbow once inside run_final_wrist_move and
        # again in its final induced-error report. Learn only from the latter,
        # complete snapshot so one wrist motion cannot be counted twice.
        if label == "replay-final-wrist-soft-hold":
            return {}
        return active_local[0].update_hold_bias(
            active, induced_errors_deg, label=label or "v2.8-pose-local-hold"
        )

    def strict_pre_wrist_settle(arm, pose, fallback_kp, fallback_kd, label="before_final_wrist"):
        return verify_pre_wrist_or_learn(arm, pose, active_local[0], label)

    def capture_error_report(label, current, pose):
        errors = original_error_report(label, current, pose)
        if label == "final":
            final_errors.clear()
            final_errors.update(errors)
        return errors

    def close_with_optional_placement1(arm):
        nonlocal placement1_ran
        try:
            if args.placement1 and final_errors and not placement1_ran:
                placement1_ran = True
                clearance_handoff = {}
                result = run_placement1_on_arm(
                    arm, args.clearance_file, args.kp, args.kd,
                    carry_hold=carry_hold,
                    clearance_handoff=clearance_handoff,
                )
                if result:
                    placement1_summary.clear()
                    placement1_summary.update(result)
                destination_pose = moves.get(
                    WHITE_BISHOP_PLACE_MOVE_NAME, {}
                ).get("pose", {})
                if not destination_pose:
                    raise RuntimeError(
                        "White Bishop Placement requires trained saved move "
                        f"{WHITE_BISHOP_PLACE_MOVE_NAME!r}"
                    )
                destination_local = LocalTargetBias(
                    LOCAL_BIAS_PATH,
                    destination_pose,
                    WHITE_BISHOP_PLACE_MOVE_NAME,
                )
                destination_local.reset_wrist_side_bias_for_active_tau()
                active_local[0] = destination_local
                try:
                    placement_result = run_white_bishop_place_on_arm(
                        arm,
                        args.moves_file,
                        args.kp,
                        args.kd,
                        args.deadband_deg,
                        api,
                        legacy,
                        destination_local,
                        clearance_handoff,
                    )
                    white_bishop_placement_summary.clear()
                    white_bishop_placement_summary.update({
                        "status": "training complete",
                        "tolerance_deg": ERROR_DEADBAND_DEG,
                        "errors_deg": placement_result["errors_deg"],
                        "blocking_errors_deg": placement_result["blocking_errors_deg"],
                    })
                finally:
                    active_local[0] = source_local
        finally:
            original_close(arm)

    def capture_wrist_hold(arm, name, target, *call_args, **call_kwargs):
        if name == "wrist" and call_kwargs.get("hold_targets"):
            wrist_kp = call_kwargs.get("kp", call_args[1] if len(call_args) > 1 else 3.0)
            wrist_kd = call_kwargs.get("kd", call_args[2] if len(call_args) > 2 else 0.3)
            gains = dict(call_kwargs.get("hold_gains", {}))
            gains["wrist"] = {"kp": wrist_kp, "kd": wrist_kd}
            tau = dict(call_kwargs.get("hold_tau", {}))
            tau["wrist"] = call_kwargs.get("active_tau", 0.0)
            targets = dict(call_kwargs["hold_targets"])
            targets["wrist"] = target
            carry_hold.clear()
            carry_hold.update({
                "hold_targets": targets,
                "hold_gains": gains,
                "hold_tau": tau,
            })
        return original_move_target_with_holds(arm, name, target, *call_args, **call_kwargs)

    def replay_move_tau_ff_with_wrist_side(name, delta_deg, low_shoulder_pose):
        return replay_move_tau_with_wrist_side_support(
            original_replay_move_tau_ff, legacy, name, delta_deg, low_shoulder_pose
        )

    print("v2.8 pose-local correction=", json.dumps({
        "build": V28_MOVE_BUILD,
        "move": move_name,
        "anchor_id": source_local.anchor_id,
        "new_anchor": source_local.created,
        "nearest_rms_deg": source_local.distance[0],
        "nearest_max_deg": source_local.distance[1],
        "shared_v2_6_baseline": True,
        "local_samples": {
            name: value.get("samples", 0)
            for name, value in source_local.anchor.get("joint_bias", {}).items()
        },
    }, ensure_ascii=False), flush=True)
    api.adaptive_target_bias_for = combined_bias
    api.update_adaptive_target_bias = update_local
    api.update_adaptive_active_tau = update_active_tau_05
    api.update_adaptive_hold_tau = update_hold_tau_05
    api.adaptive_hold_target_bias_for = combined_hold_bias
    api.update_adaptive_hold_target_bias = update_local_hold_bias
    legacy.adaptive_hold_tau_ready_for_target_bias = v28_hold_bias_ready
    api.ADAPTIVE_TARGET_BIAS_ERROR_DEADBAND_DEG = ERROR_DEADBAND_DEG
    legacy.run_replay_settle_pass = strict_pre_wrist_settle
    legacy.print_replay_error_report = capture_error_report
    api.LeftArmV2.close = close_with_optional_placement1
    api.LeftArmV2.move_target_with_holds = capture_wrist_hold
    legacy.replay_move_tau_ff = replay_move_tau_ff_with_wrist_side
    legacy.ARM_SCRIPT = str(SCRIPT_DIR / "left_arm_v2_8.py")
    try:
        legacy.replay_move(
            args.name, args.moves_file, args.clearance_file, args.python_bin,
            args.kp, args.kd, args.deadband_deg,
            replay_style=legacy.resolve_replay_style(args.sequential, args.coupled),
        )
    finally:
        api.adaptive_target_bias_for = original_get
        api.update_adaptive_target_bias = original_update
        api.update_adaptive_active_tau = original_active_tau_update
        api.update_adaptive_hold_tau = original_hold_tau_update
        api.adaptive_hold_target_bias_for = original_hold_bias_for
        api.update_adaptive_hold_target_bias = original_hold_bias_update
        legacy.adaptive_hold_tau_ready_for_target_bias = original_hold_bias_ready
        api.ADAPTIVE_TARGET_BIAS_ERROR_DEADBAND_DEG = original_api_deadband
        legacy.run_replay_settle_pass = original_settle
        legacy.print_replay_error_report = original_error_report
        api.LeftArmV2.close = original_close
        api.LeftArmV2.move_target_with_holds = original_move_target_with_holds
        legacy.replay_move_tau_ff = original_replay_move_tau_ff
        legacy.ARM_SCRIPT = original_arm_script
    if final_errors:
        validation = source_local.record_move_validation(final_errors)
        print("v2.8 final verification=", json.dumps(validation, ensure_ascii=False), flush=True)
        blockers = validation["blocking_errors_deg"]
        if blockers:
            raise RuntimeError(
                "v2.8 replay training incomplete: final joints exceed "
                f"{ERROR_DEADBAND_DEG:.1f}deg; learned data was saved; return Home and replay again: "
                + json.dumps(blockers, ensure_ascii=False)
            )
    if args.placement1 and placement1_summary:
        print(
            "v2.8 Placement1 final training status=",
            json.dumps(placement1_summary, ensure_ascii=False),
            flush=True,
        )
    if args.placement1 and white_bishop_placement_summary:
        # This is the final line of the complete user-visible workflow.
        print(
            "v2.8 White Bishop Placement final training status=",
            json.dumps(white_bishop_placement_summary, ensure_ascii=False),
            flush=True,
        )


def build_parser():
    import left_arm_v2_6_move_library as legacy
    parser = legacy.build_parser()
    subparsers = next(action for action in parser._actions if isinstance(action, argparse._SubParsersAction))
    subparsers.choices["replay-move"].add_argument(
        "--placement1",
        action="store_true",
        help=(
            "run White Bishop Placement: bishop01, pressure grasp, learned "
            "clearance, then trained white_bishop_place"
        ),
    )
    return parser


def main() -> None:
    import left_arm_v2_6_move_library as legacy
    args = build_parser().parse_args()
    if args.cmd != "replay-move":
        return legacy.main()
    if args.placement1 and legacy.normalize_move_name(args.name) != PLACEMENT1_MOVE_NAME:
        raise RuntimeError("placement1 currently requires saved move bishop01")
    replay_with_local_bias(args)


if __name__ == "__main__":
    main()
