#!/usr/bin/env python3
"""v2.8 saved-move entry point with pose-local target correction.

All motion remains in the v2.6 implementation. During replay this wrapper
adds a pose-local target bias on top of the existing shared v2.6 bias.
Non-replay commands are delegated unchanged.
"""
from __future__ import annotations

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
LOCAL_BIAS_PATH = SCRIPT_DIR / "data" / "left_arm_v2_8_local_target_bias.json"
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

    def bias_rad(self, joint: str) -> float:
        record = self.anchor.get("joint_bias", {}).get(joint, {})
        value = record.get("bias_deg", 0.0) if isinstance(record, dict) else 0.0
        return math.radians(max(-BIAS_LIMIT_DEG, min(BIAS_LIMIT_DEG, float(value))))

    def hold_bias_rad(self, active: str, joint: str) -> float:
        active_rules = self.anchor.get("hold_bias", {}).get(active, {})
        record = active_rules.get(joint, {}) if isinstance(active_rules, dict) else {}
        value = record.get("bias_deg", 0.0) if isinstance(record, dict) else 0.0
        return math.radians(max(-HOLD_BIAS_LIMIT_DEG, min(HOLD_BIAS_LIMIT_DEG, float(value))))

    def update_hold_bias(self, active: str, errors_deg: Dict[str, float], label: str = "") -> Dict[str, dict]:
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
            delta = max(
                -HOLD_BIAS_MAX_STEP_DEG * scale,
                min(HOLD_BIAS_MAX_STEP_DEG * scale, error * HOLD_BIAS_STEP_SCALE * scale),
            )
            new_bias = max(-HOLD_BIAS_LIMIT_DEG, min(HOLD_BIAS_LIMIT_DEG, current + delta))
            if new_bias == current:
                continue
            record = {
                "bias_deg": new_bias,
                "previous_bias_deg": current,
                "delta_bias_deg": new_bias - current,
                "last_error_deg": error,
                "step_scale": scale,
                "learning_state": "direction_reversal" if reversed_direction else "integrating",
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
            previous_error = abs(float(previous.get("last_error_deg", error))) if previous else None
            best_error = abs(float(previous.get("best_error_deg", error))) if previous else abs(error)
            best_bias = float(previous.get("best_bias_deg", current)) if previous else current
            scale = float(previous.get("step_scale", 1.0)) if previous else 1.0
            improved = abs(error) < best_error - IMPROVEMENT_DEADBAND_DEG
            worse = previous_error is not None and abs(error) > previous_error + IMPROVEMENT_DEADBAND_DEG
            if improved:
                best_error, best_bias = abs(error), current
            elif worse:
                current = best_bias
                scale = max(MIN_STEP_SCALE, scale * 0.5)
            elif previous_error is not None:
                scale = max(MIN_STEP_SCALE, scale * 0.8)
            delta = max(-MAX_STEP_DEG * scale, min(MAX_STEP_DEG * scale, error * STEP_SCALE * scale))
            new_bias = max(-BIAS_LIMIT_DEG, min(BIAS_LIMIT_DEG, current + delta))
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
    local = LocalTargetBias(LOCAL_BIAS_PATH, moves[move_name].get("pose", {}), move_name)
    original_get = api.adaptive_target_bias_for
    original_update = api.update_adaptive_target_bias
    original_active_tau_update = api.update_adaptive_active_tau
    original_hold_tau_update = api.update_adaptive_hold_tau
    original_hold_bias_for = api.adaptive_hold_target_bias_for
    original_hold_bias_update = api.update_adaptive_hold_target_bias
    original_hold_bias_ready = legacy.adaptive_hold_tau_ready_for_target_bias
    original_arm_script = legacy.ARM_SCRIPT
    original_api_deadband = api.ADAPTIVE_TARGET_BIAS_ERROR_DEADBAND_DEG
    original_settle = legacy.run_replay_settle_pass
    original_error_report = legacy.print_replay_error_report
    final_errors: Dict[str, float] = {}

    def combined_bias(joint: str, low_shoulder_front: bool = False,
                      path: str = api.ADAPTIVE_TARGET_BIAS_PATH) -> float:
        return original_get(joint, low_shoulder_front, path) + local.bias_rad(joint)

    def update_local(final_errors_deg: Dict[str, float], low_shoulder_front: bool = False,
                     path: str = api.ADAPTIVE_TARGET_BIAS_PATH, label: str = "",
                     force_joints: Optional[Iterable[str]] = None,
                     require_ready: bool = True) -> Dict[str, dict]:
        # The v2.6 shared correction is the inherited baseline. v2.8 learns
        # only the residual, preventing a distant pose from rewriting it.
        return local.update(final_errors_deg, label=label or "v2.8-pose-local")

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
            local_offset = local.hold_bias_rad(active, joint)
            if local_offset:
                combined[joint] = combined.get(joint, 0.0) + local_offset
        return combined

    def v28_hold_bias_ready(active, hold_name, low_shoulder_pose):
        return (
            original_hold_bias_ready(active, hold_name, low_shoulder_pose)
            or abs(local.hold_bias_rad(active, hold_name)) > 1e-12
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
        return local.update_hold_bias(active, induced_errors_deg, label=label or "v2.8-pose-local-hold")

    def strict_pre_wrist_settle(arm, pose, fallback_kp, fallback_kd, label="before_final_wrist"):
        return verify_pre_wrist_or_learn(arm, pose, local, label)

    def capture_error_report(label, current, pose):
        errors = original_error_report(label, current, pose)
        if label == "final":
            final_errors.clear()
            final_errors.update(errors)
        return errors

    print("v2.8 pose-local correction=", json.dumps({
        "move": move_name,
        "anchor_id": local.anchor_id,
        "new_anchor": local.created,
        "nearest_rms_deg": local.distance[0],
        "nearest_max_deg": local.distance[1],
        "shared_v2_6_baseline": True,
        "local_samples": {name: value.get("samples", 0) for name, value in local.anchor.get("joint_bias", {}).items()},
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
        legacy.ARM_SCRIPT = original_arm_script
    if final_errors:
        validation = local.record_move_validation(final_errors)
        print("v2.8 final verification=", json.dumps(validation, ensure_ascii=False), flush=True)
        blockers = validation["blocking_errors_deg"]
        if blockers:
            raise RuntimeError(
                "v2.8 replay training incomplete: final joints exceed "
                f"{ERROR_DEADBAND_DEG:.1f}deg; learned data was saved; return Home and replay again: "
                + json.dumps(blockers, ensure_ascii=False)
            )


def main() -> None:
    import left_arm_v2_6_move_library as legacy
    args = legacy.build_parser().parse_args()
    if args.cmd != "replay-move":
        return legacy.main()
    replay_with_local_bias(args)


if __name__ == "__main__":
    main()
