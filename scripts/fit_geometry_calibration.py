#!/usr/bin/env python3
"""Fit shoulder-frame geometry IK to empirical grasp targets.

Version 2 understands two sample groups:
  normal_samples: regular target-shoulder planned joint targets
  limit_samples: hold-wrist extreme-near-left front/elbow limit poses

If geometry_calibration_samples_v2.json exists, it is preferred. The older
geometry_calibration_samples.json remains supported for comparison.
"""

from __future__ import annotations

import json
import math
import os
from typing import Callable, Dict, Iterable, List, Optional, Tuple


INCH_TO_CM = 2.54
LINK_SHOULDER_TO_ELBOW_CM = 9.0 * INCH_TO_CM
LINK_ELBOW_TO_WRIST_CM = 7.8 * INCH_TO_CM
LINK_WRIST_TO_GRIP_CM = 8.8 * INCH_TO_CM
GRIP_FORWARD_PROJECT_CM = 0.0
ARM_Y_CM_PER_DEG = 1.4
SHOULDER_SIDE_TEST_LIMIT_DEG = 25.0

JOINTS = ("shoulder_front", "shoulder_side", "elbow", "wrist")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def geometry_offsets_from_shoulder_frame(
    target_forward_cm: float,
    target_left_cm: float,
    target_up_cm: float,
) -> Dict[str, float]:
    wrist_forward_cm = target_forward_cm - GRIP_FORWARD_PROJECT_CM
    wrist_up_cm = target_up_cm + LINK_WRIST_TO_GRIP_CM
    reach_cm = math.hypot(wrist_forward_cm, wrist_up_cm)

    l1 = LINK_SHOULDER_TO_ELBOW_CM
    l2 = LINK_ELBOW_TO_WRIST_CM
    reach_clamped_cm = clamp(reach_cm, abs(l1 - l2) + 0.5, l1 + l2 - 0.5)

    base_angle = math.atan2(wrist_up_cm, wrist_forward_cm)
    shoulder_inner = math.acos(
        clamp(
            (l1 * l1 + reach_clamped_cm * reach_clamped_cm - l2 * l2)
            / (2.0 * l1 * reach_clamped_cm),
            -1.0,
            1.0,
        )
    )
    elbow_inner = math.acos(
        clamp(
            (l1 * l1 + l2 * l2 - reach_clamped_cm * reach_clamped_cm)
            / (2.0 * l1 * l2),
            -1.0,
            1.0,
        )
    )

    shoulder_front_deg = math.degrees(base_angle + shoulder_inner)
    elbow_bend_deg = 180.0 - math.degrees(elbow_inner)
    wrist_keep_down_deg = -90.0 - shoulder_front_deg + elbow_bend_deg
    side_deg = clamp(
        -target_left_cm / ARM_Y_CM_PER_DEG,
        -SHOULDER_SIDE_TEST_LIMIT_DEG,
        SHOULDER_SIDE_TEST_LIMIT_DEG,
    )

    return {
        "shoulder_front": math.radians(shoulder_front_deg),
        "shoulder_side": math.radians(side_deg),
        "elbow": math.radians(elbow_bend_deg),
        "wrist": math.radians(wrist_keep_down_deg),
    }


def fit_line(xs: Iterable[float], ys: Iterable[float]) -> Tuple[float, float]:
    x_list = list(xs)
    y_list = list(ys)
    n = float(len(x_list))
    sx = sum(x_list)
    sy = sum(y_list)
    sxx = sum(x * x for x in x_list)
    sxy = sum(x * y for x, y in zip(x_list, y_list))
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-12:
        return 1.0, (sy / n) if n else 0.0
    scale = (n * sxy - sx * sy) / denom
    bias = (sy - scale * sx) / n
    return scale, bias


def solve_linear_system(matrix: List[List[float]], vector: List[float]) -> List[float]:
    n = len(vector)
    aug = [row[:] + [vector[i]] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise ValueError("singular matrix")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        pivot_value = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= pivot_value
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]
    return [aug[row][n] for row in range(n)]


def fit_linear_features(feature_rows: List[List[float]], ys: List[float]) -> List[float]:
    width = len(feature_rows[0])
    xtx = [[0.0 for _ in range(width)] for _ in range(width)]
    xty = [0.0 for _ in range(width)]
    for features, y in zip(feature_rows, ys):
        for i in range(width):
            xty[i] += features[i] * y
            for j in range(width):
                xtx[i][j] += features[i] * features[j]
    for i in range(width):
        xtx[i][i] += 1e-9
    return solve_linear_system(xtx, xty)


def dot(a: List[float], b: List[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def summarize_errors(errors_rad: List[float]) -> Dict[str, float]:
    abs_deg = [abs(math.degrees(err)) for err in errors_rad]
    if not abs_deg:
        return {"mean_abs_deg": 0.0, "max_abs_deg": 0.0}
    return {
        "mean_abs_deg": sum(abs_deg) / len(abs_deg),
        "max_abs_deg": max(abs_deg),
    }


def print_model_coefficients(model_name: str, coeffs: List[float]) -> str:
    if model_name == "raw+shoulder_xyz":
        coeff_label = {
            "bias_deg": math.degrees(coeffs[0]),
            "raw_scale": coeffs[1],
            "forward_coef_deg": math.degrees(coeffs[2]),
            "left_coef_deg": math.degrees(coeffs[3]),
            "up_coef_deg": math.degrees(coeffs[4]),
        }
    elif model_name == "raw+bias":
        coeff_label = {
            "bias_deg": math.degrees(coeffs[0]),
            "raw_scale": coeffs[1],
        }
    else:
        coeff_label = {
            "bias_deg": math.degrees(coeffs[0]),
            "forward_coef_deg": math.degrees(coeffs[1]),
            "left_coef_deg": math.degrees(coeffs[2]),
            "up_coef_deg": math.degrees(coeffs[3]),
        }
    return json.dumps({k: round(v, 6) for k, v in coeff_label.items()})


def load_payload(here: str) -> Tuple[dict, str]:
    v2_path = os.path.join(here, "geometry_calibration_samples_v2.json")
    if os.path.exists(v2_path):
        with open(v2_path, "r", encoding="utf-8") as f:
            return json.load(f), v2_path
    path = os.path.join(here, "geometry_calibration_samples.json")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f), path


def limit_position_t(sample: dict) -> float:
    arm_x_start = 6.0
    arm_x_full = 1.0
    forward_start = 28.0
    forward_full = 24.0
    near_x_t = clamp(
        (arm_x_start - sample["legacy_arm_x_cm"]) / (arm_x_start - arm_x_full),
        0.0,
        1.0,
    )
    near_forward_t = clamp(
        (forward_start - sample["forward_cm"]) / (forward_start - forward_full),
        0.0,
        1.0,
    )
    return max(near_x_t, near_forward_t)


def fit_limit_line(samples: List[dict], key: str) -> Optional[Tuple[float, float, List[float]]]:
    usable = [sample for sample in samples if sample.get("result") == "success"]
    if len(usable) < 2:
        return None
    ts = [limit_position_t(sample) for sample in usable]
    ys = [sample[key] for sample in usable]
    slope, intercept = fit_line(ts, ys)
    errors = [slope * limit_position_t(sample) + intercept - sample[key] for sample in usable]
    return intercept, slope, errors


def print_limit_model(limit_samples: List[dict]) -> None:
    if not limit_samples:
        return
    print()
    print("Limit hold-wrist model:")
    print("Sample count:", len(limit_samples))
    print("Success count:", sum(1 for sample in limit_samples if sample.get("result") == "success"))
    print("Samples:")
    for sample in limit_samples:
        print(
            "  ",
            sample["id"],
            "result=" + sample.get("result", "unknown"),
            "position_t=%.4f" % limit_position_t(sample),
            "front_lift_deg=%.3f" % sample["front_lift_deg"],
            "elbow_drop_deg=%.3f" % sample["elbow_drop_deg"],
        )

    for key, label in (
        ("front_lift_deg", "front_lift"),
        ("elbow_drop_deg", "elbow_drop"),
    ):
        fitted = fit_limit_line(limit_samples, key)
        if fitted is None:
            print("  ", label, "not enough successful samples to fit")
            continue
        intercept, slope, errors = fitted
        abs_errors = [abs(err) for err in errors]
        print(
            "  ",
            label,
            "success-fit min_deg=%.3f" % intercept,
            "range_deg=%.3f" % slope,
            "mean_abs_err_deg=%.3f" % (sum(abs_errors) / len(abs_errors)),
            "max_abs_err_deg=%.3f" % max(abs_errors),
        )
        if key == "front_lift_deg":
            print("     suggested LIMIT_FRONT_LIFT_MIN_DEG = %.3f" % intercept)
            print("     suggested LIMIT_FRONT_LIFT_MAX_DEG = %.3f" % (intercept + slope))
        else:
            print("     suggested LIMIT_ELBOW_DROP_MIN_DEG = %.3f" % intercept)
            print("     suggested LIMIT_ELBOW_DROP_MAX_DEG = %.3f" % (intercept + slope))


def is_success_result(sample: dict) -> bool:
    return sample.get("result") == "success"


def print_local_correction_diagnostics(all_samples: List[dict]) -> None:
    local_samples = [sample for sample in all_samples if sample.get("fit") is False]
    if not local_samples:
        return

    print()
    print("Local correction diagnostics:")
    print("Sample count:", len(local_samples))
    print(
        "These samples are intentionally excluded from affine fitting; use them to tune local correction parameters."
    )

    wrist_extreme = [
        sample
        for sample in local_samples
        if "geometry_wrist_extreme_left_extra_up_deg" in sample
    ]
    if wrist_extreme:
        print()
        print("Extreme-left wrist-up correction:")
        for sample in wrist_extreme:
            print(
                "  ",
                sample["id"],
                "result=" + sample.get("result", "unknown"),
                "extra_up_deg=%.3f" % sample["geometry_wrist_extreme_left_extra_up_deg"],
            )
        success_values = [
            sample["geometry_wrist_extreme_left_extra_up_deg"]
            for sample in wrist_extreme
            if is_success_result(sample)
        ]
        low_values = [
            sample["geometry_wrist_extreme_left_extra_up_deg"]
            for sample in wrist_extreme
            if "needs" in sample.get("result", "") or "too_down" in sample.get("result", "")
        ]
        if low_values:
            print("   insufficient wrist-up below_deg=%.3f" % max(low_values))
        if success_values:
            print("   validated wrist-up success_deg=%.3f" % min(success_values))
            print("   suggested GEOMETRY_WRIST_EXTREME_LEFT_EXTRA_UP_MAX_DEG ~= 10.0")

    extreme_side = [
        sample
        for sample in local_samples
        if "extreme_left_side_inward_fraction" in sample
    ]
    if extreme_side:
        print()
        print("Extreme-left shoulder_side inward correction:")
        for sample in extreme_side:
            print(
                "  ",
                sample["id"],
                "result=" + sample.get("result", "unknown"),
                "fraction=%.3f" % sample["extreme_left_side_inward_fraction"],
                "target_side=%.4f" % sample["target"]["shoulder_side"],
            )
        success_values = [
            sample["extreme_left_side_inward_fraction"]
            for sample in extreme_side
            if is_success_result(sample)
        ]
        if success_values:
            print("   validated side inward fraction=%.3f" % min(success_values))
            print("   suggested GEOMETRY_SHOULDER_SIDE_EXTREME_LEFT_INWARD_MAX_FRACTION ~= 0.9")

    far_right_side = [
        sample
        for sample in local_samples
        if "far_right_center_side_outward_deg" in sample
    ]
    if far_right_side:
        print()
        print("Far-right-center shoulder_side outward correction:")
        for sample in far_right_side:
            print(
                "  ",
                sample["id"],
                "result=" + sample.get("result", "unknown"),
                "far_center_deg=%.3f" % sample.get("far_center_side_outward_deg", 0.0),
                "far_right_center_deg=%.3f" % sample["far_right_center_side_outward_deg"],
            )
        success_values = [
            sample["far_right_center_side_outward_deg"]
            for sample in far_right_side
            if is_success_result(sample)
        ]
        if success_values:
            print("   validated far-right-center outward_deg=%.3f" % min(success_values))
            print("   suggested GEOMETRY_SHOULDER_SIDE_FAR_RIGHT_CENTER_OUTWARD_MAX_DEG ~= 0.9")

    far_center_side = [
        sample
        for sample in local_samples
        if "far_center_side_outward_deg" in sample
        or "far_center_side_cap" in sample
    ]
    if far_center_side:
        print()
        print("Far-center shoulder_side outward correction:")
        for sample in far_center_side:
            print(
                "  ",
                sample["id"],
                "result=" + sample.get("result", "unknown"),
                "far_center_deg=%.3f" % sample.get("far_center_side_outward_deg", 0.0),
                "legacy_cap=%.3f" % sample.get("far_center_side_cap", 0.0),
            )
        print("   keep as local geometry correction; do not fold into base affine yet.")


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    payload, path = load_payload(here)

    home = payload["home"]
    all_samples = payload.get("normal_samples", payload.get("samples", []))
    samples = [sample for sample in all_samples if sample.get("fit", True)]
    limit_samples = payload.get("limit_samples", [])
    raw_by_joint: Dict[str, List[float]] = {joint: [] for joint in JOINTS}
    target_by_joint: Dict[str, List[float]] = {joint: [] for joint in JOINTS}

    rows = []
    for sample in samples:
        raw = geometry_offsets_from_shoulder_frame(
            sample["forward_cm"],
            sample["left_cm"],
            sample["up_cm"],
        )
        target_offsets = {
            joint: sample["target"][joint] - home[joint]
            for joint in JOINTS
        }
        for joint in JOINTS:
            raw_by_joint[joint].append(raw[joint])
            target_by_joint[joint].append(target_offsets[joint])
        rows.append((sample, raw, target_offsets))

    print("Sample file:", os.path.basename(path))
    print("Fit target: empirical motor offset ~= scale * raw_geometry_offset + bias")
    print("Sample count:", len(samples))
    print()

    fitted: Dict[str, Tuple[float, float]] = {}
    for joint in JOINTS:
        scale, bias = fit_line(raw_by_joint[joint], target_by_joint[joint])
        fitted[joint] = (scale, bias)
        errors = [
            scale * raw + bias - target
            for raw, target in zip(raw_by_joint[joint], target_by_joint[joint])
        ]
        stats = summarize_errors(errors)
        print(
            joint,
            "scale=%.6f" % scale,
            "bias_deg=%.3f" % math.degrees(bias),
            "mean_abs_err_deg=%.3f" % stats["mean_abs_deg"],
            "max_abs_err_deg=%.3f" % stats["max_abs_deg"],
        )

    print()
    print("Per-sample residuals, degrees:")
    for sample, raw, target_offsets in rows:
        parts = []
        for joint in JOINTS:
            scale, bias = fitted[joint]
            err_deg = math.degrees(scale * raw[joint] + bias - target_offsets[joint])
            parts.append("%s=%+.2f" % (joint, err_deg))
        print(sample["id"] + ":", ", ".join(parts))

    feature_models: List[Tuple[str, Callable[[dict, Dict[str, float], str], List[float]]]] = [
        ("raw+bias", lambda sample, raw, joint: [1.0, raw[joint]]),
        (
            "raw+shoulder_xyz",
            lambda sample, raw, joint: [
                1.0,
                raw[joint],
                sample["forward_cm"] / 50.0,
                sample["left_cm"] / 40.0,
                sample["up_cm"] / 50.0,
            ],
        ),
        (
            "shoulder_xyz_only",
            lambda sample, raw, joint: [
                1.0,
                sample["forward_cm"] / 50.0,
                sample["left_cm"] / 40.0,
                sample["up_cm"] / 50.0,
            ],
        ),
    ]

    print()
    print("Model comparison:")
    for model_name, feature_fn in feature_models:
        print(model_name + ":")
        for joint in JOINTS:
            feature_rows = [feature_fn(sample, raw, joint) for sample, raw, _ in rows]
            ys = [target_offsets[joint] for _, _, target_offsets in rows]
            coeffs = fit_linear_features(feature_rows, ys)
            errors = [dot(coeffs, features) - y for features, y in zip(feature_rows, ys)]
            stats = summarize_errors(errors)
            print(
                "  ",
                joint,
                "mean_abs_err_deg=%.3f" % stats["mean_abs_deg"],
                "max_abs_err_deg=%.3f" % stats["max_abs_deg"],
                "coeffs=" + print_model_coefficients(model_name, coeffs),
            )

    print()
    print("Suggested raw+shoulder_xyz affine constants:")
    affine_model = feature_models[1][1]
    for joint in JOINTS:
        feature_rows = [affine_model(sample, raw, joint) for sample, raw, _ in rows]
        ys = [target_offsets[joint] for _, _, target_offsets in rows]
        coeffs = fit_linear_features(feature_rows, ys)
        prefix = "GEOMETRY_AFFINE_" + joint.upper()
        print("%s_BIAS_DEG = %.9f" % (prefix, math.degrees(coeffs[0])))
        print("%s_RAW_SCALE = %.9f" % (prefix, coeffs[1]))
        print("%s_FORWARD_COEF_DEG = %.9f" % (prefix, math.degrees(coeffs[2])))
        print("%s_LEFT_COEF_DEG = %.9f" % (prefix, math.degrees(coeffs[3])))
        print("%s_UP_COEF_DEG = %.9f" % (prefix, math.degrees(coeffs[4])))

    print_limit_model(limit_samples)
    print_local_correction_diagnostics(all_samples)


if __name__ == "__main__":
    main()
