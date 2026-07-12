#!/usr/bin/env python3
"""First-stage automatic target picker for HP60C ROS RGB-D streams.

This version intentionally avoids ML. It finds a likely tabletop object inside
a center ROI by selecting depth points that are closer than the local table
background, then exports camera/arm coordinates for left_arm_controller.py.
"""

import argparse
import json
import math
import subprocess
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


INCH_TO_CM = 2.54
CAMERA_FORWARD_FROM_SHOULDER_CM = 2.5 * INCH_TO_CM
CAMERA_LEFT_FROM_SHOULDER_CM = 7.3 * INCH_TO_CM
CAMERA_UP_FROM_SHOULDER_CM = 4.5 * INCH_TO_CM
CAMERA_PITCH_DOWN_DEG = 54.0


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def pixel_to_camera_cm(
    u: int, v: int, depth_m: float, intr: Intrinsics
) -> Tuple[float, float, float]:
    z_cm = depth_m * 100.0
    x_cm = (u - intr.cx) * depth_m / intr.fx * 100.0
    y_cm = (v - intr.cy) * depth_m / intr.fy * 100.0
    return x_cm, y_cm, z_cm


def infer_intrinsics(width: int, height: int, hfov_deg: float) -> Intrinsics:
    hfov_rad = math.radians(hfov_deg)
    fx = (width * 0.5) / math.tan(hfov_rad * 0.5)
    return Intrinsics(fx=fx, fy=fx, cx=(width - 1) * 0.5, cy=(height - 1) * 0.5)


def depth_to_meters(depth: np.ndarray, depth_scale: float, force_scale: bool) -> np.ndarray:
    original_dtype = depth.dtype
    d = depth.astype(np.float32)
    if depth.ndim == 3:
        d = d[:, :, 0]
    if (original_dtype == np.float32 or original_dtype == np.float64) and not force_scale:
        return d
    return d * depth_scale


def robust_patch_depth(depth_m: np.ndarray, u: int, v: int, radius: int) -> float:
    h, w = depth_m.shape[:2]
    x0 = max(0, u - radius)
    x1 = min(w, u + radius + 1)
    y0 = max(0, v - radius)
    y1 = min(h, v + radius + 1)
    vals = depth_m[y0:y1, x0:x1].reshape(-1)
    vals = vals[np.isfinite(vals)]
    vals = vals[vals > 0]
    if vals.size == 0:
        raise ValueError("no valid target depth")
    return float(np.median(vals))


def refine_to_dark_rgb_target(
    rgb: np.ndarray,
    detect_info: dict,
    fallback_u: int,
    fallback_v: int,
    aim_y_frac: float,
    dark_v_max: int,
    min_area: int,
    max_area: int,
) -> Tuple[int, int, dict]:
    import cv2

    h, w = rgb.shape[:2]
    roi_info = detect_info.get("roi", {})
    bbox_info = detect_info.get("bbox", {})
    x0 = int(bbox_info.get("x0", roi_info.get("x0", 0)))
    y0 = int(bbox_info.get("y0", roi_info.get("y0", 0)))
    x1 = int(bbox_info.get("x1", roi_info.get("x1", w)))
    y1 = int(bbox_info.get("y1", roi_info.get("y1", h)))

    pad_x = max(12, int((x1 - x0) * 0.25))
    pad_y = max(12, int((y1 - y0) * 0.35))
    x0 = max(0, x0 - pad_x)
    y0 = max(0, y0 - pad_y)
    x1 = min(w, x1 + pad_x)
    y1 = min(h, y1 + pad_y)
    if x1 <= x0 or y1 <= y0:
        return fallback_u, fallback_v, {"used": False, "reason": "empty refine roi"}

    patch = rgb[y0:y1, x0:x1]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    # The current training object is visually black on a light tabletop. Depth
    # first finds the near object; RGB then recenters the aim point on the
    # black body instead of on a depth bbox edge.
    mask = (hsv[:, :, 2] <= dark_v_max).astype(np.uint8) * 255
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    num, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    candidates = []
    best = None
    best_score = None
    for idx in range(1, num):
        area = int(stats[idx, cv2.CC_STAT_AREA])
        if area < min_area or area > max_area:
            continue
        bx = int(stats[idx, cv2.CC_STAT_LEFT])
        by = int(stats[idx, cv2.CC_STAT_TOP])
        bw = int(stats[idx, cv2.CC_STAT_WIDTH])
        bh = int(stats[idx, cv2.CC_STAT_HEIGHT])
        cx, cy = centroids[idx]
        u = int(round(x0 + cx))
        v = int(round(y0 + by + bh * aim_y_frac))
        dist = math.hypot(u - fallback_u, v - fallback_v)
        score = dist * 0.01 - area * 0.0002
        item = {
            "idx": idx,
            "area": area,
            "u": u,
            "v": v,
            "cx": float(x0 + cx),
            "cy": float(y0 + cy),
            "bbox": {
                "x0": x0 + bx,
                "y0": y0 + by,
                "x1": x0 + bx + bw,
                "y1": y0 + by + bh,
            },
            "score": score,
        }
        candidates.append(item)
        if best_score is None or score < best_score:
            best_score = score
            best = item

    if best is None:
        return (
            fallback_u,
            fallback_v,
            {
                "used": False,
                "reason": "no dark component found",
                "roi": {"x0": x0, "y0": y0, "x1": x1, "y1": y1},
            },
        )

    return (
        best["u"],
        best["v"],
        {
            "used": True,
            "roi": {"x0": x0, "y0": y0, "x1": x1, "y1": y1},
            "dark_v_max": dark_v_max,
            "selected": best,
            "candidate_count": len(candidates),
            "candidates": candidates[:6],
        },
    )


def refine_to_dark_rgb_in_roi(
    rgb: np.ndarray,
    roi: Tuple[int, int, int, int],
    fallback_u: int,
    fallback_v: int,
    aim_y_frac: float,
    dark_v_max: int,
    min_area: int,
    max_area: int,
) -> Tuple[int, int, dict]:
    import cv2

    h, w = rgb.shape[:2]
    x0, y0, x1, y1 = roi
    x0 = max(0, min(w, x0))
    x1 = max(0, min(w, x1))
    y0 = max(0, min(h, y0))
    y1 = max(0, min(h, y1))
    if x1 <= x0 or y1 <= y0:
        return fallback_u, fallback_v, {"used": False, "reason": "empty global dark roi"}

    patch = rgb[y0:y1, x0:x1]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    mask = (hsv[:, :, 2] <= dark_v_max).astype(np.uint8) * 255
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    num, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    candidates = []
    best = None
    best_score = None
    roi_cx = (x0 + x1) * 0.5
    roi_cy = (y0 + y1) * 0.5
    for idx in range(1, num):
        area = int(stats[idx, cv2.CC_STAT_AREA])
        if area < min_area or area > max_area:
            continue
        bx = int(stats[idx, cv2.CC_STAT_LEFT])
        by = int(stats[idx, cv2.CC_STAT_TOP])
        bw = int(stats[idx, cv2.CC_STAT_WIDTH])
        bh = int(stats[idx, cv2.CC_STAT_HEIGHT])
        if bw < 12 or bh < 12:
            continue
        aspect = bw / float(max(1, bh))
        if aspect < 0.35 or aspect > 2.2:
            continue
        cx, cy = centroids[idx]
        u = int(round(x0 + cx))
        v = int(round(y0 + by + bh * aim_y_frac))
        center_penalty = math.hypot((x0 + cx) - roi_cx, (y0 + cy) - roi_cy) * 0.004
        size_penalty = abs(area - 1100) * 0.00015
        height_penalty = abs(bh - 45) * 0.01
        score = center_penalty + size_penalty + height_penalty
        item = {
            "idx": idx,
            "area": area,
            "u": u,
            "v": v,
            "cx": float(x0 + cx),
            "cy": float(y0 + cy),
            "bbox": {
                "x0": x0 + bx,
                "y0": y0 + by,
                "x1": x0 + bx + bw,
                "y1": y0 + by + bh,
            },
            "score": score,
        }
        candidates.append(item)
        if best_score is None or score < best_score:
            best_score = score
            best = item

    if best is None:
        return (
            fallback_u,
            fallback_v,
            {
                "used": False,
                "reason": "no global dark component found",
                "roi": {"x0": x0, "y0": y0, "x1": x1, "y1": y1},
            },
        )

    return (
        best["u"],
        best["v"],
        {
            "used": True,
            "source": "global_roi_dark_fallback",
            "roi": {"x0": x0, "y0": y0, "x1": x1, "y1": y1},
            "dark_v_max": dark_v_max,
            "selected": best,
            "candidate_count": len(candidates),
            "candidates": candidates[:6],
        },
    )


def find_target_in_roi(
    depth_m: np.ndarray,
    roi: Tuple[int, int, int, int],
    min_depth_m: float,
    max_depth_m: float,
    close_margin_m: float,
    min_area: int,
    max_area: int,
    aim_x_frac: float,
    aim_y_frac: float,
    target_depth_percentile: float,
) -> Tuple[int, int, dict]:
    import cv2

    x0, y0, x1, y1 = roi
    roi_depth = depth_m[y0:y1, x0:x1]
    valid = np.isfinite(roi_depth) & (roi_depth >= min_depth_m) & (roi_depth <= max_depth_m)
    if np.count_nonzero(valid) < min_area:
        finite = roi_depth[np.isfinite(roi_depth)]
        finite = finite[finite > 0]
        if finite.size:
            stats = (
                f"finite={finite.size} min={float(np.min(finite)):.4f}m "
                f"p50={float(np.percentile(finite, 50)):.4f}m "
                f"max={float(np.max(finite)):.4f}m"
            )
        else:
            stats = "no finite positive depth"
        raise RuntimeError(
            "not enough valid depth points in ROI; "
            f"valid={int(np.count_nonzero(valid))}, required={min_area}, {stats}"
        )

    valid_depths = roi_depth[valid]
    background_m = float(np.percentile(valid_depths, 70))
    near_threshold = background_m - close_margin_m
    mask = valid & (roi_depth < near_threshold)

    mask_u8 = (mask.astype(np.uint8) * 255)
    kernel = np.ones((5, 5), np.uint8)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)

    num, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_u8, 8)
    roi_cx = (x1 - x0) * 0.5
    roi_cy = (y1 - y0) * 0.5
    best_idx = None
    best_score = None
    candidates = []
    for idx in range(1, num):
        area = int(stats[idx, cv2.CC_STAT_AREA])
        if area < min_area or area > max_area:
            continue
        bx = int(stats[idx, cv2.CC_STAT_LEFT])
        by = int(stats[idx, cv2.CC_STAT_TOP])
        bw = int(stats[idx, cv2.CC_STAT_WIDTH])
        bh = int(stats[idx, cv2.CC_STAT_HEIGHT])
        cx, cy = centroids[idx]
        blob_depth = roi_depth[labels == idx]
        blob_depth = blob_depth[np.isfinite(blob_depth)]
        blob_depth = blob_depth[blob_depth > 0]
        if blob_depth.size == 0:
            continue
        median_depth = float(np.median(blob_depth))
        percentile_depth = float(np.percentile(blob_depth, target_depth_percentile))
        center_dist = math.hypot(cx - roi_cx, cy - roi_cy)
        # Prefer closer objects near the ROI center. The center term keeps the
        # detector from jumping to a large nearby arm/table edge.
        score = median_depth + center_dist * 0.001
        candidates.append(
            {
                "idx": idx,
                "area": area,
                "cx": float(x0 + cx),
                "cy": float(y0 + cy),
                "bbox": {
                    "x0": x0 + bx,
                    "y0": y0 + by,
                    "x1": x0 + bx + bw,
                    "y1": y0 + by + bh,
                },
                "median_depth_m": median_depth,
                "percentile_depth_m": percentile_depth,
                "center_dist_px": center_dist,
                "score": score,
            }
        )
        if best_score is None or score < best_score:
            best_score = score
            best_idx = idx

    if best_idx is None:
        raise RuntimeError(
            f"no target blob found; background={background_m:.3f}m "
            f"threshold={near_threshold:.3f}m min_area={min_area} max_area={max_area}"
        )

    best_area = int(stats[best_idx, cv2.CC_STAT_AREA])
    bx = int(stats[best_idx, cv2.CC_STAT_LEFT])
    by = int(stats[best_idx, cv2.CC_STAT_TOP])
    bw = int(stats[best_idx, cv2.CC_STAT_WIDTH])
    bh = int(stats[best_idx, cv2.CC_STAT_HEIGHT])
    u = int(round(x0 + bx + bw * aim_x_frac))
    v = int(round(y0 + by + bh * aim_y_frac))
    blob_depth = roi_depth[labels == best_idx]
    blob_depth = blob_depth[np.isfinite(blob_depth)]
    blob_depth = blob_depth[blob_depth > 0]
    target_depth_m = float(np.percentile(blob_depth, target_depth_percentile))
    info = {
        "background_depth_m": background_m,
        "near_threshold_m": near_threshold,
        "blob_area_px": best_area,
        "target_depth_m": target_depth_m,
        "target_depth_percentile": target_depth_percentile,
        "aim_x_frac": aim_x_frac,
        "aim_y_frac": aim_y_frac,
        "bbox": {
            "x0": x0 + bx,
            "y0": y0 + by,
            "x1": x0 + bx + bw,
            "y1": y0 + by + bh,
        },
        "candidate_count": len(candidates),
        "candidates": candidates[:8],
        "roi": {"x0": x0, "y0": y0, "x1": x1, "y1": y1},
    }
    return u, v, info


def scan_target_windows(
    depth_m: np.ndarray,
    search_roi: Tuple[int, int, int, int],
    window_w: int,
    window_h: int,
    step: int,
    min_depth_m: float,
    max_depth_m: float,
    close_margin_m: float,
    min_area: int,
    max_area: int,
    aim_x_frac: float,
    aim_y_frac: float,
    target_depth_percentile: float,
) -> Tuple[int, int, dict]:
    sx0, sy0, sx1, sy1 = search_roi
    candidates = []

    for y0 in range(sy0, max(sy0 + 1, sy1 - window_h + 1), step):
        for x0 in range(sx0, max(sx0 + 1, sx1 - window_w + 1), step):
            roi = (x0, y0, min(x0 + window_w, sx1), min(y0 + window_h, sy1))
            if roi[2] - roi[0] < window_w * 0.7 or roi[3] - roi[1] < window_h * 0.7:
                continue
            try:
                u, v, info = find_target_in_roi(
                    depth_m,
                    roi,
                    min_depth_m,
                    max_depth_m,
                    close_margin_m,
                    min_area,
                    max_area,
                    aim_x_frac,
                    aim_y_frac,
                    target_depth_percentile,
                )
            except Exception:
                continue

            bbox = info["bbox"]
            bw = bbox["x1"] - bbox["x0"]
            bh = bbox["y1"] - bbox["y0"]
            area = info["blob_area_px"]
            depth = info["target_depth_m"]

            # Prefer a compact object-sized blob near the search ROI center.
            # Do not simply prefer the closest depth; for upright tabletop
            # objects that often selects the near lower edge instead of the
            # graspable body.
            search_cx = (sx0 + sx1) * 0.5
            search_cy = (sy0 + sy1) * 0.5
            center_penalty = math.hypot(u - search_cx, v - search_cy) * 0.0005
            size_penalty = abs(area - 3500) * 0.000002
            height_penalty = abs(bh - 55) * 0.004
            width_penalty = abs(bw - 80) * 0.001
            top_penalty = max(0, bbox["y0"] - (sy0 + 65)) * 0.002
            truncated_bottom_penalty = 0.04 if bbox["y1"] >= roi[3] - 1 else 0.0
            score = (
                center_penalty
                + size_penalty
                + height_penalty
                + width_penalty
                + top_penalty
                + truncated_bottom_penalty
            )
            candidates.append(
                {
                    "u": u,
                    "v": v,
                    "score": score,
                    "depth_m": depth,
                    "area": area,
                    "bbox_w": bw,
                    "bbox_h": bh,
                    "roi": roi,
                    "info": info,
                }
            )

    if not candidates:
        raise RuntimeError(
            f"no target window found in search_roi={search_roi}; "
            f"window={window_w}x{window_h} step={step}"
        )

    candidates.sort(key=lambda item: item["score"])
    best = candidates[0]
    info = dict(best["info"])
    info["scan"] = {
        "enabled": True,
        "candidate_count": len(candidates),
        "best_score": best["score"],
        "best_window_roi": {
            "x0": best["roi"][0],
            "y0": best["roi"][1],
            "x1": best["roi"][2],
            "y1": best["roi"][3],
        },
        "top_candidates": [
            {
                "u": c["u"],
                "v": c["v"],
                "score": c["score"],
                "depth_m": c["depth_m"],
                "area": c["area"],
                "bbox_w": c["bbox_w"],
                "bbox_h": c["bbox_h"],
                "roi": {
                    "x0": c["roi"][0],
                    "y0": c["roi"][1],
                    "x1": c["roi"][2],
                    "y1": c["roi"][3],
                },
            }
            for c in candidates[:8]
        ],
    }
    return best["u"], best["v"], info


def provisional_arm_target(
    camera_xyz_cm: Tuple[float, float, float],
    offset_cm: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    return (
        camera_xyz_cm[0] + offset_cm[0],
        camera_xyz_cm[1] + offset_cm[1],
        camera_xyz_cm[2] + offset_cm[2],
    )


def camera_to_shoulder_frame_cm(
    camera_xyz_cm: Tuple[float, float, float],
    camera_forward_cm: float,
    camera_left_cm: float,
    camera_up_cm: float,
    camera_pitch_down_deg: float,
) -> Tuple[float, float, float]:
    """Convert HP60C optical coordinates into shoulder_front-axis frame.

    Camera frame from pixel projection:
      x: image right, y: image down, z: camera forward.

    Shoulder frame used here:
      forward: from shoulder_front axis toward the tabletop target,
      left: positive to the robot/camera left side,
      up: positive upward from the shoulder_front axis.
    """
    cam_x_right_cm, cam_y_down_cm, cam_z_forward_cm = camera_xyz_cm
    pitch = math.radians(camera_pitch_down_deg)
    shoulder_forward_cm = camera_forward_cm + (
        cam_z_forward_cm * math.cos(pitch) - cam_y_down_cm * math.sin(pitch)
    )
    shoulder_left_cm = camera_left_cm - cam_x_right_cm
    shoulder_up_cm = camera_up_cm - (
        cam_z_forward_cm * math.sin(pitch) + cam_y_down_cm * math.cos(pitch)
    )
    return shoulder_forward_cm, shoulder_left_cm, shoulder_up_cm


def run_controller(controller: str, arm_xyz_cm: Tuple[float, float, float]) -> None:
    cmd = [
        "sudo",
        "python3",
        controller,
        "target",
        "--x",
        f"{arm_xyz_cm[0]:.2f}",
        "--y",
        f"{arm_xyz_cm[1]:.2f}",
        "--z",
        f"{arm_xyz_cm[2]:.2f}",
        "--execute",
    ]
    print("run:", " ".join(cmd))
    subprocess.check_call(cmd)


def main() -> None:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--rgb-topic", default="/ascamera_hp60c/rgb0/image")
    parser.add_argument("--depth-topic", default="/ascamera_hp60c/depth0/image_raw")
    parser.add_argument("--camera-info-topic", default="/ascamera_hp60c/rgb0/camera_info")
    parser.add_argument("--hfov-deg", type=float, default=73.8)
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--depth-scale-set", action="store_true")
    parser.add_argument("--wait-seconds", type=float, default=5.0)

    # Default to the usable tabletop area. Keep the top edge below the drawer
    # background, but make the left/right range wide enough for random target
    # placement tests.
    parser.add_argument("--roi-x0", type=int, default=120)
    parser.add_argument("--roi-y0", type=int, default=135)
    parser.add_argument("--roi-x1", type=int, default=540)
    parser.add_argument("--roi-y1", type=int, default=385)
    parser.add_argument("--min-depth-m", type=float, default=0.20)
    parser.add_argument("--max-depth-m", type=float, default=1.20)
    parser.add_argument("--close-margin-cm", type=float, default=1.5)
    parser.add_argument("--min-area", type=int, default=80)
    parser.add_argument("--max-area", type=int, default=8000)
    parser.add_argument("--depth-radius", type=int, default=5)
    parser.add_argument("--aim-x-frac", type=float, default=0.5)
    parser.add_argument("--aim-y-frac", type=float, default=0.35)
    parser.add_argument("--target-depth-percentile", type=float, default=70.0)
    parser.add_argument("--scan-windows", action="store_true")
    parser.add_argument("--window-w", type=int, default=80)
    parser.add_argument("--window-h", type=int, default=85)
    parser.add_argument("--window-step", type=int, default=20)
    parser.add_argument("--no-rgb-dark-refine", action="store_true")
    parser.add_argument("--dark-v-max", type=int, default=95)
    parser.add_argument("--dark-min-area", type=int, default=40)
    parser.add_argument("--dark-max-area", type=int, default=4000)

    parser.add_argument("--arm-offset-x", type=float, default=18.4)
    parser.add_argument("--arm-offset-y", type=float, default=20.9)
    parser.add_argument("--arm-offset-z", type=float, default=-27.4)
    parser.add_argument("--camera-forward-from-shoulder-cm", type=float, default=CAMERA_FORWARD_FROM_SHOULDER_CM)
    parser.add_argument("--camera-left-from-shoulder-cm", type=float, default=CAMERA_LEFT_FROM_SHOULDER_CM)
    parser.add_argument("--camera-up-from-shoulder-cm", type=float, default=CAMERA_UP_FROM_SHOULDER_CM)
    parser.add_argument("--camera-pitch-down-deg", type=float, default=CAMERA_PITCH_DOWN_DEG)
    parser.add_argument(
        "--controller-mode",
        choices=("legacy", "shoulder"),
        default="legacy",
        help="legacy uses empirical arm_cm; shoulder prints/runs shoulder-frame target.",
    )

    parser.add_argument("--save-debug-prefix", default=None)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--skip-claw", action="store_true")
    parser.add_argument("--controller", default="left_arm_controller.py")
    args = parser.parse_args()

    import cv2
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image

    bridge = CvBridge()
    state = {"rgb": None, "depth": None, "intr": None}

    def rgb_cb(msg: Image) -> None:
        try:
            state["rgb"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            state["rgb"] = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def depth_cb(msg: Image) -> None:
        state["depth"] = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def info_cb(msg: CameraInfo) -> None:
        k = msg.K
        if k and k[0] > 0 and k[4] > 0:
            state["intr"] = Intrinsics(
                fx=float(k[0]),
                fy=float(k[4]),
                cx=float(k[2]),
                cy=float(k[5]),
            )

    rospy.init_node("hp60c_auto_target", anonymous=True)
    rospy.Subscriber(args.rgb_topic, Image, rgb_cb, queue_size=1)
    rospy.Subscriber(args.depth_topic, Image, depth_cb, queue_size=1)
    rospy.Subscriber(args.camera_info_topic, CameraInfo, info_cb, queue_size=1)

    print("Waiting for RGB-D frame...")
    deadline = time.time() + args.wait_seconds
    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and time.time() < deadline:
        if state["rgb"] is not None and state["depth"] is not None:
            break
        rate.sleep()

    if state["rgb"] is None or state["depth"] is None:
        raise RuntimeError("No RGB-D frame received before timeout.")

    rgb = state["rgb"]
    raw_depth = state["depth"]
    h, w = rgb.shape[:2]
    intr = state["intr"] or infer_intrinsics(w, h, args.hfov_deg)
    depth_m = depth_to_meters(raw_depth, args.depth_scale, args.depth_scale_set)

    roi = (
        max(0, args.roi_x0),
        max(0, args.roi_y0),
        min(w, args.roi_x1),
        min(h, args.roi_y1),
    )
    if args.scan_windows:
        u, v, detect_info = scan_target_windows(
            depth_m,
            roi,
            args.window_w,
            args.window_h,
            args.window_step,
            args.min_depth_m,
            args.max_depth_m,
            args.close_margin_cm / 100.0,
            args.min_area,
            args.max_area,
            args.aim_x_frac,
            args.aim_y_frac,
            args.target_depth_percentile,
        )
    else:
        u, v, detect_info = find_target_in_roi(
            depth_m,
            roi,
            args.min_depth_m,
            args.max_depth_m,
            args.close_margin_cm / 100.0,
            args.min_area,
            args.max_area,
            args.aim_x_frac,
            args.aim_y_frac,
            args.target_depth_percentile,
        )
    raw_u, raw_v = u, v
    if not args.no_rgb_dark_refine:
        u, v, rgb_refine_info = refine_to_dark_rgb_target(
            rgb,
            detect_info,
            u,
            v,
            args.aim_y_frac,
            args.dark_v_max,
            args.dark_min_area,
            args.dark_max_area,
        )
        if not rgb_refine_info.get("used"):
            u, v, fallback_info = refine_to_dark_rgb_in_roi(
                rgb,
                roi,
                u,
                v,
                args.aim_y_frac,
                args.dark_v_max,
                args.dark_min_area,
                args.dark_max_area,
            )
            rgb_refine_info["fallback"] = fallback_info
            if fallback_info.get("used"):
                rgb_refine_info = fallback_info
        detect_info["rgb_dark_refine"] = rgb_refine_info
        detect_info["raw_depth_aim_pixel"] = {"u": raw_u, "v": raw_v}
    target_depth_m = float(detect_info["target_depth_m"])
    if detect_info.get("rgb_dark_refine", {}).get("used"):
        refined_depth_m = robust_patch_depth(depth_m, u, v, args.depth_radius)
        detect_info["depth_before_rgb_refine_m"] = target_depth_m
        detect_info["target_depth_m"] = refined_depth_m
        detect_info["target_depth_source"] = "rgb_refined_pixel_patch"
        target_depth_m = refined_depth_m
    camera_xyz = pixel_to_camera_cm(u, v, target_depth_m, intr)
    arm_xyz = provisional_arm_target(
        camera_xyz,
        (args.arm_offset_x, args.arm_offset_y, args.arm_offset_z),
    )
    shoulder_xyz = camera_to_shoulder_frame_cm(
        camera_xyz,
        args.camera_forward_from_shoulder_cm,
        args.camera_left_from_shoulder_cm,
        args.camera_up_from_shoulder_cm,
        args.camera_pitch_down_deg,
    )

    result = {
        "pixel": {"u": u, "v": v},
        "depth_cm": target_depth_m * 100.0,
        "camera_cm": {"x": camera_xyz[0], "y": camera_xyz[1], "z": camera_xyz[2]},
        "arm_cm": {"x": arm_xyz[0], "y": arm_xyz[1], "z": arm_xyz[2]},
        "shoulder_frame_cm": {
            "forward": shoulder_xyz[0],
            "left": shoulder_xyz[1],
            "up": shoulder_xyz[2],
            "camera_pitch_down_deg": args.camera_pitch_down_deg,
            "note": "Origin is shoulder_front axis; forward/left/up use measured camera offsets and pitch.",
        },
        "intrinsics": {
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.cx,
            "cy": intr.cy,
        },
        "detect": detect_info,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))

    if args.save_debug_prefix:
        debug = rgb.copy()
        cv2.rectangle(debug, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 0), 2)
        if "raw_depth_aim_pixel" in detect_info:
            raw = detect_info["raw_depth_aim_pixel"]
            cv2.circle(debug, (raw["u"], raw["v"]), 5, (255, 255, 0), 1)
        refine = detect_info.get("rgb_dark_refine", {})
        selected = refine.get("selected")
        if selected:
            rb = selected["bbox"]
            cv2.rectangle(debug, (rb["x0"], rb["y0"]), (rb["x1"], rb["y1"]), (255, 0, 255), 1)
        cv2.circle(debug, (u, v), 8, (0, 255, 255), 2)
        cv2.imwrite(args.save_debug_prefix + "_rgb_target.jpg", debug)
        depth_vis = depth_m.copy()
        depth_vis[~np.isfinite(depth_vis)] = 0
        depth_vis = np.clip(depth_vis, args.min_depth_m, args.max_depth_m)
        depth_vis = ((depth_vis - args.min_depth_m) / (args.max_depth_m - args.min_depth_m) * 255).astype(np.uint8)
        depth_vis = cv2.applyColorMap(255 - depth_vis, cv2.COLORMAP_JET)
        cv2.rectangle(depth_vis, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 0), 2)
        cv2.circle(depth_vis, (u, v), 8, (0, 255, 255), 2)
        cv2.imwrite(args.save_debug_prefix + "_depth_target.jpg", depth_vis)
        print("saved debug images:", args.save_debug_prefix + "_rgb_target.jpg")

    if args.controller_mode == "shoulder":
        cmd = [
            "sudo",
            "python3",
            args.controller,
            "target-shoulder",
            "--forward",
            f"{shoulder_xyz[0]:.2f}",
            "--left",
            f"{shoulder_xyz[1]:.2f}",
            "--up",
            f"{shoulder_xyz[2]:.2f}",
            "--execute",
            "--allow-unverified-geometry",
        ]
    else:
        cmd = [
            "sudo",
            "python3",
            args.controller,
            "target",
            "--x",
            f"{arm_xyz[0]:.2f}",
            "--y",
            f"{arm_xyz[1]:.2f}",
            "--z",
            f"{arm_xyz[2]:.2f}",
            "--execute",
        ]
    if args.skip_claw:
        cmd.append("--skip-claw")
    print("Controller command:")
    print(" ".join(cmd))
    print("Shoulder-frame dry-run command:")
    print(
        " ".join(
            [
                "sudo",
                "python3",
                args.controller,
                "target-shoulder",
                "--forward",
                f"{shoulder_xyz[0]:.2f}",
                "--left",
                f"{shoulder_xyz[1]:.2f}",
                "--up",
                f"{shoulder_xyz[2]:.2f}",
            ]
        )
    )

    if args.execute:
        subprocess.check_call(cmd)


if __name__ == "__main__":
    main()
