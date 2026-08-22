#!/usr/bin/env python3
"""Chessboard vision helpers for v2.7 geometry-first grasp planning.

Phase 1 only:
- save a manual four-corner board calibration
- project board squares through a homography
- draw square ROI and board coordinates for visual inspection
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
from typing import Iterable, Optional

import cv2
import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, "data")
DEFAULT_CALIBRATION_PATH = os.path.join(DATA_DIR, "chessboard_vision_v2_7_calibration.json")
BOARD_SIZE_MM = 440.0
SQUARE_SIZE_MM = 55.0
BOARD_FILES = "abcdefgh"
BOARD_RANKS = "12345678"

# Empty-board reference captured from live v2.7 inspection.
# This is a known false-positive baseline: with no pieces on the board, g1 can produce
# a shallow full-width bottom-band depth blob under the rank-1 constant-depth model.
EMPTY_BOARD_DEPTH_BASELINES = {
    "g1": {
        "detected_count": 1,
        "detected_squares": ["g1"],
        "method": "depth",
        "board_model": "constant",
        "confidence": 1.0,
        "area_px": 2128.5,
        "bbox_patch_px": [11, 49, 74, 36],
        "median_raise_m": 0.013000011444091797,
        "close_threshold_m": 0.008,
        "close_area_px": 2228,
        "temporal_votes": 6,
        "temporal_samples": 7,
        "center_mm": [355.7851257324219, 15.022212028503418],
    }
}


def parse_point_list(text: str) -> np.ndarray:
    values = [float(value) for value in re.split(r"[,\s]+", text.strip()) if value]
    if len(values) != 8:
        raise ValueError("expected exactly four x,y points: 'x1,y1 x2,y2 x3,y3 x4,y4'")
    return np.array(values, dtype=np.float32).reshape(4, 2)


def angle_diff_deg(a: float, b: float) -> float:
    diff = abs((a - b + 90.0) % 180.0 - 90.0)
    return float(diff)


def line_from_points(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(pts) < 2:
        raise ValueError("expected at least two points to fit a line")
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).reshape(4)
    p1 = np.array([x0, y0], dtype=np.float64)
    p2 = np.array([x0 + vx, y0 + vy], dtype=np.float64)
    line = np.cross([p1[0], p1[1], 1.0], [p2[0], p2[1], 1.0])
    norm = math.hypot(float(line[0]), float(line[1]))
    if norm <= 1e-9:
        raise ValueError("degenerate fitted line")
    return (line / norm).astype(np.float64)


def point_line_distance(point: np.ndarray, line: np.ndarray) -> float:
    x, y = np.asarray(point, dtype=np.float64).reshape(2)
    return float(abs(line[0] * x + line[1] * y + line[2]))


def intersect_lines(line_a: np.ndarray, line_b: np.ndarray) -> np.ndarray:
    p = np.cross(line_a, line_b)
    if abs(float(p[2])) <= 1e-9:
        raise ValueError("parallel lines cannot intersect")
    return (p[:2] / p[2]).astype(np.float32)


def auto_locate_board_corners(
    image,
    seed_corners: np.ndarray,
    search_radius_px: float = 24.0,
    _angle_tolerance_deg: float = 18.0,
    _min_line_length_px: float = 90.0,
) -> tuple[np.ndarray, dict]:
    """Refine a1,h1,h8,a8 corners by detecting grid-line peaks near a seeded warp."""
    if image is None:
        raise RuntimeError("missing image frame")
    seed = np.asarray(seed_corners, dtype=np.float32).reshape(4, 2)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image.copy()

    board_points = np.array(
        [[0.0, 0.0], [BOARD_SIZE_MM, 0.0], [BOARD_SIZE_MM, BOARD_SIZE_MM], [0.0, BOARD_SIZE_MM]],
        dtype=np.float32,
    )
    image_to_seed_board = cv2.getPerspectiveTransform(seed, board_points)
    seed_board_to_image = np.linalg.inv(image_to_seed_board)
    warp_size = int(BOARD_SIZE_MM)
    warped = cv2.warpPerspective(image, image_to_seed_board, (warp_size, warp_size))
    warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY) if warped.ndim == 3 else warped
    dark_score = 255.0 - warped_gray.astype(np.float32)
    smooth = np.ones(5, dtype=np.float32) / 5.0
    col_score = np.convolve(dark_score.mean(axis=0), smooth, mode="same")
    row_score = np.convolve(dark_score.mean(axis=1), smooth, mode="same")

    detected_x = []
    detected_y = []
    offsets_x = []
    offsets_y = []
    for index in range(9):
        expected = index * SQUARE_SIZE_MM
        radius = search_radius_px if index not in (0, 8) else min(search_radius_px, 18.0)
        lo = max(0, int(round(expected - radius)))
        hi = min(warp_size, int(round(expected + radius)) + 1)
        x = lo + int(np.argmax(col_score[lo:hi]))
        y = lo + int(np.argmax(row_score[lo:hi]))
        detected_x.append(float(x))
        detected_y.append(float(y))
        offsets_x.append(float(x - expected))
        offsets_y.append(float(y - expected))

    if any(np.diff(detected_x) <= 0) or any(np.diff(detected_y) <= 0):
        return seed.copy(), {
            "ok": False,
            "reason": "detected_grid_lines_not_monotonic",
            "detected_x": detected_x,
            "detected_y": detected_y,
            "corners_px": seed.astype(float).tolist(),
        }

    ideal_points = []
    measured_image_points = []
    for rank_index, measured_y in enumerate(detected_y):
        for file_index, measured_x in enumerate(detected_x):
            ideal_points.append([file_index * SQUARE_SIZE_MM, rank_index * SQUARE_SIZE_MM])
            p = np.array([measured_x, measured_y, 1.0], dtype=np.float64)
            q = seed_board_to_image @ p
            measured_image_points.append((q[:2] / q[2]).tolist())

    h, _ = cv2.findHomography(np.array(ideal_points, dtype=np.float32), np.array(measured_image_points, dtype=np.float32))
    if h is None:
        return seed.copy(), {"ok": False, "reason": "failed_to_fit_grid_homography", "corners_px": seed.astype(float).tolist()}
    corners = cv2.perspectiveTransform(board_points.reshape(-1, 1, 2), h).reshape(-1, 2).astype(np.float32)
    max_shift = float(np.max(np.linalg.norm(corners - seed, axis=1)))
    if max_shift > search_radius_px * 3.0:
        return seed.copy(), {
            "ok": False,
            "reason": "grid_refined_corners_too_far_from_seed",
            "max_shift_px": max_shift,
            "detected_x": detected_x,
            "detected_y": detected_y,
            "corners_px": seed.astype(float).tolist(),
            "rejected_corners_px": corners.astype(float).tolist(),
        }

    return corners, {
        "ok": True,
        "method": "seeded_warp_grid_peaks",
        "max_shift_px": max_shift,
        "detected_x": detected_x,
        "detected_y": detected_y,
        "offsets_x": offsets_x,
        "offsets_y": offsets_y,
        "corners_px": corners.astype(float).tolist(),
    }


def normalize_square(square: str) -> str:
    text = square.strip().lower()
    if len(text) == 2 and text[0] in BOARD_FILES and text[1] in BOARD_RANKS:
        return text
    if len(text) == 2 and text[0] in BOARD_RANKS and text[1] in BOARD_FILES:
        return text[1] + text[0]
    raise ValueError(f"invalid square: {square!r}; use a1..h8 or 1a..8h")


def parse_square_list(text: str) -> list[str]:
    tokens = [value.strip().lower() for value in re.split(r"[,\s]+", text.strip()) if value]
    if any(value in ("all", "*", "board", "full") for value in tokens):
        return [file + rank for rank in BOARD_RANKS for file in BOARD_FILES]
    squares = [normalize_square(value) for value in tokens]
    if not squares:
        raise ValueError("expected at least one square, e.g. 'a2', 'a2,h2,a7,h7', or 'all'")
    return squares


def square_bounds_mm(square: str) -> np.ndarray:
    square = normalize_square(square)
    file_index = BOARD_FILES.index(square[0])
    rank_index = BOARD_RANKS.index(square[1])
    x0 = file_index * SQUARE_SIZE_MM
    y0 = rank_index * SQUARE_SIZE_MM
    x1 = x0 + SQUARE_SIZE_MM
    y1 = y0 + SQUARE_SIZE_MM
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float32)


def square_center_mm(square: str) -> tuple[float, float]:
    bounds = square_bounds_mm(square)
    center = bounds.mean(axis=0)
    return float(center[0]), float(center[1])


def board_to_image_points(points_mm: np.ndarray, homography_board_to_image: np.ndarray) -> np.ndarray:
    pts = np.asarray(points_mm, dtype=np.float32).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(pts, homography_board_to_image)
    return projected.reshape(-1, 2)


def image_to_board_points(points_px: np.ndarray, homography_board_to_image: np.ndarray) -> np.ndarray:
    h_inv = np.linalg.inv(homography_board_to_image)
    pts = np.asarray(points_px, dtype=np.float32).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(pts, h_inv)
    return projected.reshape(-1, 2)


def load_calibration(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    h = np.array(payload["homography_board_to_image"], dtype=np.float64)
    payload["homography_board_to_image"] = h
    return payload


def save_calibration_from_points(
    path: str,
    image_path: str,
    board_points: np.ndarray,
    image_points: np.ndarray,
    corner_order: str = "custom",
    note: str = "",
) -> dict:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    board_points = np.asarray(board_points, dtype=np.float32).reshape(-1, 2)
    image_points = np.asarray(image_points, dtype=np.float32).reshape(-1, 2)
    if len(board_points) != len(image_points):
        raise ValueError("board_points and image_points must have the same length")
    if len(board_points) < 4:
        raise ValueError("expected at least four calibration points")
    h, _ = cv2.findHomography(board_points, image_points.astype(np.float32))
    if h is None:
        raise RuntimeError("failed to compute homography from calibration points")
    payload = {
        "type": "chessboard_vision_v2_7_calibration",
        "image_path": image_path,
        "corner_order": corner_order,
        "board_size_mm": BOARD_SIZE_MM,
        "square_size_mm": SQUARE_SIZE_MM,
        "image_points_px": image_points.astype(float).tolist(),
        "board_points_mm": board_points.astype(float).tolist(),
        "homography_board_to_image": h.astype(float).tolist(),
        "note": note,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    return payload


def save_calibration(path: str, image_path: str, image_points: np.ndarray, note: str = "") -> dict:
    board_points = np.array(
        [[0.0, 0.0], [BOARD_SIZE_MM, 0.0], [BOARD_SIZE_MM, BOARD_SIZE_MM], [0.0, BOARD_SIZE_MM]],
        dtype=np.float32,
    )
    return save_calibration_from_points(
        path,
        image_path,
        board_points,
        image_points,
        corner_order="a1,h1,h8,a8",
        note=note,
    )


def draw_polyline(image, points: np.ndarray, color: tuple[int, int, int], thickness: int = 2) -> None:
    pts = np.round(points).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(image, [pts], True, color, thickness, cv2.LINE_AA)


def draw_board_grid(image, homography_board_to_image: np.ndarray) -> None:
    for index in range(9):
        p = index * SQUARE_SIZE_MM
        vertical = np.array([[p, 0.0], [p, BOARD_SIZE_MM]], dtype=np.float32)
        horizontal = np.array([[0.0, p], [BOARD_SIZE_MM, p]], dtype=np.float32)
        draw_polyline(image, board_to_image_points(vertical, homography_board_to_image), (0, 255, 255), 1)
        draw_polyline(image, board_to_image_points(horizontal, homography_board_to_image), (0, 255, 255), 1)


def normalize_depth_patch(depth_patch: np.ndarray) -> np.ndarray:
    depth = depth_patch.astype(np.float32)
    if depth.size == 0:
        return depth
    finite = np.isfinite(depth) & (depth > 0)
    if not np.any(finite):
        return depth
    median = float(np.nanmedian(depth[finite]))
    # ROS depth images are commonly either meters (float32) or millimeters (uint16).
    if median > 20.0:
        depth = depth / 1000.0
    return depth


def matches_empty_board_depth_baseline(square: str, area: float, bbox: tuple, median_raise: float, close_threshold_m: float, patch_size: int) -> Optional[dict]:
    baseline = EMPTY_BOARD_DEPTH_BASELINES.get(square)
    if baseline is None:
        return None
    bx, by, bw, bh = baseline["bbox_patch_px"]
    x, y, w, h = bbox
    full_width_band = w >= patch_size * 0.68 and y >= patch_size * 0.48 and h >= patch_size * 0.30
    area_close = abs(area - float(baseline["area_px"])) <= 420.0
    bbox_close = abs(x - bx) <= 5 and abs(y - by) <= 6 and abs(w - bw) <= 8 and abs(h - bh) <= 8
    raise_close = abs(median_raise - float(baseline["median_raise_m"])) <= 0.006
    threshold_close = close_threshold_m <= 0.012
    if full_width_band and area_close and bbox_close and raise_close and threshold_close:
        return {
            "square": square,
            "baseline": "empty_board_g1_2026_08_21",
            "baseline_area_px": float(baseline["area_px"]),
            "baseline_bbox_patch_px": list(baseline["bbox_patch_px"]),
            "baseline_median_raise_m": float(baseline["median_raise_m"]),
        }
    return None


def is_rank2_compact_depth_rescue(square: str, area: float, bbox: tuple, median_raise: float, close_threshold_m: float, max_raise: float, patch_size: int) -> bool:
    if square[1] != "2":
        return False
    x, y, w, h = bbox
    compact = 8 <= w <= patch_size * 0.42 and 10 <= h <= patch_size * 0.46
    enough_shape = area >= 55.0 and area <= 520.0
    enough_raise = median_raise >= max(0.008, close_threshold_m * 0.75)
    strong_peak = max_raise >= 0.08
    not_edge_band = not (w >= patch_size * 0.55 or h >= patch_size * 0.55)
    return compact and enough_shape and enough_raise and strong_peak and not_edge_band


def detect_piece_in_square_image(image, homography_board_to_image: np.ndarray, square: str, depth_image=None) -> dict:
    square = normalize_square(square)
    bounds = square_bounds_mm(square)
    x0, y0 = bounds[0]
    patch_size = 96
    dst = np.array(
        [[0, patch_size - 1], [patch_size - 1, patch_size - 1], [patch_size - 1, 0], [0, 0]],
        dtype=np.float32,
    )
    src = board_to_image_points(bounds, homography_board_to_image).astype(np.float32)
    image_to_patch = cv2.getPerspectiveTransform(src, dst)
    patch = cv2.warpPerspective(image, image_to_patch, (patch_size, patch_size))
    depth_result = None
    if depth_image is not None:
        depth_patch = cv2.warpPerspective(depth_image.astype(np.float32), image_to_patch, (patch_size, patch_size))
        depth_result = detect_piece_in_square_depth_patch(depth_patch, square, x0, y0, homography_board_to_image, patch_size)
        if depth_result["detected"]:
            return depth_result
        if depth_result.get("reason") != "insufficient_valid_depth" and float(depth_result.get("max_raise_m", 0.0)) < 0.006:
            return depth_result
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)

    border = int(patch_size * 0.14)
    valid = np.zeros((patch_size, patch_size), dtype=np.uint8)
    valid[border : patch_size - border, border : patch_size - border] = 255

    # Conservative pawn detector: look for compact raised-piece blobs, not full-square texture.
    dark = cv2.inRange(gray, 0, 122)
    sat = cv2.inRange(hsv[:, :, 1], 45, 255)
    shadow = cv2.inRange(gray, 70, 170)
    mask = cv2.bitwise_and(cv2.bitwise_or(cv2.bitwise_and(dark, sat), shadow), valid)
    mask = cv2.medianBlur(mask, 5)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    patch_center = np.array([patch_size * 0.5, patch_size * 0.5], dtype=np.float32)
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 90.0 or area > 1700.0:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w < 9 or h < 9 or w > 54 or h > 58:
            continue
        aspect = w / float(h)
        if aspect < 0.45 or aspect > 1.9:
            continue
        moments = cv2.moments(contour)
        if abs(moments["m00"]) <= 1e-6:
            continue
        center = np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]], dtype=np.float32)
        distance = float(np.linalg.norm(center - patch_center))
        if distance > patch_size * 0.32:
            continue
        fill_ratio = area / float(w * h)
        if fill_ratio < 0.22:
            continue
        score = area + 120.0 * fill_ratio - 3.0 * distance
        candidates.append((score, area, center, (x, y, w, h), fill_ratio))

    if not candidates:
        result = {
            "detected": False,
            "square": square,
            "method": "rgb",
            "reason": "no_blob_in_square_roi",
            "confidence": 0.0,
        }
        if depth_result is not None:
            result["depth_fallback"] = depth_result
        return result

    candidates.sort(key=lambda item: item[0], reverse=True)
    _score, area, center_patch, bbox, fill_ratio = candidates[0]
    center_mm = np.array(
        [[x0 + (center_patch[0] / (patch_size - 1)) * SQUARE_SIZE_MM, y0 + ((patch_size - 1 - center_patch[1]) / (patch_size - 1)) * SQUARE_SIZE_MM]],
        dtype=np.float32,
    )
    center_px = board_to_image_points(center_mm, homography_board_to_image)[0]
    confidence = max(0.0, min(1.0, area / 900.0))
    result = {
        "detected": True,
        "square": square,
        "method": "rgb",
        "confidence": float(confidence),
        "area_px": area,
        "bbox_patch_px": [int(v) for v in bbox],
        "fill_ratio": float(fill_ratio),
        "center_patch_px": center_patch.astype(float).tolist(),
        "center_mm": center_mm[0].astype(float).tolist(),
        "center_px": center_px.astype(float).tolist(),
    }
    if depth_result is not None:
        result["depth_fallback"] = depth_result
    return result


def detect_piece_in_square_depth_patch(depth_patch: np.ndarray, square: str, x0: float, y0: float, homography_board_to_image: np.ndarray, patch_size: int) -> dict:
    depth = normalize_depth_patch(depth_patch)
    finite = np.isfinite(depth) & (depth > 0)
    border = int(patch_size * 0.12)
    valid = np.zeros((patch_size, patch_size), dtype=np.uint8)
    valid[border : patch_size - border, border : patch_size - border] = 255
    valid_bool = (valid > 0) & finite
    if int(np.count_nonzero(valid_bool)) < 180:
        return {
            "detected": False,
            "square": square,
            "method": "depth",
            "reason": "insufficient_valid_depth",
            "confidence": 0.0,
            "valid_depth_samples": int(np.count_nonzero(valid_bool)),
        }

    yy, xx = np.mgrid[0:patch_size, 0:patch_size].astype(np.float32)
    center_xy = (patch_size - 1) * 0.5
    center_radius = patch_size * 0.28
    center_region = ((xx - center_xy) ** 2 + (yy - center_xy) ** 2) <= center_radius**2
    board_sample = valid_bool & ~center_region
    if int(np.count_nonzero(board_sample)) < 120:
        board_sample = valid_bool
    board_depth_all = float(np.nanpercentile(depth[valid_bool], 70))
    board_depth_outer = float(np.nanpercentile(depth[board_sample], 72))
    board_depth = max(board_depth_all, board_depth_outer)
    plane_sample = board_sample.copy()
    board_sample_depth = depth[board_sample]
    if board_sample_depth.size:
        # Ignore the nearest outer samples so a piece touching the ROI edge does not pull the board plane upward.
        plane_min_depth = float(np.nanpercentile(board_sample_depth, 35))
        plane_sample = plane_sample & (depth >= plane_min_depth)
    if int(np.count_nonzero(plane_sample)) < 120:
        plane_sample = board_sample
    px = xx[plane_sample].reshape(-1) / float(patch_size - 1)
    py = yy[plane_sample].reshape(-1) / float(patch_size - 1)
    pz = depth[plane_sample].reshape(-1)
    plane_depth = np.full_like(depth, board_depth, dtype=np.float32)
    plane_rmse_m = None
    if len(pz) >= 120:
        design = np.column_stack([px, py, np.ones_like(px)])
        coeffs, _residuals, _rank, _singular = np.linalg.lstsq(design, pz, rcond=None)
        plane_depth = (
            coeffs[0] * (xx / float(patch_size - 1))
            + coeffs[1] * (yy / float(patch_size - 1))
            + coeffs[2]
        ).astype(np.float32)
        plane_fit = design @ coeffs
        plane_rmse_m = float(np.sqrt(np.mean((plane_fit - pz) ** 2)))
    use_constant_depth = square[1] == "1"
    board_model = "constant" if use_constant_depth else "plane"
    if not use_constant_depth and plane_rmse_m is not None and plane_rmse_m > 0.025:
        return {
            "detected": False,
            "square": square,
            "method": "depth",
            "reason": "unstable_depth_plane",
            "confidence": 0.0,
            "board_depth_m": board_depth,
            "board_depth_all_m": board_depth_all,
            "board_depth_outer_m": board_depth_outer,
            "board_model": board_model,
            "plane_depth_center_m": float(plane_depth[int(round(center_xy)), int(round(center_xy))]),
            "plane_sample_depth_samples": int(np.count_nonzero(plane_sample)),
            "plane_rmse_m": plane_rmse_m,
            "valid_depth_samples": int(np.count_nonzero(valid_bool)),
            "board_sample_depth_samples": int(np.count_nonzero(board_sample)),
        }
    close_delta = (board_depth - depth) if use_constant_depth else (plane_depth - depth)
    max_raise = float(np.nanmax(close_delta[valid_bool])) if np.any(valid_bool) else 0.0
    threshold_candidates = [0.018, 0.012, 0.008, 0.006]
    candidates = []
    patch_center = np.array([patch_size * 0.5, patch_size * 0.5], dtype=np.float32)
    best_close_mask = np.zeros((patch_size, patch_size), dtype=np.uint8)
    best_close_threshold_m = threshold_candidates[-1]

    for close_threshold_m in threshold_candidates:
        close_mask = ((close_delta > close_threshold_m) & valid_bool).astype(np.uint8) * 255
        close_mask = cv2.medianBlur(close_mask, 3)
        close_mask = cv2.morphologyEx(close_mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        close_mask = cv2.morphologyEx(close_mask, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8))
        best_close_mask = close_mask
        best_close_threshold_m = close_threshold_m

        contours, _ = cv2.findContours(close_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 24.0 or area > 2400.0:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 5 or h < 5 or w > 74 or h > 74:
                continue
            if w >= patch_size * 0.68 and h < patch_size * 0.36:
                continue
            aspect = w / float(h)
            if aspect > 2.6:
                continue
            moments = cv2.moments(contour)
            if abs(moments["m00"]) <= 1e-6:
                continue
            center = np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]], dtype=np.float32)
            distance = float(np.linalg.norm(center - patch_center))
            if distance > patch_size * 0.54:
                continue
            contour_mask = np.zeros_like(close_mask)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            mask_bool = contour_mask > 0
            median_raise = float(np.nanmedian(close_delta[mask_bool])) if np.any(mask_bool) else 0.0
            if median_raise < close_threshold_m:
                continue
            if w >= patch_size * 0.68 and median_raise < 0.013:
                continue
            score = area + 7000.0 * median_raise - 1.5 * distance
            candidates.append((score, area, center, (x, y, w, h), median_raise, close_threshold_m, close_mask))
        if candidates:
            break

    if not candidates:
        return {
            "detected": False,
            "square": square,
            "method": "depth",
            "reason": "no_close_depth_blob",
            "confidence": 0.0,
            "board_depth_m": board_depth,
            "board_depth_all_m": board_depth_all,
            "board_depth_outer_m": board_depth_outer,
            "board_model": board_model,
            "plane_depth_center_m": float(plane_depth[int(round(center_xy)), int(round(center_xy))]),
            "plane_sample_depth_samples": int(np.count_nonzero(plane_sample)),
            "plane_rmse_m": plane_rmse_m,
            "valid_depth_samples": int(np.count_nonzero(valid_bool)),
            "board_sample_depth_samples": int(np.count_nonzero(board_sample)),
            "max_raise_m": max_raise,
            "close_threshold_m": best_close_threshold_m,
            "close_area_px": int(np.count_nonzero(best_close_mask)),
        }

    candidates.sort(key=lambda item: item[0], reverse=True)
    _score, area, center_patch, bbox, median_raise, close_threshold_m, close_mask = candidates[0]
    center_mm = np.array(
        [[x0 + (center_patch[0] / (patch_size - 1)) * SQUARE_SIZE_MM, y0 + ((patch_size - 1 - center_patch[1]) / (patch_size - 1)) * SQUARE_SIZE_MM]],
        dtype=np.float32,
    )
    center_px = board_to_image_points(center_mm, homography_board_to_image)[0]
    empty_board_baseline = matches_empty_board_depth_baseline(square, area, bbox, median_raise, close_threshold_m, patch_size)
    if empty_board_baseline is not None:
        return {
            "detected": False,
            "square": square,
            "method": "depth",
            "reason": "matches_empty_board_baseline",
            "confidence": 0.0,
            "area_px": area,
            "bbox_patch_px": [int(v) for v in bbox],
            "median_raise_m": median_raise,
            "empty_board_baseline": empty_board_baseline,
            "board_depth_m": board_depth,
            "board_depth_all_m": board_depth_all,
            "board_depth_outer_m": board_depth_outer,
            "board_model": board_model,
            "plane_depth_center_m": float(plane_depth[int(round(center_xy)), int(round(center_xy))]),
            "plane_sample_depth_samples": int(np.count_nonzero(plane_sample)),
            "plane_rmse_m": plane_rmse_m,
            "valid_depth_samples": int(np.count_nonzero(valid_bool)),
            "board_sample_depth_samples": int(np.count_nonzero(board_sample)),
            "max_raise_m": max_raise,
            "close_threshold_m": close_threshold_m,
            "close_area_px": int(np.count_nonzero(close_mask)),
            "center_patch_px": center_patch.astype(float).tolist(),
            "center_mm": center_mm[0].astype(float).tolist(),
            "center_px": center_px.astype(float).tolist(),
        }
    confidence = max(0.0, min(1.0, (median_raise / 0.045) * (area / 500.0)))
    rank2_rescue = is_rank2_compact_depth_rescue(square, area, bbox, median_raise, close_threshold_m, max_raise, patch_size)
    if rank2_rescue:
        confidence = max(confidence, min(1.0, 0.22 + area / 600.0 + median_raise / 0.08))
    if confidence < 0.12:
        return {
            "detected": False,
            "square": square,
            "method": "depth",
            "reason": "low_confidence_depth_blob",
            "confidence": float(confidence),
            "area_px": area,
            "bbox_patch_px": [int(v) for v in bbox],
            "median_raise_m": median_raise,
            "board_depth_m": board_depth,
            "board_depth_all_m": board_depth_all,
            "board_depth_outer_m": board_depth_outer,
            "board_model": board_model,
            "plane_depth_center_m": float(plane_depth[int(round(center_xy)), int(round(center_xy))]),
            "plane_sample_depth_samples": int(np.count_nonzero(plane_sample)),
            "plane_rmse_m": plane_rmse_m,
            "valid_depth_samples": int(np.count_nonzero(valid_bool)),
            "board_sample_depth_samples": int(np.count_nonzero(board_sample)),
            "max_raise_m": max_raise,
            "close_threshold_m": close_threshold_m,
            "close_area_px": int(np.count_nonzero(close_mask)),
            "center_patch_px": center_patch.astype(float).tolist(),
            "center_mm": center_mm[0].astype(float).tolist(),
            "center_px": center_px.astype(float).tolist(),
        }
    return {
        "detected": True,
        "square": square,
        "method": "depth_rank2_compact_rescue" if rank2_rescue else "depth",
        "confidence": float(confidence),
        "area_px": area,
        "bbox_patch_px": [int(v) for v in bbox],
        "median_raise_m": median_raise,
        "board_depth_m": board_depth,
        "board_depth_all_m": board_depth_all,
        "board_depth_outer_m": board_depth_outer,
        "board_model": board_model,
        "plane_depth_center_m": float(plane_depth[int(round(center_xy)), int(round(center_xy))]),
        "plane_sample_depth_samples": int(np.count_nonzero(plane_sample)),
        "plane_rmse_m": plane_rmse_m,
        "valid_depth_samples": int(np.count_nonzero(valid_bool)),
        "board_sample_depth_samples": int(np.count_nonzero(board_sample)),
        "max_raise_m": max_raise,
        "close_threshold_m": close_threshold_m,
        "close_area_px": int(np.count_nonzero(close_mask)),
        "center_patch_px": center_patch.astype(float).tolist(),
        "center_mm": center_mm[0].astype(float).tolist(),
        "center_px": center_px.astype(float).tolist(),
    }


def annotate_square_image(image, calibration_path: str, square: str, output_path: str, image_path: str = "") -> dict:
    return annotate_squares_image(image, calibration_path, square, output_path, image_path=image_path)


def draw_squares_overlay_image(
    image,
    calibration_path: str,
    squares: str,
    image_path: str = "",
    detect_pieces: bool = False,
    depth_image=None,
    piece_results_override: Optional[dict] = None,
) -> dict:
    if image is None:
        raise RuntimeError("missing image frame")
    calibration = load_calibration(calibration_path)
    h = calibration["homography_board_to_image"]
    square_list = parse_square_list(squares)
    draw_board_grid(image, h)

    square_results = {}
    piece_results = {}
    detected_squares = []
    title_parts = []
    compact_detection_overlay = len(square_list) > 16
    for square in square_list:
        roi_mm = square_bounds_mm(square)
        roi_px = board_to_image_points(roi_mm, h)
        center_mm = np.array([square_center_mm(square)], dtype=np.float32)
        center_px = board_to_image_points(center_mm, h)[0]
        if not compact_detection_overlay:
            draw_polyline(image, roi_px, (0, 0, 255), 3)
        center_xy = tuple(np.round(center_px).astype(int))
        if not compact_detection_overlay:
            cv2.circle(image, center_xy, 5, (255, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(
                image,
                square,
                (center_xy[0] + 7, center_xy[1] - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        square_results[square] = {
            "center_mm": center_mm[0].astype(float).tolist(),
            "center_px": center_px.astype(float).tolist(),
            "roi_px": roi_px.astype(float).tolist(),
        }
        if detect_pieces:
            if piece_results_override is not None and square in piece_results_override:
                piece = piece_results_override[square]
            else:
                piece = detect_piece_in_square_image(image, h, square, depth_image=depth_image)
            piece_results[square] = piece
            if piece.get("detected"):
                detected_squares.append(square)
                draw_polyline(image, roi_px, (0, 255, 0), 2 if compact_detection_overlay else 3)
                piece_xy = tuple(np.round(piece["center_px"]).astype(int))
                cv2.circle(image, piece_xy, 7, (0, 255, 0), -1, cv2.LINE_AA)
                cv2.circle(image, piece_xy, 11, (0, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(
                    image,
                    square if compact_detection_overlay else "piece " + str(piece.get("method", "")),
                    (piece_xy[0] + 9, piece_xy[1] + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            elif not compact_detection_overlay:
                reason = str(piece.get("reason", "no_piece"))
                label = "no " + str(piece.get("method", "")) + " " + reason[:16]
                cv2.putText(
                    image,
                    label,
                    (center_xy[0] + 7, center_xy[1] + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (0, 180, 255),
                    1,
                    cv2.LINE_AA,
                )
        title_parts.append(square)
    cv2.putText(
        image,
        ("detected=" + ",".join(detected_squares)) if compact_detection_overlay else "squares=" + ",".join(title_parts),
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    first_square = square_list[0]
    return {
        "square": first_square,
        "squares": square_list,
        "center_mm": square_results[first_square]["center_mm"],
        "center_px": square_results[first_square]["center_px"],
        "roi_px": square_results[first_square]["roi_px"],
        "square_results": square_results,
        "piece_results": piece_results,
        "detected_squares": detected_squares,
        "detected_count": len(detected_squares),
        "image_path": image_path,
    }


def annotate_squares_image(
    image,
    calibration_path: str,
    squares: str,
    output_path: str,
    image_path: str = "",
    detect_pieces: bool = False,
    depth_image=None,
    piece_results_override: Optional[dict] = None,
) -> dict:
    result = draw_squares_overlay_image(
        image,
        calibration_path,
        squares,
        image_path=image_path,
        detect_pieces=detect_pieces,
        depth_image=depth_image,
        piece_results_override=piece_results_override,
    )
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    cv2.imwrite(output_path, image)
    result["output_path"] = output_path
    return result


def annotate_square(image_path: str, calibration_path: str, square: str, output_path: str) -> dict:
    image = cv2.imread(image_path)
    if image is None:
        raise RuntimeError(f"failed to read image: {image_path}")
    return annotate_square_image(image, calibration_path, square, output_path, image_path=image_path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="v2.7 chessboard vision phase-1 geometry tool")
    sub = parser.add_subparsers(dest="cmd", required=True)

    calibrate = sub.add_parser("calibrate")
    calibrate.add_argument("--image", required=True)
    calibrate.add_argument(
        "--corners",
        required=True,
        help="four inner board black-line corners in order a1,h1,h8,a8, e.g. '100,428 595,418 520,52 165,58'",
    )
    calibrate.add_argument("--output", default=DEFAULT_CALIBRATION_PATH)
    calibrate.add_argument("--note", default="")

    inspect = sub.add_parser("inspect-square")
    inspect.add_argument("--image", required=True)
    inspect.add_argument("--calibration", default=DEFAULT_CALIBRATION_PATH)
    inspect.add_argument("--square", required=True, help="one square or a list, e.g. 'a2' or 'a2,h2,a7,h7'")
    inspect.add_argument("--output", required=True)

    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.cmd == "calibrate":
        image_points = parse_point_list(args.corners)
        payload = save_calibration(args.output, args.image, image_points, args.note)
        print(json.dumps({k: v for k, v in payload.items() if k != "homography_board_to_image"}, ensure_ascii=False, indent=2))
    elif args.cmd == "inspect-square":
        result = annotate_square(args.image, args.calibration, args.square, args.output)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        raise RuntimeError(f"unknown command: {args.cmd}")


if __name__ == "__main__":
    main()
