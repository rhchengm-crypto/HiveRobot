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
import time
from typing import Iterable, Optional

import cv2
import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, "data")
DEFAULT_CALIBRATION_PATH = os.path.join(DATA_DIR, "chessboard_vision_v2_7_calibration.json")
DEFAULT_EMPTY_BOARD_BASELINE_PATH = os.path.join(DATA_DIR, "chessboard_vision_v2_7_empty_board_baseline.json")
BOARD_SIZE_MM = 440.0
SQUARE_SIZE_MM = 55.0
BOARD_FILES = "abcdefgh"
BOARD_RANKS = "12345678"

STANDARD_STARTING_PIECES = {
    "a1": {"piece_id": "white_rook_queen_side", "piece_type": "rook", "color": "white"},
    "b1": {"piece_id": "white_knight_queen_side", "piece_type": "knight", "color": "white"},
    "c1": {"piece_id": "white_bishop_queen_side", "piece_type": "bishop", "color": "white"},
    "d1": {"piece_id": "white_queen", "piece_type": "queen", "color": "white"},
    "e1": {"piece_id": "white_king", "piece_type": "king", "color": "white"},
    "f1": {"piece_id": "white_bishop_king_side", "piece_type": "bishop", "color": "white"},
    "g1": {"piece_id": "white_knight_king_side", "piece_type": "knight", "color": "white"},
    "h1": {"piece_id": "white_rook_king_side", "piece_type": "rook", "color": "white"},
    "a8": {"piece_id": "black_rook_queen_side", "piece_type": "rook", "color": "black"},
    "b8": {"piece_id": "black_knight_queen_side", "piece_type": "knight", "color": "black"},
    "c8": {"piece_id": "black_bishop_queen_side", "piece_type": "bishop", "color": "black"},
    "d8": {"piece_id": "black_queen", "piece_type": "queen", "color": "black"},
    "e8": {"piece_id": "black_king", "piece_type": "king", "color": "black"},
    "f8": {"piece_id": "black_bishop_king_side", "piece_type": "bishop", "color": "black"},
    "g8": {"piece_id": "black_knight_king_side", "piece_type": "knight", "color": "black"},
    "h8": {"piece_id": "black_rook_king_side", "piece_type": "rook", "color": "black"},
}
for _file in BOARD_FILES:
    STANDARD_STARTING_PIECES[f"{_file}2"] = {
        "piece_id": f"white_pawn_{_file}",
        "piece_type": "pawn",
        "color": "white",
    }
    STANDARD_STARTING_PIECES[f"{_file}7"] = {
        "piece_id": f"black_pawn_{_file}",
        "piece_type": "pawn",
        "color": "black",
    }

CHESS_PIECE_YOLO_CLASSES = [
    "white_pawn",
    "white_rook",
    "white_knight",
    "white_bishop",
    "white_queen",
    "white_king",
    "black_pawn",
    "black_rook",
    "black_knight",
    "black_bishop",
    "black_queen",
    "black_king",
]

OPENING_TARGET_SLOTS = {
    "white_pawn": ["a2", "b2", "c2", "d2", "e2", "f2", "g2", "h2"],
    "white_rook": ["a1", "h1"],
    "white_knight": ["b1", "g1"],
    "white_bishop": ["c1", "f1"],
    "white_queen": ["d1"],
    "white_king": ["e1"],
    "black_pawn": ["a7", "b7", "c7", "d7", "e7", "f7", "g7", "h7"],
    "black_rook": ["a8", "h8"],
    "black_knight": ["b8", "g8"],
    "black_bishop": ["c8", "f8"],
    "black_queen": ["d8"],
    "black_king": ["e8"],
}

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

EMPTY_BOARD_RGB_BASELINES = {
    "f1": {
        "baseline": "empty_board_f1_rgb_2026_08_23",
        "method": "rgb",
        "confidence": 0.9,
        "area_px": 816.0,
        "bbox_patch_px": [59, 31, 25, 52],
        "fill_ratio": 0.65,
        "center_patch_px": [72.2, 60.3],
        "center_mm": [316.9, 20.5],
        "note": "Observed empty-board RGB texture/shadow false positive on f1.",
    }
}
_EMPTY_BOARD_BASELINE_CACHE = {"path": None, "mtime": None, "depth_baselines": None, "rgb_baselines": None}


def load_empty_board_depth_baselines(path: Optional[str] = None) -> dict:
    baseline_path = path or DEFAULT_EMPTY_BOARD_BASELINE_PATH
    baselines = dict(EMPTY_BOARD_DEPTH_BASELINES)
    try:
        mtime = os.path.getmtime(baseline_path)
    except OSError:
        return baselines
    if (
        _EMPTY_BOARD_BASELINE_CACHE["path"] == baseline_path
        and _EMPTY_BOARD_BASELINE_CACHE["mtime"] == mtime
        and isinstance(_EMPTY_BOARD_BASELINE_CACHE["depth_baselines"], dict)
    ):
        loaded = _EMPTY_BOARD_BASELINE_CACHE["depth_baselines"]
    else:
        try:
            with open(baseline_path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            loaded = payload.get("depth_baselines", {}) if isinstance(payload, dict) else {}
        except Exception:
            loaded = {}
        _EMPTY_BOARD_BASELINE_CACHE.update({"path": baseline_path, "mtime": mtime, "depth_baselines": loaded})
    if isinstance(loaded, dict):
        for square, info in loaded.items():
            try:
                baselines[normalize_square(square)] = dict(info)
            except Exception:
                continue
    return baselines


def load_empty_board_rgb_baselines(path: Optional[str] = None) -> dict:
    baseline_path = path or DEFAULT_EMPTY_BOARD_BASELINE_PATH
    baselines = dict(EMPTY_BOARD_RGB_BASELINES)
    try:
        mtime = os.path.getmtime(baseline_path)
    except OSError:
        return baselines
    if (
        _EMPTY_BOARD_BASELINE_CACHE["path"] == baseline_path
        and _EMPTY_BOARD_BASELINE_CACHE["mtime"] == mtime
        and isinstance(_EMPTY_BOARD_BASELINE_CACHE["rgb_baselines"], dict)
    ):
        loaded = _EMPTY_BOARD_BASELINE_CACHE["rgb_baselines"]
    else:
        try:
            with open(baseline_path, "r", encoding="utf-8") as f:
                payload = json.load(f)
        except Exception:
            payload = {}
        loaded = payload.get("rgb_baselines", {}) if isinstance(payload, dict) else {}
        if not loaded and isinstance(payload, dict):
            empty_results = payload.get("metadata", {}).get("empty_piece_results", {})
            loaded = {
                square: info
                for square, info in empty_results.items()
                if isinstance(info, dict) and info.get("detected") and str(info.get("method", "")) == "rgb"
            }
        _EMPTY_BOARD_BASELINE_CACHE.update({"path": baseline_path, "mtime": mtime, "rgb_baselines": loaded})
    if isinstance(loaded, dict):
        for square, info in loaded.items():
            try:
                baselines[normalize_square(square)] = dict(info)
            except Exception:
                continue
    return baselines


def save_empty_board_depth_baselines(path: str, depth_baselines: dict, metadata: Optional[dict] = None, rgb_baselines: Optional[dict] = None) -> dict:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    payload = {
        "type": "chessboard_vision_v2_7_empty_board_baseline",
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": "Captured with no chess pieces on the board. Used to suppress empty-board depth artifacts.",
        "metadata": metadata or {},
        "depth_baselines": depth_baselines,
        "rgb_baselines": rgb_baselines or {},
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    try:
        _EMPTY_BOARD_BASELINE_CACHE.update(
            {
                "path": path,
                "mtime": os.path.getmtime(path),
                "depth_baselines": dict(depth_baselines),
                "rgb_baselines": dict(rgb_baselines or {}),
            }
        )
    except OSError:
        pass
    return payload


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


def board_point_to_square(x_mm: float, y_mm: float) -> Optional[str]:
    if x_mm < 0.0 or y_mm < 0.0 or x_mm >= BOARD_SIZE_MM or y_mm >= BOARD_SIZE_MM:
        return None
    file_index = int(x_mm // SQUARE_SIZE_MM)
    rank_index = int(y_mm // SQUARE_SIZE_MM)
    if not (0 <= file_index < 8 and 0 <= rank_index < 8):
        return None
    return BOARD_FILES[file_index] + BOARD_RANKS[rank_index]


def identify_piece_for_square(square: str, layout: Optional[dict] = None) -> dict:
    square = normalize_square(square)
    piece_layout = STANDARD_STARTING_PIECES if layout is None else layout
    identity = piece_layout.get(square)
    if not identity:
        return {
            "piece_id": "unknown_piece",
            "piece_type": "unknown",
            "color": "unknown",
            "identity_method": "square_layout",
            "identity_confidence": 0.0,
        }
    result = dict(identity)
    result["identity_method"] = "standard_starting_position" if layout is None else "custom_square_layout"
    result["identity_confidence"] = 1.0
    return result


def assign_opening_targets(piece_class_results: dict, occupied_targets: Optional[Iterable[str]] = None) -> list[dict]:
    used_targets = set(normalize_square(square) for square in (occupied_targets or []))
    placement_plan = []
    for source_square in sorted(piece_class_results):
        info = piece_class_results[source_square]
        piece_class = str(info.get("piece_class", "unknown"))
        if piece_class not in OPENING_TARGET_SLOTS:
            placement_plan.append(
                {
                    "pick": source_square,
                    "place": None,
                    "piece_class": piece_class,
                    "reason": "unknown_piece_class",
                    "confidence": float(info.get("confidence", 0.0)),
                }
            )
            continue
        target_square = None
        for candidate in OPENING_TARGET_SLOTS[piece_class]:
            if candidate not in used_targets:
                target_square = candidate
                used_targets.add(candidate)
                break
        if target_square is None:
            placement_plan.append(
                {
                    "pick": source_square,
                    "place": None,
                    "piece_class": piece_class,
                    "reason": "no_opening_slot_available",
                    "confidence": float(info.get("confidence", 0.0)),
                }
            )
            continue
        placement_plan.append(
            {
                "pick": source_square,
                "place": target_square,
                "piece_class": piece_class,
                "confidence": float(info.get("confidence", 0.0)),
            }
        )
    return placement_plan


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


def matches_empty_board_depth_baseline(
    square: str,
    area: float,
    bbox: tuple,
    median_raise: float,
    close_threshold_m: float,
    patch_size: int,
    empty_board_baselines: Optional[dict] = None,
) -> Optional[dict]:
    baselines = load_empty_board_depth_baselines() if empty_board_baselines is None else empty_board_baselines
    baseline = baselines.get(square)
    if baseline is None:
        return None
    bx, by, bw, bh = baseline["bbox_patch_px"]
    x, y, w, h = bbox
    area_tolerance = max(120.0, float(baseline["area_px"]) * 0.28)
    bbox_tolerance = max(7.0, patch_size * 0.08)
    raise_tolerance = max(0.004, min(0.010, float(baseline.get("median_raise_m", 0.0)) * 0.55))
    area_close = abs(area - float(baseline["area_px"])) <= area_tolerance
    bbox_close = (
        abs(x - bx) <= bbox_tolerance
        and abs(y - by) <= bbox_tolerance
        and abs(w - bw) <= bbox_tolerance
        and abs(h - bh) <= bbox_tolerance
    )
    raise_close = abs(median_raise - float(baseline["median_raise_m"])) <= raise_tolerance
    threshold_close = abs(close_threshold_m - float(baseline.get("close_threshold_m", close_threshold_m))) <= 0.006
    if area_close and bbox_close and raise_close and threshold_close:
        return {
            "square": square,
            "baseline": str(baseline.get("baseline", "empty_board_web_capture")),
            "baseline_area_px": float(baseline["area_px"]),
            "baseline_bbox_patch_px": list(baseline["bbox_patch_px"]),
            "baseline_median_raise_m": float(baseline["median_raise_m"]),
            "baseline_close_threshold_m": float(baseline.get("close_threshold_m", close_threshold_m)),
        }
    return None


def matches_empty_board_rgb_baseline(
    square: str,
    area: float,
    bbox: tuple,
    fill_ratio: float,
    center_patch: np.ndarray,
    patch_size: int,
    empty_board_baselines: Optional[dict] = None,
) -> Optional[dict]:
    baselines = load_empty_board_rgb_baselines() if empty_board_baselines is None else empty_board_baselines
    baseline = baselines.get(square)
    if baseline is None:
        return None
    bx, by, bw, bh = baseline["bbox_patch_px"]
    x, y, w, h = bbox
    area_tolerance = max(120.0, float(baseline["area_px"]) * 0.38)
    bbox_tolerance = max(12.0, patch_size * 0.14)
    fill_tolerance = 0.28
    area_close = abs(area - float(baseline["area_px"])) <= area_tolerance
    bbox_close = (
        abs(x - bx) <= bbox_tolerance
        and abs(y - by) <= bbox_tolerance
        and abs(w - bw) <= bbox_tolerance
        and abs(h - bh) <= bbox_tolerance
    )
    fill_close = abs(fill_ratio - float(baseline.get("fill_ratio", fill_ratio))) <= fill_tolerance
    center_close = bbox_close
    baseline_center = baseline.get("center_patch_px")
    if isinstance(baseline_center, list) and len(baseline_center) == 2:
        center_close = float(np.linalg.norm(np.asarray(center_patch, dtype=np.float32) - np.asarray(baseline_center, dtype=np.float32))) <= patch_size * 0.16
    shape_close = bbox_close or center_close
    if area_close and shape_close and fill_close:
        return {
            "square": square,
            "baseline": str(baseline.get("baseline", "empty_board_web_capture")),
            "baseline_area_px": float(baseline["area_px"]),
            "baseline_bbox_patch_px": list(baseline["bbox_patch_px"]),
            "baseline_fill_ratio": float(baseline.get("fill_ratio", fill_ratio)),
            "baseline_center_patch_px": baseline_center,
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


def is_rank1_compact_depth_rescue(square: str, area: float, bbox: tuple, median_raise: float, close_threshold_m: float, max_raise: float, patch_size: int) -> bool:
    if square[1] != "1":
        return False
    _x, y, w, h = bbox
    compact = 5 <= w <= patch_size * 0.48 and 5 <= h <= patch_size * 0.48
    enough_shape = 45.0 <= area <= 900.0
    upper_half = y <= patch_size * 0.38
    enough_raise = median_raise >= max(0.012, close_threshold_m * 0.85)
    strong_peak = max_raise >= 0.08
    not_edge_band = not (w >= patch_size * 0.62 or h >= patch_size * 0.62)
    return compact and enough_shape and upper_half and enough_raise and strong_peak and not_edge_band


def is_rank1_edge_depth_artifact(square: str, area: float, bbox: tuple, confidence: float, center_patch: np.ndarray, patch_size: int) -> bool:
    if square[1] != "1":
        return False
    _x, y, w, h = bbox
    center = np.asarray(center_patch, dtype=np.float32).reshape(2)
    lower_band = y >= patch_size * 0.45 or center[1] >= patch_size * 0.58
    full_width_band = w >= patch_size * 0.62 and h >= patch_size * 0.25 and area >= 850.0
    near_bottom_edge = y >= patch_size * 0.62 or center[1] >= patch_size * 0.72
    short_or_thin = h <= patch_size * 0.24 or w <= patch_size * 0.34
    small_edge_blob = area <= 520.0 and confidence < 0.45
    return (lower_band and full_width_band) or (near_bottom_edge and short_or_thin and small_edge_blob)


def is_rank1_rgb_artifact(square: str, area: float, bbox: tuple, confidence: float, center_patch: np.ndarray, patch_size: int) -> bool:
    if square[1] != "1":
        return False
    _x, y, w, h = bbox
    center = np.asarray(center_patch, dtype=np.float32).reshape(2)
    low_confidence = confidence < 0.42
    lower_blob = y >= patch_size * 0.48 or center[1] >= patch_size * 0.60
    compact_blob = area <= 700.0 and w <= patch_size * 0.50 and h <= patch_size * 0.50
    return low_confidence and lower_blob and compact_blob


def detect_piece_in_square_image(
    image,
    homography_board_to_image: np.ndarray,
    square: str,
    depth_image=None,
    empty_board_baselines: Optional[dict] = None,
    empty_board_depth_baselines: Optional[dict] = None,
    empty_board_rgb_baselines: Optional[dict] = None,
) -> dict:
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
    depth_baselines = empty_board_depth_baselines if empty_board_depth_baselines is not None else empty_board_baselines
    rgb_baselines = empty_board_rgb_baselines if empty_board_rgb_baselines is not None else empty_board_baselines
    if depth_image is not None:
        depth_patch = cv2.warpPerspective(depth_image.astype(np.float32), image_to_patch, (patch_size, patch_size))
        depth_result = detect_piece_in_square_depth_patch(
            depth_patch,
            square,
            x0,
            y0,
            homography_board_to_image,
            patch_size,
            empty_board_baselines=depth_baselines,
        )
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
    empty_board_baseline = matches_empty_board_rgb_baseline(
        square,
        area,
        bbox,
        fill_ratio,
        center_patch,
        patch_size,
        empty_board_baselines=rgb_baselines,
    )
    if empty_board_baseline is not None:
        result.update(
            {
                "detected": False,
                "reason": "matches_empty_board_rgb_baseline",
                "confidence": 0.0,
                "empty_board_baseline": empty_board_baseline,
            }
        )
    elif is_rank1_rgb_artifact(square, area, bbox, confidence, center_patch, patch_size):
        result.update(
            {
                "detected": False,
                "reason": "rank1_rgb_artifact",
                "confidence": 0.0,
            }
        )
    else:
        loaded_rgb_baselines = rgb_baselines if rgb_baselines is not None else load_empty_board_rgb_baselines()
        baseline = loaded_rgb_baselines.get(square) if isinstance(loaded_rgb_baselines, dict) else None
        if isinstance(baseline, dict):
            result["empty_board_rgb_baseline_candidate"] = {
                "baseline_area_px": baseline.get("area_px"),
                "baseline_bbox_patch_px": baseline.get("bbox_patch_px"),
                "baseline_fill_ratio": baseline.get("fill_ratio"),
                "baseline_center_patch_px": baseline.get("center_patch_px"),
            }
    return result


def detect_piece_in_square_depth_patch(
    depth_patch: np.ndarray,
    square: str,
    x0: float,
    y0: float,
    homography_board_to_image: np.ndarray,
    patch_size: int,
    empty_board_baselines: Optional[dict] = None,
) -> dict:
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
    empty_board_baseline = matches_empty_board_depth_baseline(
        square,
        area,
        bbox,
        median_raise,
        close_threshold_m,
        patch_size,
        empty_board_baselines=empty_board_baselines,
    )
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
    rank1_rescue = is_rank1_compact_depth_rescue(square, area, bbox, median_raise, close_threshold_m, max_raise, patch_size)
    if rank1_rescue:
        confidence = max(confidence, min(1.0, 0.18 + area / 700.0 + median_raise / 0.09))
    rank1_edge_artifact = is_rank1_edge_depth_artifact(square, area, bbox, confidence, center_patch, patch_size)
    if rank1_edge_artifact:
        return {
            "detected": False,
            "square": square,
            "method": "depth",
            "reason": "rank1_edge_depth_artifact",
            "confidence": 0.0,
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
        "method": "depth_rank2_compact_rescue" if rank2_rescue else ("depth_rank1_compact_rescue" if rank1_rescue else "depth"),
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


def square_file_rank(square: str) -> tuple[int, int]:
    return ord(square[0].lower()) - ord("a"), int(square[1])


def adjacent_square(a: str, b: str) -> bool:
    af, ar = square_file_rank(a)
    bf, br = square_file_rank(b)
    return abs(af - bf) + abs(ar - br) == 1


def same_file_adjacent_square(a: str, b: str) -> bool:
    af, ar = square_file_rank(a)
    bf, br = square_file_rank(b)
    return af == bf and abs(ar - br) == 1


def piece_detection_strength(piece: dict) -> float:
    confidence = float(piece.get("confidence", 0.0) or 0.0)
    area = float(piece.get("area_px", 0.0) or 0.0)
    return confidence * max(1.0, min(area, 1200.0) / 250.0)


def center_edge_margin_mm(square: str, piece: dict) -> float:
    center = piece.get("center_mm")
    if not center or len(center) < 2:
        return 999.0
    bounds = square_bounds_mm(square)
    xs = bounds[:, 0]
    ys = bounds[:, 1]
    x = float(center[0])
    y = float(center[1])
    return min(abs(x - float(xs.min())), abs(x - float(xs.max())), abs(y - float(ys.min())), abs(y - float(ys.max())))


def center_delta_mm(a: dict, b: dict) -> tuple[float, float]:
    ca = a.get("center_mm")
    cb = b.get("center_mm")
    if not ca or not cb or len(ca) < 2 or len(cb) < 2:
        return 999.0, 999.0
    return abs(float(ca[0]) - float(cb[0])), abs(float(ca[1]) - float(cb[1]))


def is_depth_detection_method(method: str) -> bool:
    return method.startswith("depth")


def vertical_depth_projection_duplicate_loser(a: str, pa: dict, b: str, pb: dict) -> Optional[str]:
    if not same_file_adjacent_square(a, b):
        return None
    if not is_depth_detection_method(str(pa.get("method", ""))) or not is_depth_detection_method(str(pb.get("method", ""))):
        return None
    _, ar = square_file_rank(a)
    _, br = square_file_rank(b)
    lower_square, lower_piece = (a, pa) if ar < br else (b, pb)
    upper_square, upper_piece = (b, pb) if ar < br else (a, pa)
    lower_center = lower_piece.get("center_mm")
    upper_center = upper_piece.get("center_mm")
    if not lower_center or not upper_center or len(lower_center) < 2 or len(upper_center) < 2:
        return None
    lower_bounds = square_bounds_mm(lower_square)
    upper_bounds = square_bounds_mm(upper_square)
    lower_y_to_shared_edge = abs(float(lower_bounds[:, 1].max()) - float(lower_center[1]))
    upper_y_to_shared_edge = abs(float(upper_center[1]) - float(upper_bounds[:, 1].min()))
    dx_mm, dy_mm = center_delta_mm(lower_piece, upper_piece)
    lower_area = float(lower_piece.get("area_px", 0.0) or 0.0)
    upper_area = float(upper_piece.get("area_px", 0.0) or 0.0)
    upper_bbox = upper_piece.get("bbox_patch_px") or []
    upper_w = float(upper_bbox[2]) if len(upper_bbox) >= 4 else 999.0
    upper_h = float(upper_bbox[3]) if len(upper_bbox) >= 4 else 999.0
    comparable_blob_size = lower_area > 0.0 and upper_area > 0.0 and min(lower_area, upper_area) >= max(lower_area, upper_area) * 0.55
    both_touch_shared_edge = lower_y_to_shared_edge <= 18.0 and upper_y_to_shared_edge <= 18.0
    same_column_blob = dx_mm <= 20.0 and dy_mm <= 58.0
    if both_touch_shared_edge and same_column_blob and comparable_blob_size:
        return upper_square
    upper_tiny_shard = upper_area <= 90.0 and (upper_h <= 8.0 or upper_w <= 12.0)
    lower_stable_blob = lower_area >= 180.0 and float(lower_piece.get("confidence", 0.0) or 0.0) >= 0.18
    near_lower_projection = dx_mm <= 22.0 and dy_mm <= 65.0 and lower_y_to_shared_edge <= 22.0 and upper_y_to_shared_edge <= 30.0
    if upper_tiny_shard and lower_stable_blob and near_lower_projection:
        return upper_square
    return None


def suppress_piece_detection(filtered: dict, square: str, reason: str) -> None:
    piece = filtered[square]
    piece["detected"] = False
    piece["confidence"] = 0
    piece["reason"] = reason
    piece["suppressed_detection"] = {
        key: piece.get(key)
        for key in ("method", "area_px", "bbox_patch_px", "center_mm", "center_px", "temporal_votes", "temporal_samples")
        if key in piece
    }


def suppress_adjacent_duplicate_detections(piece_results: dict) -> tuple[dict, list[str], dict]:
    filtered = {square: dict(piece) for square, piece in piece_results.items()}
    detected = [square for square, piece in filtered.items() if piece.get("detected")]
    suppress_reasons = {}
    for square in detected:
        piece = filtered[square]
        samples = int(piece.get("temporal_samples", 0) or 0)
        votes = int(piece.get("temporal_votes", 0) or 0)
        confidence = float(piece.get("confidence", 0.0) or 0.0)
        if samples >= 5 and votes < 3 and confidence < 0.75:
            suppress_reasons[square] = "low_temporal_votes_full_board"
    for i, a in enumerate(detected):
        if a in suppress_reasons:
            continue
        for b in detected[i + 1 :]:
            if b in suppress_reasons or not adjacent_square(a, b):
                continue
            pa = filtered[a]
            pb = filtered[b]
            method_a = str(pa.get("method", ""))
            method_b = str(pb.get("method", ""))
            if same_file_adjacent_square(a, b):
                dx_mm, dy_mm = center_delta_mm(pa, pb)
                votes_a = int(pa.get("temporal_votes", 0) or 0)
                votes_b = int(pb.get("temporal_votes", 0) or 0)
                if dx_mm <= 18.0 and dy_mm <= 50.0:
                    if is_depth_detection_method(method_a) and method_b == "rgb" and votes_a >= 3:
                        suppress_reasons[b] = "adjacent_rgb_duplicate_of_depth_piece"
                        continue
                    if is_depth_detection_method(method_b) and method_a == "rgb" and votes_b >= 3:
                        suppress_reasons[a] = "adjacent_rgb_duplicate_of_depth_piece"
                        break
                projection_loser = vertical_depth_projection_duplicate_loser(a, pa, b, pb)
                if projection_loser is not None:
                    suppress_reasons[projection_loser] = "adjacent_depth_projection_artifact"
                    if projection_loser == a:
                        break
                    continue
            margin_a = center_edge_margin_mm(a, pa)
            margin_b = center_edge_margin_mm(b, pb)
            strength_a = piece_detection_strength(pa)
            strength_b = piece_detection_strength(pb)
            loser = None
            if margin_a <= 14.0 and strength_b >= strength_a * 1.25:
                loser = a
            elif margin_b <= 14.0 and strength_a >= strength_b * 1.25:
                loser = b
            if loser is not None:
                suppress_reasons[loser] = "adjacent_square_duplicate_artifact"
    for square, reason in suppress_reasons.items():
        suppress_piece_detection(filtered, square, reason)
    detected_squares = [square for square, piece in filtered.items() if piece.get("detected")]
    identified_pieces = {}
    for square in detected_squares:
        piece = filtered[square]
        identity = {k: piece.get(k) for k in ("piece_id", "piece_type", "color", "identity_method", "identity_confidence")}
        if not identity.get("piece_id"):
            identity = identify_piece_for_square(square)
            piece.update(identity)
        identified_pieces[square] = {
            "square": square,
            "piece_id": piece["piece_id"],
            "piece_type": piece["piece_type"],
            "color": piece["color"],
            "identity_method": piece["identity_method"],
            "identity_confidence": piece["identity_confidence"],
            "detection_method": piece.get("method", ""),
            "detection_confidence": piece.get("confidence", 0.0),
            "center_mm": piece.get("center_mm"),
            "center_px": piece.get("center_px"),
        }
    return filtered, detected_squares, identified_pieces


def draw_squares_overlay_image(
    image,
    calibration_path: str,
    squares: str,
    image_path: str = "",
    detect_pieces: bool = False,
    depth_image=None,
    piece_results_override: Optional[dict] = None,
    empty_board_baselines: Optional[dict] = None,
    empty_board_depth_baselines: Optional[dict] = None,
    empty_board_rgb_baselines: Optional[dict] = None,
    identity_overrides: Optional[dict] = None,
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
    identified_pieces = {}
    title_parts = []
    compact_detection_overlay = len(square_list) > 16
    identity_overrides = identity_overrides or {}
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
                piece = detect_piece_in_square_image(
                    image,
                    h,
                    square,
                    depth_image=depth_image,
                    empty_board_baselines=empty_board_baselines,
                    empty_board_depth_baselines=empty_board_depth_baselines,
                    empty_board_rgb_baselines=empty_board_rgb_baselines,
                )
            if piece.get("detected"):
                identity = dict(identity_overrides.get(square) or identify_piece_for_square(square))
                piece.update(identity)
                identified_pieces[square] = {
                    "square": square,
                    "piece_id": piece["piece_id"],
                    "piece_type": piece["piece_type"],
                    "color": piece["color"],
                    "identity_method": piece["identity_method"],
                    "identity_confidence": piece["identity_confidence"],
                    "detection_method": piece.get("method", ""),
                    "detection_confidence": piece.get("confidence", 0.0),
                    "center_mm": piece.get("center_mm"),
                    "center_px": piece.get("center_px"),
                }
            piece_results[square] = piece
            if piece.get("detected") and not compact_detection_overlay:
                detected_squares.append(square)
                draw_polyline(image, roi_px, (0, 255, 0), 2 if compact_detection_overlay else 3)
                piece_xy = tuple(np.round(piece["center_px"]).astype(int))
                cv2.circle(image, piece_xy, 7, (0, 255, 0), -1, cv2.LINE_AA)
                cv2.circle(image, piece_xy, 11, (0, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(
                    image,
                    str(piece.get("piece_id", square)) if compact_detection_overlay else str(piece.get("piece_id", "piece")),
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
    if detect_pieces and compact_detection_overlay:
        piece_results, detected_squares, identified_pieces = suppress_adjacent_duplicate_detections(piece_results)
        if identity_overrides:
            yolo_squares = set(identity_overrides.keys())
            for square in list(detected_squares):
                if square in yolo_squares:
                    continue
                if any(adjacent_square(square, yolo_square) for yolo_square in yolo_squares):
                    suppress_piece_detection(piece_results, square, "adjacent_to_yolo_identity_artifact")
                    identified_pieces.pop(square, None)
            detected_squares = [square for square in detected_squares if piece_results.get(square, {}).get("detected")]
        for square in detected_squares:
            piece = piece_results[square]
            identity = identity_overrides.get(square)
            if isinstance(identity, dict):
                piece.update(
                    {
                        "piece_id": identity.get("piece_id", piece.get("piece_id", square)),
                        "piece_type": identity.get("piece_type", piece.get("piece_type", "unknown")),
                        "color": identity.get("color", piece.get("color", "unknown")),
                        "identity_method": identity.get("identity_method", piece.get("identity_method", "unknown")),
                        "identity_confidence": identity.get("identity_confidence", piece.get("identity_confidence", 0.0)),
                    }
                )
                identified_pieces[square] = {
                    "square": square,
                    "piece_id": piece["piece_id"],
                    "piece_type": piece["piece_type"],
                    "color": piece["color"],
                    "identity_method": piece["identity_method"],
                    "identity_confidence": piece["identity_confidence"],
                    "detection_method": piece.get("method", ""),
                    "detection_confidence": piece.get("confidence", 0.0),
                    "center_mm": piece.get("center_mm"),
                    "center_px": piece.get("center_px"),
                }
            roi_px = np.asarray(square_results[square]["roi_px"], dtype=np.float32)
            draw_polyline(image, roi_px, (0, 255, 0), 2)
            piece_xy = tuple(np.round(piece["center_px"]).astype(int))
            cv2.circle(image, piece_xy, 7, (0, 255, 0), -1, cv2.LINE_AA)
            cv2.circle(image, piece_xy, 11, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(
                image,
                str(piece.get("piece_id", square)),
                (piece_xy[0] + 9, piece_xy[1] + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
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
        "identified_pieces": identified_pieces,
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
    empty_board_baselines: Optional[dict] = None,
    empty_board_depth_baselines: Optional[dict] = None,
    empty_board_rgb_baselines: Optional[dict] = None,
    identity_overrides: Optional[dict] = None,
) -> dict:
    result = draw_squares_overlay_image(
        image,
        calibration_path,
        squares,
        image_path=image_path,
        detect_pieces=detect_pieces,
        depth_image=depth_image,
        piece_results_override=piece_results_override,
        empty_board_baselines=empty_board_baselines,
        empty_board_depth_baselines=empty_board_depth_baselines,
        empty_board_rgb_baselines=empty_board_rgb_baselines,
        identity_overrides=identity_overrides,
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
