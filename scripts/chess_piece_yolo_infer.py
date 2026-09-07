#!/usr/bin/env python3
"""Run a trained chess-piece YOLO model and build an opening placement plan."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from chessboard_vision_v2_7 import (
    CHESS_PIECE_YOLO_CLASSES,
    DEFAULT_CALIBRATION_PATH,
    assign_opening_targets,
    board_point_to_square,
    image_to_board_points,
    load_calibration,
    draw_board_grid,
    parse_square_list,
)


def run_yolo(model_path: str, image_path: str, imgsz: int, conf: float) -> list[dict]:
    try:
        from ultralytics import YOLO
    except Exception as exc:
        raise RuntimeError(f"ultralytics is not available in this Python environment: {exc}") from exc
    model = YOLO(model_path)
    result = model.predict(source=image_path, imgsz=imgsz, conf=conf, verbose=False)[0]
    names = result.names
    detections = []
    if result.boxes is None:
        return detections
    for box in result.boxes:
        xyxy = box.xyxy[0].detach().cpu().numpy().astype(float).tolist()
        class_id = int(box.cls[0].detach().cpu().item())
        confidence = float(box.conf[0].detach().cpu().item())
        piece_class = str(names.get(class_id, CHESS_PIECE_YOLO_CLASSES[class_id] if class_id < len(CHESS_PIECE_YOLO_CLASSES) else class_id))
        detections.append(
            {
                "piece_class": piece_class,
                "class_id": class_id,
                "confidence": confidence,
                "bbox_xyxy": xyxy,
                "center_px": [(xyxy[0] + xyxy[2]) * 0.5, (xyxy[1] + xyxy[3]) * 0.5],
            }
        )
    return detections


def save_prediction_image(image_path: str, detections: list[dict], calibration_path: str | None = None) -> str:
    """Draw actual model boxes on the exact saved input, leaving it unchanged."""
    source = Path(image_path)
    frame = cv2.imread(str(source))
    if frame is None:
        raise RuntimeError(f"failed to read prediction input: {source}")
    height, width = frame.shape[:2]
    if calibration_path is not None:
        draw_board_grid(frame, load_calibration(calibration_path)["homography_board_to_image"])
    for detection in detections:
        box = np.asarray(detection["bbox_xyxy"], dtype=float)
        if box.shape != (4,) or not np.isfinite(box).all() or box[2] <= box[0] or box[3] <= box[1]:
            continue
        x1, y1, x2, y2 = np.rint(box).astype(int)
        x1, x2 = np.clip([x1, x2], 0, width - 1)
        y1, y2 = np.clip([y1, y2], 0, height - 1)
        color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{detection['piece_class']} {detection['confidence']:.1%}"
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)
        tx = max(0, min(int(x1), width - tw - 4))
        ty = int(y1) - 6 if y1 >= th + 8 else min(height - baseline - 1, int(y1) + th + 5)
        cv2.rectangle(frame, (tx, max(0, ty-th-3)), (min(width-1, tx+tw+3), ty+baseline), (0, 0, 0), -1)
        cv2.putText(frame, label, (tx+1, ty), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    if not detections:
        cv2.putText(frame, "No YOLO detections", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0,255,0), 1, cv2.LINE_AA)
    target = source.with_name(source.stem + "_pred.jpg")
    if not cv2.imwrite(str(target), frame):
        raise RuntimeError(f"failed to save prediction image: {target}")
    return str(target)


def map_detections_to_squares(detections: list[dict], calibration_path: str, allowed_squares: list[str]) -> dict:
    calibration = load_calibration(calibration_path)
    h = calibration["homography_board_to_image"]
    allowed = set(allowed_squares)
    mapped = {}
    for detection in detections:
        # The center of a tall piece projects into the square behind its base.
        # Use a point near the bottom of the full-piece box for square assignment.
        # This assumes the deployed upright camera view (crowns above bases).
        x1, y1, x2, y2 = detection["bbox_xyxy"]
        base_px = [(x1 + x2) * 0.5, y2 - 0.1 * (y2 - y1)]
        if not np.isfinite([x1, y1, x2, y2]).all() or x2 <= x1 or y2 <= y1:
            continue
        board_xy = image_to_board_points(np.array([base_px], dtype=np.float32), h)[0]
        square = board_point_to_square(float(board_xy[0]), float(board_xy[1]))
        if square is None or square not in allowed:
            continue
        current = mapped.get(square)
        if current is not None and float(current.get("confidence", 0.0)) >= float(detection["confidence"]):
            continue
        item = dict(detection)
        item["bbox_center_px"] = detection["center_px"]
        item["center_px"] = base_px
        item["square_anchor"] = "bbox_base_90_percent"
        item["square"] = square
        item["center_mm"] = board_xy.astype(float).tolist()
        mapped[square] = item
    return mapped


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run HiveRobot chess-piece YOLO inference")
    parser.add_argument("--model", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION_PATH)
    parser.add_argument("--squares", default="a4,b4,c4,d4,e4,f4,g4,h4")
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--occupied-targets", default="", help="opening squares already occupied, e.g. a2,e1")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if cv2.imread(str(Path(args.image))) is None:
        raise RuntimeError(f"failed to read image: {args.image}")
    allowed_squares = parse_square_list(args.squares)
    occupied_targets = [] if not args.occupied_targets.strip() else parse_square_list(args.occupied_targets)
    detections = run_yolo(args.model, args.image, args.imgsz, args.conf)
    prediction_path = save_prediction_image(args.image, detections, args.calibration)
    piece_class_results = map_detections_to_squares(detections, args.calibration, allowed_squares)
    payload = {
        "ok": True,
        "model": args.model,
        "image": args.image,
        "prediction_image_path": prediction_path,
        "squares": allowed_squares,
        "detections": detections,
        "piece_class_results": piece_class_results,
        "placement_plan": assign_opening_targets(piece_class_results, occupied_targets=occupied_targets),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
