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


def map_detections_to_squares(detections: list[dict], calibration_path: str, allowed_squares: list[str]) -> dict:
    calibration = load_calibration(calibration_path)
    h = calibration["homography_board_to_image"]
    allowed = set(allowed_squares)
    mapped = {}
    for detection in detections:
        board_xy = image_to_board_points(np.array([detection["center_px"]], dtype=np.float32), h)[0]
        square = board_point_to_square(float(board_xy[0]), float(board_xy[1]))
        if square is None or square not in allowed:
            continue
        current = mapped.get(square)
        if current is not None and float(current.get("confidence", 0.0)) >= float(detection["confidence"]):
            continue
        item = dict(detection)
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
    piece_class_results = map_detections_to_squares(detections, args.calibration, allowed_squares)
    payload = {
        "ok": True,
        "model": args.model,
        "image": args.image,
        "squares": allowed_squares,
        "detections": detections,
        "piece_class_results": piece_class_results,
        "placement_plan": assign_opening_targets(piece_class_results, occupied_targets=occupied_targets),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
