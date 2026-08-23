#!/usr/bin/env python3
"""Create and label the HiveRobot chess-piece YOLO dataset."""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path

import cv2
import numpy as np

from chessboard_vision_v2_7 import (
    CHESS_PIECE_YOLO_CLASSES,
    DEFAULT_CALIBRATION_PATH,
    board_to_image_points,
    load_calibration,
    normalize_square,
    square_bounds_mm,
)


DEFAULT_DATASET_DIR = Path("datasets") / "chess_pieces_yolo"


def parse_placements(text: str) -> dict[str, str]:
    placements = {}
    for item in text.replace("\n", ",").split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            raise ValueError(f"expected square:class, got {item!r}")
        square, piece_class = [part.strip().lower() for part in item.split(":", 1)]
        square = normalize_square(square)
        if piece_class not in CHESS_PIECE_YOLO_CLASSES:
            raise ValueError(f"unknown piece class {piece_class!r}; expected one of {CHESS_PIECE_YOLO_CLASSES}")
        placements[square] = piece_class
    if not placements:
        raise ValueError("expected at least one placement, e.g. 'a4:white_pawn,b4:black_king'")
    return placements


def write_data_yaml(dataset_dir: Path) -> Path:
    path = dataset_dir / "data.yaml"
    lines = [
        f"path: {dataset_dir.as_posix()}",
        "train: images/train",
        "val: images/val",
        "test: images/test",
        "names:",
    ]
    for index, name in enumerate(CHESS_PIECE_YOLO_CLASSES):
        lines.append(f"  {index}: {name}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def init_dataset(dataset_dir: Path) -> None:
    for split in ("train", "val", "test"):
        (dataset_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (dataset_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
    write_data_yaml(dataset_dir)


def square_yolo_box(square: str, homography: np.ndarray, image_shape: tuple[int, int, int], shrink: float) -> tuple[float, float, float, float]:
    roi_px = board_to_image_points(square_bounds_mm(square), homography)
    center = roi_px.mean(axis=0)
    shrunk = center + (roi_px - center) * float(shrink)
    x_min = float(np.min(shrunk[:, 0]))
    x_max = float(np.max(shrunk[:, 0]))
    y_min = float(np.min(shrunk[:, 1]))
    y_max = float(np.max(shrunk[:, 1]))
    height, width = image_shape[:2]
    x_min = max(0.0, min(float(width - 1), x_min))
    x_max = max(0.0, min(float(width - 1), x_max))
    y_min = max(0.0, min(float(height - 1), y_min))
    y_max = max(0.0, min(float(height - 1), y_max))
    cx = ((x_min + x_max) * 0.5) / width
    cy = ((y_min + y_max) * 0.5) / height
    bw = (x_max - x_min) / width
    bh = (y_max - y_min) / height
    return cx, cy, bw, bh


def add_labeled_image(
    image_path: Path,
    calibration_path: Path,
    dataset_dir: Path,
    split: str,
    placements: dict[str, str],
    name: str,
    shrink: float,
) -> None:
    if split not in ("train", "val", "test"):
        raise ValueError("split must be train, val, or test")
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"failed to read image: {image_path}")
    calibration = load_calibration(str(calibration_path))
    h = calibration["homography_board_to_image"]
    image_name = name + image_path.suffix.lower()
    label_name = name + ".txt"
    dst_image = dataset_dir / "images" / split / image_name
    dst_label = dataset_dir / "labels" / split / label_name
    dst_image.parent.mkdir(parents=True, exist_ok=True)
    dst_label.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(image_path, dst_image)
    rows = []
    for square, piece_class in sorted(placements.items()):
        class_id = CHESS_PIECE_YOLO_CLASSES.index(piece_class)
        cx, cy, bw, bh = square_yolo_box(square, h, image.shape, shrink)
        rows.append(f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    dst_label.write_text("\n".join(rows) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="HiveRobot chess-piece YOLO dataset helper")
    sub = parser.add_subparsers(dest="cmd", required=True)

    init = sub.add_parser("init")
    init.add_argument("--dataset-dir", default=str(DEFAULT_DATASET_DIR))

    add = sub.add_parser("add-image")
    add.add_argument("--image", required=True)
    add.add_argument("--calibration", default=DEFAULT_CALIBRATION_PATH)
    add.add_argument("--dataset-dir", default=str(DEFAULT_DATASET_DIR))
    add.add_argument("--split", default="train", choices=["train", "val", "test"])
    add.add_argument("--placements", required=True, help="comma separated square:class labels, e.g. a4:white_pawn,b4:black_king")
    add.add_argument("--name", default="", help="dataset item basename; defaults to image stem")
    add.add_argument("--shrink", type=float, default=0.72, help="shrink each square ROI before writing YOLO bbox")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    dataset_dir = Path(args.dataset_dir)
    if args.cmd == "init":
        init_dataset(dataset_dir)
        print(write_data_yaml(dataset_dir))
    elif args.cmd == "add-image":
        init_dataset(dataset_dir)
        image_path = Path(args.image)
        name = args.name or image_path.stem
        add_labeled_image(
            image_path,
            Path(args.calibration),
            dataset_dir,
            args.split,
            parse_placements(args.placements),
            name,
            args.shrink,
        )
        print(dataset_dir / "data.yaml")
    else:
        raise RuntimeError(f"unknown command: {args.cmd}")


if __name__ == "__main__":
    main()
