#!/usr/bin/env python3
"""Web verifier for chessboard vision v2.7 phase 1.

This is intentionally separate from the v2.6 arm web controller.
It does not expose arm motion commands.
"""

from __future__ import annotations

import argparse
import html
import json
import os
import shutil
import subprocess
import tempfile
import threading
import time
import uuid
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from chessboard_vision_v2_7 import (
    BOARD_SIZE_MM,
    DEFAULT_CALIBRATION_PATH,
    annotate_squares_image,
    auto_locate_board_corners,
    draw_squares_overlay_image,
    parse_point_list,
    parse_square_list,
    save_calibration_from_points,
    square_bounds_mm,
)
from chess_piece_yolo_dataset import add_labeled_image, init_dataset, parse_placements, write_data_yaml


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CORNERS = "100,428 595,418 520,52 165,58"
DEFAULT_SQUARES = "all"
DEFAULT_OUTPUT_DIR = os.path.join(tempfile.gettempdir(), "hive_robot_chessboard_vision_v2_7")
WEB_VERSION = "v2.7"
DEFAULT_YOLO_DATASET_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "datasets", "chess_pieces_yolo")
DEFAULT_YOLO_DOCKER_IMAGE = "ultralytics/ultralytics:latest-jetson-jetpack5"


@dataclass
class CameraConfig:
    live_camera: bool
    rgb_topic: str
    depth_topic: str
    jpeg_quality: int


class LiveCameraState:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.rgb = None
        self.rgb_stamp = 0.0
        self.rgb_seq = 0
        self.depth = None
        self.depth_stamp = 0.0
        self.depth_seq = 0

    def set_rgb(self, frame) -> None:
        with self.condition:
            self.rgb = frame.copy()
            self.rgb_stamp = time.time()
            self.rgb_seq += 1
            self.condition.notify_all()

    def set_depth(self, frame) -> None:
        with self.condition:
            self.depth = frame.copy()
            self.depth_stamp = time.time()
            self.depth_seq += 1
            self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            rgb = None if self.rgb is None else self.rgb.copy()
            depth = None if self.depth is None else self.depth.copy()
            return rgb, self.rgb_stamp, self.rgb_seq, depth, self.depth_stamp, self.depth_seq

    def wait_for_newer(self, last_seq: int, timeout_s: float = 1.0):
        deadline = time.time() + timeout_s
        with self.condition:
            while self.rgb_seq <= last_seq:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self.condition.wait(remaining)
            return None if self.rgb is None else self.rgb.copy(), self.rgb_stamp, self.rgb_seq

    def status(self) -> dict:
        with self.condition:
            return {
                "has_frame": self.rgb is not None,
                "rgb_seq": self.rgb_seq,
                "rgb_stamp": self.rgb_stamp,
                "rgb_age_s": None if self.rgb is None else round(time.time() - self.rgb_stamp, 3),
                "shape": None if self.rgb is None else list(self.rgb.shape),
                "has_depth": self.depth is not None,
                "depth_seq": self.depth_seq,
                "depth_stamp": self.depth_stamp,
                "depth_age_s": None if self.depth is None else round(time.time() - self.depth_stamp, 3),
                "depth_shape": None if self.depth is None else list(self.depth.shape),
            }


def encode_jpeg(frame, quality: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buf.tobytes()


def draw_live_status(frame, seq: int, stamp: float) -> None:
    age = time.time() - stamp if stamp else 0.0
    cv2.putText(
        frame,
        f"live seq={seq} age={age:.2f}s",
        (10, frame.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


def convert_ros_image(msg, desired_encoding: str = "bgr8"):
    try:
        from cv_bridge import CvBridge

        return CvBridge().imgmsg_to_cv2(msg, desired_encoding=desired_encoding)
    except Exception:
        encoding = (msg.encoding or "").lower()
        channels_by_encoding = {
            "bgr8": 3,
            "rgb8": 3,
            "mono8": 1,
            "bgra8": 4,
            "rgba8": 4,
        }
        dtype_by_encoding = {
            "bgr8": np.uint8,
            "rgb8": np.uint8,
            "mono8": np.uint8,
            "bgra8": np.uint8,
            "rgba8": np.uint8,
        }
        if encoding not in channels_by_encoding:
            raise ValueError(f"unsupported ROS image encoding: {msg.encoding}")
        dtype = dtype_by_encoding[encoding]
        channels = channels_by_encoding[encoding]
        arr = np.frombuffer(msg.data, dtype=dtype)
        if channels == 1:
            frame = arr.reshape(msg.height, msg.step // np.dtype(dtype).itemsize)[:, : msg.width]
        else:
            frame = arr.reshape(msg.height, msg.step // np.dtype(dtype).itemsize // channels, channels)[:, : msg.width, :]
        if encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        elif encoding == "bgra8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif encoding == "mono8":
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame.copy()


def convert_ros_depth_image(msg):
    try:
        from cv_bridge import CvBridge

        return CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception:
        encoding = (msg.encoding or "").lower()
        dtype_by_encoding = {
            "16uc1": np.uint16,
            "32fc1": np.float32,
            "mono16": np.uint16,
        }
        if encoding not in dtype_by_encoding:
            raise ValueError(f"unsupported ROS depth encoding: {msg.encoding}")
        dtype = dtype_by_encoding[encoding]
        arr = np.frombuffer(msg.data, dtype=dtype)
        width_step = msg.step // np.dtype(dtype).itemsize
        return arr.reshape(msg.height, width_step)[:, : msg.width].copy()


def parse_square_corner_calibration(text: str) -> tuple[np.ndarray, np.ndarray, list[str]]:
    board_points = []
    image_points = []
    labels = []
    for raw_line in text.replace(";", "\n").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            raise ValueError(f"square corner line must look like 'a2: x,y x,y x,y x,y': {raw_line!r}")
        square, points_text = line.split(":", 1)
        square = square.strip().lower()
        points = parse_point_list(points_text)
        square_board_points = square_bounds_mm(square)
        board_points.extend(square_board_points.tolist())
        image_points.extend(points.tolist())
        labels.append(square)
    if not board_points:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32), []
    return (
        np.array(board_points, dtype=np.float32),
        np.array(image_points, dtype=np.float32),
        labels,
    )


def save_live_calibration(
    calibration_path: str,
    frame,
    seed_corners: np.ndarray,
    auto_locate: bool,
    extra_board_points: Optional[np.ndarray] = None,
    extra_image_points: Optional[np.ndarray] = None,
    extra_labels: Optional[list] = None,
) -> tuple[np.ndarray, dict]:
    corner_image_points = np.asarray(seed_corners, dtype=np.float32).reshape(4, 2)
    auto_location = {"ok": False, "reason": "disabled", "corners_px": corner_image_points.astype(float).tolist()}
    if auto_locate:
        corner_image_points, auto_location = auto_locate_board_corners(frame, corner_image_points)
    corner_board_points = np.array(
        [[0.0, 0.0], [BOARD_SIZE_MM, 0.0], [BOARD_SIZE_MM, BOARD_SIZE_MM], [0.0, BOARD_SIZE_MM]],
        dtype=np.float32,
    )
    extra_board = np.empty((0, 2), dtype=np.float32) if extra_board_points is None else np.asarray(extra_board_points, dtype=np.float32).reshape(-1, 2)
    extra_image = np.empty((0, 2), dtype=np.float32) if extra_image_points is None else np.asarray(extra_image_points, dtype=np.float32).reshape(-1, 2)
    labels = [] if extra_labels is None else extra_labels
    board_points = np.vstack([corner_board_points, extra_board])
    image_points = np.vstack([corner_image_points, extra_image])
    save_calibration_from_points(
        calibration_path,
        "live",
        board_points,
        image_points,
        corner_order="a1,h1,h8,a8" + ("," + ",".join(labels) if labels else ""),
        note="web-control-v2.7 live calibration",
    )
    return corner_image_points, auto_location


def should_temporal_merge(squares: str, detect_pieces: bool) -> bool:
    if not detect_pieces:
        return False
    try:
        return len(parse_square_list(squares)) > 16
    except Exception:
        return False


def choose_best_piece_result(results: list) -> dict:
    detected = [item for item in results if item and item.get("detected")]
    if detected:
        best = max(
            detected,
            key=lambda item: (
                float(item.get("confidence", 0.0)),
                float(item.get("area_px", 0.0)),
                float(item.get("median_raise_m", 0.0)),
            ),
        ).copy()
        best["temporal_votes"] = len(detected)
        best["temporal_samples"] = len(results)
        return best
    latest = (results[-1] if results else {"detected": False, "confidence": 0.0}).copy()
    latest["temporal_votes"] = 0
    latest["temporal_samples"] = len(results)
    return latest


def merge_temporal_piece_results(sample_results: list) -> dict:
    if not sample_results:
        return {}
    square_list = sample_results[-1].get("squares", [])
    merged = {}
    for square in square_list:
        square_samples = [
            sample.get("piece_results", {}).get(square)
            for sample in sample_results
            if square in sample.get("piece_results", {})
        ]
        merged[square] = choose_best_piece_result(square_samples)
    return merged


def read_text_tail(path: str, max_bytes: int = 12000) -> str:
    if not os.path.exists(path):
        return ""
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        size = f.tell()
        f.seek(max(0, size - max_bytes), os.SEEK_SET)
        return f.read().decode("utf-8", errors="replace")


def repo_root_for_script() -> str:
    return os.path.dirname(SCRIPT_DIR)


def docker_repo_mount(repo_root: str) -> tuple[str, str]:
    marker = os.path.join("hive_robot", "DM_Control_Python")
    normalized = repo_root.replace("\\", "/")
    if normalized.endswith(marker):
        host_root = repo_root[: -len("DM_Control_Python")].rstrip("\\/")
        return host_root, "/workspace/hive_robot/DM_Control_Python"
    return repo_root, "/workspace/hive_robot/DM_Control_Python"


def build_yolo_train_command(state: "VisionState") -> list[str]:
    repo_root = repo_root_for_script()
    data_yaml = os.path.relpath(os.path.join(state.yolo_dataset_dir, "data.yaml"), repo_root).replace("\\", "/")
    train_args = [
        "detect",
        "train",
        "model=yolo11n.pt",
        f"data={data_yaml}",
        "imgsz=960",
        "epochs=120",
        "batch=8",
        "project=runs/chess_piece_yolo",
        "name=yolo11n_rank4",
    ]
    yolo_exe = shutil.which("yolo")
    if yolo_exe:
        return [yolo_exe, *train_args]
    host_mount, container_repo = docker_repo_mount(repo_root)
    inner = "cd " + container_repo + " && yolo " + " ".join(train_args)
    return [
        "sudo",
        "docker",
        "run",
        "--rm",
        "--runtime",
        "nvidia",
        "--network",
        "host",
        "--ipc",
        "host",
        "-v",
        f"{host_mount}:/workspace/hive_robot",
        state.yolo_docker_image,
        "bash",
        "-lc",
        inner,
    ]


HTML_PAGE = """<!doctype html>
<html lang="en" translate="no">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="google" content="notranslate">
  <title>Chessboard Vision v2.7</title>
  <style>
    :root { color-scheme: dark; }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: #101316;
      color: #eef2f5;
      font-family: Arial, Helvetica, sans-serif;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 10px 14px;
      background: #171b20;
      border-bottom: 1px solid #2b333c;
    }
    h1 { margin: 0; font-size: 18px; }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 390px;
      gap: 12px;
      padding: 12px;
    }
    .viewer {
      min-height: 0;
      background: #050607;
      border: 1px solid #2c343d;
      border-radius: 8px;
      overflow: hidden;
    }
    img {
      display: block;
      width: 100%;
      max-height: calc(100vh - 78px);
      object-fit: contain;
      background: #000;
    }
    aside { display: grid; gap: 10px; align-content: start; }
    .panel {
      padding: 12px;
      background: #181d22;
      border: 1px solid #303944;
      border-radius: 8px;
    }
    label {
      display: grid;
      gap: 5px;
      margin-bottom: 10px;
      color: #cbd5df;
      font-size: 12px;
    }
    input {
      min-height: 38px;
      width: 100%;
      padding: 8px 9px;
      border: 1px solid #485565;
      border-radius: 6px;
      background: #10151a;
      color: #eef2f5;
      font-size: 13px;
    }
    textarea {
      min-height: 92px;
      width: 100%;
      padding: 8px 9px;
      border: 1px solid #485565;
      border-radius: 6px;
      background: #10151a;
      color: #eef2f5;
      font-size: 13px;
      resize: vertical;
    }
    button {
      min-height: 40px;
      width: 100%;
      border: 1px solid #5a6b7d;
      border-radius: 6px;
      background: #2f77bd;
      color: white;
      font-size: 14px;
      font-weight: 700;
      cursor: pointer;
    }
    pre {
      min-height: 160px;
      max-height: 360px;
      overflow: auto;
      padding: 9px;
      background: #0b0f13;
      border: 1px solid #2c343d;
      border-radius: 6px;
      color: #cde6ff;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      font-size: 12px;
    }
    .hint { color: #98a6b3; font-size: 12px; line-height: 1.45; }
  </style>
</head>
<body>
  <header>
    <h1>Chessboard Vision v2.7 Phase 1</h1>
    <div class="hint">Geometry only. No arm commands.</div>
  </header>
  <main>
    <section class="viewer">
      <img id="overlay" src="__INITIAL_VIEWER_SRC__" alt="chessboard vision viewer">
    </section>
    <aside>
      <section class="panel">
        <label>Inner board black-line corners in order a1,h1,h8,a8
          <input id="corners" value="__CORNERS__">
        </label>
        <label><input id="autoLocate" type="checkbox"> Auto locate board from live frame</label>
        <label><input id="detectPieces" type="checkbox" checked> Detect pieces in selected squares</label>
        <label>Squares
          <input id="squares" value="__SQUARES__">
        </label>
        <label>Square corner calibration
          <textarea id="squareCornerInput" placeholder="a2: x,y x,y x,y x,y&#10;h2: x,y x,y x,y x,y"></textarea>
        </label>
        <button onclick="showInputFrame()">Show Input Frame</button>
        <button onclick="inspectSquare()">Inspect Square</button>
        <button onclick="detectWholeBoard()">Detect Whole Board</button>
        <button onclick="useShownSquareCorners()">Use Shown Square Corners</button>
        <p class="hint">
          The a-file origin is the black line to the right of the rank numbers, not the outer decorative border.
          Use comma or space separated squares. Example square input accepts both a2 and 2a.
        </p>
      </section>
      <section class="panel">
        <pre id="result">Ready.</pre>
      </section>
      <section class="panel">
        <label>YOLO rank-4 labels
          <textarea id="yoloPlacements" placeholder="a4:white_pawn&#10;b4:black_king&#10;c4:white_rook"></textarea>
        </label>
        <label>YOLO split
          <input id="yoloSplit" value="train">
        </label>
        <button onclick="saveYoloSample()">Save YOLO Sample</button>
        <button onclick="startYoloTrain()">Start YOLO Train</button>
        <button onclick="refreshYoloStatus()">YOLO Train Status</button>
        <p class="hint">
          Place pieces on rank 4, enter square:class labels, save samples, then train.
          Classes include white_pawn, white_rook, white_knight, white_bishop, white_queen, white_king, and black_*.
        </p>
        <pre id="yoloStatus">YOLO training idle.</pre>
      </section>
      <section class="panel">
        <pre id="liveStatus">Live status pending.</pre>
      </section>
      <section class="panel">
        <pre id="squareCornerOutput">Square corners will appear after Inspect.</pre>
      </section>
    </aside>
  </main>
  <script>
    let lastInspectData = null;

    function showInputFrame() {
      document.getElementById('overlay').src = '/live-rgb.mjpg?ts=' + Date.now();
    }

    function useShownSquareCorners() {
      if (!lastInspectData || !lastInspectData.square_results) {
        return;
      }
      const lines = [];
      for (const [square, info] of Object.entries(lastInspectData.square_results)) {
        const coords = info.roi_px.map((pt) => pt[0].toFixed(1) + ',' + pt[1].toFixed(1));
        lines.push(square + ': ' + coords.join(' '));
      }
      document.getElementById('squareCornerInput').value = lines.join('\\n');
    }

    function detectWholeBoard() {
      document.getElementById('squares').value = 'all';
      inspectSquare();
    }

    async function inspectSquare() {
      const result = document.getElementById('result');
      const squareCornerOutput = document.getElementById('squareCornerOutput');
      result.textContent = 'Running...';
      squareCornerOutput.textContent = 'Running...';
      const params = new URLSearchParams({
        corners: document.getElementById('corners').value,
        auto_locate: document.getElementById('autoLocate').checked ? '1' : '0',
        detect_pieces: document.getElementById('detectPieces').checked ? '1' : '0',
        squares: document.getElementById('squares').value,
        square_corners: document.getElementById('squareCornerInput').value
      });
      const res = await fetch('/api/inspect?' + params.toString());
      const data = await res.json();
      lastInspectData = data.ok ? data : null;
      if (data.ok && data.piece_results) {
        const detected = data.detected_squares || [];
        const identities = data.identified_pieces || {};
        const identityLines = Object.keys(identities).sort().map((square) => {
          const info = identities[square];
          return square + ':' + info.piece_id;
        });
        result.textContent =
          'detected_count=' + detected.length + '\\n' +
          'detected_squares=' + detected.join(',') + '\\n\\n' +
          'identified_pieces=' + (identityLines.join(',') || 'none') + '\\n\\n' +
          JSON.stringify(data, null, 2);
      } else {
        result.textContent = JSON.stringify(data, null, 2);
      }
      if (data.ok && data.square_results) {
        const entries = Object.entries(data.square_results);
        const lines = [];
        if (entries.length > 16) {
          lines.push('Whole-board mode: ' + entries.length + ' squares inspected.');
          lines.push('Detected: ' + ((data.detected_squares || []).join(',') || 'none'));
          if (data.identified_pieces) {
            const identityLines = Object.keys(data.identified_pieces).sort().map((square) => {
              return square + ':' + data.identified_pieces[square].piece_id;
            });
            lines.push('Pieces: ' + (identityLines.join(', ') || 'none'));
          }
          squareCornerOutput.textContent = lines.join('\\n');
        } else {
          for (const [square, info] of entries) {
            lines.push(square + ' corners px:');
            const names = ['bottom-left', 'bottom-right', 'top-right', 'top-left'];
            info.roi_px.forEach((pt, index) => {
              lines.push('  ' + names[index] + ': ' + pt[0].toFixed(1) + ',' + pt[1].toFixed(1));
            });
            lines.push('  center: ' + info.center_px[0].toFixed(1) + ',' + info.center_px[1].toFixed(1));
          }
          squareCornerOutput.textContent = lines.join('\\n');
        }
      } else {
        squareCornerOutput.textContent = data.error || 'No square corner data.';
      }
      if (data.ok) {
        document.getElementById('overlay').src = '/live-overlay.mjpg?ts=' + Date.now();
      }
    }

    async function saveYoloSample() {
      const status = document.getElementById('yoloStatus');
      status.textContent = 'Saving YOLO sample...';
      const params = new URLSearchParams({
        corners: document.getElementById('corners').value,
        auto_locate: document.getElementById('autoLocate').checked ? '1' : '0',
        square_corners: document.getElementById('squareCornerInput').value,
        placements: document.getElementById('yoloPlacements').value,
        split: document.getElementById('yoloSplit').value || 'train'
      });
      const res = await fetch('/api/yolo/add-sample?' + params.toString());
      const data = await res.json();
      status.textContent = JSON.stringify(data, null, 2);
    }

    async function startYoloTrain() {
      const status = document.getElementById('yoloStatus');
      status.textContent = 'Starting YOLO training...';
      const res = await fetch('/api/yolo/train');
      const data = await res.json();
      status.textContent = JSON.stringify(data, null, 2);
    }

    async function refreshYoloStatus() {
      const status = document.getElementById('yoloStatus');
      const res = await fetch('/api/yolo/status?ts=' + Date.now());
      const data = await res.json();
      status.textContent = JSON.stringify(data, null, 2);
    }

    async function refreshLiveStatus() {
      try {
        const res = await fetch('/api/status?ts=' + Date.now());
        const data = await res.json();
        document.getElementById('liveStatus').textContent = JSON.stringify(data, null, 2);
      } catch (err) {
        document.getElementById('liveStatus').textContent = String(err);
      }
    }
    showInputFrame();
    refreshLiveStatus();
    setInterval(refreshLiveStatus, 1000);
  </script>
</body>
</html>
"""


class VisionState:
    def __init__(
        self,
        calibration_path: str,
        output_dir: str,
        camera: LiveCameraState,
        camera_cfg: CameraConfig,
        yolo_dataset_dir: str,
        yolo_docker_image: str,
    ) -> None:
        self.calibration_path = calibration_path
        self.output_dir = output_dir
        self.overlay_path = os.path.join(output_dir, "overlay.jpg")
        self.yolo_dataset_dir = yolo_dataset_dir
        self.yolo_docker_image = yolo_docker_image
        self.yolo_train_log_path = os.path.join(output_dir, "yolo_train.log")
        self.yolo_train_process = None
        self.yolo_train_command = []
        self.yolo_lock = threading.Lock()
        self.camera = camera
        self.camera_cfg = camera_cfg
        self.overlay_lock = threading.Lock()
        self.overlay_squares = DEFAULT_SQUARES
        self.overlay_seed_corners = parse_point_list(DEFAULT_CORNERS)
        self.overlay_auto_locate = False
        self.overlay_detect_pieces = True
        self.overlay_extra_board_points = np.empty((0, 2), dtype=np.float32)
        self.overlay_extra_image_points = np.empty((0, 2), dtype=np.float32)
        os.makedirs(output_dir, exist_ok=True)
        init_dataset(Path(self.yolo_dataset_dir))

    def set_overlay_config(
        self,
        squares: str,
        seed_corners: np.ndarray,
        auto_locate: bool,
        detect_pieces: bool,
        extra_board_points: np.ndarray,
        extra_image_points: np.ndarray,
    ) -> None:
        with self.overlay_lock:
            self.overlay_squares = squares
            self.overlay_seed_corners = np.asarray(seed_corners, dtype=np.float32).reshape(4, 2)
            self.overlay_auto_locate = auto_locate
            self.overlay_detect_pieces = detect_pieces
            self.overlay_extra_board_points = np.asarray(extra_board_points, dtype=np.float32).reshape(-1, 2)
            self.overlay_extra_image_points = np.asarray(extra_image_points, dtype=np.float32).reshape(-1, 2)

    def get_overlay_config(self) -> tuple[str, np.ndarray, bool, bool, np.ndarray, np.ndarray]:
        with self.overlay_lock:
            return (
                self.overlay_squares,
                self.overlay_seed_corners.copy(),
                self.overlay_auto_locate,
                self.overlay_detect_pieces,
                self.overlay_extra_board_points.copy(),
                self.overlay_extra_image_points.copy(),
            )


def make_handler(state: VisionState):
    class Handler(BaseHTTPRequestHandler):
        server_version = "HiveRobotChessboardVisionV27/1.0"

        def log_message(self, fmt, *args) -> None:  # type: ignore[override]
            print(f"[{time.strftime('%H:%M:%S')}] {self.address_string()} {fmt % args}", flush=True)

        def send_bytes(self, payload: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
            self.send_bytes(json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8"), "application/json; charset=utf-8", status)

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/":
                page = (
                    HTML_PAGE
                    .replace("__CORNERS__", html.escape(DEFAULT_CORNERS, quote=True))
                    .replace("__SQUARES__", html.escape(DEFAULT_SQUARES, quote=True))
                    .replace("__INITIAL_VIEWER_SRC__", "/live-rgb.mjpg?ts=0")
                )
                self.send_bytes(page.encode("utf-8"), "text/html; charset=utf-8")
                return
            if parsed.path == "/overlay.jpg":
                if not os.path.exists(state.overlay_path):
                    self.send_bytes(b"", "image/jpeg", HTTPStatus.NOT_FOUND)
                    return
                with open(state.overlay_path, "rb") as f:
                    self.send_bytes(f.read(), "image/jpeg")
                return
            if parsed.path == "/live-rgb.mjpg":
                self.stream_live_rgb()
                return
            if parsed.path == "/live-overlay.mjpg":
                self.stream_live_overlay()
                return
            if parsed.path == "/api/status":
                self.send_json({
                    "ok": True,
                    "version": WEB_VERSION,
                    "live_camera": state.camera_cfg.live_camera,
                    "rgb_topic": state.camera_cfg.rgb_topic,
                    "depth_topic": state.camera_cfg.depth_topic,
                    **state.camera.status(),
                })
                return
            if parsed.path == "/api/inspect":
                self.inspect(parsed.query)
                return
            if parsed.path == "/api/yolo/add-sample":
                self.yolo_add_sample(parsed.query)
                return
            if parsed.path == "/api/yolo/train":
                self.yolo_train()
                return
            if parsed.path == "/api/yolo/status":
                self.yolo_status()
                return
            self.send_json({"ok": False, "error": "not found"}, HTTPStatus.NOT_FOUND)

        def stream_live_rgb(self) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            last_seq = -1
            while True:
                frame, stamp, seq = state.camera.wait_for_newer(last_seq, timeout_s=1.0)
                if frame is None:
                    time.sleep(0.1)
                    continue
                try:
                    draw_live_status(frame, seq, stamp)
                    jpg = encode_jpeg(frame, state.camera_cfg.jpeg_quality)
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpg)}\r\n".encode("ascii")
                        + b"Cache-Control: no-store\r\n\r\n"
                    )
                    self.wfile.write(header + jpg + b"\r\n")
                    self.wfile.flush()
                    last_seq = seq
                except (BrokenPipeError, ConnectionResetError):
                    break

        def stream_live_overlay(self) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            last_seq = -1
            while True:
                frame, stamp, seq = state.camera.wait_for_newer(last_seq, timeout_s=1.0)
                if frame is None:
                    time.sleep(0.1)
                    continue
                try:
                    _rgb, _rgb_stamp, _rgb_seq, depth, depth_stamp, depth_seq = state.camera.snapshot()
                    squares, seed_corners, auto_locate, detect_pieces, extra_board_points, extra_image_points = state.get_overlay_config()
                    save_live_calibration(
                        state.calibration_path,
                        frame,
                        seed_corners,
                        auto_locate,
                        extra_board_points,
                        extra_image_points,
                    )
                    draw_squares_overlay_image(
                        frame,
                        state.calibration_path,
                        squares,
                        image_path="live",
                        detect_pieces=detect_pieces,
                        depth_image=depth,
                    )
                    draw_live_status(frame, seq, stamp)
                    if depth is not None:
                        cv2.putText(
                            frame,
                            f"depth seq={depth_seq} age={time.time() - depth_stamp:.2f}s",
                            (10, frame.shape[0] - 34),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 255, 255),
                            2,
                            cv2.LINE_AA,
                        )
                    jpg = encode_jpeg(frame, state.camera_cfg.jpeg_quality)
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpg)}\r\n".encode("ascii")
                        + b"Cache-Control: no-store\r\n\r\n"
                    )
                    self.wfile.write(header + jpg + b"\r\n")
                    self.wfile.flush()
                    last_seq = seq
                except (BrokenPipeError, ConnectionResetError):
                    break

        def inspect(self, query: str) -> None:
            try:
                params = parse_qs(query)
                corners_text = params.get("corners", [DEFAULT_CORNERS])[0]
                auto_locate = params.get("auto_locate", ["1"])[0].strip().lower() not in ("0", "false", "no", "off")
                detect_pieces = params.get("detect_pieces", ["1"])[0].strip().lower() not in ("0", "false", "no", "off")
                squares = params.get("squares", params.get("square", [DEFAULT_SQUARES]))[0]
                corner_image_points = parse_point_list(corners_text)
                frame, stamp, seq, depth, depth_stamp, depth_seq = state.camera.snapshot()
                if frame is None:
                    raise RuntimeError("live RGB frame not received yet")
                extra_text = params.get("square_corners", [""])[0]
                extra_board_points, extra_image_points, extra_labels = parse_square_corner_calibration(extra_text)
                corner_image_points, auto_location = save_live_calibration(
                    state.calibration_path,
                    frame,
                    corner_image_points,
                    auto_locate,
                    extra_board_points,
                    extra_image_points,
                    extra_labels,
                )
                if should_temporal_merge(squares, detect_pieces):
                    samples = []
                    sample_results = []
                    sample_count = 7
                    last_seq = seq
                    samples.append((frame, stamp, seq, depth, depth_stamp, depth_seq))
                    for _index in range(sample_count - 1):
                        next_frame, next_stamp, next_seq = state.camera.wait_for_newer(last_seq, timeout_s=0.18)
                        if next_frame is None or next_seq == last_seq:
                            break
                        _rgb, _rgb_stamp, _rgb_seq, next_depth, next_depth_stamp, next_depth_seq = state.camera.snapshot()
                        samples.append((next_frame, next_stamp, next_seq, next_depth, next_depth_stamp, next_depth_seq))
                        last_seq = next_seq
                    for sample_frame, _sample_stamp, _sample_seq, sample_depth, _sample_depth_stamp, _sample_depth_seq in samples:
                        sample_results.append(
                            draw_squares_overlay_image(
                                sample_frame.copy(),
                                state.calibration_path,
                                squares,
                                image_path="live",
                                detect_pieces=True,
                                depth_image=sample_depth,
                            )
                        )
                    piece_results_override = merge_temporal_piece_results(sample_results)
                    frame, stamp, seq, depth, depth_stamp, depth_seq = samples[-1]
                    result = annotate_squares_image(
                        frame,
                        state.calibration_path,
                        squares,
                        state.overlay_path,
                        image_path="live",
                        detect_pieces=True,
                        depth_image=depth,
                        piece_results_override=piece_results_override,
                    )
                    result["temporal_inspect"] = True
                    result["temporal_samples"] = len(samples)
                else:
                    result = annotate_squares_image(
                        frame,
                        state.calibration_path,
                        squares,
                        state.overlay_path,
                        image_path="live",
                        detect_pieces=detect_pieces,
                        depth_image=depth,
                    )
                result["rgb_age_s"] = round(time.time() - stamp, 3)
                result["rgb_seq"] = seq
                result["rgb_topic"] = state.camera_cfg.rgb_topic
                result["has_depth"] = depth is not None
                result["depth_age_s"] = None if depth is None else round(time.time() - depth_stamp, 3)
                result["depth_seq"] = depth_seq
                result["depth_topic"] = state.camera_cfg.depth_topic
                result["auto_location"] = auto_location
                result["calibration_corners_px"] = corner_image_points.astype(float).tolist()
                state.set_overlay_config(squares, parse_point_list(corners_text), auto_locate, detect_pieces, extra_board_points, extra_image_points)
                self.send_json({"ok": True, "version": WEB_VERSION, "live_camera": state.camera_cfg.live_camera, **result})
            except Exception as exc:
                self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)

        def yolo_add_sample(self, query: str) -> None:
            try:
                params = parse_qs(query)
                placements_text = params.get("placements", [""])[0]
                placements = parse_placements(placements_text)
                split = params.get("split", ["train"])[0].strip().lower() or "train"
                corners_text = params.get("corners", [DEFAULT_CORNERS])[0]
                auto_locate = params.get("auto_locate", ["0"])[0].strip().lower() not in ("0", "false", "no", "off")
                frame, stamp, seq, _depth, _depth_stamp, _depth_seq = state.camera.snapshot()
                if frame is None:
                    raise RuntimeError("live RGB frame not received yet")
                extra_text = params.get("square_corners", [""])[0]
                extra_board_points, extra_image_points, extra_labels = parse_square_corner_calibration(extra_text)
                corner_image_points = parse_point_list(corners_text)
                corner_image_points, auto_location = save_live_calibration(
                    state.calibration_path,
                    frame,
                    corner_image_points,
                    auto_locate,
                    extra_board_points,
                    extra_image_points,
                    extra_labels,
                )
                dataset_dir = Path(state.yolo_dataset_dir)
                init_dataset(dataset_dir)
                capture_dir = dataset_dir / "captures"
                capture_dir.mkdir(parents=True, exist_ok=True)
                sample_name = params.get("name", [""])[0].strip()
                if not sample_name:
                    sample_name = f"rank4_{time.strftime('%Y%m%d_%H%M%S')}_{seq}_{uuid.uuid4().hex[:6]}"
                image_path = capture_dir / f"{sample_name}.jpg"
                cv2.imwrite(str(image_path), frame)
                add_labeled_image(
                    image_path,
                    Path(state.calibration_path),
                    dataset_dir,
                    split,
                    placements,
                    sample_name,
                    shrink=float(params.get("shrink", ["0.72"])[0]),
                )
                self.send_json(
                    {
                        "ok": True,
                        "version": WEB_VERSION,
                        "dataset_dir": str(dataset_dir),
                        "data_yaml": str(write_data_yaml(dataset_dir)),
                        "split": split,
                        "sample_name": sample_name,
                        "image_path": str(image_path),
                        "placements": placements,
                        "rgb_seq": seq,
                        "rgb_age_s": round(time.time() - stamp, 3),
                        "auto_location": auto_location,
                        "calibration_corners_px": corner_image_points.astype(float).tolist(),
                    }
                )
            except Exception as exc:
                self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)

        def yolo_train(self) -> None:
            try:
                with state.yolo_lock:
                    process = state.yolo_train_process
                    if process is not None and process.poll() is None:
                        self.send_json(
                            {
                                "ok": False,
                                "error": "YOLO training is already running",
                                "pid": process.pid,
                                "log_path": state.yolo_train_log_path,
                            },
                            HTTPStatus.CONFLICT,
                        )
                        return
                    init_dataset(Path(state.yolo_dataset_dir))
                    command = build_yolo_train_command(state)
                    os.makedirs(os.path.dirname(state.yolo_train_log_path), exist_ok=True)
                    log = open(state.yolo_train_log_path, "ab", buffering=0)
                    log.write(("\n\n=== YOLO train started " + time.strftime("%Y-%m-%d %H:%M:%S") + " ===\n").encode("utf-8"))
                    process = subprocess.Popen(
                        command,
                        cwd=repo_root_for_script(),
                        stdout=log,
                        stderr=subprocess.STDOUT,
                    )
                    state.yolo_train_process = process
                    state.yolo_train_command = command
                self.send_json(
                    {
                        "ok": True,
                        "version": WEB_VERSION,
                        "pid": process.pid,
                        "command": command,
                        "dataset_dir": state.yolo_dataset_dir,
                        "log_path": state.yolo_train_log_path,
                    }
                )
            except Exception as exc:
                self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)

        def yolo_status(self) -> None:
            with state.yolo_lock:
                process = state.yolo_train_process
                if process is None:
                    running = False
                    returncode = None
                    pid = None
                else:
                    returncode = process.poll()
                    running = returncode is None
                    pid = process.pid
                payload = {
                    "ok": True,
                    "version": WEB_VERSION,
                    "running": running,
                    "pid": pid,
                    "returncode": returncode,
                    "command": state.yolo_train_command,
                    "dataset_dir": state.yolo_dataset_dir,
                    "log_path": state.yolo_train_log_path,
                    "log_tail": read_text_tail(state.yolo_train_log_path),
                }
            self.send_json(payload)

    return Handler


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Chessboard vision v2.7 web verifier")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8097)
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION_PATH)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--live-camera", action="store_true", help="accepted for compatibility; this verifier is live-only")
    parser.add_argument("--rgb-topic", default="/ascamera_hp60c/rgb0/image")
    parser.add_argument("--depth-topic", default="/ascamera_hp60c/depth0/image_raw")
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--yolo-dataset-dir", default=DEFAULT_YOLO_DATASET_DIR)
    parser.add_argument("--yolo-docker-image", default=DEFAULT_YOLO_DOCKER_IMAGE)
    return parser


def start_ros_camera(camera: LiveCameraState, rgb_topic: str, depth_topic: str) -> None:
    try:
        import rospy
        from sensor_msgs.msg import Image
    except Exception as exc:
        raise RuntimeError(f"ROS camera requested but rospy/sensor_msgs import failed: {exc}") from exc

    def rgb_cb(msg) -> None:
        camera.set_rgb(convert_ros_image(msg, desired_encoding="bgr8"))
        status = camera.status()
        if status["rgb_seq"] <= 3 or status["rgb_seq"] % 30 == 0:
            print(
                f"live RGB frame seq={status['rgb_seq']} shape={status['shape']} encoding={getattr(msg, 'encoding', '')}",
                flush=True,
            )

    def depth_cb(msg) -> None:
        camera.set_depth(convert_ros_depth_image(msg))
        status = camera.status()
        if status["depth_seq"] <= 3 or status["depth_seq"] % 30 == 0:
            print(
                f"live depth frame seq={status['depth_seq']} shape={status['depth_shape']} encoding={getattr(msg, 'encoding', '')}",
                flush=True,
            )

    if not rospy.core.is_initialized():
        rospy.init_node("chessboard_vision_v2_7_web_control", anonymous=True, disable_signals=True)
    rospy.Subscriber(rgb_topic, Image, rgb_cb, queue_size=1)
    rospy.Subscriber(depth_topic, Image, depth_cb, queue_size=1)


def main() -> None:
    args = build_parser().parse_args()
    camera = LiveCameraState()
    camera_cfg = CameraConfig(
        live_camera=True,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        jpeg_quality=args.jpeg_quality,
    )
    start_ros_camera(camera, args.rgb_topic, args.depth_topic)
    state = VisionState(args.calibration, args.output_dir, camera, camera_cfg, args.yolo_dataset_dir, args.yolo_docker_image)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    print(f"Chessboard vision v2.7 web verifier listening on http://{args.host}:{args.port}/", flush=True)
    print("This verifier does not expose arm motion commands.", flush=True)
    print(f"Live RGB topic: {args.rgb_topic}", flush=True)
    print(f"Live depth topic: {args.depth_topic}", flush=True)
    print(f"Live RGB stream: http://<orin-ip>:{args.port}/live-rgb.mjpg", flush=True)
    print(f"Live overlay stream: http://<orin-ip>:{args.port}/live-overlay.mjpg", flush=True)
    print(f"YOLO dataset dir: {args.yolo_dataset_dir}", flush=True)
    print(f"YOLO docker image: {args.yolo_docker_image}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
