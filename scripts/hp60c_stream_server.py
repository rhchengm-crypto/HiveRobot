#!/usr/bin/env python3
"""Serve live HP60C ROS RGB-D streams to a browser as MJPEG.

Run this on the Orin where the HP60C ROS topics are available, then open the
shown URL from Windows. The browser receives a continuous multipart stream; no
per-frame image files are written.
"""

import argparse
import json
import math
import os
import subprocess
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple
from urllib.parse import parse_qs, urlparse

# On Jetson/ROS Noetic, importing cv2 after rospy may fail with:
# "libgomp.so.1: cannot allocate memory in static TLS block". Load OpenCV
# before any ROS Python modules and before importing helpers that use cv2.
import cv2
import numpy as np

DEFAULT_CONTROLLER = "/home/nvidia/hive_robot/DM_Control_Python/left_arm_controller.py"

from hp60c_auto_target import (
    CAMERA_FORWARD_FROM_SHOULDER_CM,
    CAMERA_LEFT_FROM_SHOULDER_CM,
    CAMERA_PITCH_DOWN_DEG,
    CAMERA_UP_FROM_SHOULDER_CM,
    Intrinsics,
    camera_to_shoulder_frame_cm,
    choose_shoulder_grasp_strategy,
    depth_to_meters,
    find_target_in_roi,
    infer_intrinsics,
    pixel_to_camera_cm,
    provisional_arm_target,
    refine_to_dark_rgb_in_roi,
    refine_to_dark_rgb_target,
    robust_patch_depth,
    scan_target_windows,
)


# Kept only as a reference for the first debug UI. The server serves
# CLEAN_HTML_PAGE below.
LEGACY_HTML_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HiveRobot HP60C Live</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: Arial, sans-serif; }
    header { padding: 10px 12px; background: #1b1b1b; border-bottom: 1px solid #333; }
    h1 { margin: 0; font-size: 18px; }
    main { display: grid; grid-template-columns: minmax(0, 1fr) 360px; gap: 10px; padding: 10px; }
    img { display: block; width: 100%; max-height: calc(100vh - 70px); object-fit: contain; background: #000; }
    aside { display: grid; gap: 8px; align-content: start; }
    button { padding: 8px 10px; border: 1px solid #555; background: #242424; color: #eee; border-radius: 5px; cursor: pointer; }
    button:hover { background: #303030; }
    label { font-size: 13px; display: inline-flex; align-items: center; gap: 6px; margin-right: 10px; }
    pre { margin: 0; padding: 10px; height: calc(100vh - 190px); overflow: auto; background: #191919; border: 1px solid #333; font-size: 12px; line-height: 1.35; }
    a { color: #8ecbff; }
    @media (max-width: 900px) { main { grid-template-columns: 1fr; } pre { height: 260px; } }
  </style>
</head>
<body>
  <header>
    <h1>HiveRobot HP60C Live</h1>
    <div><a href="/stream.mjpg">stream</a> | <a href="/debug.jpg">debug.jpg</a> | <a href="/rgb.mjpg">rgb</a> | <a href="/depth.mjpg">depth</a></div>
  </header>
  <main>
    <img id="stream" src="/stream.mjpg?t=live" alt="HP60C live stream">
    <aside>
      <div>
        <label><input id="skipClaw" type="checkbox"> skip claw</label>
        <button onclick="copyCommand()">复制命令</button>
        <button onclick="executeTarget()">执行当前目标</button>
        <button onclick="goHome()">Home</button>
      </div>
      <div>
        <button onclick="annotate('perfect')">记录完美</button>
        <button onclick="annotate('success')">记录成功</button>
        <button onclick="annotate('fail')">记录失败</button>
      </div>
      <pre id="command">waiting command...</pre>
      <pre id="state">waiting...</pre>
    </aside>
  </main>
  <script>
    let latestCommand = '';
    async function refreshCommand() {
      const skip = document.getElementById('skipClaw').checked ? '1' : '0';
      const res = await fetch('/command.json?skip_claw=' + skip + '&t=' + Date.now(), { cache: 'no-store' });
      const data = await res.json();
      latestCommand = data.command_text || '';
      document.getElementById('command').textContent = JSON.stringify(data, null, 2);
    }
    async function refreshState() {
      try {
        await refreshCommand();
        const res = await fetch('/state.json?t=' + Date.now(), { cache: 'no-store' });
        document.getElementById('state').textContent =
          JSON.stringify(await res.json(), null, 2);
      } catch (err) {
        document.getElementById('state').textContent = String(err);
      }
    }
    document.getElementById('stream').onerror = function () {
      document.getElementById('state').textContent = 'stream image failed to load';
    };
    async function copyCommand() {
      await navigator.clipboard.writeText(latestCommand);
    }
    async function executeTarget() {
      if (!confirm('执行当前识别目标？机械臂会运动。')) return;
      const skip = document.getElementById('skipClaw').checked ? '1' : '0';
      const res = await fetch('/execute-target?skip_claw=' + skip, { method: 'POST' });
      document.getElementById('command').textContent = JSON.stringify(await res.json(), null, 2);
    }
    async function goHome() {
      if (!confirm('机械臂回 Home？')) return;
      const res = await fetch('/home', { method: 'POST' });
      document.getElementById('command').textContent = JSON.stringify(await res.json(), null, 2);
    }
    async function annotate(result) {
      const note = prompt('备注，可留空：', '');
      const res = await fetch('/annotate-last?result=' + encodeURIComponent(result) + '&note=' + encodeURIComponent(note || ''), { method: 'POST' });
      document.getElementById('command').textContent = JSON.stringify(await res.json(), null, 2);
    }
    refreshState();
    setInterval(refreshState, 500);
  </script>
</body>
</html>
"""


CLEAN_HTML_PAGE = """<!doctype html>
<html lang="en" translate="no">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="google" content="notranslate">
  <title>HiveRobot HP60C Live</title>
  <style>
    :root { color-scheme: dark; }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: #101214;
      color: #eef1f4;
      font-family: Arial, Helvetica, sans-serif;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 10px 14px;
      background: #171a1d;
      border-bottom: 1px solid #2b3035;
    }
    h1 { margin: 0; font-size: 18px; font-weight: 700; }
    nav { display: flex; gap: 12px; flex-wrap: wrap; font-size: 13px; }
    a { color: #8ecbff; text-decoration: none; }
    a:hover { text-decoration: underline; }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 340px;
      gap: 12px;
      padding: 12px;
    }
    .video {
      min-height: 0;
      background: #050607;
      border: 1px solid #2b3035;
    }
    img {
      display: block;
      width: 100%;
      max-height: calc(100vh - 76px);
      object-fit: contain;
      background: #000;
    }
    aside { display: grid; gap: 10px; align-content: start; }
    .panel {
      padding: 12px;
      background: #181c20;
      border: 1px solid #2d343b;
      border-radius: 8px;
    }
    .panel h2 {
      margin: 0 0 10px;
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.04em;
      color: #aeb8c2;
    }
    .coords {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
    }
    .coord {
      padding: 8px;
      background: #101418;
      border: 1px solid #283039;
      border-radius: 6px;
    }
    .coord span {
      display: block;
      margin-bottom: 4px;
      font-size: 11px;
      color: #9aa6b2;
    }
    .coord strong { font-size: 18px; color: #f5f75b; }
    .buttons { display: flex; flex-wrap: wrap; gap: 8px; }
    button {
      min-height: 34px;
      padding: 7px 10px;
      border: 1px solid #44505c;
      background: #242b32;
      color: #f5f7fa;
      border-radius: 6px;
      cursor: pointer;
      font-size: 13px;
    }
    button:hover { background: #303944; }
    button.primary { border-color: #4e8fd8; background: #1e4f86; }
    button.danger { border-color: #8f5555; background: #563030; }
    label {
      display: inline-flex;
      align-items: center;
      gap: 7px;
      min-height: 34px;
      padding-right: 6px;
      font-size: 13px;
      color: #d8dee5;
    }
    .status {
      margin: 0;
      white-space: pre-wrap;
      color: #d7dee6;
      font-size: 13px;
      line-height: 1.45;
    }
    .command {
      margin-top: 8px;
      padding-top: 8px;
      border-top: 1px solid #2d343b;
      color: #aeb8c2;
      font-size: 12px;
      line-height: 1.4;
      word-break: break-word;
    }
    @media (max-width: 900px) {
      main { grid-template-columns: 1fr; }
      img { max-height: none; }
    }
  </style>
</head>
<body>
  <header>
    <h1>HiveRobot HP60C Live</h1>
    <nav>
      <a href="/stream.mjpg">Live Stream</a>
      <a href="/debug.jpg">Snapshot</a>
      <a href="/rgb.mjpg">RGB</a>
      <a href="/depth.mjpg">Depth</a>
    </nav>
  </header>
  <main>
    <section class="video">
      <img id="stream" src="/stream.mjpg?t=live" alt="HP60C live stream">
    </section>
    <aside>
      <section class="panel">
        <h2>Target Coordinates</h2>
        <div class="coords">
          <div class="coord"><span>Forward</span><strong id="forward">--</strong></div>
          <div class="coord"><span>Left</span><strong id="left">--</strong></div>
          <div class="coord"><span>Up</span><strong id="up">--</strong></div>
        </div>
      </section>
      <section class="panel">
        <h2>Control</h2>
        <div class="buttons">
          <label><input id="skipClaw" type="checkbox"> Skip claw</label>
          <button type="button" onclick="copyCommand()">Copy Command</button>
          <button id="executeBtn" type="button" class="primary" onclick="executeTarget()">Execute Target</button>
          <button id="homeBtn" type="button" onclick="goHome()">Home</button>
        </div>
      </section>
      <section class="panel">
        <h2>Run Result</h2>
        <div class="buttons">
          <button type="button" onclick="annotate('perfect')">Mark Perfect</button>
          <button type="button" onclick="annotate('success')">Mark Success</button>
          <button type="button" class="danger" onclick="annotate('fail')">Mark Failure</button>
        </div>
      </section>
      <section class="panel">
        <h2>Status</h2>
        <p id="status" class="status">Waiting for camera and target...</p>
        <div id="commandSummary" class="command">Command will appear after target lock.</div>
      </section>
    </aside>
  </main>
  <script>
    let latestCommand = '';
    let lastMessage = '';

    function fmt(value) {
      return Number.isFinite(value) ? value.toFixed(2) : '--';
    }

    function targetCoords(payload) {
      const target = payload && payload.target;
      return target && target.shoulder_frame_cm ? target.shoulder_frame_cm : null;
    }

    function setStatus(text) {
      lastMessage = text || '';
      document.getElementById('status').textContent = lastMessage || 'Ready.';
    }

    function resultDetails(data) {
      const parts = [];
      if (data.error) parts.push(data.error);
      if (Number.isFinite(data.returncode)) parts.push('return code ' + data.returncode);
      const stderr = (data.stderr || '').trim();
      const stdout = (data.stdout || '').trim();
      if (stderr) parts.push('stderr: ' + stderr.slice(-700));
      else if (stdout) parts.push('stdout: ' + stdout.slice(-700));
      return parts.join('\\n') || 'unknown error';
    }

    async function refresh() {
      const skip = document.getElementById('skipClaw').checked ? '1' : '0';
      try {
        const [commandRes, stateRes] = await Promise.all([
          fetch('/command.json?skip_claw=' + skip + '&t=' + Date.now(), { cache: 'no-store' }),
          fetch('/state.json?t=' + Date.now(), { cache: 'no-store' })
        ]);
        const commandData = await commandRes.json();
        const stateData = await stateRes.json();
        latestCommand = commandData.command_text || '';

        const coords = targetCoords(stateData);
        document.getElementById('forward').textContent = coords ? fmt(coords.forward) : '--';
        document.getElementById('left').textContent = coords ? fmt(coords.left) : '--';
        document.getElementById('up').textContent = coords ? fmt(coords.up) : '--';

        const targetAge = stateData.target_age_s;
        const ageText = Number.isFinite(targetAge) ? targetAge.toFixed(2) + ' s ago' : 'not locked';
        const targetLine = coords ? 'Target locked: ' + ageText : 'No target lock';
        const errorLine = stateData.target_error ? '\\nDetection: ' + stateData.target_error : '';
        document.getElementById('status').textContent = (lastMessage ? lastMessage + '\\n' : '') + targetLine + errorLine;
        document.getElementById('commandSummary').textContent = latestCommand || 'Command will appear after target lock.';
      } catch (err) {
        document.getElementById('status').textContent = 'Connection error: ' + err;
      }
    }

    async function copyCommand() {
      if (!latestCommand) {
        setStatus('No command available yet.');
        return;
      }
      await navigator.clipboard.writeText(latestCommand);
      setStatus('Command copied.');
    }

    async function executeTarget() {
      if (!confirm('Execute the current target? The robot arm will move.')) return;
      const skip = document.getElementById('skipClaw').checked ? '1' : '0';
      const btn = document.getElementById('executeBtn');
      btn.disabled = true;
      btn.textContent = 'Executing...';
      setStatus('Executing target...');
      try {
        const res = await fetch('/execute-target?skip_claw=' + skip, { method: 'POST' });
        const data = await res.json();
        setStatus(data.ok ? 'Execute finished. Run ID: ' + data.run_id : 'Execute failed:\\n' + resultDetails(data));
      } catch (err) {
        setStatus('Execute request failed:\\n' + err);
      } finally {
        btn.disabled = false;
        btn.textContent = 'Execute Target';
        await refresh();
      }
    }

    async function goHome() {
      if (!confirm('Move the robot arm to Home?')) return;
      const btn = document.getElementById('homeBtn');
      btn.disabled = true;
      btn.textContent = 'Homing...';
      setStatus('Moving to Home...');
      try {
        const res = await fetch('/home', { method: 'POST' });
        const data = await res.json();
        setStatus(data.ok ? 'Home finished. Run ID: ' + data.run_id : 'Home failed:\\n' + resultDetails(data));
      } catch (err) {
        setStatus('Home request failed:\\n' + err);
      } finally {
        btn.disabled = false;
        btn.textContent = 'Home';
        await refresh();
      }
    }

    async function annotate(result) {
      const note = prompt('Optional note:', '') || '';
      const res = await fetch('/annotate-last?result=' + encodeURIComponent(result) + '&note=' + encodeURIComponent(note), { method: 'POST' });
      const data = await res.json();
      setStatus(data.ok ? 'Saved ' + result + ' annotation.' : 'Annotation failed: ' + (data.error || 'unknown error'));
      await refresh();
    }

    document.getElementById('stream').onerror = function () {
      setStatus('Live stream failed to load.');
    };
    setStatus('Web controls ready.');
    refresh();
    setInterval(refresh, 800);
  </script>
</body>
</html>
"""


@dataclass
class DetectorConfig:
    hfov_deg: float
    depth_scale: float
    depth_scale_set: bool
    roi: Tuple[int, int, int, int]
    min_depth_m: float
    max_depth_m: float
    close_margin_cm: float
    min_area: int
    max_area: int
    depth_radius: int
    aim_x_frac: float
    aim_y_frac: float
    target_depth_percentile: float
    scan_windows: bool
    window_w: int
    window_h: int
    window_step: int
    no_rgb_dark_refine: bool
    dark_v_max: int
    dark_min_area: int
    dark_max_area: int
    arm_offset: Tuple[float, float, float]
    camera_forward_cm: float
    camera_left_cm: float
    camera_up_cm: float
    camera_pitch_down_deg: float
    grasp_strategy: str


@dataclass
class ControllerConfig:
    controller: str
    python_bin: str
    geometry_execution_blend: float
    use_sudo: bool
    execute_enabled: bool


class RunJournal:
    def __init__(self, path: str) -> None:
        self.path = path
        self.lock = threading.Lock()
        self.last_run_id: Optional[str] = None

    def append(self, record: dict) -> str:
        run_id = record.get("run_id") or (
            time.strftime("%Y%m%d_%H%M%S") + f"_{int((time.time() % 1) * 1000):03d}_{record.get('type', 'record')}"
        )
        record["run_id"] = run_id
        record["logged_at"] = time.time()
        directory = os.path.dirname(os.path.abspath(self.path))
        if directory:
            os.makedirs(directory, exist_ok=True)
        line = json.dumps(record, ensure_ascii=False, sort_keys=True)
        with self.lock:
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(line + "\n")
            if record.get("type") in ("execute_target", "home"):
                self.last_run_id = run_id
        return run_id


class StreamState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.rgb = None
        self.depth = None
        self.intr: Optional[Intrinsics] = None
        self.rgb_stamp = 0.0
        self.depth_stamp = 0.0
        self.target = None
        self.target_error: Optional[str] = None
        self.target_stamp = 0.0

    def set_rgb(self, frame) -> None:
        with self.lock:
            self.rgb = frame.copy()
            self.rgb_stamp = time.time()

    def set_depth(self, frame) -> None:
        with self.lock:
            self.depth = frame.copy()
            self.depth_stamp = time.time()

    def set_intr(self, intr: Intrinsics) -> None:
        with self.lock:
            self.intr = intr

    def snapshot(self):
        with self.lock:
            return (
                None if self.rgb is None else self.rgb.copy(),
                None if self.depth is None else self.depth.copy(),
                self.intr,
                self.rgb_stamp,
                self.depth_stamp,
                self.target,
                self.target_error,
                self.target_stamp,
            )

    def set_target(self, target, error: Optional[str]) -> None:
        with self.lock:
            self.target = target
            self.target_error = error
            self.target_stamp = time.time()


def make_depth_vis(depth_m: np.ndarray, min_depth_m: float, max_depth_m: float):
    depth_vis = depth_m.copy()
    depth_vis[~np.isfinite(depth_vis)] = 0
    depth_vis = np.clip(depth_vis, min_depth_m, max_depth_m)
    scale = max(1e-6, max_depth_m - min_depth_m)
    depth_vis = ((depth_vis - min_depth_m) / scale * 255).astype(np.uint8)
    return cv2.applyColorMap(255 - depth_vis, cv2.COLORMAP_JET)


def detect_target(rgb, raw_depth, intr: Optional[Intrinsics], cfg: DetectorConfig) -> dict:
    h, w = rgb.shape[:2]
    intr = intr or infer_intrinsics(w, h, cfg.hfov_deg)
    depth_m = depth_to_meters(raw_depth, cfg.depth_scale, cfg.depth_scale_set)
    roi = (
        max(0, cfg.roi[0]),
        max(0, cfg.roi[1]),
        min(w, cfg.roi[2]),
        min(h, cfg.roi[3]),
    )
    if cfg.scan_windows:
        u, v, detect_info = scan_target_windows(
            depth_m,
            roi,
            cfg.window_w,
            cfg.window_h,
            cfg.window_step,
            cfg.min_depth_m,
            cfg.max_depth_m,
            cfg.close_margin_cm / 100.0,
            cfg.min_area,
            cfg.max_area,
            cfg.aim_x_frac,
            cfg.aim_y_frac,
            cfg.target_depth_percentile,
        )
    else:
        u, v, detect_info = find_target_in_roi(
            depth_m,
            roi,
            cfg.min_depth_m,
            cfg.max_depth_m,
            cfg.close_margin_cm / 100.0,
            cfg.min_area,
            cfg.max_area,
            cfg.aim_x_frac,
            cfg.aim_y_frac,
            cfg.target_depth_percentile,
        )

    raw_u, raw_v = u, v
    if not cfg.no_rgb_dark_refine:
        u, v, rgb_refine_info = refine_to_dark_rgb_target(
            rgb,
            detect_info,
            u,
            v,
            cfg.aim_y_frac,
            cfg.dark_v_max,
            cfg.dark_min_area,
            cfg.dark_max_area,
        )
        if not rgb_refine_info.get("used"):
            u, v, fallback_info = refine_to_dark_rgb_in_roi(
                rgb,
                roi,
                u,
                v,
                cfg.aim_y_frac,
                cfg.dark_v_max,
                cfg.dark_min_area,
                cfg.dark_max_area,
            )
            rgb_refine_info["fallback"] = fallback_info
            if fallback_info.get("used"):
                rgb_refine_info = fallback_info
        detect_info["rgb_dark_refine"] = rgb_refine_info
        detect_info["raw_depth_aim_pixel"] = {"u": raw_u, "v": raw_v}
        selected = rgb_refine_info.get("selected") if isinstance(rgb_refine_info, dict) else None
        if selected and selected.get("bbox"):
            detect_info["display_bbox"] = selected["bbox"]

    target_depth_m = float(detect_info["target_depth_m"])
    if detect_info.get("rgb_dark_refine", {}).get("used"):
        refined_depth_m = robust_patch_depth(depth_m, u, v, cfg.depth_radius)
        detect_info["depth_before_rgb_refine_m"] = target_depth_m
        detect_info["target_depth_m"] = refined_depth_m
        detect_info["target_depth_source"] = "rgb_refined_pixel_patch"
        target_depth_m = refined_depth_m

    camera_xyz = pixel_to_camera_cm(u, v, target_depth_m, intr)
    arm_xyz = provisional_arm_target(camera_xyz, cfg.arm_offset)
    shoulder_xyz = camera_to_shoulder_frame_cm(
        camera_xyz,
        cfg.camera_forward_cm,
        cfg.camera_left_cm,
        cfg.camera_up_cm,
        cfg.camera_pitch_down_deg,
    )
    strategy, strategy_info = choose_shoulder_grasp_strategy(
        arm_xyz,
        shoulder_xyz,
        cfg.grasp_strategy,
    )
    return {
        "pixel": {"u": u, "v": v},
        "depth_cm": target_depth_m * 100.0,
        "camera_cm": {"x": camera_xyz[0], "y": camera_xyz[1], "z": camera_xyz[2]},
        "arm_cm": {"x": arm_xyz[0], "y": arm_xyz[1], "z": arm_xyz[2]},
        "shoulder_frame_cm": {
            "forward": shoulder_xyz[0],
            "left": shoulder_xyz[1],
            "up": shoulder_xyz[2],
            "camera_pitch_down_deg": cfg.camera_pitch_down_deg,
        },
        "intrinsics": {
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.cx,
            "cy": intr.cy,
        },
        "detect": detect_info,
        "controller_strategy": strategy_info,
        "strategy": strategy,
    }


def overlay_target(rgb, target, error: Optional[str], cfg: DetectorConfig):
    frame = rgb.copy()
    h, w = frame.shape[:2]
    roi = (
        max(0, cfg.roi[0]),
        max(0, cfg.roi[1]),
        min(w, cfg.roi[2]),
        min(h, cfg.roi[3]),
    )
    cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 0), 2)
    if target:
        detect = target.get("detect", {})
        depth_bbox = detect.get("bbox")
        refine = detect.get("rgb_dark_refine", {})
        refined_bbox = None
        selected = refine.get("selected") if isinstance(refine, dict) else None
        if selected:
            refined_bbox = selected.get("bbox")
        if refined_bbox is None and isinstance(refine, dict):
            fallback = refine.get("fallback")
            selected = fallback.get("selected") if isinstance(fallback, dict) else None
            if selected:
                refined_bbox = selected.get("bbox")

        if depth_bbox:
            cv2.rectangle(
                frame,
                (depth_bbox["x0"], depth_bbox["y0"]),
                (depth_bbox["x1"], depth_bbox["y1"]),
                (255, 255, 0),
                1,
            )
        bbox = detect.get("display_bbox") or refined_bbox or depth_bbox
        if bbox:
            cv2.rectangle(
                frame,
                (bbox["x0"], bbox["y0"]),
                (bbox["x1"], bbox["y1"]),
                (255, 0, 255),
                2,
            )
        raw = detect.get("raw_depth_aim_pixel")
        if raw:
            cv2.circle(frame, (raw["u"], raw["v"]), 5, (255, 255, 0), 1)
        pixel = target["pixel"]
        cv2.circle(frame, (pixel["u"], pixel["v"]), 8, (0, 255, 255), 2)
        shoulder = target["shoulder_frame_cm"]
        label = (
            f"F {shoulder['forward']:.2f} L {shoulder['left']:.2f} "
            f"U {shoulder['up']:.2f} cm"
        )
        cv2.putText(frame, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    elif error:
        cv2.putText(frame, error[:90], (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 255), 2)
    return frame


def encode_jpeg(frame, quality: int) -> Optional[bytes]:
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    return buf.tobytes()


def ros_image_to_numpy(msg, desired_encoding: Optional[str] = None):
    """Convert a sensor_msgs/Image into a numpy array without cv_bridge."""
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
        "8uc1": 1,
        "16uc1": 1,
        "mono16": 1,
        "32fc1": 1,
    }
    dtype_by_encoding = {
        "bgr8": np.uint8,
        "rgb8": np.uint8,
        "bgra8": np.uint8,
        "rgba8": np.uint8,
        "mono8": np.uint8,
        "8uc1": np.uint8,
        "16uc1": np.uint16,
        "mono16": np.uint16,
        "32fc1": np.float32,
    }
    if encoding not in dtype_by_encoding:
        raise ValueError(f"unsupported ROS image encoding: {msg.encoding}")

    dtype = np.dtype(dtype_by_encoding[encoding])
    if msg.is_bigendian:
        dtype = dtype.newbyteorder(">")
    channels = channels_by_encoding[encoding]
    bytes_per_pixel = dtype.itemsize * channels
    row_items = int(msg.step // dtype.itemsize)
    arr = np.frombuffer(msg.data, dtype=dtype)
    arr = arr.reshape((msg.height, row_items))
    useful_items = msg.width * channels
    arr = arr[:, :useful_items]
    if channels > 1:
        arr = arr.reshape((msg.height, msg.width, channels))
    else:
        arr = arr.reshape((msg.height, msg.width))
    if msg.is_bigendian:
        arr = arr.byteswap().newbyteorder()
    arr = arr.copy()

    desired = (desired_encoding or "").lower()
    if desired == "bgr8":
        if encoding == "rgb8":
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        if encoding in ("mono8", "8uc1"):
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    return arr


def make_image_converter():
    try:
        from cv_bridge import CvBridge

        bridge = CvBridge()
    except Exception as exc:
        print(f"cv_bridge unavailable, using manual sensor_msgs/Image conversion: {exc}")
        bridge = None

    def convert(msg, desired_encoding: Optional[str] = None):
        if bridge is not None:
            try:
                return bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding or "passthrough")
            except Exception as exc:
                print(f"cv_bridge conversion failed, falling back to manual conversion: {exc}")
        return ros_image_to_numpy(msg, desired_encoding)

    return convert


def detector_loop(state: StreamState, cfg: DetectorConfig, hz: float) -> None:
    period = 1.0 / max(0.1, hz)
    while True:
        started = time.time()
        rgb, depth, intr, _rgb_stamp, _depth_stamp, _target, _err, _target_stamp = state.snapshot()
        if rgb is not None and depth is not None:
            try:
                target = detect_target(rgb, depth, intr, cfg)
                state.set_target(target, None)
            except Exception as exc:
                state.set_target(None, str(exc))
        time.sleep(max(0.0, period - (time.time() - started)))


def build_controller_command(target: Optional[dict], ctrl: ControllerConfig, skip_claw: bool) -> Optional[list]:
    if not target:
        return None
    shoulder = target.get("shoulder_frame_cm")
    if not shoulder:
        return None
    cmd = []
    if ctrl.use_sudo:
        cmd.extend(["sudo", "-n"])
    cmd.extend(
        [
            ctrl.python_bin,
            ctrl.controller,
            "target-shoulder",
            "--forward",
            f"{float(shoulder['forward']):.2f}",
            "--left",
            f"{float(shoulder['left']):.2f}",
            "--up",
            f"{float(shoulder['up']):.2f}",
            "--geometry-execution-blend",
            f"{ctrl.geometry_execution_blend:.2f}",
            "--execute",
            "--allow-unverified-geometry",
        ]
    )
    if skip_claw:
        cmd.append("--skip-claw")
    strategy = target.get("strategy")
    if strategy == "extreme-near-left":
        cmd.extend(["--auto-pre-claw-front-lift", "--hold-wrist-lift"])
    return cmd


def command_payload(state: StreamState, ctrl: ControllerConfig, skip_claw: bool) -> dict:
    _rgb, _depth, _intr, _rgb_stamp, _depth_stamp, target, err, target_stamp = state.snapshot()
    cmd = build_controller_command(target, ctrl, skip_claw)
    return {
        "execute_enabled": ctrl.execute_enabled,
        "skip_claw": skip_claw,
        "target_age_s": None if target is None else round(time.time() - target_stamp, 3),
        "target_error": err,
        "command": cmd,
        "command_text": "" if cmd is None else " ".join(cmd),
        "target": target,
    }


def build_home_command(ctrl: ControllerConfig) -> list:
    cmd = []
    if ctrl.use_sudo:
        cmd.extend(["sudo", "-n"])
    cmd.extend([ctrl.python_bin, ctrl.controller, "home"])
    return cmd


def make_handler(
    state: StreamState,
    cfg: DetectorConfig,
    ctrl: ControllerConfig,
    journal: RunJournal,
    jpeg_quality: int,
    stream_fps: float,
):
    class Handler(BaseHTTPRequestHandler):
        server_version = "HiveRobotHP60CStream/1.0"

        def log_message(self, fmt, *args) -> None:
            message = fmt % args
            if "GET /state.json" in message or "GET /command.json" in message:
                return
            print("%s - - [%s] %s" % (self.client_address[0], self.log_date_time_string(), message))

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            path = parsed.path
            query = parse_qs(parsed.query)
            if path in ("/", "/index.html"):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(CLEAN_HTML_PAGE.encode("utf-8"))
                return
            if path == "/state.json":
                self.send_json_state()
                return
            if path == "/command.json":
                self.send_command_json(query)
                return
            if path == "/debug.jpg":
                self.send_debug_jpeg()
                return
            if path in ("/stream.mjpg", "/rgb.mjpg", "/depth.mjpg", "/target.mjpg"):
                self.send_mjpeg(path)
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/execute-target":
                self.execute_target(parse_qs(parsed.query))
                return
            if parsed.path == "/home":
                self.go_home()
                return
            if parsed.path == "/annotate-last":
                self.annotate_last(parse_qs(parsed.query))
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
            data = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def query_bool(self, query: dict, name: str) -> bool:
            return query.get(name, ["0"])[0].lower() in ("1", "true", "yes", "on")

        def send_json_state(self) -> None:
            rgb, depth, _intr, rgb_stamp, depth_stamp, target, err, target_stamp = state.snapshot()
            now = time.time()
            payload = {
                "rgb_age_s": None if rgb is None else round(now - rgb_stamp, 3),
                "depth_age_s": None if depth is None else round(now - depth_stamp, 3),
                "target_age_s": None if target is None else round(now - target_stamp, 3),
                "target_error": err,
                "target": target,
            }
            self.send_json(payload)

        def send_command_json(self, query: dict) -> None:
            self.send_json(command_payload(state, ctrl, self.query_bool(query, "skip_claw")))

        def execute_target(self, query: dict) -> None:
            skip_claw = self.query_bool(query, "skip_claw")
            payload = command_payload(state, ctrl, skip_claw)
            cmd = payload.get("command")
            started_at = time.time()
            if cmd is None:
                payload["ok"] = False
                payload["error"] = "no target available"
                self.send_json(payload, HTTPStatus.CONFLICT)
                return
            if not ctrl.execute_enabled:
                payload["ok"] = False
                payload["error"] = "controller execution disabled; restart with --enable-controller-execute"
                self.send_json(payload, HTTPStatus.FORBIDDEN)
                return
            print("execute controller command:", " ".join(cmd))
            try:
                completed = subprocess.run(cmd, check=False, text=True, capture_output=True)
            except Exception as exc:
                payload["ok"] = False
                payload["error"] = str(exc)
                payload["duration_s"] = round(time.time() - started_at, 3)
                payload["run_id"] = journal.append(
                    {
                        "type": "execute_target",
                        "ok": False,
                        "error": str(exc),
                        "started_at": started_at,
                        "duration_s": payload["duration_s"],
                        "skip_claw": skip_claw,
                        "command": cmd,
                        "target": payload.get("target"),
                    }
                )
                self.send_json(payload, HTTPStatus.INTERNAL_SERVER_ERROR)
                return
            payload["ok"] = completed.returncode == 0
            payload["returncode"] = completed.returncode
            payload["duration_s"] = round(time.time() - started_at, 3)
            payload["stdout"] = completed.stdout[-4000:]
            payload["stderr"] = completed.stderr[-4000:]
            payload["run_id"] = journal.append(
                {
                    "type": "execute_target",
                    "ok": payload["ok"],
                    "returncode": completed.returncode,
                    "started_at": started_at,
                    "duration_s": payload["duration_s"],
                    "skip_claw": skip_claw,
                    "command": cmd,
                    "target": payload.get("target"),
                    "stdout": completed.stdout,
                    "stderr": completed.stderr,
                }
            )
            self.send_json(payload, HTTPStatus.OK if completed.returncode == 0 else HTTPStatus.INTERNAL_SERVER_ERROR)

        def go_home(self) -> None:
            cmd = build_home_command(ctrl)
            started_at = time.time()
            payload = {
                "execute_enabled": ctrl.execute_enabled,
                "command": cmd,
                "command_text": " ".join(cmd),
            }
            if not ctrl.execute_enabled:
                payload["ok"] = False
                payload["error"] = "controller execution disabled; restart with --enable-controller-execute"
                self.send_json(payload, HTTPStatus.FORBIDDEN)
                return
            print("execute home command:", " ".join(cmd))
            try:
                completed = subprocess.run(cmd, check=False, text=True, capture_output=True)
            except Exception as exc:
                payload["ok"] = False
                payload["error"] = str(exc)
                payload["duration_s"] = round(time.time() - started_at, 3)
                payload["run_id"] = journal.append(
                    {
                        "type": "home",
                        "ok": False,
                        "error": str(exc),
                        "started_at": started_at,
                        "duration_s": payload["duration_s"],
                        "command": cmd,
                    }
                )
                self.send_json(payload, HTTPStatus.INTERNAL_SERVER_ERROR)
                return
            payload["ok"] = completed.returncode == 0
            payload["returncode"] = completed.returncode
            payload["duration_s"] = round(time.time() - started_at, 3)
            payload["stdout"] = completed.stdout[-4000:]
            payload["stderr"] = completed.stderr[-4000:]
            payload["run_id"] = journal.append(
                {
                    "type": "home",
                    "ok": payload["ok"],
                    "returncode": completed.returncode,
                    "started_at": started_at,
                    "duration_s": payload["duration_s"],
                    "command": cmd,
                    "stdout": completed.stdout,
                    "stderr": completed.stderr,
                }
            )
            self.send_json(payload, HTTPStatus.OK if completed.returncode == 0 else HTTPStatus.INTERNAL_SERVER_ERROR)

        def annotate_last(self, query: dict) -> None:
            result = query.get("result", [""])[0]
            note = query.get("note", [""])[0]
            if result not in ("perfect", "success", "fail"):
                self.send_json({"ok": False, "error": "result must be perfect/success/fail"}, HTTPStatus.BAD_REQUEST)
                return
            if journal.last_run_id is None:
                self.send_json({"ok": False, "error": "no previous run to annotate"}, HTTPStatus.CONFLICT)
                return
            record = {
                "type": "annotation",
                "annotates_run_id": journal.last_run_id,
                "result": result,
                "note": note,
            }
            annotation_id = journal.append(record)
            self.send_json(
                {
                    "ok": True,
                    "annotation_id": annotation_id,
                    "annotates_run_id": journal.last_run_id,
                    "result": result,
                    "note": note,
                    "log_path": journal.path,
                }
            )

        def send_debug_jpeg(self) -> None:
            rgb, _depth, _intr, _rgb_stamp, _depth_stamp, target, err, _target_stamp = state.snapshot()
            if rgb is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "RGB frame not received yet")
                return
            frame = overlay_target(rgb, target, err, cfg)
            jpg = encode_jpeg(frame, jpeg_quality)
            if jpg is None:
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "JPEG encode failed")
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)

        def send_mjpeg(self, path: str) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            period = 1.0 / max(1.0, stream_fps)
            while True:
                started = time.time()
                rgb, depth, _intr, _rgb_stamp, _depth_stamp, target, err, _target_stamp = state.snapshot()
                frame = None
                if path == "/rgb.mjpg":
                    frame = rgb
                elif path == "/depth.mjpg" and depth is not None:
                    depth_m = depth_to_meters(depth, cfg.depth_scale, cfg.depth_scale_set)
                    frame = make_depth_vis(depth_m, cfg.min_depth_m, cfg.max_depth_m)
                elif path in ("/stream.mjpg", "/target.mjpg") and rgb is not None:
                    frame = overlay_target(rgb, target, err, cfg)
                if frame is None:
                    time.sleep(0.05)
                    continue
                jpg = encode_jpeg(frame, jpeg_quality)
                if jpg is None:
                    time.sleep(0.05)
                    continue
                try:
                    self.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                    )
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
                time.sleep(max(0.0, period - (time.time() - started)))

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--stream-fps", type=float, default=15.0)
    parser.add_argument("--detect-hz", type=float, default=5.0)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--controller", default=DEFAULT_CONTROLLER)
    parser.add_argument("--controller-python", default="python3")
    parser.add_argument("--geometry-execution-blend", type=float, default=1.0)
    parser.add_argument("--no-sudo-controller", action="store_true")
    parser.add_argument("--enable-controller-execute", action="store_true")
    parser.add_argument("--run-log", default="data/hp60c_web_runs.jsonl")
    parser.add_argument("--rgb-topic", default="/ascamera_hp60c/rgb0/image")
    parser.add_argument("--depth-topic", default="/ascamera_hp60c/depth0/image_raw")
    parser.add_argument("--camera-info-topic", default="/ascamera_hp60c/rgb0/camera_info")
    parser.add_argument("--hfov-deg", type=float, default=73.8)
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--depth-scale-set", action="store_true")
    parser.add_argument("--roi-x0", type=int, default=55)
    parser.add_argument("--roi-y0", type=int, default=115)
    parser.add_argument("--roi-x1", type=int, default=605)
    parser.add_argument("--roi-y1", type=int, default=455)
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
        "--grasp-strategy",
        choices=("auto", "normal", "extreme-near-left"),
        default="auto",
    )
    args = parser.parse_args()

    import rospy
    from sensor_msgs.msg import CameraInfo, Image

    cfg = DetectorConfig(
        hfov_deg=args.hfov_deg,
        depth_scale=args.depth_scale,
        depth_scale_set=args.depth_scale_set,
        roi=(args.roi_x0, args.roi_y0, args.roi_x1, args.roi_y1),
        min_depth_m=args.min_depth_m,
        max_depth_m=args.max_depth_m,
        close_margin_cm=args.close_margin_cm,
        min_area=args.min_area,
        max_area=args.max_area,
        depth_radius=args.depth_radius,
        aim_x_frac=args.aim_x_frac,
        aim_y_frac=args.aim_y_frac,
        target_depth_percentile=args.target_depth_percentile,
        scan_windows=args.scan_windows,
        window_w=args.window_w,
        window_h=args.window_h,
        window_step=args.window_step,
        no_rgb_dark_refine=args.no_rgb_dark_refine,
        dark_v_max=args.dark_v_max,
        dark_min_area=args.dark_min_area,
        dark_max_area=args.dark_max_area,
        arm_offset=(args.arm_offset_x, args.arm_offset_y, args.arm_offset_z),
        camera_forward_cm=args.camera_forward_from_shoulder_cm,
        camera_left_cm=args.camera_left_from_shoulder_cm,
        camera_up_cm=args.camera_up_from_shoulder_cm,
        camera_pitch_down_deg=args.camera_pitch_down_deg,
        grasp_strategy=args.grasp_strategy,
    )
    ctrl = ControllerConfig(
        controller=args.controller,
        python_bin=args.controller_python,
        geometry_execution_blend=args.geometry_execution_blend,
        use_sudo=not args.no_sudo_controller,
        execute_enabled=args.enable_controller_execute,
    )
    journal = RunJournal(args.run_log)
    state = StreamState()
    convert_image = make_image_converter()

    def rgb_cb(msg: Image) -> None:
        frame = convert_image(msg, desired_encoding="bgr8")
        state.set_rgb(frame)

    def depth_cb(msg: Image) -> None:
        state.set_depth(convert_image(msg, desired_encoding="passthrough"))

    def info_cb(msg: CameraInfo) -> None:
        k = msg.K
        if k and k[0] > 0 and k[4] > 0:
            state.set_intr(Intrinsics(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5])))

    rospy.init_node("hp60c_stream_server", anonymous=True, disable_signals=True)
    rospy.Subscriber(args.rgb_topic, Image, rgb_cb, queue_size=1)
    rospy.Subscriber(args.depth_topic, Image, depth_cb, queue_size=1)
    rospy.Subscriber(args.camera_info_topic, CameraInfo, info_cb, queue_size=1)

    detector = threading.Thread(target=detector_loop, args=(state, cfg, args.detect_hz), daemon=True)
    detector.start()

    handler = make_handler(state, cfg, ctrl, journal, args.jpeg_quality, args.stream_fps)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"HP60C live stream server listening on http://{args.host}:{args.port}/")
    print("Open from Windows with: http://<orin-ip>:%d/" % args.port)
    print("Direct stream: http://<orin-ip>:%d/stream.mjpg" % args.port)
    print("Single-frame debug: http://<orin-ip>:%d/debug.jpg" % args.port)
    print("Controller execute enabled:", ctrl.execute_enabled)
    print("Run log:", os.path.abspath(args.run_log))
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
