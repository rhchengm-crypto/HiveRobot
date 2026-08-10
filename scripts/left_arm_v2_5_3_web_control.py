#!/usr/bin/env python3
"""Web control panel for left_arm_v2_5_3 with HP60C RGB-D stream.

Run on the Orin where ROS HP60C topics and the left arm serial port exist.
The page exposes only the tested v2.5.3 actions plus a debug command/output
panel so every button press leaves a visible command trail.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple
from urllib.parse import parse_qs, urlparse

# Import OpenCV before ROS on Jetson to avoid libgomp static TLS issues.
import cv2
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from hp60c_auto_target import (
    Intrinsics,
    camera_to_shoulder_frame_cm,
    choose_shoulder_grasp_strategy,
    depth_to_meters as hp_depth_to_meters,
    find_target_in_roi,
    infer_intrinsics,
    pixel_to_camera_cm,
    provisional_arm_target,
    refine_to_dark_rgb_in_roi,
    refine_to_dark_rgb_target,
    robust_patch_depth,
    scan_target_windows,
)


DEFAULT_ARM_SCRIPT = os.path.join(SCRIPT_DIR, "left_arm_v2_5_3.py")
DEFAULT_RUN_LOG = os.path.join(SCRIPT_DIR, "data", "left_arm_v2_5_3_web_runs.jsonl")
DEFAULT_ROI = (55, 115, 605, 455)
DEFAULT_CAMERA_FORWARD_FROM_SHOULDER_CM = 10.5
DEFAULT_CAMERA_LEFT_FROM_SHOULDER_CM = -13.0
DEFAULT_CAMERA_UP_FROM_SHOULDER_CM = 12.0
DEFAULT_CAMERA_PITCH_DOWN_DEG = 70.0
WEB_CONTROLLER_VERSION = "v1.2"
DEFAULT_JOINTS = [
    "shoulder_front",
    "shoulder_side",
    "shoulder_rotate",
    "elbow",
    "arm_roll",
    "wrist_side",
    "wrist",
]
NUDGE_DEGREES = {2.0, 5.0, 10.0}
NUDGE_SECONDS = {
    2.0: 1.6,
    5.0: 2.2,
    10.0: 3.2,
}
WRIST_NUDGE_SECONDS = 6.0
JOINT_DIRECTION_LABELS = {
    "shoulder_front": {"positive": "Backward", "negative": "Forward"},
    "shoulder_side": {"positive": "Outward", "negative": "Inward"},
    "elbow": {"positive": "Forward", "negative": "Backward"},
    "shoulder_rotate": {"positive": "Outward Rotate", "negative": "Inward Rotate"},
    "arm_roll": {"positive": "Inward Roll", "negative": "Outward Roll"},
    "wrist_side": {"positive": "Wrist Outward", "negative": "Wrist Inward"},
    "wrist": {"positive": "Wrist Forward", "negative": "Wrist Backward"},
}


HTML_PAGE = """<!doctype html>
<html lang="en" translate="no">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="google" content="notranslate">
  <title>HiveRobot Left Arm Control</title>
  <style>
    :root { color-scheme: dark; }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: #0f1215;
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
    h1 { margin: 0; font-size: 18px; font-weight: 700; }
    nav { display: flex; gap: 12px; flex-wrap: wrap; font-size: 13px; }
    a { color: #87c7ff; text-decoration: none; }
    a:hover { text-decoration: underline; }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 380px;
      gap: 12px;
      padding: 12px;
    }
    .video {
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
    .panel h2 {
      margin: 0 0 10px;
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.04em;
      color: #aeb8c3;
    }
    .panel-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      margin-bottom: 10px;
    }
    .panel-head h2 { margin: 0; }
    .buttons {
      display: grid;
      grid-template-columns: 1fr;
      gap: 8px;
    }
    .nudge-degrees {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      margin-bottom: 10px;
    }
    .nudge-degrees label {
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 5px;
      min-height: 34px;
      border: 1px solid #485565;
      border-radius: 6px;
      background: #10151a;
      font-size: 13px;
    }
    .joint-grid {
      display: grid;
      gap: 8px;
    }
    .joint-row {
      display: grid;
      grid-template-columns: 118px minmax(0, 1fr) minmax(0, 1fr);
      gap: 6px;
      align-items: center;
    }
    .joint-name {
      color: #cbd5df;
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    button {
      min-height: 42px;
      padding: 9px 11px;
      border: 1px solid #485565;
      border-radius: 6px;
      background: #242b33;
      color: #eef2f5;
      font-size: 14px;
      text-align: left;
      cursor: pointer;
    }
    .copy-btn {
      min-height: 30px;
      padding: 5px 9px;
      font-size: 12px;
      text-align: center;
    }
    button:hover { background: #2d3540; }
    button:disabled { opacity: 0.55; cursor: wait; }
    .primary { border-color: #4f8fcb; background: #203247; }
    .danger { border-color: #d64b4b; background: #4a2020; }
    .status {
      min-height: 24px;
      color: #c7d1db;
      font-size: 13px;
      line-height: 1.35;
      white-space: pre-wrap;
    }
    pre {
      margin: 0;
      padding: 10px;
      min-height: 110px;
      max-height: 260px;
      overflow: auto;
      background: #0f1317;
      border: 1px solid #2a333c;
      border-radius: 6px;
      color: #dce5ed;
      font-size: 12px;
      line-height: 1.4;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
    }
    .meta {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      font-size: 12px;
      color: #cad3dc;
    }
    .coords {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 8px;
      margin-top: 8px;
      font-size: 12px;
      color: #cad3dc;
    }
    .meta div,
    .coords div {
      padding: 8px;
      background: #10151a;
      border: 1px solid #2a333c;
      border-radius: 6px;
    }
    .coords strong {
      display: block;
      margin-top: 3px;
      color: #f5f75b;
      font-size: 15px;
    }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      img { max-height: 58vh; }
      pre { max-height: 220px; }
    }
  </style>
</head>
<body>
  <header>
    <h1>HiveRobot Left Arm Control <span style="color:#7f8c98;font-size:13px;">v1.2</span></h1>
    <nav>
      <a href="/stream.mjpg">RGB + Depth</a>
      <a href="/rgb.mjpg">RGB</a>
      <a href="/depth.mjpg">Depth</a>
    </nav>
  </header>
  <main>
    <section class="video">
      <img id="stream" src="/stream.mjpg?t=live" alt="HP60C RGB-D live stream">
    </section>
    <aside>
      <section class="panel">
        <h2>Controls</h2>
        <div class="buttons">
          <button type="button" onclick="runAction('capture-home')">Capture New Home</button>
          <button type="button" onclick="runAction('capture-clearance')">Capture New Clearance</button>
          <button type="button" class="primary" onclick="runAction('table-clearance')">Table Clearance Move</button>
          <button type="button" class="primary" onclick="runAction('home')">Home Move</button>
          <button type="button" class="danger" onclick="runAction('emergency-stop', true)">Emergency Stop</button>
        </div>
      </section>
      <section class="panel">
        <h2>Joint Nudge <span style="color:#7f8c98;font-size:12px;">v1.2</span></h2>
        <div class="nudge-degrees">
          <label><input type="radio" name="nudgeDeg" value="2" checked>2°</label>
          <label><input type="radio" name="nudgeDeg" value="5">5°</label>
          <label><input type="radio" name="nudgeDeg" value="10">10°</label>
        </div>
        <div class="joint-grid" id="jointGrid"></div>
      </section>
      <section class="panel">
        <h2>Status</h2>
        <div class="meta">
          <div>RGB<br><span id="rgbAge">--</span></div>
          <div>Depth<br><span id="depthAge">--</span></div>
          <div>Run<br><span id="runState">idle</span></div>
        </div>
        <div class="coords">
          <div>Forward<strong id="forward">--</strong></div>
          <div>Left<strong id="left">--</strong></div>
          <div>Up<strong id="up">--</strong></div>
          <div>Depth<strong id="targetDepth">--</strong></div>
        </div>
        <div id="status" class="status"></div>
      </section>
      <section class="panel">
        <h2>Debug Command</h2>
        <pre id="command">waiting...</pre>
      </section>
      <section class="panel">
        <div class="panel-head">
          <h2>Debug Output</h2>
          <button type="button" class="copy-btn" onclick="copyOutput()">Copy</button>
        </div>
        <pre id="output">waiting...</pre>
      </section>
    </aside>
  </main>
  <script>
    const actionNames = {
      'capture-home': 'Capture New Home',
      'capture-clearance': 'Capture New Clearance',
      'table-clearance': 'Table Clearance Move',
      'home': 'Home Move',
      'emergency-stop': 'Emergency Stop'
    };
    const jointDirections = [
      ['shoulder_front', 'Backward', 'Forward'],
      ['shoulder_side', 'Outward', 'Inward'],
      ['elbow', 'Forward', 'Backward'],
      ['shoulder_rotate', 'Outward Rotate', 'Inward Rotate'],
      ['arm_roll', 'Inward Roll', 'Outward Roll'],
      ['wrist_side', 'Wrist Outward', 'Wrist Inward'],
      ['wrist', 'Wrist Forward', 'Wrist Backward']
    ];

    function setStatus(text) {
      document.getElementById('status').textContent = text || '';
    }

    function formatOutput(data) {
      if (!data) return 'waiting...';
      const parts = [];
      if (data.started_at) parts.push('started_at: ' + data.started_at);
      if (data.finished_at) parts.push('finished_at: ' + data.finished_at);
      if (data.duration_s !== null && data.duration_s !== undefined) parts.push('duration_s: ' + data.duration_s);
      if (data.returncode !== null && data.returncode !== undefined) parts.push('returncode: ' + data.returncode);
      if (data.error) parts.push('error:\\n' + data.error);
      if (data.stdout) parts.push('stdout:\\n' + data.stdout);
      if (data.stderr) parts.push('stderr:\\n' + data.stderr);
      return parts.join('\\n\\n') || JSON.stringify(data, null, 2);
    }

    function selectedNudgeDeg() {
      const selected = document.querySelector('input[name="nudgeDeg"]:checked');
      return selected ? Number(selected.value) : 2;
    }

    function buildJointGrid() {
      const grid = document.getElementById('jointGrid');
      grid.innerHTML = '';
      jointDirections.forEach(([joint, positive, negative]) => {
        const row = document.createElement('div');
        row.className = 'joint-row';
        row.innerHTML =
          '<div class="joint-name">' + joint + '</div>' +
          '<button type="button" onclick="runNudge(\\'' + joint + '\\', 1)">' + positive + '</button>' +
          '<button type="button" onclick="runNudge(\\'' + joint + '\\', -1)">' + negative + '</button>';
        grid.appendChild(row);
      });
    }

    async function runAction(action, force) {
      const label = actionNames[action] || action;
      if (!force && !confirm(label + '?')) return;
      if (action === 'emergency-stop' && !confirm('Emergency stop: disable all left-arm motors now?')) return;
      setStatus(label + ' requested...');
      try {
        const res = await fetch('/api/action/' + action, { method: 'POST', cache: 'no-store' });
        const data = await res.json();
        document.getElementById('command').textContent = data.command_text || JSON.stringify(data.command || [], null, 2);
        document.getElementById('output').textContent = formatOutput(data);
        setStatus(data.ok ? label + ' running/finished.' : label + ' failed to start.');
      } catch (err) {
        setStatus(label + ' request failed:\\n' + err);
      }
      refresh();
    }

    async function runNudge(joint, direction) {
      const deg = selectedNudgeDeg();
      const label = joint + ' ' + (direction > 0 ? '+' : '-') + deg + '°';
      if (!confirm('Nudge ' + label + '?')) return;
      setStatus('Nudge requested: ' + label);
      try {
        const res = await fetch('/api/nudge', {
          method: 'POST',
          cache: 'no-store',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ joint, direction, deg })
        });
        const data = await res.json();
        document.getElementById('command').textContent = data.command_text || JSON.stringify(data.command || [], null, 2);
        document.getElementById('output').textContent = formatOutput(data);
        setStatus(data.ok ? 'Nudge running/finished: ' + label : 'Nudge failed to start: ' + label);
      } catch (err) {
        setStatus('Nudge request failed:\\n' + err);
      }
      refresh();
    }

    async function copyOutput() {
      const text = document.getElementById('output').textContent || '';
      try {
        if (navigator.clipboard && navigator.clipboard.writeText) {
          await navigator.clipboard.writeText(text);
        } else {
          const area = document.createElement('textarea');
          area.value = text;
          area.setAttribute('readonly', '');
          area.style.position = 'fixed';
          area.style.left = '-9999px';
          document.body.appendChild(area);
          area.select();
          const ok = document.execCommand('copy');
          document.body.removeChild(area);
          if (!ok) throw new Error('document.execCommand copy returned false');
        }
        setStatus('Debug output copied.');
      } catch (err) {
        setStatus('Copy failed:\\n' + err);
      }
    }

    async function refresh() {
      try {
        const res = await fetch('/api/state?t=' + Date.now(), { cache: 'no-store' });
        const data = await res.json();
        const target = data.target || null;
        const coords = target && target.shoulder_frame_cm ? target.shoulder_frame_cm : null;
        const fmt = (value) => Number.isFinite(value) ? value.toFixed(1) + ' cm' : '--';
        document.getElementById('rgbAge').textContent = data.rgb_age_s === null ? '--' : data.rgb_age_s + 's';
        document.getElementById('depthAge').textContent = data.depth_age_s === null ? '--' : data.depth_age_s + 's';
        document.getElementById('runState').textContent = data.run && data.run.running ? 'running' : 'idle';
        document.getElementById('forward').textContent = coords ? fmt(coords.forward) : '--';
        document.getElementById('left').textContent = coords ? fmt(coords.left) : '--';
        document.getElementById('up').textContent = coords ? fmt(coords.up) : '--';
        document.getElementById('targetDepth').textContent =
          target && Number.isFinite(target.depth_cm) ? target.depth_cm.toFixed(1) + ' cm' : '--';
        if (data.target_error && !(data.run && data.run.running)) {
          setStatus('Detection: ' + data.target_error);
        }
        if (data.run) {
          document.getElementById('command').textContent = data.run.command_text || 'waiting...';
          document.getElementById('output').textContent = formatOutput(data.run);
          if (data.run.running) setStatus('Running: ' + (data.run.action || 'command'));
        }
      } catch (err) {
        setStatus('State refresh failed:\\n' + err);
      }
    }

    document.getElementById('stream').onerror = function () {
      setStatus('RGB-D stream failed to load.');
    };
    buildJointGrid();
    refresh();
    setInterval(refresh, 750);
  </script>
</body>
</html>
"""


@dataclass
class ControlConfig:
    arm_script: str
    python_bin: str
    use_sudo: bool
    execute_enabled: bool
    run_log: str


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

    def set_target(self, target, error: Optional[str]) -> None:
        with self.lock:
            self.target = target
            self.target_error = error
            self.target_stamp = time.time()

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


class RunState:
    def __init__(self, log_path: str) -> None:
        self.lock = threading.Lock()
        self.log_path = log_path
        self.proc: Optional[subprocess.Popen] = None
        self.current = None
        self.last = None

    def snapshot(self) -> Optional[dict]:
        with self.lock:
            run = self.current if self.current is not None else self.last
            return None if run is None else dict(run)

    def start(self, action: str, cmd: list, popen_kwargs: dict) -> dict:
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return {
                    "ok": False,
                    "running": True,
                    "error": "another command is already running",
                    "command": cmd,
                    "command_text": " ".join(cmd),
                }
            started_at = time.time()
            try:
                proc = subprocess.Popen(
                    cmd,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    **popen_kwargs,
                )
            except Exception as exc:
                payload = {
                    "ok": False,
                    "running": False,
                    "action": action,
                    "command": cmd,
                    "command_text": " ".join(cmd),
                    "error": str(exc),
                    "started_at": iso_time(started_at),
                    "finished_at": iso_time(time.time()),
                }
                self.last = payload
                self._append_log(payload)
                return payload
            self.proc = proc
            self.current = {
                "ok": True,
                "running": True,
                "action": action,
                "command": cmd,
                "command_text": " ".join(cmd),
                "started_at": iso_time(started_at),
                "started_epoch": started_at,
                "stdout": "",
                "stderr": "",
                "returncode": None,
                "duration_s": None,
            }
            threading.Thread(target=self._waiter, args=(proc,), daemon=True).start()
            return dict(self.current)

    def terminate_current(self) -> Optional[dict]:
        with self.lock:
            proc = self.proc
            if proc is None or proc.poll() is not None:
                return None
            run = self.current
            try:
                proc.terminate()
            except Exception as exc:
                if run is not None:
                    run["terminate_error"] = str(exc)
                return None
            return None if run is None else dict(run)

    def cancel_current(self, timeout: float = 1.0) -> Optional[dict]:
        with self.lock:
            proc = self.proc
            run = self.current
            if proc is None or proc.poll() is not None:
                return None
            command_text = "" if run is None else run.get("command_text", "")
            print("emergency stop: terminating current command:", command_text)
            try:
                proc.terminate()
            except Exception as exc:
                cancelled = {"ok": False, "error": str(exc), "command_text": command_text}
                self.last = cancelled
                return cancelled
        killed = False
        try:
            stdout, stderr = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            killed = True
            proc.kill()
            stdout, stderr = proc.communicate(timeout=timeout)
        finished_at = time.time()
        with self.lock:
            if self.proc is proc:
                started_epoch = finished_at if run is None else run.get("started_epoch", finished_at)
                cancelled = dict(run or {})
                cancelled.update(
                    {
                        "ok": False,
                        "running": False,
                        "cancelled_by_emergency_stop": True,
                        "killed": killed,
                        "returncode": proc.returncode,
                        "duration_s": round(finished_at - started_epoch, 3),
                        "finished_at": iso_time(finished_at),
                        "stdout": stdout[-4000:],
                        "stderr": stderr[-4000:],
                    }
                )
                cancelled.pop("started_epoch", None)
                self.last = cancelled
                self.current = None
                self.proc = None
                self._append_log(cancelled)
                return cancelled
        return None

    def set_last(self, payload: dict) -> None:
        with self.lock:
            self.last = dict(payload)
            self._append_log(payload)

    def _waiter(self, proc: subprocess.Popen) -> None:
        stdout, stderr = proc.communicate()
        finished_at = time.time()
        with self.lock:
            if self.proc is proc:
                run = self.current or {}
                started_epoch = run.get("started_epoch", finished_at)
                run.update(
                    {
                        "ok": proc.returncode == 0,
                        "running": False,
                        "returncode": proc.returncode,
                        "duration_s": round(finished_at - started_epoch, 3),
                        "finished_at": iso_time(finished_at),
                        "stdout": stdout[-8000:],
                        "stderr": stderr[-8000:],
                    }
                )
                run.pop("started_epoch", None)
                self.last = dict(run)
                self.current = None
                self.proc = None
                self._append_log(run)

    def _append_log(self, payload: dict) -> None:
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.log_path)), exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(payload, ensure_ascii=False) + "\n")
        except Exception as exc:
            print("failed to append web run log:", exc)


def iso_time(value: float) -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(value))


def cmd_prefix(cfg: ControlConfig) -> list:
    cmd = []
    if cfg.use_sudo:
        cmd.extend(["sudo", "-n"])
    cmd.extend([cfg.python_bin, cfg.arm_script])
    return cmd


def build_arm_command(cfg: ControlConfig, action: str) -> list:
    if action == "capture-home":
        return cmd_prefix(cfg) + ["capture-home", "--note", "web-control"]
    if action == "capture-clearance":
        return cmd_prefix(cfg) + ["capture-clearance", "--note", "web-control"]
    if action == "table-clearance":
        return cmd_prefix(cfg) + [
            "clearance",
            "--deadband-deg",
            "0.5",
            "--max-delta-deg",
            "120",
            "--step-deg",
            "5",
            "--execute",
        ]
    if action == "home":
        return cmd_prefix(cfg) + [
            "home",
            "--deadband-deg",
            "0.5",
            "--max-delta-deg",
            "120",
            "--execute",
        ]
    raise ValueError("unknown action: " + action)


def build_nudge_command(cfg: ControlConfig, joint: str, delta_deg: float) -> list:
    abs_deg = abs(float(delta_deg))
    seconds = WRIST_NUDGE_SECONDS if joint == "wrist" else NUDGE_SECONDS.get(abs_deg, 2.2)
    return cmd_prefix(cfg) + [
        "nudge-hold",
        "--joint",
        joint,
        "--deg",
        f"{delta_deg:.3f}",
        "--seconds",
        f"{seconds:.2f}",
        "--max-deg",
        "10",
        "--hold-joints",
        ",".join(DEFAULT_JOINTS),
        "--auto-gains",
    ]


def build_emergency_stop_command(cfg: ControlConfig) -> list:
    script_dir = os.path.dirname(os.path.abspath(cfg.arm_script))
    code = (
        "import sys\n"
        f"sys.path.insert(0, {script_dir!r})\n"
        "from left_arm_v2_5_3 import LeftArmV2, DEFAULT_JOINTS\n"
        "arm = LeftArmV2()\n"
        "try:\n"
        "    arm.disable(DEFAULT_JOINTS)\n"
        "    print('emergency stop: disabled all left-arm motors')\n"
        "finally:\n"
        "    arm.close()\n"
    )
    cmd = []
    if cfg.use_sudo:
        cmd.extend(["sudo", "-n"])
    cmd.extend([cfg.python_bin, "-c", code])
    return cmd


def depth_to_meters(depth, scale: float, scale_set: bool):
    if depth is None:
        return None
    if np.issubdtype(depth.dtype, np.floating):
        return depth.astype(np.float32)
    if scale_set:
        return depth.astype(np.float32) * scale
    if depth.dtype == np.uint16:
        return depth.astype(np.float32) * scale
    return depth.astype(np.float32)


def make_depth_vis(depth_m: np.ndarray, min_depth_m: float, max_depth_m: float):
    depth_vis = depth_m.copy()
    depth_vis[~np.isfinite(depth_vis)] = 0
    depth_vis = np.clip(depth_vis, min_depth_m, max_depth_m)
    scale = max(1e-6, max_depth_m - min_depth_m)
    depth_vis = ((depth_vis - min_depth_m) / scale * 255).astype(np.uint8)
    return cv2.applyColorMap(255 - depth_vis, cv2.COLORMAP_JET)


def clamp_roi(roi: Tuple[int, int, int, int], width: int, height: int) -> Optional[Tuple[int, int, int, int]]:
    x0 = max(0, min(width - 1, int(roi[0])))
    y0 = max(0, min(height - 1, int(roi[1])))
    x1 = max(0, min(width - 1, int(roi[2])))
    y1 = max(0, min(height - 1, int(roi[3])))
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def draw_green_roi(frame, roi: Tuple[int, int, int, int]):
    if frame is None:
        return None
    out = frame.copy()
    clamped = clamp_roi(roi, out.shape[1], out.shape[0])
    if clamped is None:
        return out
    x0, y0, x1, y1 = clamped
    cv2.rectangle(out, (x0, y0), (x1, y1), (0, 255, 0), 2)
    return out


def detect_target(rgb, raw_depth, intr: Optional[Intrinsics], cfg: DetectorConfig) -> dict:
    h, w = rgb.shape[:2]
    intr = intr or infer_intrinsics(w, h, cfg.hfov_deg)
    depth_m = hp_depth_to_meters(raw_depth, cfg.depth_scale, cfg.depth_scale_set)
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
        "intrinsics": {"fx": intr.fx, "fy": intr.fy, "cx": intr.cx, "cy": intr.cy},
        "detect": detect_info,
        "controller_strategy": strategy_info,
        "strategy": strategy,
    }


def detector_loop(stream_state: StreamState, cfg: DetectorConfig, hz: float) -> None:
    period = 1.0 / max(0.1, hz)
    while True:
        started = time.time()
        rgb, depth, intr, _rgb_stamp, _depth_stamp, _target, _err, _target_stamp = stream_state.snapshot()
        if rgb is not None and depth is not None:
            try:
                stream_state.set_target(detect_target(rgb, depth, intr, cfg), None)
            except Exception as exc:
                stream_state.set_target(None, str(exc))
        time.sleep(max(0.0, period - (time.time() - started)))


def overlay_target(frame, target, error: Optional[str], roi: Tuple[int, int, int, int]):
    out = draw_green_roi(frame, roi)
    if out is None:
        return None
    if target:
        detect = target.get("detect", {})
        depth_bbox = detect.get("bbox")
        bbox = detect.get("display_bbox") or depth_bbox
        if depth_bbox:
            cv2.rectangle(out, (depth_bbox["x0"], depth_bbox["y0"]), (depth_bbox["x1"], depth_bbox["y1"]), (255, 255, 0), 1)
        if bbox:
            cv2.rectangle(out, (bbox["x0"], bbox["y0"]), (bbox["x1"], bbox["y1"]), (255, 0, 255), 2)
        raw = detect.get("raw_depth_aim_pixel")
        if raw:
            cv2.circle(out, (raw["u"], raw["v"]), 5, (255, 255, 0), 1)
        pixel = target.get("pixel")
        if pixel:
            cv2.circle(out, (pixel["u"], pixel["v"]), 8, (0, 255, 255), 2)
        shoulder = target.get("shoulder_frame_cm", {})
        label = f"F {shoulder.get('forward', 0):.2f} L {shoulder.get('left', 0):.2f} U {shoulder.get('up', 0):.2f} cm"
        cv2.putText(out, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    elif error:
        cv2.putText(out, error[:90], (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 255), 2)
    return out


def compose_rgb_depth(
    rgb,
    depth,
    depth_scale: float,
    depth_scale_set: bool,
    min_depth_m: float,
    max_depth_m: float,
    roi: Tuple[int, int, int, int],
    target=None,
    error: Optional[str] = None,
):
    if rgb is None and depth is None:
        return None
    if rgb is None:
        depth_vis = make_depth_vis(depth_to_meters(depth, depth_scale, depth_scale_set), min_depth_m, max_depth_m)
        return draw_green_roi(depth_vis, roi)
    if depth is None:
        return overlay_target(rgb, target, error, roi)
    depth_vis = make_depth_vis(depth_to_meters(depth, depth_scale, depth_scale_set), min_depth_m, max_depth_m)
    if depth_vis.shape[:2] != rgb.shape[:2]:
        depth_vis = cv2.resize(depth_vis, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
    rgb_vis = overlay_target(rgb, target, error, roi)
    depth_vis = draw_green_roi(depth_vis, roi)
    divider = np.full((rgb.shape[0], 6, 3), 24, dtype=np.uint8)
    return np.hstack([rgb_vis, divider, depth_vis])


def encode_jpeg(frame, quality: int):
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    return buf.tobytes()


def ros_image_to_numpy(msg, desired_encoding: Optional[str] = None):
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
    row_items = int(msg.step // dtype.itemsize)
    arr = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, row_items))
    arr = arr[:, : msg.width * channels]
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


def make_handler(
    stream_state: StreamState,
    run_state: RunState,
    cfg: ControlConfig,
    jpeg_quality: int,
    stream_fps: float,
    depth_scale: float,
    depth_scale_set: bool,
    min_depth_m: float,
    max_depth_m: float,
    roi: Tuple[int, int, int, int],
):
    class Handler(BaseHTTPRequestHandler):
        server_version = "HiveRobotLeftArmWeb/1.0"

        def log_message(self, fmt, *args) -> None:
            message = fmt % args
            if "GET /api/state" in message:
                return
            print("%s - - [%s] %s" % (self.client_address[0], self.log_date_time_string(), message))

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(HTML_PAGE.encode("utf-8"))
                return
            if path == "/api/state":
                self.send_state()
                return
            if path in ("/stream.mjpg", "/rgb.mjpg", "/depth.mjpg"):
                self.send_mjpeg(path)
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:
            path = urlparse(self.path).path
            if path.startswith("/api/action/"):
                action = path.rsplit("/", 1)[-1]
                self.start_action(action)
                return
            if path == "/api/nudge":
                self.start_nudge()
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

        def send_state(self) -> None:
            rgb, depth, _intr, rgb_stamp, depth_stamp, target, err, target_stamp = stream_state.snapshot()
            now = time.time()
            self.send_json(
                {
                    "rgb_age_s": None if rgb is None else round(now - rgb_stamp, 3),
                    "depth_age_s": None if depth is None else round(now - depth_stamp, 3),
                    "target_age_s": None if target is None else round(now - target_stamp, 3),
                    "target_error": err,
                    "target": target,
                    "run": run_state.snapshot(),
                    "execute_enabled": cfg.execute_enabled,
                }
            )

        def start_action(self, action: str) -> None:
            allowed = {"capture-home", "capture-clearance", "table-clearance", "home", "emergency-stop"}
            if action not in allowed:
                self.send_json({"ok": False, "error": "unknown action"}, HTTPStatus.BAD_REQUEST)
                return
            if not cfg.execute_enabled:
                command = build_emergency_stop_command(cfg) if action == "emergency-stop" else build_arm_command(cfg, action)
                self.send_json(
                    {
                        "ok": False,
                        "error": "execution disabled; restart with --enable-execute",
                        "command": command,
                        "command_text": " ".join(command),
                    },
                    HTTPStatus.FORBIDDEN,
                )
                return
            if action == "emergency-stop":
                cmd = build_emergency_stop_command(cfg)
                cancelled = run_state.cancel_current(timeout=1.0)
                started_at = time.time()
                try:
                    completed = subprocess.run(cmd, check=False, text=True, capture_output=True, timeout=6.0)
                    payload = {
                        "ok": completed.returncode == 0,
                        "running": False,
                        "action": action,
                        "command": cmd,
                        "command_text": " ".join(cmd),
                        "started_at": iso_time(started_at),
                        "finished_at": iso_time(time.time()),
                        "returncode": completed.returncode,
                        "duration_s": round(time.time() - started_at, 3),
                        "stdout": completed.stdout[-8000:],
                        "stderr": completed.stderr[-8000:],
                        "cancelled_previous": cancelled,
                    }
                except Exception as exc:
                    payload = {
                        "ok": False,
                        "running": False,
                        "action": action,
                        "command": cmd,
                        "command_text": " ".join(cmd),
                        "started_at": iso_time(started_at),
                        "finished_at": iso_time(time.time()),
                        "error": str(exc),
                        "cancelled_previous": cancelled,
                    }
                run_state.set_last(payload)
                self.send_json(payload, HTTPStatus.OK if payload.get("ok") else HTTPStatus.INTERNAL_SERVER_ERROR)
                return
            else:
                try:
                    cmd = build_arm_command(cfg, action)
                except ValueError as exc:
                    self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
                    return
            payload = run_state.start(action, cmd, popen_kwargs={})
            status = HTTPStatus.OK if payload.get("ok") else HTTPStatus.CONFLICT
            self.send_json(payload, status)

        def read_json_body(self) -> dict:
            length = int(self.headers.get("Content-Length", "0") or "0")
            if length <= 0:
                return {}
            raw = self.rfile.read(length)
            return json.loads(raw.decode("utf-8"))

        def start_nudge(self) -> None:
            try:
                body = self.read_json_body()
                joint = str(body.get("joint", ""))
                direction = 1 if float(body.get("direction", 0)) > 0 else -1
                deg = float(body.get("deg", 0))
            except Exception as exc:
                self.send_json({"ok": False, "error": f"invalid nudge payload: {exc}"}, HTTPStatus.BAD_REQUEST)
                return
            if joint not in DEFAULT_JOINTS:
                self.send_json({"ok": False, "error": "unknown joint"}, HTTPStatus.BAD_REQUEST)
                return
            if deg not in NUDGE_DEGREES:
                self.send_json({"ok": False, "error": "degree must be 2, 5, or 10"}, HTTPStatus.BAD_REQUEST)
                return
            delta_deg = deg * direction
            cmd = build_nudge_command(cfg, joint, delta_deg)
            if not cfg.execute_enabled:
                self.send_json(
                    {
                        "ok": False,
                        "error": "execution disabled; restart with --enable-execute",
                        "command": cmd,
                        "command_text": " ".join(cmd),
                    },
                    HTTPStatus.FORBIDDEN,
                )
                return
            payload = run_state.start(f"nudge:{joint}:{delta_deg:+.0f}deg", cmd, popen_kwargs={})
            status = HTTPStatus.OK if payload.get("ok") else HTTPStatus.CONFLICT
            self.send_json(payload, status)

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
                rgb, depth, _intr, _rgb_stamp, _depth_stamp, target, err, _target_stamp = stream_state.snapshot()
                if path == "/rgb.mjpg":
                    frame = overlay_target(rgb, target, err, roi)
                elif path == "/depth.mjpg":
                    frame = (
                        None
                        if depth is None
                        else draw_green_roi(
                            make_depth_vis(
                                depth_to_meters(depth, depth_scale, depth_scale_set),
                                min_depth_m,
                                max_depth_m,
                            ),
                            roi,
                        )
                    )
                else:
                    frame = compose_rgb_depth(rgb, depth, depth_scale, depth_scale_set, min_depth_m, max_depth_m, roi, target, err)
                if frame is None:
                    time.sleep(0.05)
                    continue
                jpg = encode_jpeg(frame, jpeg_quality)
                if jpg is None:
                    time.sleep(0.05)
                    continue
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
                time.sleep(max(0.0, period - (time.time() - started)))

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument("--stream-fps", type=float, default=15.0)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--arm-script", default=DEFAULT_ARM_SCRIPT)
    parser.add_argument("--python-bin", default="python3")
    parser.add_argument("--no-sudo", action="store_true")
    parser.add_argument("--enable-execute", action="store_true")
    parser.add_argument("--run-log", default=DEFAULT_RUN_LOG)
    parser.add_argument("--rgb-topic", default="/ascamera_hp60c/rgb0/image")
    parser.add_argument("--depth-topic", default="/ascamera_hp60c/depth0/image_raw")
    parser.add_argument("--camera-info-topic", default="/ascamera_hp60c/rgb0/camera_info")
    parser.add_argument("--detect-hz", type=float, default=4.0)
    parser.add_argument("--hfov-deg", type=float, default=73.8)
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--depth-scale-set", action="store_true")
    parser.add_argument("--min-depth-m", type=float, default=0.20)
    parser.add_argument("--max-depth-m", type=float, default=1.20)
    parser.add_argument("--roi-x0", type=int, default=DEFAULT_ROI[0])
    parser.add_argument("--roi-y0", type=int, default=DEFAULT_ROI[1])
    parser.add_argument("--roi-x1", type=int, default=DEFAULT_ROI[2])
    parser.add_argument("--roi-y1", type=int, default=DEFAULT_ROI[3])
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
    parser.add_argument("--camera-forward-from-shoulder-cm", type=float, default=DEFAULT_CAMERA_FORWARD_FROM_SHOULDER_CM)
    parser.add_argument("--camera-left-from-shoulder-cm", type=float, default=DEFAULT_CAMERA_LEFT_FROM_SHOULDER_CM)
    parser.add_argument("--camera-up-from-shoulder-cm", type=float, default=DEFAULT_CAMERA_UP_FROM_SHOULDER_CM)
    parser.add_argument("--camera-pitch-down-deg", type=float, default=DEFAULT_CAMERA_PITCH_DOWN_DEG)
    parser.add_argument("--grasp-strategy", choices=("auto", "normal", "extreme-near-left"), default="auto")
    args = parser.parse_args()

    import rospy
    from sensor_msgs.msg import CameraInfo, Image

    ctrl_cfg = ControlConfig(
        arm_script=os.path.abspath(args.arm_script),
        python_bin=args.python_bin,
        use_sudo=not args.no_sudo,
        execute_enabled=args.enable_execute,
        run_log=args.run_log,
    )
    stream_state = StreamState()
    run_state = RunState(args.run_log)
    convert_image = make_image_converter()
    roi = (args.roi_x0, args.roi_y0, args.roi_x1, args.roi_y1)
    detector_cfg = DetectorConfig(
        hfov_deg=args.hfov_deg,
        depth_scale=args.depth_scale,
        depth_scale_set=args.depth_scale_set,
        roi=roi,
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

    def rgb_cb(msg: Image) -> None:
        stream_state.set_rgb(convert_image(msg, desired_encoding="bgr8"))

    def depth_cb(msg: Image) -> None:
        stream_state.set_depth(convert_image(msg, desired_encoding="passthrough"))

    def info_cb(msg: CameraInfo) -> None:
        k = msg.K
        if k and k[0] > 0 and k[4] > 0:
            stream_state.set_intr(Intrinsics(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5])))

    rospy.init_node("left_arm_v253_web_control", anonymous=True, disable_signals=True)
    rospy.Subscriber(args.rgb_topic, Image, rgb_cb, queue_size=1)
    rospy.Subscriber(args.depth_topic, Image, depth_cb, queue_size=1)
    rospy.Subscriber(args.camera_info_topic, CameraInfo, info_cb, queue_size=1)
    detector = threading.Thread(target=detector_loop, args=(stream_state, detector_cfg, args.detect_hz), daemon=True)
    detector.start()

    handler = make_handler(
        stream_state,
        run_state,
        ctrl_cfg,
        args.jpeg_quality,
        args.stream_fps,
        args.depth_scale,
        args.depth_scale_set,
        args.min_depth_m,
        args.max_depth_m,
        roi,
    )
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"Left arm v2.5.3 web control listening on http://{args.host}:{args.port}/")
    print("Web controller version:", WEB_CONTROLLER_VERSION)
    print("Open from Windows with: http://<orin-ip>:%d/" % args.port)
    print("RGB-D stream: http://<orin-ip>:%d/stream.mjpg" % args.port)
    print("Execute enabled:", ctrl_cfg.execute_enabled)
    print("Arm script:", ctrl_cfg.arm_script)
    print("Run log:", os.path.abspath(args.run_log))
    print("ROI:", roi)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
