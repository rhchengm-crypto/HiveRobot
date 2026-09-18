#!/usr/bin/env python3
"""HiveRobot v3.0: confirm a C4 vision result before a trained multi-flow.

This module composes the existing v2.8 controller.  It does not modify the
v2.7/v2.8 implementations, trained moves, calibration, or learned data.
"""
from __future__ import annotations

import json
import os
import secrets
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http import HTTPStatus
from http.server import ThreadingHTTPServer
from pathlib import Path

import chessboard_vision_v2_8_web_control as v28


WEB_VERSION = "v3.0-c4-confirmed-piece-routing"
DEFAULT_PORT = 8100
CONFIRMATION_TTL_SECONDS = 120.0
V28_MAKE_HANDLER = v28.make_handler
V30_MOVE_SCRIPT = str(Path(__file__).parent / "left_arm_v3_0_move_library.py")
V3_C4_BISHOP_ACTION = "v3-c4-white-bishop-placement"
V3_C4_KNIGHT_ACTION = "v3-c4-white-knight-place-b1"
V3_C4_ROOK_ACTION = "v3-c4-white-rook-place-a1"


def classify_c4_route(piece_class: str) -> dict | None:
    """Return the existing trained multi-flow selected by a C4 class."""
    normalized = str(piece_class).strip().lower().replace("-", "_")
    piece_type = normalized.rsplit("_", 1)[-1]
    if piece_type == "bishop":
        return {
            "piece_type": "bishop",
            "saved_move": "white_knight_c4",
            "flow_key": "white_bishop_placement",
            "flow_label": "White Bishop Placement",
        }
    if piece_type == "knight":
        return {
            "piece_type": "knight",
            "saved_move": "white_knight_c4",
            "flow_key": "white_knight_place_b1",
            "flow_label": "white knight place_B1",
        }
    if piece_type == "rook":
        return {
            "piece_type": "rook",
            "saved_move": "white_knight_c4",
            "flow_key": "white_rook_place_a1",
            "flow_label": "white rook place_A1",
        }
    return None


class PendingC4Confirmations:
    """Short-lived, one-time server-side confirmations."""

    def __init__(self, ttl_seconds: float = CONFIRMATION_TTL_SECONDS) -> None:
        self.ttl_seconds = float(ttl_seconds)
        self.lock = threading.Lock()
        self.pending: dict[str, dict] = {}

    def issue(self, detection: dict, route: dict) -> tuple[str, float]:
        token = secrets.token_urlsafe(24)
        expires_at = time.time() + self.ttl_seconds
        with self.lock:
            self._discard_expired_locked()
            self.pending[token] = {
                "expires_at": expires_at,
                "detection": dict(detection),
                "route": dict(route),
            }
        return token, expires_at

    def consume(self, token: str) -> dict:
        now = time.time()
        with self.lock:
            self._discard_expired_locked(now)
            record = self.pending.pop(str(token), None)
        if record is None:
            raise ValueError("确认已失效或已使用，请重新识别 C4")
        return record

    def _discard_expired_locked(self, now: float | None = None) -> None:
        current = time.time() if now is None else float(now)
        expired = [token for token, item in self.pending.items()
                   if float(item.get("expires_at", 0.0)) <= current]
        for token in expired:
            self.pending.pop(token, None)


def build_arm_page() -> str:
    page = v28.ARM_HTML_PAGE
    replay_button = '<button type="button" class="primary" onclick="replayMove()">Replay</button>'
    c4_button = replay_button + """
            <button type="button" class="primary" onclick="inspectC4AndConfirm()">
              识别 C4 并确认执行
            </button>"""
    if replay_button not in page:
        raise RuntimeError("v3.0 page injection failed: Replay button was not found")
    page = page.replace(replay_button, c4_button, 1)
    style_end = "</style>"
    modal_css = """
.v3-modal{position:fixed;inset:0;z-index:9999;background:rgba(0,0,0,.76);display:flex;align-items:center;justify-content:center;padding:24px}
.v3-modal[hidden]{display:none}.v3-dialog{width:min(720px,95vw);max-height:94vh;overflow:auto;background:#172430;border:1px solid #7392a8;border-radius:10px;padding:18px;box-shadow:0 20px 60px #000}
.v3-dialog img{display:block;width:100%;max-height:58vh;object-fit:contain;background:#080d11;margin:12px 0}.v3-actions{display:flex;gap:12px;justify-content:flex-end}.v3-result{font-size:18px;line-height:1.5;white-space:pre-line}
""" + style_end
    if style_end not in page:
        raise RuntimeError("v3.0 page injection failed: style end was not found")
    page = page.replace(style_end, modal_css, 1)
    modal = """
<div id="c4ConfirmModal" class="v3-modal" hidden role="dialog" aria-modal="true" aria-labelledby="c4ConfirmTitle">
  <div class="v3-dialog">
    <h2 id="c4ConfirmTitle">确认 C4 棋子</h2>
    <div id="c4ConfirmText" class="v3-result"></div>
    <img id="c4ConfirmImage" alt="C4 YOLO prediction">
    <div class="v3-actions">
      <button type="button" onclick="resolveC4Confirmation(false)">取消，不执行</button>
      <button type="button" class="primary" onclick="resolveC4Confirmation(true)">识别正确，执行机械臂</button>
    </div>
  </div>
</div>
"""
    if "</body>" not in page:
        raise RuntimeError("v3.0 page injection failed: body end was not found")
    page = page.replace("</body>", modal + "</body>", 1)
    script = r"""
    let pendingC4Confirmation = null;

    function resolveC4Confirmation(confirmed) {
      const modal = document.getElementById('c4ConfirmModal');
      modal.hidden = true;
      const pending = pendingC4Confirmation;
      pendingC4Confirmation = null;
      if (pending) pending.resolve(Boolean(confirmed));
    }

    function showC4Confirmation(data) {
      const confidence = Number(data.confidence || 0) * 100;
      document.getElementById('c4ConfirmText').textContent =
        'C4 识别结果：' + data.piece_class + '（' + confidence.toFixed(1) + '%）\n' +
        '确认后将执行：' + data.flow_label + '\n请检查预测框和棋子类别是否正确。';
      const image = document.getElementById('c4ConfirmImage');
      image.src = data.prediction_image_url + (data.prediction_image_url.includes('?') ? '&' : '?') + 'ts=' + Date.now();
      document.getElementById('c4ConfirmModal').hidden = false;
      return new Promise((resolve) => { pendingC4Confirmation = {resolve}; });
    }

    async function inspectC4AndConfirm() {
      if (pendingC4Confirmation) return;
      setStatus('正在识别 C4；此阶段不会启动机械臂。');
      try {
        const response = await fetch('/api/v3/c4/inspect', {
          method: 'POST', cache: 'no-store',
          headers: {'Content-Type': 'application/json'}, body: '{}'
        });
        const data = await response.json();
        document.getElementById('output').textContent = formatOutput(data);
        if (!response.ok || !data.ok) throw new Error(data.error || 'C4 识别失败');
        if (!data.actionable) {
          setStatus('C4 识别为 ' + (data.piece_class || 'none') + '；不是 bishop/knight/rook，机械臂不动作。');
          alert('C4 识别结果：' + (data.piece_class || '未识别到棋子') + '\n当前只处理 bishop、knight 或 rook，机械臂不会动作。');
          return;
        }
        const confirmed = await showC4Confirmation(data);
        if (!confirmed) {
          setStatus('已取消；机械臂未启动。');
          return;
        }
        setStatus('确认完成，正在启动 ' + data.flow_label + '。');
        const execute = await fetch('/api/v3/c4/execute', {
          method: 'POST', cache: 'no-store',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            confirmation_token: data.confirmation_token,
            confirmed: true,
            enable_gate: document.getElementById('enableMotionGate').checked
          })
        });
        const result = await execute.json();
        document.getElementById('command').textContent = result.command_text || JSON.stringify(result.command || [], null, 2);
        document.getElementById('output').textContent = formatOutput(result);
        if (!execute.ok || !result.ok) throw new Error(result.error || '机械臂流程启动失败');
        setStatus('已启动：' + data.flow_label);
      } catch (error) {
        setStatus('C4 自动流程失败：\n' + error);
      }
      refresh();
    }
"""
    marker = "</script>"
    index = page.rfind(marker)
    if index < 0:
        raise RuntimeError("v3.0 page injection failed: script end was not found")
    return page[:index] + script + page[index:]


ARM_HTML_PAGE = build_arm_page()


def make_handler(vision_state, stream_state, run_state, ctrl_cfg, args):
    base_handler = V28_MAKE_HANDLER(
        vision_state, stream_state, run_state, ctrl_cfg, args
    )
    confirmations = PendingC4Confirmations()

    class Handler(base_handler):
        server_version = "HiveRobotV30/1.0"

        def _loopback_json(self, path: str, payload: dict | None = None) -> tuple[int, dict]:
            port = int(self.server.server_address[1])
            url = f"http://127.0.0.1:{port}{path}"
            body = None if payload is None else json.dumps(payload).encode("utf-8")
            request = urllib.request.Request(
                url, data=body,
                headers={"Content-Type": "application/json"} if body is not None else {},
                method="POST" if body is not None else "GET",
            )
            try:
                with urllib.request.urlopen(request, timeout=180) as response:
                    return response.status, json.loads(response.read().decode("utf-8"))
            except urllib.error.HTTPError as exc:
                raw = exc.read().decode("utf-8", errors="replace")
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    data = {"ok": False, "error": raw or str(exc)}
                return exc.code, data

        def inspect_c4(self) -> None:
            config = vision_state.overlay_config
            params = urllib.parse.urlencode({
                "squares": "c4",
                "corners": str(config.get("corners", v28.vision.DEFAULT_CORNERS)),
                "auto_locate": "1" if config.get("auto_locate", False) else "0",
                "square_corners": str(config.get("square_corners", "")),
                "model": str(config.get("yolo_model", "")),
            })
            status, detected = self._loopback_json("/api/yolo/detect?" + params)
            if status >= 400 or not detected.get("ok"):
                return self.send_json(detected, status)
            info = (detected.get("piece_class_results") or {}).get("c4")
            if not isinstance(info, dict):
                return self.send_json({
                    "ok": True,
                    "version": WEB_VERSION,
                    "square": "c4",
                    "piece_class": None,
                    "confidence": 0.0,
                    "actionable": False,
                    "reason": "no_piece_detected_at_c4",
                    "prediction_image_url": detected.get("prediction_image_url"),
                    "detection": detected,
                })
            piece_class = str(info.get("piece_class", "")).strip()
            route = classify_c4_route(piece_class)
            response = {
                "ok": True,
                "version": WEB_VERSION,
                "square": "c4",
                "piece_class": piece_class,
                "confidence": float(info.get("confidence", 0.0)),
                "actionable": route is not None,
                "prediction_image_url": detected.get("prediction_image_url"),
                "detection_info": info,
            }
            if route is None:
                response["reason"] = "unsupported_piece_class"
                return self.send_json(response)
            token, expires_at = confirmations.issue(info, route)
            response.update(route)
            response.update({
                "confirmation_required": True,
                "confirmation_token": token,
                "confirmation_expires_at": expires_at,
                "confirmation_ttl_seconds": CONFIRMATION_TTL_SECONDS,
            })
            return self.send_json(response)

        def execute_confirmed_c4(self) -> None:
            try:
                body = self.read_json_body()
                if body.get("confirmed") is not True:
                    raise ValueError("必须在弹窗中确认识别正确后才能执行")
                enable_gate = body.get("enable_gate") is True
                record = confirmations.consume(str(body.get("confirmation_token", "")))
            except (ValueError, TypeError) as exc:
                return self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            route = record["route"]
            command = v28.arm.build_replay_move_command(ctrl_cfg, route["saved_move"])
            if enable_gate:
                command.append("--enable-gate")
            flow_flags = {
                "bishop": "--white-bishop-placement",
                "knight": "--white-knight-place-b1",
                "rook": "--white-rook-place-a1",
            }
            command.append(flow_flags[route["piece_type"]])
            if not ctrl_cfg.execute_enabled:
                result = {
                    "ok": False,
                    "error": "execution disabled; restart with --enable-execute",
                    "command": command,
                    "command_text": " ".join(command),
                }
                status = HTTPStatus.FORBIDDEN
            else:
                cancelled = run_state.cancel_current(
                    timeout=1.0, reason="v3_confirmed_c4",
                    expected_actions={"low-torque"},
                )
                if cancelled is not None:
                    time.sleep(0.2)
                action_names = {
                    "bishop": V3_C4_BISHOP_ACTION,
                    "knight": V3_C4_KNIGHT_ACTION,
                    "rook": V3_C4_ROOK_ACTION,
                }
                result = run_state.start(
                    action_names[route["piece_type"]],
                    command,
                    popen_kwargs={},
                )
                if cancelled is not None:
                    result["cancelled_previous"] = cancelled
                status = HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT
            result["v3_c4_confirmation"] = {
                "confirmed": True,
                "square": "c4",
                "piece_class": record["detection"].get("piece_class"),
                "confidence": record["detection"].get("confidence"),
                "flow_label": route["flow_label"],
                "gate_enabled": enable_gate,
            }
            return self.send_json(result, status)

        def do_GET(self):
            path = urllib.parse.urlparse(self.path).path
            if path in ("/arm", "/arm/"):
                return self.send_bytes(
                    ARM_HTML_PAGE.encode("utf-8"), "text/html; charset=utf-8"
                )
            return super().do_GET()

        def do_POST(self):
            path = urllib.parse.urlparse(self.path).path
            if path == "/api/v3/c4/inspect":
                return self.inspect_c4()
            if path == "/api/v3/c4/execute":
                return self.execute_confirmed_c4()
            return super().do_POST()

    return Handler


def build_parser():
    parser = v28.build_parser()
    parser.description = "HiveRobot v3.0 confirmed C4 vision-to-motion controller"
    parser.set_defaults(port=DEFAULT_PORT, move_script=V30_MOVE_SCRIPT)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    ctrl_cfg = v28.arm.ControlConfig(
        arm_script=os.path.abspath(args.arm_script),
        move_script=os.path.abspath(args.move_script),
        python_bin=args.python_bin,
        use_sudo=False,
        execute_enabled=args.enable_execute,
        run_log=args.run_log,
        moves_file=os.path.abspath(args.moves_file),
    )
    stream_state = v28.arm.StreamState()
    run_state = v28.FullRunState(args.run_log)
    detector_cfg = v28.arm.DetectorConfig(
        hfov_deg=args.hfov_deg, depth_scale=args.depth_scale,
        depth_scale_set=args.depth_scale_set,
        roi=(args.roi_x0, args.roi_y0, args.roi_x1, args.roi_y1),
        min_depth_m=args.min_depth_m, max_depth_m=args.max_depth_m,
        close_margin_cm=args.close_margin_cm, min_area=args.min_area,
        max_area=args.max_area, depth_radius=args.depth_radius,
        aim_x_frac=args.aim_x_frac, aim_y_frac=args.aim_y_frac,
        target_depth_percentile=args.target_depth_percentile,
        scan_windows=args.scan_windows, window_w=args.window_w,
        window_h=args.window_h, window_step=args.window_step,
        no_rgb_dark_refine=args.no_rgb_dark_refine,
        dark_v_max=args.dark_v_max, dark_min_area=args.dark_min_area,
        dark_max_area=args.dark_max_area,
        arm_offset=(args.arm_offset_x, args.arm_offset_y, args.arm_offset_z),
        camera_forward_cm=args.camera_forward_from_shoulder_cm,
        camera_left_cm=args.camera_left_from_shoulder_cm,
        camera_up_cm=args.camera_up_from_shoulder_cm,
        camera_pitch_down_deg=args.camera_pitch_down_deg,
        grasp_strategy=args.grasp_strategy,
    )
    camera = v28.SharedCamera(stream_state)
    camera_cfg = v28.vision.CameraConfig(
        live_camera=True, rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic, jpeg_quality=args.jpeg_quality,
    )
    state = v28.vision.VisionState(
        args.calibration, args.output_dir, camera, camera_cfg,
        args.yolo_dataset_dir, args.yolo_docker_image, args.overlay_config,
    )
    v28.vision.start_ros_camera(camera, args.rgb_topic, args.depth_topic)
    import rospy
    from sensor_msgs.msg import CameraInfo

    def info_cb(msg):
        k = msg.K
        if k and k[0] > 0 and k[4] > 0:
            stream_state.set_intr(v28.arm.Intrinsics(
                fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5])
            ))

    rospy.Subscriber(args.camera_info_topic, CameraInfo, info_cb, queue_size=1)
    threading.Thread(
        target=v28.arm.detector_loop,
        args=(stream_state, detector_cfg, args.detect_hz), daemon=True,
    ).start()
    server = ThreadingHTTPServer(
        (args.host, args.port),
        make_handler(state, stream_state, run_state, ctrl_cfg, args),
    )
    print(f"{WEB_VERSION}: http://{args.host}:{args.port}/", flush=True)
    print(f"Arm execution enabled: {ctrl_cfg.execute_enabled}; C4 confirmation required: True", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        rospy.signal_shutdown("v3.0 web server stopped")


if __name__ == "__main__":
    main()
