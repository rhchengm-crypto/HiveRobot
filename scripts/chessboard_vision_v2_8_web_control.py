#!/usr/bin/env python3
"""HiveRobot v2.8: integrated chess vision and left-arm web controller.

Reuses the v2.7/v2.6 implementations without changing either source file.
Existing data, calibration, saved moves and model defaults are preserved.
"""
from __future__ import annotations
import os
import argparse
import json
import time
from pathlib import Path
import threading
from http import HTTPStatus
from http.server import ThreadingHTTPServer
from urllib.parse import urlparse
import chessboard_vision_v2_7_web_control as vision
import left_arm_v2_6_web_control as arm
from chess_crown_geometry_v2_8 import GeometryStore, PAGE as CROWN_PAGE

WEB_VERSION = "v2.8-integrated-vision-arm"
V28_MOVE_SCRIPT = str(Path(__file__).parent / 'left_arm_v2_8_move_library.py')
V28_ARM_SCRIPT = str(Path(__file__).parent / 'left_arm_v2_8.py')

HTML_PAGE = """<!doctype html><html lang="zh"><meta charset="utf-8">
<title>HiveRobot v2.8</title><style>
body{margin:0;background:#101821;color:#eee;font:16px system-ui}header{padding:12px;display:flex;gap:12px;align-items:center}button,a{font:inherit;padding:8px;color:inherit}button{background:#28445b;border:1px solid #66849a;cursor:pointer}button[aria-selected=true]{background:#326d58}iframe{border:0;width:100%;height:calc(100vh - 76px)}[hidden]{display:none}a{margin-left:auto}
</style><header><strong>HiveRobot v2.8</strong>
<button id="vision-tab" aria-selected="true" onclick="showPanel('vision')">棋盘识别与训练</button>
<button id="arm-tab" aria-selected="false" onclick="showPanel('arm')">机械臂控制</button>
<button id="crown-tab" aria-selected="false" onclick="showPanel('crown')">棋冠几何示教</button>
<a id="open" href="/vision" target="_blank" rel="noopener">单独打开当前页面</a></header>
<iframe id="vision" src="/vision" title="棋盘识别与训练"></iframe>
<iframe id="arm" src="/arm" title="机械臂控制" hidden></iframe>
<iframe id="crown" src="/crown" title="棋冠几何示教" hidden></iframe>
<script>function showPanel(name){for(const id of ['vision','arm','crown']){document.getElementById(id).hidden=id!==name;document.getElementById(id+'-tab').setAttribute('aria-selected',String(id===name));}document.getElementById('open').href='/'+name;}</script></html>"""


def build_arm_page() -> str:
    """Add the v2.8 replay/claw option without changing the v2.6 page."""
    page = arm.HTML_PAGE
    capture_clearance_button = '          <button type="button" onclick="runAction(\'capture-clearance\')">Capture New Clearance</button>'
    restore_button = capture_clearance_button + "\n" + (
        '          <button type="button" onclick="runAction(\'restore-clearance-history\')">'
        'Restore Pre-Placement1 Clearance Data</button>'
    )
    if capture_clearance_button not in page:
        raise RuntimeError("v2.8 arm page injection failed: Capture New Clearance button was not found")
    page = page.replace(capture_clearance_button, restore_button, 1)
    action_name = "      'capture-clearance': 'Capture New Clearance',"
    restore_action_name = action_name + "\n      'restore-clearance-history': 'Restore Pre-Placement1 Clearance Data',"
    if action_name not in page:
        raise RuntimeError("v2.8 arm page injection failed: action name map was not found")
    page = page.replace(action_name, restore_action_name, 1)
    replay_button = '            <button type="button" class="primary" onclick="replayMove()">Replay</button>'
    replay_controls = replay_button + """
            <label class="inline-option" title="Replay 命令结束后调用现有 Claw Close 压力停止流程">
              <input id="closeClawAfterReplay" type="checkbox"> Replay 后合拢夹爪
            </label>
            <label class="inline-option" title="选择一个多流程动作；none 只执行所选 Saved Move 的 Replay">
              多流程
              <select id="multiFlowSelect">
                <option value="none">none</option>
                <option value="placement1">Placement1：夹取后到 Clearance</option>
                <option value="placement_c4">Placement_C4：夹取后到 Clearance</option>
                <option value="white_bishop_placement">White Bishop Placement</option>
                <option value="white_knight_place_b1">white knight place_B1</option>
              </select>
            </label>"""
    if replay_button not in page:
        raise RuntimeError("v2.8 arm page injection failed: Replay button was not found")
    page = page.replace(replay_button, replay_controls, 1)

    function_start = page.find("    async function replayMove() {")
    function_end = page.find("\n\n    async function copyOutput()", function_start)
    if function_start < 0 or function_end < 0:
        raise RuntimeError("v2.8 arm page injection failed: replayMove function was not found")
    replay_function = """    async function replayMove() {
      const name = document.getElementById('moveSelect').value;
      const closeAfterReplay = document.getElementById('closeClawAfterReplay').checked;
      const multiFlow = document.getElementById('multiFlowSelect').value;
      const placement1 = multiFlow === 'placement1';
      const placementC4 = multiFlow === 'placement_c4';
      const whiteBishopPlacement = multiFlow === 'white_bishop_placement';
      const whiteKnightPlaceB1 = multiFlow === 'white_knight_place_b1';
      if (!name) {
        setStatus('No saved move selected.');
        return;
      }
      const suffix = whiteKnightPlaceB1
        ? '，随后完整执行 Placement_C4，从 Clearance 执行 white_knight_place_b1，最后 Claw Home 张爪'
        : whiteBishopPlacement
        ? '，随后完整执行 Placement1，从 Clearance 执行 White Bishop Placement，最后 Claw Home 张爪'
        : placementC4
        ? '，随后从 C4 夹取并回到 Clearance'
        : placement1
        ? '，随后夹取并回到 Clearance'
        : (closeAfterReplay ? '，随后使用压力停止逻辑合拢夹爪' : '');
      if (!confirm('Replay move via table clearance first: ' + name + suffix + '?')) return;
      try {
        const res = await fetch('/api/move/replay', {
          method: 'POST',
          cache: 'no-store',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ name, placement1, placement_c4: placementC4, white_bishop_placement: whiteBishopPlacement, white_knight_place_b1: whiteKnightPlaceB1 })
        });
        const data = await res.json();
        document.getElementById('command').textContent = data.command_text || JSON.stringify(data.command || [], null, 2);
        document.getElementById('output').textContent = formatOutput(data);
        setStatus(data.ok ? 'Replay running/finished: ' + name : 'Replay failed to start: ' + name);
        if (data.ok && closeAfterReplay && !placement1 && !placementC4 && !whiteBishopPlacement && !whiteKnightPlaceB1) {
          setStatus('Replay running: ' + name + '；结束后将合拢夹爪。');
          const finished = await waitForRunIdle('replay-move:' + name, 300000);
          if (finished) {
            document.getElementById('command').textContent = finished.command_text || document.getElementById('command').textContent;
            document.getElementById('output').textContent = formatOutput(finished);
            setStatus('Replay finished: ' + name + '；正在合拢夹爪。');
            await runAction('claw-close', true);
            return;
          }
          setStatus('Replay wait timed out; claw close was not started: ' + name);
        }
      } catch (err) {
        setStatus('Replay request failed:\\n' + err);
      }
      refresh();
    }"""
    return page[:function_start] + replay_function + page[function_end:]


ARM_HTML_PAGE = build_arm_page()
PLACEMENT1_ACTION = 'placement1:bishop01'
PLACEMENT_C4_ACTION = 'placement-c4:white_knight_c4'
WHITE_BISHOP_PLACEMENT_ACTION = 'white-bishop-placement'
WHITE_KNIGHT_PLACE_B1_ACTION = 'white-knight-place-b1'
CLAW_HOME_INTERRUPT_ACTIONS = {
    'claw-close',
    PLACEMENT1_ACTION,
    PLACEMENT_C4_ACTION,
    WHITE_BISHOP_PLACEMENT_ACTION,
    WHITE_KNIGHT_PLACE_B1_ACTION,
    # Compatibility with the first Placement1 build, which used this name.
    'replay-move:bishop01',
}


def build_parser():
    parser = vision.build_parser()
    parser.description = 'HiveRobot v2.8 integrated chess vision and left arm'
    parser.allow_abbrev = False
    parser.set_defaults(port=8098)
    parser.add_argument('--crown-config', default=str(Path(__file__).parent/'data'/'chess_crown_geometry_v2_8.json'))
    parser.add_argument('--stream-fps', type=float, default=15.0)
    parser.add_argument('--arm-script', default=V28_ARM_SCRIPT)
    parser.add_argument('--move-script', default=V28_MOVE_SCRIPT)
    parser.add_argument('--python-bin', default='python3')
    parser.add_argument('--sudo', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--no-sudo', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--enable-execute', action='store_true')
    parser.add_argument('--run-log', default=arm.DEFAULT_RUN_LOG)
    parser.add_argument('--moves-file', default=arm.DEFAULT_MOVES_FILE)
    parser.add_argument('--camera-info-topic', default='/ascamera_hp60c/rgb0/camera_info')
    parser.add_argument('--detect-hz', type=float, default=4.0)
    parser.add_argument('--hfov-deg', type=float, default=73.8)
    parser.add_argument('--depth-scale', type=float, default=0.001)
    parser.add_argument('--depth-scale-set', action='store_true')
    parser.add_argument('--min-depth-m', type=float, default=0.2)
    parser.add_argument('--max-depth-m', type=float, default=1.2)
    parser.add_argument('--roi-x0', type=int, default=arm.DEFAULT_ROI[0])
    parser.add_argument('--roi-y0', type=int, default=arm.DEFAULT_ROI[1])
    parser.add_argument('--roi-x1', type=int, default=arm.DEFAULT_ROI[2])
    parser.add_argument('--roi-y1', type=int, default=arm.DEFAULT_ROI[3])
    parser.add_argument('--close-margin-cm', type=float, default=1.5)
    parser.add_argument('--min-area', type=int, default=80)
    parser.add_argument('--max-area', type=int, default=8000)
    parser.add_argument('--depth-radius', type=int, default=5)
    parser.add_argument('--aim-x-frac', type=float, default=0.5)
    parser.add_argument('--aim-y-frac', type=float, default=0.35)
    parser.add_argument('--target-depth-percentile', type=float, default=70.0)
    parser.add_argument('--scan-windows', action='store_true')
    parser.add_argument('--window-w', type=int, default=80)
    parser.add_argument('--window-h', type=int, default=85)
    parser.add_argument('--window-step', type=int, default=20)
    parser.add_argument('--no-rgb-dark-refine', action='store_true')
    parser.add_argument('--dark-v-max', type=int, default=95)
    parser.add_argument('--dark-min-area', type=int, default=40)
    parser.add_argument('--dark-max-area', type=int, default=4000)
    parser.add_argument('--arm-offset-x', type=float, default=18.4)
    parser.add_argument('--arm-offset-y', type=float, default=20.9)
    parser.add_argument('--arm-offset-z', type=float, default=-27.4)
    parser.add_argument('--camera-forward-from-shoulder-cm', type=float, default=arm.DEFAULT_CAMERA_FORWARD_FROM_SHOULDER_CM)
    parser.add_argument('--camera-left-from-shoulder-cm', type=float, default=arm.DEFAULT_CAMERA_LEFT_FROM_SHOULDER_CM)
    parser.add_argument('--camera-up-from-shoulder-cm', type=float, default=arm.DEFAULT_CAMERA_UP_FROM_SHOULDER_CM)
    parser.add_argument('--camera-pitch-down-deg', type=float, default=arm.DEFAULT_CAMERA_PITCH_DOWN_DEG)
    parser.add_argument('--grasp-strategy', choices=('auto', 'normal', 'extreme-near-left'), default='auto')
    return parser


class SharedCamera(vision.LiveCameraState):
    """Fan out each converted frame to both consumers; preserve vision metadata."""
    def __init__(self, stream):
        super().__init__()
        self.arm_stream = stream

    def set_rgb(self, frame, metadata=None):
        super().set_rgb(frame, metadata)
        self.arm_stream.set_rgb(frame)

    def set_depth(self, frame, metadata=None):
        super().set_depth(frame, metadata)
        self.arm_stream.set_depth(frame)


def make_handler(vision_state, stream_state, run_state, ctrl_cfg, args):
    geometry = GeometryStore(Path(args.crown_config))
    geometry_lock = threading.Lock()
    vision_handler = vision.make_handler(vision_state)
    arm_handler = arm.make_handler(stream_state, run_state, ctrl_cfg,
        args.jpeg_quality, args.stream_fps, args.depth_scale, args.depth_scale_set,
        args.min_depth_m, args.max_depth_m,
        (args.roi_x0,args.roi_y0,args.roi_x1,args.roi_y1))

    class Handler(vision_handler, arm_handler):
        server_version = 'HiveRobotV28/1.0'

        def send_bytes(self, payload, content_type, status=200):
            if content_type.startswith('text/html'):
                # Legacy subpages return to the vision panel, not a nested shell.
                payload = payload.replace(b'href="/"', b'href="/vision"')
            return vision_handler.send_bytes(self, payload, content_type, status)

        def send_json(self, payload, status=200):
            # Both legacy handlers have the same JSON signature; keep no-store.
            return arm_handler.send_json(self, payload, status)

        def start_action(self, action):
            if action == 'restore-clearance-history':
                cmd = [ctrl_cfg.python_bin, '-u', ctrl_cfg.arm_script, 'restore-clearance-history']
                payload = run_state.start(action, cmd, popen_kwargs={})
                status = HTTPStatus.OK if payload.get('ok') else HTTPStatus.CONFLICT
                return self.send_json(payload, status)
            if action != 'claw-home':
                return arm_handler.start_action(self, action)
            cmd = arm.build_arm_command(ctrl_cfg, action)
            if not ctrl_cfg.execute_enabled:
                return self.send_json({
                    'ok': False,
                    'error': 'execution disabled; restart with --enable-execute',
                    'command': cmd,
                    'command_text': ' '.join(cmd),
                }, HTTPStatus.FORBIDDEN)
            cancelled = run_state.cancel_current(
                timeout=1.0,
                reason='claw_home',
                expected_actions=CLAW_HOME_INTERRUPT_ACTIONS,
            )
            if cancelled is not None:
                time.sleep(0.2)
            payload = run_state.start('claw-home', cmd, popen_kwargs={})
            if cancelled is not None:
                payload['cancelled_previous'] = cancelled
            status = HTTPStatus.OK if payload.get('ok') else HTTPStatus.CONFLICT
            return self.send_json(payload, status)

        def start_replay_move(self):
            try:
                body = self.read_json_body()
                name = str(body.get('name', '')).strip()
                placement1 = body.get('placement1') is True
                placement_c4 = body.get('placement_c4') is True
                white_bishop_placement = body.get('white_bishop_placement') is True
                white_knight_place_b1 = body.get('white_knight_place_b1') is True
            except Exception as exc:
                return self.send_json({'ok': False, 'error': f'invalid replay payload: {exc}'}, HTTPStatus.BAD_REQUEST)
            if not name:
                return self.send_json({'ok': False, 'error': 'move name is required'}, HTTPStatus.BAD_REQUEST)
            if sum((placement1, placement_c4, white_bishop_placement, white_knight_place_b1)) > 1:
                return self.send_json(
                    {'ok': False, 'error': 'select only one multi-step placement flow'},
                    HTTPStatus.BAD_REQUEST,
                )
            if (placement1 or white_bishop_placement) and name.casefold() != 'bishop01':
                return self.send_json(
                    {'ok': False, 'error': 'Placement1 and White Bishop Placement require saved move bishop01'},
                    HTTPStatus.BAD_REQUEST,
                )
            if (placement_c4 or white_knight_place_b1) and name.casefold() != 'white_knight_c4':
                return self.send_json(
                    {'ok': False, 'error': 'Placement_C4 and White Knight Place_B1 require saved move white_knight_c4'},
                    HTTPStatus.BAD_REQUEST,
                )
            cmd = arm.build_replay_move_command(ctrl_cfg, name)
            if white_knight_place_b1:
                cmd.append('--white-knight-place-b1')
            elif white_bishop_placement:
                cmd.append('--white-bishop-placement')
            elif placement1:
                cmd.append('--placement1')
            elif placement_c4:
                cmd.append('--placement-c4')
            if not ctrl_cfg.execute_enabled:
                return self.send_json({
                    'ok': False,
                    'error': 'execution disabled; restart with --enable-execute',
                    'command': cmd,
                    'command_text': ' '.join(cmd),
                }, HTTPStatus.FORBIDDEN)
            cancelled = run_state.cancel_current(
                timeout=1.0,
                reason='replay_move',
                expected_actions={'low-torque'},
            )
            if cancelled is not None:
                time.sleep(0.2)
            action_name = (
                WHITE_KNIGHT_PLACE_B1_ACTION if white_knight_place_b1
                else WHITE_BISHOP_PLACEMENT_ACTION if white_bishop_placement
                else PLACEMENT1_ACTION if placement1
                else PLACEMENT_C4_ACTION if placement_c4
                else f'replay-move:{name}'
            )
            payload = run_state.start(action_name, cmd, popen_kwargs={})
            if cancelled is not None:
                payload['cancelled_previous'] = cancelled
            status = HTTPStatus.OK if payload.get('ok') else HTTPStatus.CONFLICT
            return self.send_json(payload, status)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/crown':
                return self.send_bytes(CROWN_PAGE.encode('utf-8'), 'text/html; charset=utf-8')
            if path in ('/', '/index.html'):
                return self.send_bytes(HTML_PAGE.encode('utf-8'), 'text/html; charset=utf-8')
            if path in ('/arm','/arm/'):
                return self.send_bytes(ARM_HTML_PAGE.encode('utf-8'), 'text/html; charset=utf-8')
            if path in ('/api/state','/api/moves','/stream.mjpg','/rgb.mjpg','/depth.mjpg'):
                return arm_handler.do_GET(self)
            if path in ('/vision','/vision/'):
                original = self.path
                self.path = '/' + ('?' + urlparse(original).query if urlparse(original).query else '')
                try:
                    return vision_handler.do_GET(self)
                finally:
                    self.path = original
            return vision_handler.do_GET(self)

        def do_POST(self):
            path = urlparse(self.path).path
            if path.startswith('/api/crown/'):
                try:
                    size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=100000:raise ValueError('请求大小无效')
                    data=json.loads(self.rfile.read(size))
                    if not isinstance(data,dict):raise ValueError('参数必须是对象')
                    with geometry_lock:
                        result=geometry.action(path.rsplit('/',1)[-1],data)
                    return self.send_json(result)
                except (ValueError,KeyError,TypeError) as exc:
                    return self.send_json({'ok':False,'error':str(exc)},400)
            if path.startswith('/api/action/') or path in ('/api/nudge','/api/move/capture','/api/move/replay'):
                return arm_handler.do_POST(self)
            return vision_handler.do_POST(self)

    return Handler


def main():
    args = build_parser().parse_args()
    ctrl_cfg = arm.ControlConfig(
        arm_script=os.path.abspath(args.arm_script),
        move_script=os.path.abspath(args.move_script),
        python_bin=args.python_bin,
        use_sudo=False,
        execute_enabled=args.enable_execute,
        run_log=args.run_log,
        moves_file=os.path.abspath(args.moves_file),
    )
    stream_state = arm.StreamState()
    run_state = arm.RunState(args.run_log)
    roi = (args.roi_x0, args.roi_y0, args.roi_x1, args.roi_y1)
    detector_cfg = arm.DetectorConfig(
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

    camera = SharedCamera(stream_state)
    camera_cfg = vision.CameraConfig(live_camera=True, rgb_topic=args.rgb_topic,
                                    depth_topic=args.depth_topic, jpeg_quality=args.jpeg_quality)
    state = vision.VisionState(args.calibration, args.output_dir, camera, camera_cfg,
                              args.yolo_dataset_dir, args.yolo_docker_image, args.overlay_config)
    # One ROS initialization and one image subscription per stream.
    vision.start_ros_camera(camera, args.rgb_topic, args.depth_topic)
    import rospy
    from sensor_msgs.msg import CameraInfo
    def info_cb(msg):
        k = msg.K
        if k and k[0] > 0 and k[4] > 0:
            stream_state.set_intr(arm.Intrinsics(fx=float(k[0]), fy=float(k[4]),
                                                cx=float(k[2]), cy=float(k[5])))
    info_subscriber = rospy.Subscriber(args.camera_info_topic, CameraInfo, info_cb, queue_size=1)
    detector = threading.Thread(target=arm.detector_loop,
        args=(stream_state, detector_cfg, args.detect_hz), daemon=True)
    detector.start()
    server = ThreadingHTTPServer((args.host,args.port),
        make_handler(state,stream_state,run_state,ctrl_cfg,args))
    print(f'{WEB_VERSION}: http://{args.host}:{args.port}/', flush=True)
    print(f'Arm execution enabled: {ctrl_cfg.execute_enabled}; sudo: False', flush=True)
    print(f'YOLO data: {args.yolo_dataset_dir}; models: {vision.find_latest_yolo_model_path()}', flush=True)
    print(f'Calibration: {args.calibration}; saved moves: {ctrl_cfg.moves_file}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        rospy.signal_shutdown('v2.8 web server stopped')


if __name__ == '__main__':
    main()
