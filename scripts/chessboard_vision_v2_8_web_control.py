#!/usr/bin/env python3
"""HiveRobot v2.8: integrated chess vision and left-arm web controller.

Reuses the v2.7/v2.6 implementations without changing either source file.
Existing data, calibration, saved moves and model defaults are preserved.
"""
from __future__ import annotations
import os
import argparse
import json
from pathlib import Path
import threading
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

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/crown':
                return self.send_bytes(CROWN_PAGE.encode('utf-8'), 'text/html; charset=utf-8')
            if path in ('/', '/index.html'):
                return self.send_bytes(HTML_PAGE.encode('utf-8'), 'text/html; charset=utf-8')
            if path in ('/arm','/arm/'):
                return self.send_bytes(arm.HTML_PAGE.encode('utf-8'), 'text/html; charset=utf-8')
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
