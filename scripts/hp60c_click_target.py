#!/usr/bin/env python3
"""Click a target in HP60C RGB-D frames and export a first-stage arm target.

This is intentionally simple for early tabletop grasp training:
1. Show RGB image.
2. Click the target center.
3. Read median depth in a small window around the click.
4. Convert pixel + depth to camera XYZ.
5. Apply a provisional camera-to-arm offset.
6. Print JSON that can be copied into left_arm_controller.py target args.

The script supports OpenCV UVC and ROS image topics.
"""

import argparse
import json
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def infer_intrinsics(width: int, height: int, hfov_deg: float) -> Intrinsics:
    hfov_rad = math.radians(hfov_deg)
    fx = (width * 0.5) / math.tan(hfov_rad * 0.5)
    fy = fx
    return Intrinsics(fx=fx, fy=fy, cx=(width - 1) * 0.5, cy=(height - 1) * 0.5)


def pixel_to_camera_cm(
    u: int, v: int, depth_m: float, intr: Intrinsics
) -> Tuple[float, float, float]:
    z_cm = depth_m * 100.0
    x_cm = (u - intr.cx) * depth_m / intr.fx * 100.0
    y_cm = (v - intr.cy) * depth_m / intr.fy * 100.0
    return x_cm, y_cm, z_cm


def robust_depth_m(depth: np.ndarray, u: int, v: int, radius: int, scale: float) -> float:
    h, w = depth.shape[:2]
    x0 = max(0, u - radius)
    x1 = min(w, u + radius + 1)
    y0 = max(0, v - radius)
    y1 = min(h, v + radius + 1)
    patch = depth[y0:y1, x0:x1]

    if patch.ndim == 3:
        patch = patch[:, :, 0]

    vals = patch.astype(np.float32).reshape(-1)
    vals = vals[np.isfinite(vals)]
    vals = vals[vals > 0]
    if vals.size == 0:
        raise ValueError("No valid depth around clicked point.")

    # If depth is already float meters, use scale=1.0. If uint16 mm, use 0.001.
    return float(np.median(vals) * scale)


def provisional_arm_target(
    camera_xyz_cm: Tuple[float, float, float],
    offset_cm: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    return (
        camera_xyz_cm[0] + offset_cm[0],
        camera_xyz_cm[1] + offset_cm[1],
        camera_xyz_cm[2] + offset_cm[2],
    )


def run_controller(controller: str, arm_xyz_cm: Tuple[float, float, float]) -> None:
    cmd = [
        "sudo",
        "python3",
        controller,
        "target",
        "--x",
        f"{arm_xyz_cm[0]:.2f}",
        "--y",
        f"{arm_xyz_cm[1]:.2f}",
        "--z",
        f"{arm_xyz_cm[2]:.2f}",
        "--execute",
    ]
    print("run:", " ".join(cmd))
    subprocess.check_call(cmd)


def opencv_click(args: argparse.Namespace) -> None:
    import cv2

    rgb_cap = cv2.VideoCapture(args.rgb_index)
    if not rgb_cap.isOpened():
        raise RuntimeError(f"Cannot open RGB camera index {args.rgb_index}")

    depth_cap = None
    if args.depth_index is not None:
        depth_cap = cv2.VideoCapture(args.depth_index)
        if not depth_cap.isOpened():
            raise RuntimeError(f"Cannot open depth camera index {args.depth_index}")

    clicked: Optional[Tuple[int, int]] = None

    def on_mouse(event, x, y, flags, param):
        nonlocal clicked
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked = (int(x), int(y))

    cv2.namedWindow("hp60c_rgb")
    cv2.setMouseCallback("hp60c_rgb", on_mouse)

    print("Click target center in RGB window. Press q to quit.")
    rgb = None
    depth = None

    while True:
        ok, frame = rgb_cap.read()
        if ok and frame is not None:
            rgb = frame

        if depth_cap is not None:
            ok_d, dframe = depth_cap.read()
            if ok_d and dframe is not None:
                depth = dframe

        if rgb is None:
            time.sleep(0.02)
            continue

        view = rgb.copy()
        if clicked is not None:
            cv2.circle(view, clicked, 6, (0, 255, 255), 2)
        cv2.imshow("hp60c_rgb", view)

        key = cv2.waitKey(10) & 0xFF
        if key == ord("q"):
            break
        if clicked is not None:
            break

    rgb_cap.release()
    if depth_cap is not None:
        depth_cap.release()
    cv2.destroyAllWindows()

    if clicked is None:
        print("No point clicked.")
        return

    u, v = clicked
    h, w = rgb.shape[:2]

    if depth is None:
        if args.manual_depth_cm is None:
            raise RuntimeError(
                "No depth frame. Provide --depth-index or --manual-depth-cm."
            )
        depth_m = args.manual_depth_cm / 100.0
    else:
        depth_m = robust_depth_m(depth, u, v, args.depth_radius, args.depth_scale)

    intr = Intrinsics(
        fx=args.fx,
        fy=args.fy,
        cx=args.cx,
        cy=args.cy,
    )
    if args.fx <= 0 or args.fy <= 0:
        intr = infer_intrinsics(w, h, args.hfov_deg)

    camera_xyz = pixel_to_camera_cm(u, v, depth_m, intr)
    arm_xyz = provisional_arm_target(
        camera_xyz,
        (args.arm_offset_x, args.arm_offset_y, args.arm_offset_z),
    )

    result = {
        "pixel": {"u": u, "v": v},
        "depth_cm": depth_m * 100.0,
        "camera_cm": {
            "x": camera_xyz[0],
            "y": camera_xyz[1],
            "z": camera_xyz[2],
        },
        "arm_cm": {
            "x": arm_xyz[0],
            "y": arm_xyz[1],
            "z": arm_xyz[2],
        },
        "intrinsics": {
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.cx,
            "cy": intr.cy,
        },
        "note": "arm_cm uses provisional translation offset only; calibrate before autonomous use.",
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))

    print()
    print("Controller command:")
    print(
        "sudo python3 left_arm_controller.py target "
        f"--x {arm_xyz[0]:.2f} --y {arm_xyz[1]:.2f} --z {arm_xyz[2]:.2f} --execute"
    )

    if args.execute:
        run_controller(args.controller, arm_xyz)


def ros_click(args: argparse.Namespace) -> None:
    import cv2
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image

    bridge = CvBridge()
    state = {
        "rgb": None,
        "depth": None,
        "intrinsics": None,
        "clicked": None,
    }

    def rgb_cb(msg: Image) -> None:
        try:
            state["rgb"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            state["rgb"] = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def depth_cb(msg: Image) -> None:
        state["depth"] = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def info_cb(msg: CameraInfo) -> None:
        k = msg.K
        if k and k[0] > 0 and k[4] > 0:
            state["intrinsics"] = Intrinsics(
                fx=float(k[0]),
                fy=float(k[4]),
                cx=float(k[2]),
                cy=float(k[5]),
            )

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["clicked"] = (int(x), int(y))

    rospy.init_node("hp60c_click_target", anonymous=True)
    rospy.Subscriber(args.rgb_topic, Image, rgb_cb, queue_size=1)
    rospy.Subscriber(args.depth_topic, Image, depth_cb, queue_size=1)
    rospy.Subscriber(args.camera_info_topic, CameraInfo, info_cb, queue_size=1)

    if args.u is not None and args.v is not None:
        print("ROS headless mode")
        print("RGB topic:", args.rgb_topic)
        print("Depth topic:", args.depth_topic)
        print("Camera info topic:", args.camera_info_topic)
        print("Waiting for one RGB-D frame...")
        deadline = time.time() + args.wait_seconds
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and time.time() < deadline:
            if state["rgb"] is not None and (
                state["depth"] is not None or args.manual_depth_cm is not None
            ):
                break
            rate.sleep()

        if state["rgb"] is None:
            raise RuntimeError("No RGB frame received before timeout.")
        if args.save_rgb:
            cv2.imwrite(args.save_rgb, state["rgb"])
            print("saved RGB frame:", args.save_rgb)
        state["clicked"] = (int(args.u), int(args.v))
    else:
        if args.save_rgb:
            print("Waiting for one RGB frame to save...")
            deadline = time.time() + args.wait_seconds
            rate = rospy.Rate(30)
            while not rospy.is_shutdown() and time.time() < deadline:
                if state["rgb"] is not None:
                    break
                rate.sleep()
            if state["rgb"] is None:
                raise RuntimeError("No RGB frame received before timeout.")
            cv2.imwrite(args.save_rgb, state["rgb"])
            print("saved RGB frame:", args.save_rgb)
            print("image shape:", state["rgb"].shape)
            print("Re-run with --u PIXEL_X --v PIXEL_Y to read target depth.")
            return

        if args.no_window:
            raise RuntimeError("Use --u and --v in --no-window mode.")

    if state["clicked"] is None:
        cv2.namedWindow("hp60c_ros_rgb")
        cv2.setMouseCallback("hp60c_ros_rgb", on_mouse)

        print("ROS mode")
        print("RGB topic:", args.rgb_topic)
        print("Depth topic:", args.depth_topic)
        print("Camera info topic:", args.camera_info_topic)
        print("Click target center in RGB window. Press q to quit.")

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rgb = state["rgb"]
            if rgb is not None:
                view = rgb.copy()
                clicked = state["clicked"]
                if clicked is not None:
                    cv2.circle(view, clicked, 6, (0, 255, 255), 2)
                cv2.imshow("hp60c_ros_rgb", view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if state["clicked"] is not None:
                break
            rate.sleep()

        cv2.destroyAllWindows()

    if state["clicked"] is None:
        print("No point clicked.")
        return
    if state["rgb"] is None:
        raise RuntimeError("No RGB frame received.")

    u, v = state["clicked"]
    rgb = state["rgb"]
    h, w = rgb.shape[:2]

    depth = state["depth"]
    if depth is None:
        if args.manual_depth_cm is None:
            raise RuntimeError(
                "No depth frame received. Check --depth-topic or provide --manual-depth-cm."
            )
        depth_m = args.manual_depth_cm / 100.0
    else:
        if depth.dtype == np.float32 or depth.dtype == np.float64:
            scale = args.depth_scale if args.depth_scale_set else 1.0
        else:
            scale = args.depth_scale
        depth_m = robust_depth_m(depth, u, v, args.depth_radius, scale)

    intr = state["intrinsics"]
    if intr is None:
        if args.fx > 0 and args.fy > 0 and args.cx >= 0 and args.cy >= 0:
            intr = Intrinsics(fx=args.fx, fy=args.fy, cx=args.cx, cy=args.cy)
        else:
            intr = infer_intrinsics(w, h, args.hfov_deg)
            print("warning: camera_info not received; inferred intrinsics from FOV")

    camera_xyz = pixel_to_camera_cm(u, v, depth_m, intr)
    arm_xyz = provisional_arm_target(
        camera_xyz,
        (args.arm_offset_x, args.arm_offset_y, args.arm_offset_z),
    )

    result = {
        "pixel": {"u": u, "v": v},
        "depth_cm": depth_m * 100.0,
        "camera_cm": {
            "x": camera_xyz[0],
            "y": camera_xyz[1],
            "z": camera_xyz[2],
        },
        "arm_cm": {
            "x": arm_xyz[0],
            "y": arm_xyz[1],
            "z": arm_xyz[2],
        },
        "intrinsics": {
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.cx,
            "cy": intr.cy,
        },
        "topics": {
            "rgb": args.rgb_topic,
            "depth": args.depth_topic,
            "camera_info": args.camera_info_topic,
        },
        "note": "arm_cm uses provisional translation offset only; calibrate before autonomous use.",
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))

    print()
    print("Controller command:")
    print(
        "sudo python3 left_arm_controller.py target "
        f"--x {arm_xyz[0]:.2f} --y {arm_xyz[1]:.2f} --z {arm_xyz[2]:.2f} --execute"
    )

    if args.execute:
        run_controller(args.controller, arm_xyz)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=["opencv", "ros"], default="opencv")
    parser.add_argument("--rgb-index", type=int, default=0)
    parser.add_argument("--depth-index", type=int, default=None)
    parser.add_argument(
        "--depth-scale",
        type=float,
        default=0.001,
        help="uint16 mm -> meters uses 0.001; float meter depth uses 1.0",
    )
    parser.add_argument(
        "--depth-scale-set",
        action="store_true",
        help="force --depth-scale even when ROS depth is float",
    )
    parser.add_argument("--manual-depth-cm", type=float, default=None)
    parser.add_argument("--depth-radius", type=int, default=5)

    parser.add_argument("--rgb-topic", default="/ascamera_hp60c/rgb0/image")
    parser.add_argument("--depth-topic", default="/ascamera_hp60c/depth0/image_raw")
    parser.add_argument(
        "--camera-info-topic", default="/ascamera_hp60c/rgb0/camera_info"
    )

    parser.add_argument("--hfov-deg", type=float, default=73.8)
    parser.add_argument("--fx", type=float, default=-1.0)
    parser.add_argument("--fy", type=float, default=-1.0)
    parser.add_argument("--cx", type=float, default=-1.0)
    parser.add_argument("--cy", type=float, default=-1.0)

    # Provisional offset based on the current tested target:
    # camera=(-1.9, 0.7, 68.0) -> arm=(16.5, 21.6, 40.6).
    parser.add_argument("--arm-offset-x", type=float, default=18.4)
    parser.add_argument("--arm-offset-y", type=float, default=20.9)
    parser.add_argument("--arm-offset-z", type=float, default=-27.4)

    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--controller", default="left_arm_controller.py")
    parser.add_argument("--u", type=int, default=None, help="target pixel x")
    parser.add_argument("--v", type=int, default=None, help="target pixel y")
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--save-rgb", default=None, help="save one RGB frame and exit")
    parser.add_argument("--wait-seconds", type=float, default=5.0)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.backend == "opencv":
        opencv_click(args)
    else:
        ros_click(args)


if __name__ == "__main__":
    main()
