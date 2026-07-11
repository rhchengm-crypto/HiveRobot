#!/usr/bin/env python3
"""Probe HP60C camera interfaces on Linux/Jetson.

This script does not control the robot. It only prints useful information:
- /dev/video* devices
- v4l2-ctl capabilities when available
- OpenCV-readable camera indexes and frame shapes
- ROS image topics when rostopic is available
"""

import glob
import os
import subprocess
import sys
from typing import Iterable, List


def run(cmd: List[str]) -> str:
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        return out.strip()
    except Exception as exc:
        return f"<failed: {exc}>"


def print_block(title: str, lines: Iterable[str]) -> None:
    print()
    print("=" * 12, title, "=" * 12)
    for line in lines:
        print(line)


def probe_video_devices() -> None:
    devices = sorted(glob.glob("/dev/video*"))
    if not devices:
        print_block("video devices", ["No /dev/video* devices found."])
        return

    lines = []
    for dev in devices:
        lines.append(dev)
        if shutil_which("v4l2-ctl"):
            lines.append(run(["v4l2-ctl", "-d", dev, "--all"]))
            lines.append(run(["v4l2-ctl", "-d", dev, "--list-formats-ext"]))
    print_block("video devices", lines)


def shutil_which(name: str) -> bool:
    paths = os.environ.get("PATH", "").split(os.pathsep)
    for path in paths:
        candidate = os.path.join(path, name)
        if os.path.exists(candidate) and os.access(candidate, os.X_OK):
            return True
    return False


def probe_opencv(max_index: int) -> None:
    try:
        import cv2
    except Exception as exc:
        print_block("opencv", [f"cv2 import failed: {exc}"])
        return

    lines = []
    for idx in range(max_index + 1):
        cap = cv2.VideoCapture(idx)
        opened = cap.isOpened()
        if not opened:
            cap.release()
            continue

        ok, frame = cap.read()
        if ok and frame is not None:
            lines.append(
                f"index={idx} opened=True shape={frame.shape} dtype={frame.dtype}"
            )
        else:
            lines.append(f"index={idx} opened=True read_failed")
        cap.release()

    if not lines:
        lines = ["No OpenCV-readable camera index found."]
    print_block("opencv cameras", lines)


def probe_ros() -> None:
    if not shutil_which("rostopic"):
        print_block("ros topics", ["rostopic not found. Skip ROS probe."])
        return

    topics = run(["rostopic", "list"])
    if topics.startswith("<failed"):
        print_block("ros topics", [topics])
        return

    image_topics = [
        line
        for line in topics.splitlines()
        if any(key in line.lower() for key in ("image", "depth", "camera", "rgb"))
    ]
    if not image_topics:
        image_topics = ["No camera-like ROS topic found."]
    print_block("ros topics", image_topics)


def main() -> None:
    max_index = 8
    if len(sys.argv) > 1:
        max_index = int(sys.argv[1])

    print("HP60C interface probe")
    print("Run on Orin, for example:")
    print("  python3 hp60c_probe.py")
    probe_video_devices()
    probe_opencv(max_index)
    probe_ros()


if __name__ == "__main__":
    main()

