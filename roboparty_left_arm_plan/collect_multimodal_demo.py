import argparse
import csv
import json
import math
import os
import time

import cv2

from damiao_left_arm import LeftArm


def open_camera(camera_cfg):
    cap = cv2.VideoCapture(camera_cfg["device"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(camera_cfg["width"]))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(camera_cfg["height"]))
    cap.set(cv2.CAP_PROP_FPS, int(camera_cfg["fps"]))
    if not cap.isOpened():
        raise RuntimeError(f"camera open failed: {camera_cfg['name']} device={camera_cfg['device']}")
    return cap


def capture_frame(cap, path):
    ok, frame = cap.read()
    if not ok:
        return False
    cv2.imwrite(path, frame)
    return True


def build_header(joint_names):
    header = ["sample", "t", "phase", "active_joint", "depth_image", "hand_image"]
    for name in joint_names:
        header += [f"{name}.q", f"{name}.dq", f"{name}.tau", f"{name}.target_q"]
    return header


def build_row(sample_id, t0, phase, active_joint, depth_path, hand_path, joint_names, state, targets):
    row = [sample_id, time.time() - t0, phase, active_joint, depth_path, hand_path]
    for name in joint_names:
        s = state[name]
        row += [s["q"], s["dq"], s["tau"], targets[name]]
    return row


def main():
    parser = argparse.ArgumentParser(description="Collect left-arm motor data plus depth/hand camera frames.")
    parser.add_argument("--config", default=None)
    parser.add_argument("--out-dir", default="data/left_arm_mm_demo")
    parser.add_argument("--amp", type=float, default=0.08)
    parser.add_argument("--freq", type=float, default=0.18)
    parser.add_argument("--move-time", type=float, default=3.0)
    parser.add_argument("--settle-tol", type=float, default=0.018)
    args = parser.parse_args()

    arm = LeftArm(args.config)
    camera_cfg = arm.config.get("cameras", {})
    depth_cfg = camera_cfg.get("depth_camera", {})
    hand_cfg = camera_cfg.get("hand_camera", {})
    capture_hz = float(camera_cfg.get("capture_hz", 15))

    os.makedirs(args.out_dir, exist_ok=True)
    depth_dir = os.path.join(args.out_dir, "depth_camera")
    hand_dir = os.path.join(args.out_dir, "hand_camera")
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(hand_dir, exist_ok=True)

    depth_cap = open_camera(depth_cfg) if depth_cfg.get("installed", False) else None
    hand_cap = open_camera(hand_cfg) if hand_cfg.get("installed", False) else None

    arm.connect()
    arm.enable_all()
    base = arm.home_targets()
    joint_names = [spec.name for spec in arm.specs]

    metadata = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "config": arm.config,
        "schema": "HiveRobot left arm multimodal demo v1",
        "notes": [
            "depth_image is RGB frame from the depth camera unless a native depth SDK is added later.",
            "hand_image is wrist/gripper camera RGB frame.",
            "target_q values are the MIT position targets used during collection."
        ]
    }
    with open(os.path.join(args.out_dir, "metadata.json"), "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)

    csv_path = os.path.join(args.out_dir, "samples.csv")
    sample_period = 1.0 / max(capture_hz, 1.0)
    sample_id = 0
    t0 = time.time()

    print("move to configured home")
    arm.settle(base, tol=args.settle_tol, timeout=15.0)

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(build_header(joint_names))

        try:
            for active_name in joint_names:
                print("collect:", active_name)
                start = time.time()
                next_sample = time.time()
                last_offset = 0.0

                while time.time() - start < args.move_time:
                    elapsed = time.time() - start
                    targets, last_offset = arm.cosine_sine_target(base, active_name, args.amp, args.freq, elapsed)
                    arm.command_targets(targets, moving_name=active_name)

                    if time.time() >= next_sample:
                        depth_rel = ""
                        hand_rel = ""
                        if depth_cap is not None:
                            depth_rel = f"depth_camera/{sample_id:06d}.jpg"
                            capture_frame(depth_cap, os.path.join(args.out_dir, depth_rel))
                        if hand_cap is not None:
                            hand_rel = f"hand_camera/{sample_id:06d}.jpg"
                            capture_frame(hand_cap, os.path.join(args.out_dir, hand_rel))

                        state = arm.state()
                        writer.writerow(build_row(sample_id, t0, "move", active_name, depth_rel, hand_rel, joint_names, state, targets))
                        sample_id += 1
                        next_sample += sample_period

                    time.sleep(arm.period)

                return_time = 1.5
                return_start = time.time()
                next_sample = time.time()
                while time.time() - return_start < return_time:
                    alpha = min(max((time.time() - return_start) / return_time, 0.0), 1.0)
                    smooth = 0.5 - 0.5 * math.cos(math.pi * alpha)
                    targets = dict(base)
                    targets[active_name] = base[active_name] + last_offset * (1.0 - smooth)
                    arm.command_targets(targets, moving_name=active_name)

                    if time.time() >= next_sample:
                        depth_rel = ""
                        hand_rel = ""
                        if depth_cap is not None:
                            depth_rel = f"depth_camera/{sample_id:06d}.jpg"
                            capture_frame(depth_cap, os.path.join(args.out_dir, depth_rel))
                        if hand_cap is not None:
                            hand_rel = f"hand_camera/{sample_id:06d}.jpg"
                            capture_frame(hand_cap, os.path.join(args.out_dir, hand_rel))

                        state = arm.state()
                        writer.writerow(build_row(sample_id, t0, "return", active_name, depth_rel, hand_rel, joint_names, state, targets))
                        sample_id += 1
                        next_sample += sample_period

                    time.sleep(arm.period)

                arm.settle(base, tol=args.settle_tol, timeout=12.0)

            print("hold at home, Ctrl+C to stop")
            while True:
                arm.command_targets(base)
                time.sleep(arm.period)

        except KeyboardInterrupt:
            print("stop requested")
        finally:
            arm.ramp_down_hold(base)
            if depth_cap is not None:
                depth_cap.release()
            if hand_cap is not None:
                hand_cap.release()
            print("saved", csv_path)


if __name__ == "__main__":
    main()

