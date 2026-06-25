# HiveRobot Left Arm Test And Training Plan

This package is a lightweight bridge between the current hardware bring-up setup and the Roboparty training/deploy stack.

Roboparty's public repositories are organized around:

- `roboparty_train`: Isaac Lab + RSL-RL training, AMP/BeyondMimic/Parkour, Sim2Sim, and GMR retargeted motion data.
- `roboparty_deploy`: ROS2 deployment, motor SDK, inference launch, zeroing tools, and controller services.

For the current robot state, do not jump directly into whole-body RL. The left arm already works through Damiao USB-CAN and `DM_Control_Python`, while the DM4310 claw is not installed yet. The safest path is:

1. Finish electrical validation.
   - Left arm CAN bus has 120 ohm termination.
   - Five DM4340 joints respond on `/dev/ttyACM0`.
   - DM4310 claw is configured but not enabled until installed.

2. Establish a fixed software home.
   - Use `left_arm_config.json`.
   - Do not treat the current position at every script start as the new zero.
   - Command the arm to configured `home_rad` and keep sending hold frames.

3. Collect safe multimodal demonstrations.
   - Run `collect_multimodal_demo.py` first. It records measured state, commanded targets, depth-camera frames, and hand-camera frames.
   - The CSV stores image filenames and motor data in the same timeline.
   - Later add human teleop or ROS2 joystick data, but keep the same CSV schema.

4. Replay before training.
   - Run `replay_demo.py` on a small speed scale first.
   - Only accept a dataset if replay does not hit joint limits or produce CAN errors.

5. Train a minimal behavior-cloning baseline.
   - Run `train_bc_ridge.py` as a sanity check, not as the final policy.
   - This catches ordering/sign mistakes before moving into Isaac Lab/RSL-RL.
   - After the camera pipeline is stable, switch the policy input from only `(q, dq, tau)` to `(q, dq, tau, depth_image, hand_image)`.

6. Upgrade path toward Roboparty.
   - Convert the CSV joint order to the Roboparty/URDF order.
   - Add the arm and claw to a URDF/MJCF model.
   - Use Roboparty GMR/retargeting if using human motion data.
   - Use Roboparty ROS2 deployment only after the low-level motor mapping is stable.

## Current Motor Map

| Joint | Model | Master ID | Slave ID | Installed |
| --- | --- | ---: | ---: | --- |
| 左肩前摆 | DM4340 | 0x0E | 0x1E | yes |
| 左肩侧摆 | DM4340 | 0x0F | 0x1F | yes |
| 左肩旋转 | DM4340 | 0x10 | 0x20 | yes |
| 左肘 | DM4340 | 0x11 | 0x21 | yes |
| 左臂旋转 | DM4340 | 0x12 | 0x22 | yes |
| 左爪夹持 | DM4310 | 0x18 | 0x28 | yes |

## Claw Calibration

Latest measured claw range:

| State | Motor Position |
| --- | ---: |
| Maximum open / home | `-5.9451056 rad` |
| Maximum close | `7.5548944 rad` |

Use this abstraction in control and training:

```python
gripper = 0.0  # closed
gripper = 1.0  # open

claw_open = -5.9451056
claw_close = 7.5548944
claw_rad = claw_close * (1.0 - gripper) + claw_open * gripper
```

For repeated tests, prefer a small safety margin such as `gripper=0.05` to `0.95`. Use the exact endpoints briefly for calibration and verification.

## Orin Usage

Put this folder under:

```bash
~/hive_robot/DM_Control_Python/roboparty_left_arm_plan
```

Run from `~/hive_robot/DM_Control_Python` so Python can import `DM_CAN.py`:

```bash
cd ~/hive_robot/DM_Control_Python
sudo python3 roboparty_left_arm_plan/collect_scripted_demo.py --out data/left_arm_demo.csv
sudo python3 roboparty_left_arm_plan/collect_multimodal_demo.py --out-dir data/left_arm_mm_demo
sudo python3 roboparty_left_arm_plan/replay_demo.py --csv data/left_arm_demo.csv --speed-scale 0.4
python3 roboparty_left_arm_plan/train_bc_ridge.py --csv data/left_arm_demo.csv --out data/left_arm_bc_model.json
```

## Vision Plan

Use two camera streams:

- Depth camera: fixed on the body or table, used to see object position and arm/object geometry.
- Hand camera: mounted near the wrist or gripper, used for close-range grasp alignment.

The current implementation records RGB frames through OpenCV. If your depth camera exposes true depth through RealSense/Orbbec SDK, keep the same dataset layout and add `.npy` depth frames beside the RGB images later.

Recommended dataset layout:

```text
data/left_arm_mm_demo/
  metadata.json
  samples.csv
  depth_camera/
    000000.jpg
  hand_camera/
    000000.jpg
```

Training stages:

1. No vision: replay and behavior cloning from joint states only.
2. Passive vision: collect synchronized images, but still control with scripted/teleop actions.
3. Visual servo tests: use hand-camera image center error to adjust the DM4310 claw approach pose.
4. Imitation learning: train a policy from `(q, dq, tau, depth_camera, hand_camera)` to next joint target.
5. Sim/RL bridge: export the joint order and camera assumptions into Roboparty/Isaac Lab once the real hardware dataset is stable.

## Safety Defaults

- The claw is disabled by default in `left_arm_config.json`.
- Per-step target changes are clamped during replay.
- The controller keeps sending hold commands until interrupted.
- `Ctrl+C` ramps stiffness down but does not send a hard disable by default.
