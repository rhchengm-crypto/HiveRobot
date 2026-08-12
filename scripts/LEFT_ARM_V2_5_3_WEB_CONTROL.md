# Left Arm v2.5.3 Web Control

Browser control panel for `left_arm_v2_5_3.py` with HP60C RGB-D video.

## Start

Run on the Orin where HP60C ROS topics and the left-arm CAN serial device are available:

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 left_arm_v2_5_3_web_control.py --enable-execute
```

Open from another computer:

```text
http://<orin-ip>:8091/
```

The web server defaults to running arm commands directly with unbuffered Python:

```text
python3 -u left_arm_v2_5_3.py ...
```

Only use sudo if the serial device permissions require it:

```bash
python3 left_arm_v2_5_3_web_control.py --enable-execute --sudo
```

## Video

The main image is `/stream.mjpg`, a side-by-side RGB and depth visualization.

Direct streams:

```text
/stream.mjpg  RGB + depth
/rgb.mjpg     RGB only
/depth.mjpg   depth colormap only
```

Default ROS topics:

```text
/ascamera_hp60c/rgb0/image
/ascamera_hp60c/depth0/image_raw
```

## Buttons

| Button | Command |
| --- | --- |
| Capture New Home | `python3 -u left_arm_v2_5_3.py capture-home --note web-control` |
| Capture New Clearance | `python3 -u left_arm_v2_5_3.py capture-clearance --note web-control` |
| Capture Claw Home | `python3 -u left_arm_v2_5_3.py capture-claw-home --note web-control` |
| Table Clearance Move | `python3 -u left_arm_v2_5_3.py clearance --deadband-deg 0.5 --max-delta-deg 120 --step-deg 5 --execute` |
| Home Move | `python3 -u left_arm_v2_5_3.py home --deadband-deg 0.5 --max-delta-deg 120 --prehome-clearance --execute` |
| Claw Home | `python3 -u left_arm_v2_5_3.py claw-home --execute` |
| Claw Close | `python3 -u left_arm_v2_5_3.py claw-close --execute` |
| Emergency Stop | terminates the current web-started command, then directly disables all left-arm and claw motors |

## Debug Panel

The right-side debug panel shows:

- `Debug Command`: exact command triggered by the button.
- `Debug Output`: return code, duration, stdout, stderr, and emergency-stop cancellation details.

Run records are appended to:

```text
scripts/data/left_arm_v2_5_3_web_runs.jsonl
```

## Safety Notes

- Start without `--enable-execute` for a dry UI check. Buttons will show the command but refuse to execute.
- Emergency Stop is software-level motor disable for the left arm; keep hardware power safety available.
- Home Move first returns joints to table clearance one by one in this order: wrist, wrist_side, arm_roll, elbow, shoulder_rotate, shoulder_side, shoulder_front. It then runs the normal home move.
- Claw Close stops on pressure/stall detection and keeps holding the contact position until interrupted. During hold it prints claw status once per second.
- Claw Home can interrupt a running Claw Close hold, then moves the claw back to the captured claw home position.
- Long-running commands are launched in their own process group so Claw Home/Emergency Stop can terminate child processes cleanly.
- The web controller streams command stdout/stderr while a command is running, so long holding actions still update the debug output.
- Only one non-emergency action runs at a time. Emergency Stop can interrupt a web-started action.
