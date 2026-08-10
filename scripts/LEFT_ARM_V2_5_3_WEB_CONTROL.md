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

If `sudo -n` is not configured for the arm script, either install the sudoers rule or run the web server in an environment where the configured command can access the serial port. The web server defaults to running arm commands with:

```text
sudo -n python3 left_arm_v2_5_3.py ...
```

To disable sudo wrapping:

```bash
python3 left_arm_v2_5_3_web_control.py --enable-execute --no-sudo
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
| Capture New Home | `sudo -n python3 left_arm_v2_5_3.py capture-home --note web-control` |
| Capture New Clearance | `sudo -n python3 left_arm_v2_5_3.py capture-clearance --note web-control` |
| Table Clearance Move | `sudo -n python3 left_arm_v2_5_3.py clearance --deadband-deg 0.5 --max-delta-deg 120 --step-deg 5 --execute` |
| Home Move | `sudo -n python3 left_arm_v2_5_3.py home --deadband-deg 0.5 --max-delta-deg 120 --execute` |
| Emergency Stop | terminates the current web-started command, then directly disables all left-arm motors |

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
- Only one non-emergency action runs at a time. Emergency Stop can interrupt a web-started action.
