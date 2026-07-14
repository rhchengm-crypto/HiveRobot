# HP60C Live Browser Stream

Start the HP60C ROS driver first on the Orin:

```bash
cd ~/ascam_ws
source ~/.bashrc
roslaunch ascamera hp60c.launch
```

Keep that terminal open. In a second Orin terminal, run the browser stream server:

```bash
cd ~/hive_robot/DM_Control_Python
source ~/.bashrc
python3 hp60c_stream_server.py \
  --scan-windows \
  --window-w 80 \
  --window-h 85 \
  --window-step 20 \
  --min-area 30 \
  --max-area 6000 \
  --aim-y-frac 0.25 \
  --target-depth-percentile 75 \
  --port 8090
```

Open this from Windows:

```text
http://<orin-ip>:8090/
```

The page shows continuous MJPEG streams:

- `/target.mjpg`: RGB stream with ROI, target box, aim point, and shoulder-frame coordinates.
- `/rgb.mjpg`: raw RGB stream.
- `/depth.mjpg`: depth colormap stream.
- `/state.json`: current detection result and target coordinates.
- `/command.json`: current `left_arm_controller.py target-shoulder` command.

By default, the web page only shows/copies the current controller command. To allow
the page's execute button to actually move the arm, restart the stream server with:

```bash
sudo -v
python3 hp60c_stream_server.py \
  --scan-windows \
  --window-w 80 \
  --window-h 85 \
  --window-step 20 \
  --min-area 30 \
  --max-area 6000 \
  --aim-y-frac 0.25 \
  --target-depth-percentile 75 \
  --port 8090 \
  --enable-controller-execute
```

The execute button sends the latest `shoulder_frame_cm` to:

```bash
sudo -n python3 left_arm_controller.py target-shoulder --forward ... --left ... --up ... \
  --geometry-execution-blend 1.00 --execute --allow-unverified-geometry
```

The `Home` button sends:

```bash
sudo -n python3 left_arm_controller.py home
```

The server uses `sudo -n` so the browser cannot hang waiting for a password. If it reports
that a password is required, run `sudo -v` in the Orin terminal before starting the server,
or configure passwordless sudo for the controller command.

To configure passwordless sudo only for the arm controller:

```bash
cd ~/hive_robot/DM_Control_Python
sudo bash install_hive_robot_sudoers.sh /home/nvidia/hive_robot/DM_Control_Python/left_arm_controller.py
sudo -n python3 /home/nvidia/hive_robot/DM_Control_Python/left_arm_controller.py status
```

After that succeeds, the web page execute and Home buttons can run the controller without
prompting for a password.

## Run Logging

Every web-triggered target execution and Home command is appended to:

```text
data/hp60c_web_runs.jsonl
```

Each line is one JSON record containing the target coordinates, generated controller command,
return code, stdout/stderr, and timing. After a run, click one of:

- `记录完美`
- `记录成功`
- `记录失败`

Those buttons append an annotation record linked to the latest run id. This replaces manually
copying terminal logs into the chat.

Useful commands on Orin:

```bash
tail -n 20 data/hp60c_web_runs.jsonl
cp data/hp60c_web_runs.jsonl ~/hp60c_web_runs_$(date +%Y%m%d_%H%M%S).jsonl
```

This does not write per-frame debug images to disk. Use `hp60c_auto_target.py --save-debug-prefix ...`
only when a still-frame debug snapshot is intentionally needed.
