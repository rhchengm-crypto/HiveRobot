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

## YOLO on Orin

The Orin is JetPack 5 / L4T R35.6.1, so use the Jetson JetPack 5 Ultralytics Docker image.
The verified setup uses Docker with the NVIDIA runtime and host networking so the container can
read the HP60C stream server at `http://127.0.0.1:8090/debug.jpg`.

First confirm the NVIDIA runtime can see Jetson devices:

```bash
sudo docker run --rm --runtime nvidia nvcr.io/nvidia/l4t-base:35.4.1 \
  bash -lc "ls /dev/nvhost* | head"
```

Start YOLO:

```bash
mkdir -p ~/hive_robot/DM_Control_Python/yolo_runs

sudo docker run -it --rm \
  --runtime nvidia \
  --network host \
  --ipc host \
  -v ~/hive_robot:/workspace/hive_robot \
  ultralytics/ultralytics:latest-jetson-jetpack5
```

Inside the container, verify CUDA and YOLO:

```bash
python3 - <<'PY'
import torch
from ultralytics import YOLO

print("torch:", torch.__version__)
print("cuda:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("gpu:", torch.cuda.get_device_name(0))

model = YOLO("yolo11n.pt")
print("YOLO loaded")
PY
```

Expected verified output includes `cuda: True`, `gpu: Orin`, and `YOLO loaded`.

To test the HP60C current frame without reusing a cached `debug.jpg`, save a timestamped image and
predict on that file:

```bash
ts=$(date +%s%3N)
img=/tmp/hp60c_${ts}.jpg
wget -O "$img" "http://127.0.0.1:8090/debug.jpg?t=${ts}"

yolo predict model=yolo11n.pt \
  source="$img" \
  imgsz=1280 \
  conf=0.01 \
  classes=39,76 \
  project=/workspace/hive_robot/DM_Control_Python/yolo_runs \
  name=bottle_scissors_${ts}
```

COCO class ids used during testing:

- `39`: bottle
- `76`: scissors
- `73`: book
- `67`: cell phone

Notes from 2026-07-15 testing:

- `yolo11n.pt` runs on Orin CUDA successfully.
- Built-in `/ultralytics/ultralytics/assets/bus.jpg` detects `4 persons, 1 bus`, confirming the model and GPU are healthy.
- HP60C default COCO detection is unreliable in the overhead worktable view.
- Class-limited detection is usable for quick checks: scissors detected around `0.41-0.44`; bottle was weak around `0.03`.
- For robot use, keep a class whitelist and per-class confidence thresholds. Long term, train a HP60C-specific chess/screw/object model.

## ESP32 USB Serial Test

The ESP32 connected through CH340 appears as `/dev/ttyUSB0`. A simple Arduino sketch was compiled
and uploaded with `arduino-cli`, then verified with `ping -> pong` at 115200 baud.

Useful commands:

```bash
cd ~/hive_robot/esp32_ping/esp32_ping
export PATH="$PWD/bin:$PATH"

arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 .
```

Serial verification:

```bash
python3 - <<'PY'
import serial, time

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
time.sleep(2)
ser.write(b"ping\n")
time.sleep(0.3)
for _ in range(10):
    line = ser.readline()
    if line:
        print(line.decode(errors="replace").rstrip())
ser.close()
PY
```
