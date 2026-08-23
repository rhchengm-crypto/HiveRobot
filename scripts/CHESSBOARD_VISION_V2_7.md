# Chessboard Vision v2.7

Phase 1 is geometry only. It does not move the arm.

Goal:

```text
camera image -> board homography -> square ROI and center in board millimeters
```

Board model:

```text
board size: 440mm x 440mm
square size: 55mm x 55mm
corner order for calibration: a1, h1, h8, a8
corners are the inner board black-line corners, not the decorative coordinate border
```

Offline helper example only. Current verification should use the live web stream:

```powershell
python scripts/chessboard_vision_v2_7.py calibrate `
  --image C:\Users\rhche\Desktop\debug\chess.jpg `
  --corners "100,428 595,418 520,52 165,58" `
  --output scripts\data\chessboard_vision_v2_7_calibration.json `
  --note "manual corners from chess.jpg"

python scripts/chessboard_vision_v2_7.py inspect-square `
  --image C:\Users\rhche\Desktop\debug\chess.jpg `
  --calibration scripts\data\chessboard_vision_v2_7_calibration.json `
  --square a2,h2,a7,h7 `
  --output C:\Users\rhche\Desktop\debug\chess_v2_7_a2_overlay.jpg
```

The `inspect-square` output reports:

```text
center_mm: target square center in board coordinates
center_px: projected image pixel position
roi_px: projected square polygon in the image
```

Notes:

- The sample corner values are approximate. For real use, click or measure the four inner board black-line corners more carefully.
- The a-file origin is the black line to the right of the left rank numbers, not the far-left decorative edge.
- This phase should validate live frames from the camera; do not use old saved screenshots for current detection tuning.
- Next phase will add piece detection inside the target square ROI.

## Web Verification

Run the v2.7 live-only web verifier on the Orin:

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 scripts/chessboard_vision_v2_7_web_control.py --port 8097
```

Then open from Windows:

```text
http://<orin-ip>:8097/
```

This verifier:

```text
uses live RGB plus live depth for piece detection
accepts four inner board black-line corners in order a1,h1,h8,a8
accepts target squares such as a2,h2,a7,h7, or all for the full board
draws the board grid and target square overlay
returns center_mm, center_px, and roi_px
shows each selected square's four pixel corners for manual convergence
```

It intentionally does not expose arm movement buttons.

Live RGB stream for camera sanity check:

```text
http://<orin-ip>:8097/live-rgb.mjpg
```

Default topics:

```text
RGB:   /ascamera_hp60c/rgb0/image
Depth: /ascamera_hp60c/depth0/image_raw
```

Override them when needed:

```bash
python3 scripts/chessboard_vision_v2_7_web_control.py --port 8097 \
  --rgb-topic /ascamera_hp60c/rgb0/image \
  --depth-topic /ascamera_hp60c/depth0/image_raw
```

The main viewer shows the live RGB MJPEG stream.
Click `Inspect Square` to freeze the current live frame and draw the grid overlay.
Use `a2,h2,a7,h7` when tuning edge-case detection.
The default detection input is `all`, which inspects all 64 squares.
`Detect Whole Board` sets `Squares` to `all` and returns `detected_squares` plus `detected_count`.
`Auto locate board from live frame` is experimental and is off by default.
Leave it off for the current stable manually calibrated grid.
`Detect pieces in selected squares` marks detected piece centers in green and returns `piece_results`.
When valid depth exists for a square, depth is trusted first: a raised depth blob means piece present, and no raised depth blob means empty.
RGB detection is only a fallback when that square does not have enough valid depth samples.
Use `/api/status` or the Live status panel to confirm `has_depth: true`, increasing `depth_seq`, and a small `depth_age_s`.
For a grasp target, set `Squares` to one target square such as `a2` and verify the green point lands on the pawn.
Square corner order is bottom-left, bottom-right, top-right, top-left.
Use `Use Shown Square Corners` to copy the currently projected square corners into the editable calibration box.
After editing those square corner pixels, click `Inspect Square` again; the homography is refit from the outer board corners plus the edited square corners.
Click `Show Input Frame` to return from the overlay snapshot to the live stream.

## 2026-08-21 live piece-detection tuning notes

Starting stable reference with all pawns on ranks 1 and 2:

```text
detected_count=16
detected_squares=a1,b1,c1,d1,e1,f1,g1,h1,a2,b2,c2,d2,e2,f2,g2,h2
temporal_samples=7
```

Initial issue: `a2` and `e2` were occasionally missed. A broad rank-2 fallback was tested, but when rank 2 was cleared it falsely detected the entire second rank because shallow board slope under the constant-depth model looked like low raised pieces. That broad fallback was removed.

Rank-2 depth model change:

```text
rank 1: constant depth model
rank 2-8: local plane depth model
```

This made the cleared second rank clean. Empty-board second-rank samples showed `max_raise_m` around `0.001-0.003m`, while real rank-2 pawn samples have much stronger peaks.

Empty-board baseline captured from live inspection:

```text
empty board
detected_count=1
detected_squares=g1

g1 false positive:
method=depth
confidence=1
area_px=2128.5
bbox_patch_px=[11,49,74,36]
median_raise_m=0.013000011444091797
close_threshold_m=0.008
close_area_px=2228
temporal_votes=6
temporal_samples=7
center_mm=[355.7851257324219,15.022212028503418]
```

This baseline is now encoded in `EMPTY_BOARD_DEPTH_BASELINES`. If a future `g1` candidate matches this shallow full-width bottom-band shape, it is reported as:

```text
detected=false
reason=matches_empty_board_baseline
```

Follow-up with rank 2 occupied showed `e2` and `f2` could still miss as `low_confidence_depth_blob` even though they had strong peak evidence:

```text
e2 missed sample:
area_px=173
bbox_patch_px=[62,20,13,24]
median_raise_m=0.00941622257232666
close_threshold_m=0.008
max_raise_m=0.5365564823150635

f2 missed sample:
area_px=67.5
bbox_patch_px=[57,29,11,13]
median_raise_m=0.015368819236755371
close_threshold_m=0.012
max_raise_m=0.5505222678184509
```

Added a rank-2 compact depth rescue:

```text
rank == 2
area_px between 55 and 520
bbox is compact, not a full-square edge band
median_raise_m is at least 0.008m or close to the active threshold
max_raise_m >= 0.08m
```

Rescued detections are labeled:

```text
method=depth_rank2_compact_rescue
```

Direct checks against the recorded values:

```text
e2_missing -> rescue true
f2_missing -> rescue true
rank2_empty -> rescue false
g1_empty_baseline -> rescue false
g1 baseline matcher -> true
```

Validation run locally:

```text
python -m py_compile scripts/chessboard_vision_v2_7.py scripts/chessboard_vision_v2_7_web_control.py
```

Next live test should confirm that `e2` and `f2` return under `depth_rank2_compact_rescue`, while the empty-board `g1` false positive remains suppressed by `matches_empty_board_baseline`.

## 2026-08-23 piece identity layer

The v2.7 detector now reports piece identity fields for detected squares using a standard starting-position square layout.

Example output:

```text
identified_pieces=a1:white_rook_queen_side,b1:white_knight_queen_side,c1:white_bishop_queen_side,d1:white_queen,e1:white_king,f1:white_bishop_king_side,g1:white_knight_king_side,h1:white_rook_king_side,a2:white_pawn_a,...
```

Each detected `piece_results[square]` now includes:

```text
piece_id
piece_type
color
identity_method
identity_confidence
```

The top-level JSON also includes:

```text
identified_pieces
```

Current limitation:

```text
identity_method=standard_starting_position
```

This means identity is assigned from the square when the board is in the standard start layout. After pieces move, identity should be maintained by game-state tracking or replaced by a trained visual/tag classifier. Unknown occupied non-starting squares are reported as `unknown_piece` instead of being guessed.

## 2026-08-23 YOLO chess-piece training path

The project now uses YOLO as the long-term piece classifier. The fixed class set is:

```text
white_pawn
white_rook
white_knight
white_bishop
white_queen
white_king
black_pawn
black_rook
black_knight
black_bishop
black_queen
black_king
```

For setup games, pieces can be placed on rank 4 and YOLO classifies each visible piece. Same-class pieces do not need unique IDs: any `white_pawn` can be assigned to any open white-pawn starting square.

Dataset config:

```text
datasets/chess_pieces_yolo/data.yaml
```

Initialize the dataset:

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 scripts/chess_piece_yolo_dataset.py init \
  --dataset-dir datasets/chess_pieces_yolo
```

Add a labeled board image. The labels use square/class pairs; the script converts square ROIs into YOLO boxes using the v2.7 board calibration:

```bash
python3 scripts/chess_piece_yolo_dataset.py add-image \
  --image /tmp/chess_rank4_001.jpg \
  --calibration scripts/data/chessboard_vision_v2_7_calibration.json \
  --dataset-dir datasets/chess_pieces_yolo \
  --split train \
  --placements "a4:white_pawn,b4:white_rook,c4:black_king,d4:black_queen"
```

Start the Orin Ultralytics container:

```bash
sudo docker run -it --rm \
  --runtime nvidia \
  --network host \
  --ipc host \
  -v ~/hive_robot:/workspace/hive_robot \
  ultralytics/ultralytics:latest-jetson-jetpack5
```

Train:

```bash
cd /workspace/hive_robot/DM_Control_Python
yolo detect train \
  model=yolo11n.pt \
  data=datasets/chess_pieces_yolo/data.yaml \
  imgsz=960 \
  epochs=120 \
  batch=8 \
  project=runs/chess_piece_yolo \
  name=yolo11n_rank4
```

Run inference and produce a placement plan:

```bash
python3 scripts/chess_piece_yolo_infer.py \
  --model runs/chess_piece_yolo/yolo11n_rank4/weights/best.pt \
  --image /tmp/chess_rank4_test.jpg \
  --calibration scripts/data/chessboard_vision_v2_7_calibration.json \
  --squares a4,b4,c4,d4,e4,f4,g4,h4
```

Output includes:

```text
piece_class_results
placement_plan
```

Example placement plan:

```json
[
  {"pick": "a4", "place": "a2", "piece_class": "white_pawn", "confidence": 0.91},
  {"pick": "b4", "place": "e8", "piece_class": "black_king", "confidence": 0.88}
]
```

## Web YOLO training

The v2.7 web verifier can now collect YOLO samples and start training from the browser.

Start it on the Orin:

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 scripts/chessboard_vision_v2_7_web_control.py --port 8097
```

Open:

```text
http://<orin-ip>:8097/
```

Workflow:

```text
1. Put pieces on rank 4.
2. Enter labels in the YOLO rank-4 labels box, one per line or comma-separated.
3. Click Save YOLO Sample.
4. Repeat with different pieces, positions, lighting, and angles.
5. Set YOLO split to val for validation samples and save some validation images.
6. Click Start YOLO Train.
7. Click YOLO Train Status to watch the log tail.
8. After training, click YOLO Detect Table to classify the current rank-4 pieces.
```

Label examples:

```text
a4:white_pawn
b4:white_rook
c4:black_king
d4:black_queen
```

The web server saves the current live RGB frame, uses the active board calibration to create YOLO labels, and writes them under:

```text
datasets/chess_pieces_yolo
```

Training runs in the background. If `yolo` is available on the host, it uses it directly. Otherwise it launches the Orin Ultralytics Docker image:

```text
ultralytics/ultralytics:latest-jetson-jetpack5
```

Training log:

```text
/tmp/hive_robot_chessboard_vision_v2_7/yolo_train.log
```

The YOLO result table shows:

```text
Square: board square where the piece was recognized
Piece: YOLO class, such as white_pawn or black_king
Conf: YOLO confidence
Place: assigned opening target square
```

The default model path used by the web page is:

```text
runs/chess_piece_yolo/yolo11n_rank4/weights/best.pt
```
