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
