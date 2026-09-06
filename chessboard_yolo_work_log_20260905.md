# Chessboard YOLO work log - 2026-09-05

## Context

Today focused on HiveRobot chessboard vision v2.7 and the YOLO chess-piece classifier. The live system runs on the Jetson/Orin project path:

```text
/home/nvidia/hive_robot/DM_Control_Python
```

The Windows workspace is a local mirror used for editing and GitHub sync:

```text
C:\Users\rhche\OneDrive\文档\HiveRobot
```

## Main progress

- Extended chessboard v2.7 piece detection with loadable empty-board depth/RGB baselines.
- Added rank-1 artifact suppression and compact depth rescue logic to reduce false positives near the first rank.
- Added adjacent-square duplicate suppression for whole-board detection, including vertical projection artifacts and low temporal-vote artifacts.
- Added YOLO identity override support so trained YOLO class results can replace geometry-only `unknown_piece` identities.
- Updated the v2.7 web verifier to chain whole-board occupancy into YOLO classification and return `yolo_result`, `yolo_identified_pieces`, and `placement_plan`.
- Added Docker fallback for YOLO inference/training on Jetson using `ultralytics/ultralytics:latest-jetson-jetpack5`.
- Added YOLO training status metadata, dataset counts, latest-model resolution, and a stop endpoint for long-running training.
- Added an empty-board baseline capture endpoint in the web verifier.
- Added a step-by-step YOLO training workflow document at `scripts/CHESSBOARD_YOLO_TRAINING_WORKFLOW.md`.

## Verified inference samples

Two user-confirmed YOLO detections were recorded in:

```text
datasets/chess_pieces_yolo/verified_inference_samples.json
```

Confirmed samples:

| Piece | Square | Confidence | YOLO image |
| --- | --- | ---: | --- |
| `white_king` | `d4` | `0.9287429451942444` | `/tmp/hive_robot_chessboard_vision_v2_7/yolo_detect/whole_board_20260905_200516_119_ce2d0c.jpg` |
| `white_queen` | `d4` | `0.9944300055503845` | `/tmp/hive_robot_chessboard_vision_v2_7/yolo_detect/whole_board_20260905_200629_1908_5f9778.jpg` |

Important note: this JSON was created in the Windows workspace first. For Jetson-side training and live operation, copy it to:

```text
/home/nvidia/hive_robot/DM_Control_Python/datasets/chess_pieces_yolo/verified_inference_samples.json
```

Windows PowerShell copy command:

```powershell
scp "C:\Users\rhche\OneDrive\文档\HiveRobot\datasets\chess_pieces_yolo\verified_inference_samples.json" jetson:/home/nvidia/hive_robot/DM_Control_Python/datasets/chess_pieces_yolo/verified_inference_samples.json
```

## Jetson sync status

Attempted non-interactive SSH sync to `jetson` (`nvidia@192.168.0.199`), but Windows currently has no SSH key/agent configured for passwordless access:

```text
Permission denied (publickey,password).
```

Manual password-based `scp` from PowerShell is still available.

## Next useful steps

- Copy `verified_inference_samples.json` to the Jetson training directory.
- Continue collecting confirmed examples for the remaining chess pieces, especially visually similar classes.
- Use the verified samples as a small audit set when retraining or changing confidence thresholds.
- After each new model, run whole-board detection on single-piece cases and verify `placement_plan` target assignment.
