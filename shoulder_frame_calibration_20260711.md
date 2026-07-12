# Shoulder Frame Calibration Log - 2026-07-11

## Current Control Strategy

Camera target is now converted into `shoulder_frame_cm`:

- `forward`: from shoulder_front axis toward the tabletop target
- `left`: positive toward robot/camera left
- `up`: positive upward from shoulder_front axis

`target-shoulder` currently executes the `legacy-equivalent` target, while calibrated geometry IK is only logged for comparison.

This is intentional. The shoulder-frame coordinate chain is now stable, but pure geometry IK still needs more samples before it should drive the arm directly.

## Current Correction Model

The empirical wrist and shoulder-side corrections have been consolidated into named rule tables in `left_arm_controller.py`.

Wrist correction rules:

```text
wrist_far_x_up_deg              : far x upward correction, currently gain 0 deg
wrist_near_z_up_deg             : near x + rising z upward correction
wrist_mid_z_up_deg              : mid/high z upward correction
wrist_very_low_extra_down_deg   : low z + far x downward correction, from S8
wrist_very_near_high_up_deg     : very near x + high z upward correction, from S11
wrist_far_high_extra_down_deg   : far x + high z downward correction, from S12
```

Shoulder-side correction rules:

```text
side_very_near_t                : very near x outward soft-limit correction, from S10/S11
```

This keeps the current successful behavior but makes future fitting safer: new samples should add or adjust named rules instead of adding one-off formulas inside the motion planner.

## Geometry Calibration Fit - First Pass

Calibration data has been moved into `scripts/geometry_calibration_samples.json`.

Fitting tool:

```bash
python3 scripts/fit_geometry_calibration.py
```

The simple per-joint `raw_geometry * scale + bias` model is not accurate enough:

```text
shoulder_front mean_abs_err ~= 3.0 deg, max ~= 6.5 deg
shoulder_side  mean_abs_err ~= 1.4 deg, max ~= 2.6 deg
elbow          mean_abs_err ~= 4.0 deg, max ~= 9.6 deg
wrist          mean_abs_err ~= 7.2 deg, max ~= 22.0 deg
```

The first usable fit is an affine correction using:

```text
raw geometry offset
shoulder-frame forward
shoulder-frame left
shoulder-frame up
```

Fit quality:

```text
shoulder_front mean_abs_err ~= 0.37 deg, max ~= 1.37 deg
shoulder_side  mean_abs_err ~= 0.67 deg, max ~= 1.24 deg
elbow          mean_abs_err ~= 1.00 deg, max ~= 2.83 deg
wrist          mean_abs_err ~= 1.54 deg, max ~= 3.17 deg
```

This affine geometry calibration is now wired into `calibrated geometry planned targets` logging only. Execution still uses `legacy-equivalent planned targets` until the calibrated geometry targets are validated by dry-run and skip-claw tests.

## Successful Shoulder-Frame Samples

### Sample S1

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 35.16 --left 20.74 --up -38.41 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 35.16 cm
left    = 20.74 cm
up      = -38.41 cm
```

Result:

```text
Full grasp succeeded.
```

Notes:

- Position looked good after switching `target-shoulder` execution to legacy-equivalent targets.
- This sample confirmed that the shoulder-frame coordinate input can drive the existing stable grasp path.

### Sample S2

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 35.65 --left 20.27 --up -38.48 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 35.65 cm
left    = 20.27 cm
up      = -38.48 cm
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.12392742744118057,
  "shoulder_side": -3.094162336190278,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.31592337392057546,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.29094616449089883
}
```

Result:

```text
claw contact detected pos=11.8153 tau=0.3346
safe path stopped after eight motions. Object should be lifted.
Full grasp succeeded.
```

### Sample S3

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 39.61 --left 27.69 --up -38.82 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 39.61 cm
left    = 27.69 cm
up      = -38.82 cm
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.04454985306047832,
  "shoulder_side": -3.0320178490252427,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.2318056554664084,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.0769425405635038
}
```

Result:

```text
claw contact detected pos=11.2995 tau=0.3150
safe path stopped after eight motions. Object should be lifted.
Full grasp succeeded.
```

### Sample S4

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 34.83 --left 22.85 --up -38.21 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 34.83 cm
left    = 22.85 cm
up      = -38.21 cm
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.12392742744118057,
  "shoulder_side": -3.094162336190278,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.31592337392057546,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.2972339390813341
}
```

Result:

```text
claw contact detected pos=11.5288 tau=0.3004
safe path stopped after eight motions. Object should be lifted.
Full grasp succeeded.
```

### Sample S5

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 37.79 --left 21.52 --up -38.78 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 37.79 cm
left    = 21.52 cm
up      = -38.78 cm
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.12392742744118057,
  "shoulder_side": -3.094162336190278,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.3166364097482919,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.24380450777992957
}
```

Result:

```text
claw contact detected pos=11.8442 tau=0.3248
Full grasp succeeded.
```

### Sample S6

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 35.69 --left 20.53 --up -39.31 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 35.69 cm
left    = 20.53 cm
up      = -39.31 cm
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.12392742744118057,
  "shoulder_side": -3.094162336190278,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.31592337392057546,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.2691122364355938
}
```

Result:

```text
claw contact detected pos=11.8137 tau=0.3297
Full grasp succeeded.
```

### Sample S7

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 41.06 --left 23.81 --up -39.13 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 41.06 cm
left    = 23.81 cm
up      = -39.13 cm
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.11226862803785842,
  "shoulder_side": -3.094162336190278,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.3628222979606849,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.04803463540667274
}
```

Result:

```text
claw contact detected pos=11.8133 tau=0.3639
Full grasp succeeded.
```

### Sample S8

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 31.28 --left 22.49 --up -37.95 --execute --skip-claw --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 31.28 cm
left    = 22.49 cm
up      = -37.95 cm
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.12392742744118057,
  "shoulder_side": -3.0897236885339456,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.31592337392057546,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.2972339390813341
}
```

Initial result:

```text
Pre-grasp failed. Wrist needs more downward pressure.
```

Adjustment after S8:

```text
WRIST_LOW_Z_EXTRA_MAX_DEG: 18.0 -> 24.0
```

### Sample S8 Retest

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 31.28 --left 22.49 --up -37.95 --execute --skip-claw --allow-unverified-geometry
```

Result after first wrist adjustment:

```text
wrist_down_offset_deg = -87.04
Pre-grasp still failed. Wrist still needs more downward pressure.
```

Second adjustment after S8 retest:

```text
Added very-low near-target wrist down correction:
WRIST_VERY_LOW_Z_START_CM = 29.0
WRIST_VERY_LOW_Z_FULL_CM = 27.0
WRIST_VERY_LOW_X_START_CM = 12.0
WRIST_VERY_LOW_X_FULL_CM = 14.0
WRIST_VERY_LOW_EXTRA_DOWN_MAX_DEG = 5.0
```

Expected S8 wrist command after second adjustment:

```text
wrist_very_low_extra_down_deg ~= 4.49
wrist_down_offset_deg ~= -91.53, clamped by wrist safety limit near -90 deg
```

Final retest result:

```text
wrist_very_low_extra_down_deg = 4.4931
wrist_down_offset_deg = -91.53
limit clamp wrist requested -1.5975 used -1.5708
claw contact detected pos=11.5585 tau=0.2955
Full grasp succeeded.
```

### Sample S9

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 33.91 --left 29.67 --up -38.14 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 33.91 cm
left    = 29.67 cm
up      = -38.14 cm
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.009992333870990633,
  "shoulder_side": -2.9752305321347223,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.1588437412410858,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.35607517395731203
}
```

Result:

```text
claw contact detected pos=11.2808 tau=0.2857
safe path stopped after eight motions. Object should be lifted.
Full grasp succeeded.
```

### Sample S10

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 41.73 --left 36.36 --up -38.64 --execute --skip-claw --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 41.73 cm
left    = 36.36 cm
up      = -38.64 cm
```

Legacy-equivalent planned targets before adjustment:

```json
{
  "shoulder_front": 0.050605497758252405,
  "shoulder_side": -3.0068958735905618,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.20568816475993779,
  "arm_roll": 2.0243000984191895,
  "wrist": 0.05624003212271811
}
```

Initial result:

```text
Pre-grasp failed. Shoulder_side was too inward.
```

Adjustment after S10:

```text
Added very-near side outward soft limit for legacy arm_x < 4 cm:
SHOULDER_SIDE_VERY_NEAR_X_START_CM = 4.0
SHOULDER_SIDE_VERY_NEAR_X_FULL_CM = 2.0
SHOULDER_SIDE_VERY_NEAR_SOFT_LIMIT_DEG = -10.0
```

Expected S10 side command after adjustment:

```text
side_very_near_t = 1.0
shoulder_side_offset_deg = -10.0
shoulder_side target ~= -2.9545
```

Retest after first side adjustment:

```text
side_very_near_t = 1.0
shoulder_side_offset_deg = -10.0
shoulder_side target = -2.9545
Pre-grasp still failed. Shoulder_side was still too inward.
```

Second adjustment after S10 retest:

```text
SHOULDER_SIDE_VERY_NEAR_SOFT_LIMIT_DEG: -10.0 -> -6.0
Expected shoulder_side target ~= -2.8847
```

Retest after second side adjustment:

```text
side_very_near_t = 1.0
shoulder_side_offset_deg = -6.0
shoulder_side target = -2.8847
Pre-grasp still failed. Shoulder_side still needs to move outward.
```

Third adjustment after S10 retest:

```text
SHOULDER_SIDE_VERY_NEAR_SOFT_LIMIT_DEG: -6.0 -> -2.0
Expected shoulder_side target ~= -2.8149
```

Final retest result:

```text
side_very_near_t = 1.0
shoulder_side_offset_deg = -2.0
shoulder_side target = -2.8149
claw contact detected pos=10.0711 tau=0.2808
Full grasp succeeded.
```

## Observations

- The HP60C pitch correction is necessary. Without camera pitch, `up` was physically impossible.
- `camera_pitch_down_deg = 54.0` produced plausible tabletop targets around `up = -38` to `-43 cm`.
- Pure calibrated geometry IK matched one sample but drifted at other positions.
- Therefore, pure geometry IK should remain logging-only until more samples are collected.
- S8 showed that very low/near targets around legacy `arm_z ~= 27 cm` need more wrist-down range than the earlier 18 degree low-z cap.
- S10 showed that extreme near/left targets around legacy `arm_x ~= 0.6 cm` need a more outward shoulder_side soft limit than the normal near target cap.

## Next Samples Needed

Minimum target: 12 successful/failed shoulder-frame samples.

Good coverage:

```text
forward: 35, 40, 45 cm
left:    15, 20, 25, 30 cm
up:      natural tabletop/object-top values, usually around -38 to -43 cm
```

For each sample, record:

```text
shoulder_frame_cm
legacy-equivalent planned targets
calibrated geometry planned targets
result: success/failure
failure note: side inward/outward, wrist up/down, elbow up/down, forward/back error
```

## S11 - Far Forward / Extreme Left Edge

Shoulder-frame target:

```text
forward = 46.77 cm
left    = 37.05 cm
up      = -39.30 cm
```

Legacy-equivalent target:

```text
x = -0.108 cm
y = 18.0179 cm
z = 37.3997 cm
```

Pre-adjustment execution:

```text
side_very_near_t = 1.0
side_inward_soft_limit_deg = -2.0
shoulder_side_offset_deg = -2.0
wrist_mid_z_up_deg = 24.0
wrist_down_offset_deg = -40.6805
planned wrist = 0.4239 rad
actual wrist pos = 0.4152 rad
```

Result:

```text
Failed pre-grasp with claw skipped.
Failure note: wrist needs to lift upward.
```

Adjustment after S11:

```text
Added very-near/high-z wrist-up correction.
Trigger: arm_x <= 4 cm and arm_z >= 36 cm.
Max correction: +10 deg upward.
Expected S11 wrist_up_extra ~= +7 deg.
Expected S11 planned wrist ~= 0.546 rad.
```

Retest after wrist-up adjustment:

```text
wrist_very_near_high_up_deg = 6.9986
wrist_down_offset_deg = -33.6820
planned wrist = 0.5461 rad
claw contact detected pos = 11.3426
claw contact tau = 0.3346
final wrist pos after lift = 0.7078 rad
```

Final result:

```text
Full grasp succeeded.
This is the observed upper-left reachable limit on the current tabletop.
Keep this sample as a boundary anchor for future correction fitting.
```

## S12 - Far Forward / Mid-Left / High-Z

Shoulder-frame target:

```text
forward = 46.94 cm
left    = 21.65 cm
up      = -44.00 cm
```

Legacy-equivalent target:

```text
x = 15.292 cm
y = 20.6429 cm
z = 41.3020 cm
```

Pre-adjustment execution:

```text
wrist_mid_z_up_deg = 24.0
wrist_very_near_high_up_deg = 0.0
wrist_down_offset_deg = -40.0
planned wrist = 0.4358 rad
actual wrist pos = 0.4274 rad
shoulder_side_offset_deg = -18.0
```

Result:

```text
Failed pre-grasp with claw skipped.
Failure note: wrist needs to press downward.
```

Adjustment after S12:

```text
Added far/high-z wrist-down correction.
Trigger: arm_x >= 14 cm and arm_z >= 40 cm.
Max correction: -10 deg downward.
Expected S12 wrist_down_extra ~= 7.5 deg.
Expected S12 planned wrist ~= 0.305 rad.
```

Retest after far/high-z wrist-down adjustment:

```text
wrist_far_high_extra_down_deg = 7.4765
wrist_down_offset_deg = -47.4765
planned wrist = 0.3053 rad
claw contact detected pos = 11.0824
claw contact tau = 0.3297
final wrist pos after lift = 0.4660 rad
```

Final result:

```text
Full grasp succeeded.
This is the observed upper-right reachable limit for the current target size on the tabletop.
Keep this sample as the opposite top-edge boundary anchor from S11.
```

## S13 - Post-Refactor Validation Sample

Shoulder-frame target:

```text
forward = 36.43 cm
left    = 28.07 cm
up      = -38.65 cm
```

Legacy-equivalent target:

```text
x = 8.872 cm
y = 26.0011 cm
z = 30.7962 cm
```

Rule-table planner output:

```text
wrist_x_delta_deg = 0.16
wrist_extra_down_bias_deg = 14.18
wrist_near_z_up_deg = 2.4904
wrist_down_offset_deg = -78.0966
side_very_near_t = 0.0
shoulder_side_offset_deg = -12.5107
planned wrist = -0.2291 rad
planned shoulder_side = -2.9984 rad
```

Result:

```text
claw contact detected pos = 10.8200
claw contact tau = 0.2906
Full grasp succeeded.
```

Notes:

```text
This is the first full real-world grasp after consolidating wrist and shoulder-side corrections into rule tables.
It validates that the refactor preserved behavior on a newly moved target, not only on dry-run boundary samples.
```
