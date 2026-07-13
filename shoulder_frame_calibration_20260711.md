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

## S14 - 25% Geometry Blend / Extreme Near-Left Side Feedback

Shoulder-frame target:

```text
forward = 30.00 cm
left    = 37.35 cm
up      = -39.49 cm
```

Legacy-equivalent target:

```text
x = -0.408 cm
y = 31.6968 cm
z = 27.6963 cm
```

25% geometry blend pre-grasp:

```text
legacy shoulder_side = -2.8149
calibrated geometry shoulder_side = -2.6992
blended shoulder_side = -2.7860
```

Result:

```text
Pre-grasp with skip-claw showed shoulder_side needs to move inward slightly.
This means the very-near side correction from S10 should dominate over geometry blend.
```

Adjustment after S14:

```text
When legacy arm_x is in the very-near side region, shoulder_side geometry blend is reduced toward 0.
Other joints still use the requested geometry blend.
Expected S14 shoulder_side target after adjustment: -2.8149
```

## S15 - 25% Geometry Blend / Very-Near Left Full Grasp Success

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 30.77 --left 34.51 --up -37.94 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 30.77 cm
left    = 34.51 cm
up      = -37.94 cm
```

Legacy-equivalent target:

```text
x = 2.432 cm
y = 30.1628 cm
z = 26.8949 cm
```

Calibrated geometry planned targets:

```json
{
  "shoulder_front": 0.06059483494737372,
  "shoulder_side": -2.8943079039892825,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.060591239914950634,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.43686027924088044
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": 0.050605497758252405,
  "shoulder_side": -2.8563786788985706,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.1588437412410858,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.43686027924088044
}
```

Blend:

```text
legacy_weight = 0.75
geometry_weight = 0.25
shoulder_side_geometry_weight = 0.054
```

Blended planned targets:

```json
{
  "shoulder_front": 0.053102832055532734,
  "shoulder_side": -2.8584268570534688,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.134280615909552,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.43686027924088044
}
```

Result:

```text
claw contact detected pos = 10.2996
claw contact tau = 0.3297
safe path step 8: lift grasped object about 5cm
Full grasp succeeded.
```

Final status:

```text
shoulder_front pos = 0.0719
shoulder_side pos = -2.8574
elbow pos = 0.2405
wrist pos = -0.2710
claw pos = 10.2657
```

Conclusion:

```text
This is the first full successful grasp after reducing shoulder_side geometry blend in the very-near left region.
25% geometry blend is usable here when shoulder_side geometry influence is reduced to about 5.4%.
Keep shoulder_side mostly empirical in this region while allowing shoulder_front/elbow/wrist to use the requested blend.
```

## S16 - 25% Geometry Blend / Mid-Near Left Side Too Inward

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 36.90 --left 31.95 --up -39.05 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --skip-claw
```

Shoulder-frame target:

```text
forward = 36.90 cm
left    = 31.95 cm
up      = -39.05 cm
```

Legacy-equivalent target:

```text
x = 4.992 cm
y = 25.8559 cm
z = 31.3960 cm
```

Calibrated geometry planned targets:

```json
{
  "shoulder_front": 0.024937691039455423,
  "shoulder_side": -2.948458866313236,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.14303951638196,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.23545299678058162
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": 0.029801173074480047,
  "shoulder_side": -2.9987186275881546,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.1588437412410858,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.26063612066262953
}
```

Original blend result:

```text
geometry_weight = 0.25
shoulder_side_geometry_weight = 0.25
blended shoulder_side = -2.9862
actual shoulder_side pos = -2.9822
```

Result:

```text
Pre-grasp with skip-claw was almost touching the target.
User feedback: shoulder_side was too inward and nearly scraped the object.
```

Adjustment after S16:

```text
The shoulder_side geometry-blend protection now starts earlier.
For legacy arm_x <= 4cm, shoulder_side geometry weight is 0.
For legacy arm_x >= 8cm, shoulder_side uses the requested geometry blend.
This S16 target should now use about 0.062 geometry weight on shoulder_side instead of 0.25.
Expected shoulder_side target after adjustment is close to -2.9956 rad, much nearer to the empirical target.
```

## S17 - 25% Geometry Blend / Mid-Near Left Still Too Inward

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 36.86 --left 30.97 --up -38.59 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --skip-claw
```

Shoulder-frame target:

```text
forward = 36.86 cm
left    = 30.97 cm
up      = -38.59 cm
```

Legacy-equivalent target:

```text
x = 5.972 cm
y = 25.6179 cm
z = 31.0004 cm
```

Calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.0004123647554900023,
  "shoulder_side": -2.9670661875905866,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.178817216288504,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.20995536099161138
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": 0.012696946404935572,
  "shoulder_side": -3.001685943334898,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.1588437412410858,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.2707462538852059
}
```

Blend result:

```text
geometry_weight = 0.25
shoulder_side_geometry_weight = 0.12325
blended shoulder_side = -2.9974
actual shoulder_side pos = -2.9932
```

Result:

```text
Pre-grasp with skip-claw still nearly scraped the target.
User feedback: shoulder_side is still too inward.
```

Adjustment after S17:

```text
For this mid-near-left area, the empirical side soft limit itself is too inward.
Expand the very-near shoulder_side soft-limit ramp from x=4..2cm to x=8..4cm.
This makes side_very_near_t active around x=6cm and raises the side target outward before geometry blending.
```

## S18 - 25% Geometry Blend / Mid-Near Left Clear But Still Too Inward

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 36.59 --left 30.44 --up -38.54 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --skip-claw
```

Shoulder-frame target:

```text
forward = 36.59 cm
left    = 30.44 cm
up      = -38.54 cm
```

Legacy-equivalent target:

```text
x = 6.502 cm
y = 25.8070 cm
z = 30.8012 cm
```

Blend result:

```text
geometry_weight = 0.25
shoulder_side_geometry_weight = 0.156375
blended shoulder_side = -2.9954
actual shoulder_side pos = -2.9894
```

Result:

```text
Pre-grasp with skip-claw no longer scraped the target.
User feedback: side is still visually too inward and can move outward a little.
```

Adjustment after S18:

```text
Add a narrow mid-near-left outward side correction for arm_x from 4cm to 8cm.
The correction peaks at x=6cm with +2deg outward and fades to 0deg at x<=4cm and x>=8cm.
For S18 at x=6.502cm, expected outward correction is about +1.5deg.
This should raise shoulder_side from about -2.995rad toward about -2.969rad.
```

## S19 - 25% Geometry Blend / Mid-Near Left Skip-Claw Grasp Pose OK

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 36.54 --left 30.32 --up -38.45 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --skip-claw
```

Shoulder-frame target:

```text
forward = 36.54 cm
left    = 30.32 cm
up      = -38.45 cm
```

Legacy-equivalent target:

```text
x = 6.622 cm
y = 25.7945 cm
z = 30.6990 cm
```

Coarse lateral after S18 correction:

```text
side_very_near_t = 0.3445
side_inward_soft_limit_deg = -9.2105
side_mid_near_outward_deg = 1.3780
shoulder_side_offset_deg = -9.2105
```

Planned targets:

```json
{
  "shoulder_front": -0.002156500239036102,
  "shoulder_side": -2.9465832618175782,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.16761230718537787,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.2606054048251118
}
```

Final skip-claw pose:

```text
shoulder_front pos = 0.0250
shoulder_side pos = -2.9425
elbow pos = -0.1864
wrist pos = -0.2546
```

Result:

```text
User feedback: this position is acceptable and can grasp.
The mid-near-left outward correction fixed the previous near-scrape side placement.
```

## S20 - Extreme Near-Left Path Semantics Correction

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 24.87 --left 35.77 --up -37.16 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --skip-claw --pre-claw-front-lift-deg 4 --hold-wrist-lift
```

Observed behavior:

```text
safe path step 6: wrist down test angle
safe path step 6b: shoulder_front lift before claw; wrist holds front_lift_deg= 4.0
wrist pos = -0.4301
```

Result:

```text
User feedback: this was still the old motion path because the wrist moved down before the front-lift step.
The intended extreme test is: reach the safe/side pose, keep wrist angle unchanged, then lift shoulder_front only before claw close.
```

Adjustment after S20:

```text
When --hold-wrist-lift is used together with --pre-claw-front-lift-deg, step 6 now skips wrist-down entirely.
Expected log: "safe path step 6: skip wrist down; wrist holds current safe angle".
Then step 6b lifts shoulder_front only while wrist remains at the safe/side-pose angle.
```

## S21 - Extreme Near-Left Front-Lift Must Start From Safe Angle

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 24.87 --left 35.77 --up -37.16 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --skip-claw --pre-claw-front-lift-deg 4 --hold-wrist-lift
```

Observed behavior after S20:

```text
safe path step 6: skip wrist down; wrist holds current safe angle
safe path step 6b: shoulder_front lift before claw; wrist holds front_lift_deg= 4.0
shoulder_front pos = 0.1516
wrist pos = 1.1130
```

Result:

```text
Wrist correctly stayed at the safe angle, but shoulder_front still followed the old path first:
step 3 moved shoulder_front to the planned grasp angle, then step 6b added another lift.
User feedback: the front lift should start from the safe angle and only needs about 5-10deg.
```

Adjustment after S21:

```text
When --hold-wrist-lift is combined with --pre-claw-front-lift-deg, step 3 now keeps shoulder_front at the safe angle.
Expected log: "safe path step 3: shoulder_front holds safe angle for front-lift test".
Then step 6b adds only the requested front lift from that safe angle.
```

## S22 - Extreme Near-Left Front-Lift Direction And Elbow Hold

User feedback after S21:

```text
The command still did not compute the desired limit grasp pose.
After reaching the safe angle, the elbow should not move to the old planned elbow target.
The wrist should stay unchanged.
The shoulder_front only needs to move forward about 5-10deg from the safe angle.
```

Observed problem:

```text
The previous implementation added +4deg to shoulder_front.
On this arm, forward shoulder_front motion for this limit test is the negative direction.
The previous implementation also still ran step 4 to planned elbow.
```

Adjustment after S22:

```text
In front-lift-from-safe mode, step 4 now holds the elbow at its safe angle.
Step 6 still skips wrist-down.
Step 6b now applies shoulder_front -= pre_claw_front_lift_deg.
Expected log includes: "sign= forward_negative".
```

## S23 - Auto Front-Lift Calculation For Extreme Near-Left

User feedback:

```text
Manual 8deg front-lift was still not enough.
The front-lift angle should be computed from the target position instead of being guessed visually.
```

Adjustment after S23:

```text
Add --auto-pre-claw-front-lift for target-shoulder.
The auto lift uses legacy arm_x and shoulder-frame forward.
For extreme near-left targets, front lift ramps from 8deg to 18deg.
Current sample:
  legacy arm_x = 1.172cm
  shoulder forward = 24.87cm
  computed front_lift_deg ~= 17.4deg
The value is capped at 18deg so it remains far smaller than the old full planned shoulder_front move.
```

## S24 - Auto Limit Pose Must Include Elbow

User feedback after S23:

```text
The auto 17.4deg front lift was not enough by itself.
The extreme near-left pose also needs elbow calculation, not only shoulder_front.
```

Adjustment after S24:

```text
Auto pre-claw limit pose now computes:
  front_lift_deg from legacy arm_x and shoulder forward
  elbow_drop_deg from legacy arm_x and shoulder forward

For the current sample:
  legacy arm_x = 1.172cm
  shoulder forward = 24.87cm
  front_lift_deg ~= 17.4deg
  elbow_drop_deg ~= 26.9deg

The elbow drop is applied after side placement and before front lift, while wrist remains unchanged.
Expected log:
  safe path step 6a: elbow calculated drop before claw; wrist holds
  safe path step 6b: shoulder_front forward lift before claw; wrist holds
```

## S25 - Auto Limit Pose Should Be Shoulder-Dominant

User feedback after S24:

```text
The front swing still did not reach the needed height, and the elbow moved first.
The limit pose should be computed so shoulder_front reaches the main approach height first,
then elbow provides only the remaining correction.
```

Adjustment after S25:

```text
Auto front lift now uses the actual angular gap between the safe shoulder_front angle and the planned shoulder_front target.
It applies 72% of that gap, capped at 70deg.
For the current sample, this should compute about 53deg instead of 17deg.

Execution order changed:
  step 6a: shoulder_front calculated forward lift first
  step 6b: elbow calculated drop second
Wrist remains unchanged.
```

## S26 - Auto Limit Front Swing Overreached

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 24.87 --left 35.77 --up -37.16 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --skip-claw --auto-pre-claw-front-lift --hold-wrist-lift
```

Observed auto output:

```text
front_lift_deg = 66.6747
elbow_drop_deg = 16.9680
```

Final skip-claw pose:

```text
shoulder_front pos = 0.2195
shoulder_side pos = -2.8140
elbow pos = 0.3706
wrist pos = 1.1152
```

Result:

```text
User feedback: shoulder_front moved too far forward.
```

Adjustment after S26:

```text
The previous shoulder-front target-gap fraction 0.72 was too aggressive.
Reduce LIMIT_FRONT_LIFT_TARGET_FRACTION from 0.72 to 0.48.
Reduce LIMIT_FRONT_LIFT_MAX_DEG from 70deg to 48deg.
For this sample, expected front_lift_deg is now about 44.4deg instead of 66.7deg.
Expected shoulder_front final target moves back from about 0.22rad toward about 0.61rad.
```

## S27 - Auto Limit Front Swing Still Too Large At 44deg

User feedback:

```text
The recalculated 44.4deg front lift is still too large.
```

Adjustment after S27:

```text
Reduce LIMIT_FRONT_LIFT_TARGET_FRACTION from 0.48 to 0.32.
Reduce LIMIT_FRONT_LIFT_MAX_DEG from 48deg to 34deg.
For this sample, expected front_lift_deg is now about 32.8deg.
This keeps the auto value above the known-insufficient 17deg test, but well below the too-large 44-66deg range.
```

## S28 - Auto Limit Front Swing Still Too Large At 32deg

User feedback:

```text
The recalculated 32.8deg front lift is still too large.
```

Adjustment after S28:

```text
Reduce LIMIT_FRONT_LIFT_TARGET_FRACTION from 0.32 to 0.24.
Reduce LIMIT_FRONT_LIFT_MAX_DEG from 34deg to 25deg.
For this sample, expected front_lift_deg is now about 24-25deg.
This keeps the value above the known-insufficient 17deg test while staying well below the too-large 32-66deg range.
```

## S29 - Auto Limit Front Swing Still Too Large At 24deg

User feedback:

```text
The recalculated 24-25deg front lift is still too large.
```

Adjustment after S29:

```text
Reduce LIMIT_FRONT_LIFT_TARGET_FRACTION from 0.24 to 0.20.
Reduce LIMIT_FRONT_LIFT_MAX_DEG from 25deg to 21deg.
For this sample, expected front_lift_deg is now about 20-21deg.
This narrows the current bracket between the known-insufficient 17deg test and the too-large 24deg test.
```

## S30 - Auto Limit Front Swing Reduced Slightly

User feedback:

```text
Reduce the 20-21deg front lift slightly.
```

Adjustment after S30:

```text
Reduce LIMIT_FRONT_LIFT_TARGET_FRACTION from 0.20 to 0.18.
Reduce LIMIT_FRONT_LIFT_MAX_DEG from 21deg to 19deg.
For this sample, expected front_lift_deg is now about 18-19deg.
```

## S31 - Elbow Slightly Higher After Front Swing Accepted

User feedback:

```text
The front swing angle is acceptable.
Raise the elbow slightly, but not too much.
```

Adjustment after S31:

```text
Keep the current front swing constants unchanged.
Reduce LIMIT_ELBOW_DROP_MAX_DEG from 18deg to 14deg.
For this sample, expected elbow_drop_deg changes from about 17deg to about 13deg.
This keeps the elbow slightly higher while preserving the same wrist-hold limit path.
```

## S32 - Front Swing Slightly Smaller, Elbow Accepted

User feedback:

```text
The elbow angle is acceptable.
Reduce the front swing slightly.
```

Adjustment after S32:

```text
Keep LIMIT_ELBOW_DROP_MAX_DEG at 14deg.
Reduce LIMIT_FRONT_LIFT_TARGET_FRACTION from 0.18 to 0.17.
Reduce LIMIT_FRONT_LIFT_MAX_DEG from 19deg to 18deg.
For this sample, expected front_lift_deg is now about 17-18deg.
```

## S33 - Extreme Near-Left Full Grasp Success, Post-Lift Limit Error

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 24.87 --left 35.77 --up -37.16 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --auto-pre-claw-front-lift --hold-wrist-lift
```

Auto limit pose:

```text
front_lift_deg = 17.6560
elbow_drop_deg = 13.1973
```

Result:

```text
claw contact detected pos = 10.9775
claw contact tau = 0.3541
Full grasp succeeded.
```

Error after contact:

```text
ValueError: elbow target outside safe limits: offset=1.8990, limit=[-0.2618, 1.7453]
```

Cause:

```text
The grasp pose was valid, but step 8 reused the normal post-contact lift:
LIFT_ELBOW_DEG = 22deg.
In the hold-wrist extreme path, the elbow is already close to its upper safe limit,
so adding 22deg can exceed the limit after a successful grasp.
```

Adjustment after S33:

```text
In --hold-wrist-lift mode, post-contact elbow lift is now limited to 8deg maximum.
It is also clamped to stay 2deg below the elbow max safe offset.
The code logs requested_deg and used_deg before step 8.
```

## S34 - Extreme Near-Left Full Grasp Success With Auto Limit Path

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 24.87 --left 35.77 --up -37.16 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --auto-pre-claw-front-lift --hold-wrist-lift
```

Auto limit pose:

```text
front_lift_deg = 17.6560
elbow_drop_deg = 13.1973
```

Successful path:

```text
safe path step 6a: shoulder_front calculated forward lift before claw; wrist holds
safe path step 6b: elbow calculated drop before claw; wrist holds
claw contact detected pos = 11.2118
claw contact tau = 0.3590
hold-wrist post-contact elbow lift limited: requested_deg = 8.0 used_deg = 8.0
safe path step 8: lift grasped object about 5cm hold_wrist_lift = True
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 1.0538
shoulder_side pos = -2.8140
elbow pos = 0.5766
wrist pos = 1.1122
claw pos = 11.1820
```

Result:

```text
Full grasp succeeded at the extreme near-left tabletop position.
This validates the auto limit path:
  hold wrist
  calculated small shoulder_front forward lift
  calculated elbow drop
  limited post-contact elbow lift
```

## S35 - Vision Strategy Fusion

Goal:

```text
After camera detection, automatically select the grasp path:
  normal 75/25 shoulder-frame grasp for regular tabletop targets
  extreme-near-left hold-wrist limit grasp for the validated left-front boundary
```

Implementation:

```text
hp60c_auto_target.py now has --grasp-strategy with choices:
  auto
  normal
  extreme-near-left

Default is auto.
```

Auto near-left limit condition:

```text
legacy arm_x <= 2.5cm
shoulder_forward <= 28.0cm
shoulder_left >= 33.0cm
```

Auto selected command behavior:

```text
normal:
  target-shoulder ... --geometry-execution-blend 0.25

extreme-near-left:
  target-shoulder ... --geometry-execution-blend 0.25 --auto-pre-claw-front-lift --hold-wrist-lift
```

Notes:

```text
The JSON output includes controller_strategy with the selected mode, reason, measured values, and thresholds.
The console also prints Controller strategy before Controller command.
```

## S36 - Vision Strategy Threshold Correction

Observed camera target:

```text
legacy arm_x = 3.6292cm
shoulder_forward = 25.9749cm
shoulder_left = 33.3128cm
```

Initial auto strategy:

```text
mode = normal
reason = target outside validated near-left limit region
threshold arm_x_max_cm = 2.5
```

User feedback:

```text
This judgment is wrong; this target should use the near-left limit path.
```

Adjustment after S36:

```text
Increase EXTREME_NEAR_LEFT_ARM_X_MAX_CM from 2.5cm to 4.0cm.
The current sample now satisfies:
  arm_x <= 4.0cm
  shoulder_forward <= 28.0cm
  shoulder_left >= 33.0cm
and should select extreme-near-left automatically.
```

## S37 - Vision Strategy Threshold Correction 2

Observed camera target:

```text
legacy arm_x = 5.5069cm
shoulder_forward = 27.1350cm
shoulder_left = 31.4351cm
```

Initial auto strategy:

```text
mode = normal
reason = target outside validated near-left limit region
threshold arm_x_max_cm = 4.0
threshold shoulder_left_min_cm = 33.0
```

User feedback:

```text
This judgment is still wrong; the target is still in the front-left near-limit area
and should use the hold-wrist extreme grasp path.
```

Adjustment after S37:

```text
Increase EXTREME_NEAR_LEFT_ARM_X_MAX_CM from 4.0cm to 6.0cm.
Decrease EXTREME_NEAR_LEFT_LEFT_MIN_CM from 33.0cm to 31.0cm.
Keep EXTREME_NEAR_LEFT_FORWARD_MAX_CM at 28.0cm.

The current sample now satisfies:
  arm_x <= 6.0cm
  shoulder_forward <= 28.0cm
  shoulder_left >= 31.0cm
and should select extreme-near-left automatically.
```

## S38 - Extreme Near-Left Edge Auto Angle Correction

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 27.28 --left 31.26 --up -37.52 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --auto-pre-claw-front-lift --hold-wrist-lift --skip-claw
```

Shoulder-frame target:

```text
forward = 27.28cm
left = 31.26cm
up = -37.52cm
```

Legacy-equivalent target:

```text
arm_x = 5.6820cm
arm_y = 32.7394cm
arm_z = 24.5037cm
```

Auto limit pose before correction:

```text
front_lift_deg = 13.0800
elbow_drop_deg = 2.5200
```

Final skip-claw pose:

```text
shoulder_front pos = 1.1324
shoulder_side pos = -2.8830
elbow pos = 0.6212
wrist pos = 1.1152
```

Result:

```text
User feedback: this pose cannot grasp.
The strategy correctly selected the extreme hold-wrist path, but the auto angle formula still faded out too quickly at the edge of the newly expanded region.
```

Adjustment after S38:

```text
Align the auto limit-pose ramp with the expanded strategy region:
  LIMIT_FRONT_LIFT_ARM_X_START_CM from 4.0cm to 6.0cm
  LIMIT_ELBOW_DROP_ARM_X_START_CM from 4.0cm to 6.0cm

Raise the minimum limit-pose correction inside the extreme region:
  LIMIT_FRONT_LIFT_MIN_DEG from 12.0deg to 15.0deg
  LIMIT_ELBOW_DROP_MIN_DEG from 0.0deg to 6.0deg

Expected recalculated S38 pose:
  front_lift_deg = 15.54deg
  elbow_drop_deg = 7.44deg

Expected effect:
  The edge of the extreme-near-left region keeps enough elbow participation instead of almost falling back to the normal path.
```

## S39 - Extreme Near-Left Edge Still Short

Follow-up feedback after S38:

```text
The recalculated edge pose still lacks distance.
S38 expected values were:
  front_lift_deg = 15.54deg
  elbow_drop_deg = 7.44deg
```

Adjustment after S39:

```text
Raise the edge-region minimum corrections again, but keep the successful extreme-point upper limits almost unchanged:
  LIMIT_FRONT_LIFT_MIN_DEG from 15.0deg to 16.5deg
  LIMIT_ELBOW_DROP_MIN_DEG from 6.0deg to 9.0deg

Expected recalculated S38/S39 pose:
  front_lift_deg = 16.77deg
  elbow_drop_deg = 9.90deg

Expected S34 successful extreme pose remains near the cap:
  front_lift_deg = 17.95deg
  elbow_drop_deg = 13.83deg

Expected effect:
  The edge of the extreme-near-left strategy gets more forward reach and more elbow participation,
  while the known successful deep extreme sample changes only slightly.
```

## S40 - Extreme Near-Left Edge Full Grasp Success

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 27.02 --left 33.16 --up -37.58 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry --auto-pre-claw-front-lift --hold-wrist-lift
```

Shoulder-frame target:

```text
forward = 27.02cm
left = 33.16cm
up = -37.58cm
```

Legacy-equivalent target:

```text
arm_x = 3.7820cm
arm_y = 32.9850cm
arm_z = 24.3994cm
```

Auto limit pose:

```text
front_lift_deg = 17.1654
elbow_drop_deg = 11.2180
```

Successful path:

```text
safe path step 6a: shoulder_front calculated forward lift before claw; wrist holds
safe path step 6b: elbow calculated drop before claw; wrist holds
claw contact detected pos = 11.7073
claw contact tau = 0.3150
hold-wrist post-contact elbow lift limited: requested_deg = 8.0 used_deg = 8.0
safe path step 8: lift grasped object about 5cm hold_wrist_lift = True
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 1.0649
shoulder_side pos = -2.8140
elbow pos = 0.6106
wrist pos = 1.1130
claw pos = 11.6760
```

Result:

```text
Perfect full grasp.
This validates the S39 raised edge-region limit-pose correction on a target that is less deep than S34 but still inside the extreme-near-left strategy.
Keep current S39 constants.
```

## S41 - Geometry Calibration V2 Fit

Goal:

```text
Use the later samples instead of fitting only the original 12-point geometry set.
Separate normal shoulder-frame geometry calibration from the hold-wrist extreme-near-left strategy.
```

Implementation:

```text
Added scripts/geometry_calibration_samples_v2.json.
Updated scripts/fit_geometry_calibration.py to prefer the v2 sample file when present.
The v2 file contains:
  normal_samples: 14 structured target-shoulder joint targets
  limit_samples: S34, S38, S40 hold-wrist extreme-near-left samples
```

V2 normal affine fit:

```text
raw+shoulder_xyz model:
  shoulder_front mean_abs_err_deg = 0.403, max_abs_err_deg = 1.321
  shoulder_side  mean_abs_err_deg = 0.740, max_abs_err_deg = 1.500
  elbow          mean_abs_err_deg = 1.199, max_abs_err_deg = 2.642
  wrist          mean_abs_err_deg = 1.553, max_abs_err_deg = 3.572
```

V2 hold-wrist extreme fit:

```text
Success samples: S34, S40
Short sample kept as boundary context: S38

Fitted from successful limit samples:
  LIMIT_FRONT_LIFT_MIN_DEG = 16.748
  LIMIT_FRONT_LIFT_MAX_DEG = 17.688
  LIMIT_ELBOW_DROP_MIN_DEG = 9.536
  LIMIT_ELBOW_DROP_MAX_DEG = 13.328
```

Applied after S41:

```text
Updated GEOMETRY_AFFINE_COEFFS in left_arm_controller.py with v2 raw+shoulder_xyz constants.
Updated hold-wrist limit constants from the successful S34/S40 fit.
The limit fit reproduces the two successful extreme samples and gives S38 a slightly stronger correction than S39.
```

## S42 - Geometry Calibration V2 Normal Path Full Grasp Success

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 36.86 --left 29.69 --up -38.59 --geometry-execution-blend 0.25 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 36.86cm
left = 29.69cm
up = -38.59cm
```

Legacy-equivalent target:

```text
arm_x = 7.2520cm
arm_y = 25.6179cm
arm_z = 31.0004cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.01797928338024546,
  "shoulder_side": -2.9729193092849666,
  "elbow": -0.20901341096401926,
  "wrist": -0.2134594919844257
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.009643268020591789,
  "shoulder_side": -2.970994450877038,
  "elbow": -0.1588437412410858,
  "wrist": -0.22110133293958967
}
```

25% geometry blend:

```json
{
  "shoulder_front": -0.011727271860505206,
  "shoulder_side": -2.9713856783484496,
  "elbow": -0.17138615867181917,
  "wrist": -0.21919087270079868
}
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 11.2915
claw contact tau = 0.3199
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 0.0067
shoulder_side pos = -2.9669
elbow pos = 0.2020
wrist pos = -0.0544
claw pos = 11.2610
```

Result:

```text
Perfect full grasp.
This is the first full normal-path real grasp after applying v2 geometry affine constants.
The 25% geometry blend remains stable in the mid-near-left area.
```

## S43 - Middle Region 40% Geometry Blend Full Grasp Success

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 36.33 --left 29.81 --up -38.48 --geometry-execution-blend 0.40 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 36.33cm
left = 29.81cm
up = -38.48cm
```

Legacy-equivalent target:

```text
arm_x = 7.1320cm
arm_y = 25.9820cm
arm_z = 30.5998cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.018846023387889432,
  "shoulder_side": -2.9692938756938,
  "elbow": -0.21048988984993755,
  "wrist": -0.22529932759621185
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.0075488729181986125,
  "shoulder_side": -2.965234864345457,
  "elbow": -0.1588437412410858,
  "wrist": -0.2662996249714433
}
```

40% geometry blend:

```json
{
  "shoulder_front": -0.012067733106074941,
  "shoulder_side": -2.966506146699758,
  "elbow": -0.17950220068462652,
  "wrist": -0.24989950602135075
}
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 11.5272
claw contact tau = 0.3736
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 0.0067
shoulder_side pos = -2.9623
elbow pos = 0.1944
wrist pos = -0.0849
claw pos = 11.4963
```

Result:

```text
Perfect full grasp.
This validates 40% geometry execution blend in the middle/mid-near-left test region after v2 geometry calibration.
Next test can raise this same region to 60% geometry blend.
```

## S44 - Middle Region 60% Geometry Blend Skip-Claw Pose OK

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 36.62 --left 29.34 --up -38.39 --geometry-execution-blend 0.6 --execute --allow-unverified-geometry --skip-claw
```

Shoulder-frame target:

```text
forward = 36.62cm
left = 29.34cm
up = -38.39cm
```

Legacy-equivalent target:

```text
arm_x = 7.6020cm
arm_y = 25.6945cm
arm_z = 30.6975cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.027688132527877718,
  "shoulder_side": -2.9798433225657472,
  "elbow": -0.22060726208405856,
  "wrist": -0.21054211400103817
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.015751920402571895,
  "shoulder_side": -2.9877932449274835,
  "elbow": -0.1588437412410858,
  "wrist": -0.23818599458844303
}
```

60% geometry blend:

```json
{
  "shoulder_front": -0.02291364767775539,
  "shoulder_side": -2.9834979018754373,
  "elbow": -0.19590185374686947,
  "wrist": -0.2215996662360001
}
```

Final skip-claw pose:

```text
shoulder_front pos = 0.0048
shoulder_side pos = -2.9784
elbow pos = -0.2142
wrist pos = -0.2192
```

Result:

```text
User feedback: pose is perfect.
This is a middle-region 60% geometry blend pose-ok sample.
Store in v2 JSON as skip_claw_pose_ok; full grasp still needs confirmation before counting it as a complete success sample.
```

Full grasp retest:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 36.62 --left 29.34 --up -38.39 --geometry-execution-blend 0.6 --execute --allow-unverified-geometry
```

Successful path:

```text
claw contact detected pos = 11.2957
claw contact tau = 0.2955
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = -0.0021
shoulder_side pos = -2.9784
elbow pos = 0.1776
wrist pos = -0.0578
claw pos = 11.2648
```

Updated result:

```text
Perfect full grasp.
Upgrade S44 in geometry_calibration_samples_v2.json from skip_claw_pose_ok to success.
This validates 60% geometry execution blend in the middle/mid-near-left test region.
```

## S45 - Middle Region 80% Geometry Blend Full Grasp Success

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 37.44 --left 29.58 --up -38.79 --geometry-execution-blend 0.8 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 37.44cm
left = 29.58cm
up = -38.79cm
```

Legacy-equivalent target:

```text
arm_x = 7.3620cm
arm_y = 25.2662cm
arm_z = 31.5031cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.013599308828191048,
  "shoulder_side": -2.9754835148156173,
  "elbow": -0.2014625977441281,
  "wrist": -0.20960186726090346
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.011563130197785543,
  "shoulder_side": -2.9762740718643212,
  "elbow": -0.1588437412410858,
  "wrist": -0.1659451979940878
}
```

80% geometry blend:

```json
{
  "shoulder_front": -0.013192073102109946,
  "shoulder_side": -2.9757425013047727,
  "elbow": -0.19293882644351965,
  "wrist": -0.20087053340754032
}
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 11.5471
claw contact tau = 0.3394
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 0.0055
shoulder_side pos = -2.9707
elbow pos = 0.1802
wrist pos = -0.0357
claw pos = 11.5188
```

Result:

```text
Perfect full grasp.
This validates 80% geometry execution blend in the middle/mid-near-left test region.
Store in v2 JSON as a successful normal-path sample.
```

## S46 - Middle Region 100% Geometry Blend Full Grasp Success

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 37.47 --left 30.06 --up -38.64 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 37.47cm
left = 30.06cm
up = -38.64cm
```

Legacy-equivalent target:

```text
arm_x = 6.8820cm
arm_y = 25.1538cm
arm_z = 31.3994cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.010915553927722477,
  "shoulder_side": -2.9698492048887584,
  "elbow": -0.19990374330653937,
  "wrist": -0.1982017514580141
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.003185549788212838,
  "shoulder_side": -2.9532357257379958,
  "elbow": -0.1588437412410858,
  "wrist": -0.19506142316898245
}
```

100% geometry execution target:

```json
{
  "shoulder_front": -0.010915553927722477,
  "shoulder_side": -2.9652057374661203,
  "elbow": -0.19990374330653937,
  "wrist": -0.1982017514580141
}
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 10.7734
claw contact tau = 0.2857
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 0.0101
shoulder_side pos = -2.9612
elbow pos = 0.1711
wrist pos = -0.0345
claw pos = 10.7410
```

Result:

```text
Successful full grasp.
This validates 100% geometry execution blend in the middle/mid-near-left test region.
The middle region now has successful 40%, 60%, 80%, and 100% geometry blend samples.
```

## S47 - Right/Far-Front 40% Geometry Full Grasp, Wrist Slightly High

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 44.77 --left 23.26 --up -39.39 --geometry-execution-blend 0.4 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 44.77cm
left = 23.26cm
up = -39.39cm
```

Legacy-equivalent target:

```text
arm_x = 13.6820cm
arm_y = 19.6888cm
arm_z = 36.2970cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.1567549095755567,
  "shoulder_side": -3.113605421080539,
  "elbow": -0.468657447320881,
  "wrist": 0.2989988613177914
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.17422781648365704,
  "shoulder_side": -3.094162336190278,
  "elbow": -0.48271303518126196,
  "wrist": 0.2845631106886184
}
```

40% geometry blend:

```json
{
  "shoulder_front": -0.1672386537204169,
  "shoulder_side": -3.1019395701463823,
  "elbow": -0.4770908000371096,
  "wrist": 0.29033741094028764
}
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw anomaly stop occurred after contact-like closure, then held/backed off safely.
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = -0.1455
shoulder_side pos = -3.0943
elbow pos = -0.1055
wrist pos = 0.4545
claw pos = 12.2051
```

Result:

```text
Successful grasp, but user feedback says the wrist should press down slightly for a perfect pose.
Store in v2 JSON as result=success_wrist_needs_down and fit=false.
```

Adjustment after S47:

```text
Add a small far-front/mid-high wrist-down correction:
  WRIST_FAR_MID_EXTRA_DOWN_MAX_DEG = 4.0
  active around arm_x 12-14cm
  active around arm_z 34.5-38.5cm, peaking near 36.5cm

For this sample, expected additional coarse wrist down is about 3deg.
Because execution blend is 40% geometry, effective blended wrist movement should be about 2deg down.
This should improve the right/far-front region without changing the middle region or the higher S12 far-high correction.
```

## S48 - Right/Far-Front 40% Geometry Full Grasp Success After Wrist Correction

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 44.44 --left 23.12 --up -39.14 --geometry-execution-blend 0.4 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 44.44cm
left = 23.12cm
up = -39.14cm
```

Legacy-equivalent target:

```text
arm_x = 13.8220cm
arm_y = 19.8088cm
arm_z = 35.9007cm
```

Wrist correction:

```text
wrist_far_mid_extra_down_deg = 2.5521
wrist_down_offset_deg = -52.7250
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.1588808136075307,
  "shoulder_side": -3.1172471360443934,
  "elbow": -0.4629691736816681,
  "wrist": 0.27787415831166795
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.17628730500101053,
  "shoulder_side": -3.094162336190278,
  "elbow": -0.47441455207725336,
  "wrist": 0.21371157067595647
}
```

40% geometry blend:

```json
{
  "shoulder_front": -0.1693247084436186,
  "shoulder_side": -3.1033962561319246,
  "elbow": -0.4698364007190192,
  "wrist": 0.23937660573024108
}
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 12.4489
claw contact tau = 0.3053
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = -0.1459
shoulder_side pos = -3.0962
elbow pos = -0.0975
wrist pos = 0.4023
claw pos = 12.4157
```

Result:

```text
Perfect full grasp.
This validates the S47 far-front/mid-high wrist-down correction.
Store in v2 JSON as a successful right/far-front 40% geometry sample.
```

## S49 - Right/Far-Front 80% Geometry Blend Full Grasp Success

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 44.04 --left 23.33 --up -39.31 --geometry-execution-blend 0.8 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 44.04cm
left = 23.33cm
up = -39.31cm
```

Legacy-equivalent target:

```text
arm_x = 13.6120cm
arm_y = 20.2324cm
arm_z = 35.8031cm
```

Wrist correction:

```text
wrist_far_mid_extra_down_deg = 2.1007
wrist_down_offset_deg = -53.2298
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.14765494454668637,
  "shoulder_side": -3.1092117171765206,
  "elbow": -0.4424743287577456,
  "wrist": 0.23539124930102573
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.17300608600726108,
  "shoulder_side": -3.094162336190278,
  "elbow": -0.47237081595492414,
  "wrist": 0.20490042023202193
}
```

80% geometry blend:

```json
{
  "shoulder_front": -0.15272517283880133,
  "shoulder_side": -3.1062018409792724,
  "elbow": -0.4484536261971813,
  "wrist": 0.22929308348722496
}
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 12.2684
claw contact tau = 0.2906
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = -0.1310
shoulder_side pos = -3.0997
elbow pos = -0.0757
wrist pos = 0.3916
claw pos = 12.2372
```

Result:

```text
Perfect full grasp.
This validates 80% geometry execution blend in the right/far-front region after the far-mid wrist-down correction.
Store in v2 JSON as a successful right/far-front 80% geometry sample.
```

## S50 - Right/Far-Front Pure Geometry Full Grasp, Wrist Needs More Down

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 45.54 --left 23.50 --up -39.33 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 45.54cm
left = 23.50cm
up = -39.33cm
```

Legacy-equivalent target:

```text
arm_x = 13.4420cm
arm_y = 19.0306cm
arm_z = 36.7010cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.16469525591725187,
  "shoulder_side": -3.115054601753311,
  "elbow": -0.4943483857942408,
  "wrist": 0.3723721138177619
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 1.0
```

Result:

```text
Full grasp succeeded, but wrist needs more downward pressure.
Store in v2 JSON with fit=false. This is a correction sample, not an ideal-fit sample.
```

Adjustment after S50:

```text
Added geometry-only far-front/right-upper wrist-down correction.
At forward ~=45.54cm, left ~=23.5cm, up ~= -39.3cm it applies nearly the full extra down correction.
At the previously successful ~=44.0cm right/far-front samples it applies only a small correction.
```

## S51 - Right/Far-Front Pure Geometry Full Grasp Success After Geometry Wrist Correction

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 45.15 --left 23.16 --up -39.36 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 45.15cm
left = 23.16cm
up = -39.36cm
```

Geometry correction:

```text
geometry wrist far-front extra down = 4.7196deg
```

V2 calibrated geometry planned targets after correction:

```json
{
  "shoulder_front": -0.16364806580028102,
  "shoulder_side": -3.117876543604499,
  "elbow": -0.48443245221914955,
  "wrist": 0.2523364588346374
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 1.0
```

Result:

```text
Perfect full grasp.
This validates pure geometry execution in the right/far-front region with the local geometry wrist-down correction.
Store in v2 JSON with fit=false for now to avoid double-counting the local wrist correction in the affine fit.
```

## S52 - Left/Far-Front 80% Geometry Blend Full Grasp Success

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 43.92 --left 36.75 --up -39.15 --geometry-execution-blend 0.8 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 43.92cm
left = 36.75cm
up = -39.15cm
```

Legacy-equivalent target:

```text
arm_x = 0.1920cm
arm_y = 20.2354cm
arm_z = 35.6032cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": 0.032949108828635776,
  "shoulder_side": -2.810011984363458,
  "elbow": -0.25107878913585246,
  "wrist": 0.26080925229530105
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.0017543798015774437,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.3111028758418074,
  "wrist": 0.3120793652666194
}
```

80% geometry blend:

```json
{
  "shoulder_front": 0.026008411102593134,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.26308360647704343,
  "wrist": 0.2710632748895647
}
```

Execution blend:

```text
geometry_weight = 0.8
shoulder_side_geometry_weight = 0.0
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 11.0302
claw contact tau = 0.2955
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 0.0444
shoulder_side pos = -2.8136
elbow pos = 0.1097
wrist pos = 0.4335
claw pos = 10.9985
```

Result:

```text
Perfect full grasp.
This validates left/far-front 80% geometry blend for shoulder_front, elbow, and wrist.
Shoulder-side remained protected at legacy weight because this is near the extreme-left limit.
```

## S53 - Left/Far-Front Transition 80% Geometry, Shoulder-Side Needs More Inward

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 44.28 --left 32.73 --up -39.63 --geometry-execution-blend 0.8 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 44.28cm
left = 32.73cm
up = -39.63cm
```

Legacy-equivalent target:

```text
arm_x = 4.2120cm
arm_y = 20.2263cm
arm_z = 36.2031cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.017522087783452878,
  "shoulder_side": -2.9535872402297523,
  "elbow": -0.312696102521668,
  "wrist": 0.2666040097523582
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.0089451363197941,
  "shoulder_side": -2.8250849254103123,
  "elbow": -0.323667798142689,
  "wrist": 0.38633445926965304
}
```

80% geometry blend:

```json
{
  "shoulder_front": -0.01580669749072112,
  "shoulder_side": -2.8305334235586566,
  "elbow": -0.3148904416458722,
  "wrist": 0.2905500996558172
}
```

Execution blend:

```text
geometry_weight = 0.8
shoulder_side_geometry_weight = 0.0424
```

Result:

```text
Full grasp succeeded, but shoulder_side needs more inward motion.
Store in v2 JSON with fit=false. This is a side correction sample.
```

Adjustment after S53:

```text
Added local far-left/transition shoulder_side geometry release.
This increases shoulder_side geometry weight near forward >42cm, left ~=32.5cm, legacy_arm_x ~=4.5cm.
Extreme-left samples such as S52 remain protected.
```

## S54 - Left/Far-Front Transition 80% Geometry Success After Side Release

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 44.09 --left 32.06 --up -39.02 --geometry-execution-blend 0.8 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 44.09cm
left = 32.06cm
up = -39.02cm
```

Legacy-equivalent target:

```text
arm_x = 4.8820cm
arm_y = 20.0215cm
arm_z = 35.5979cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.035261426829337195,
  "shoulder_side": -2.971449736845481,
  "elbow": -0.32586564748417657,
  "wrist": 0.27059257834443484
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.020638842308156047,
  "shoulder_side": -2.857242616878308,
  "elbow": -0.3109929469830268,
  "wrist": 0.31118161291991087
}
```

80% geometry blend:

```json
{
  "shoulder_front": -0.032336909925100965,
  "shoulder_side": -2.877388752840517,
  "elbow": -0.3228911073839466,
  "wrist": 0.27871038525953007
}
```

Execution blend:

```text
geometry_weight = 0.8
shoulder_side_geometry_weight = 0.1764
far_left_side_release = 0.2150
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 11.3010
claw contact tau = 0.2808
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = -0.0120
shoulder_side pos = -2.8754
elbow pos = 0.0494
wrist pos = 0.4397
claw pos = 11.2720
```

Result:

```text
Perfect full grasp.
This validates the local far-left transition shoulder_side release.
Store in v2 JSON with fit=false for now because shoulder_side still used mostly protected/legacy blending.
```

## S55 - Left/Far-Front Transition 100% Geometry Main Joints Success

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 44.24 --left 32.47 --up -39.41 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 44.24cm
left = 32.47cm
up = -39.41cm
```

Legacy-equivalent target:

```text
arm_x = 4.4720cm
arm_y = 20.1293cm
arm_z = 36.0016cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.024540154551215276,
  "shoulder_side": -2.960496611929281,
  "elbow": -0.31874426091495944,
  "wrist": 0.27062487715486705
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.013482992374979297,
  "shoulder_side": -2.837564029562072,
  "elbow": -0.3194476936135269,
  "wrist": 0.3800043024759099
}
```

100% geometry blend with side protection:

```json
{
  "shoulder_front": -0.024540154551215276,
  "shoulder_side": -2.879280594920978,
  "elbow": -0.31874426091495944,
  "wrist": 0.27062487715486705
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 0.3393
far_left_side_release = 0.3393
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 11.5360
claw contact tau = 0.2955
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = -0.0059
shoulder_side pos = -2.8773
elbow pos = 0.0532
wrist pos = 0.4332
claw pos = 11.5116
```

Result:

```text
Perfect full grasp.
This validates 100% geometry for shoulder_front, elbow, and wrist in the left/far-front transition region.
Shoulder_side still uses protected partial geometry, but the local side release is sufficient.
Store in v2 JSON with fit=false for now because shoulder_side is not a pure affine geometry sample.
```

## S56 - Left/Far-Front More-Left 100% Geometry Failed, Wrist Needs Up

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 44.24 --left 34.40 --up -39.41 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 44.24cm
left = 34.40cm
up = -39.41cm
```

Legacy-equivalent target:

```text
arm_x = 2.5420cm
arm_y = 20.1293cm
arm_z = 36.0016cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": 0.0016273364828697234,
  "shoulder_side": -2.929124423433609,
  "elbow": -0.29144683377433744,
  "wrist": 0.2749965583964372
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.0017543798015774437,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.3194476936135269,
  "wrist": 0.3800724663835594
}
```

100% geometry blend with side protection:

```json
{
  "shoulder_front": 0.0016273364828697234,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.29144683377433744,
  "wrist": 0.2749965583964372
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 0.0
far_left_side_release = 0.0
```

Result:

```text
Failed to grasp. Wrist needs upward correction.
Store in v2 JSON with fit=false. This is a wrist correction sample.
```

Adjustment after S56:

```text
Added local far-left/left ~=34.4cm geometry wrist-up correction.
At this point wrist target should move from about 0.275rad toward about 0.379rad.
The correction is localized away from the successful left ~=32.5cm transition band and the far-left protected left ~=36.75cm band.
```

## S57 - Left/Far-Front More-Left 100% Geometry Success After Wrist-Up Correction

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 44.62 --left 34.45 --up -39.38 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 44.62cm
left = 34.45cm
up = -39.38cm
```

Geometry correction:

```text
geometry wrist far-left extra up = 5.6571deg
```

V2 calibrated geometry planned targets after correction:

```json
{
  "shoulder_front": -0.0032475308203236963,
  "shoulder_side": -2.930957292965677,
  "elbow": -0.3051002769777389,
  "wrist": 0.4097286916448266
}
```

100% geometry blend with side protection:

```json
{
  "shoulder_front": -0.0032475308203236963,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.3051002769777389,
  "wrist": 0.4097286916448266
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 0.0
far_left_side_release = 0.0
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 11.2747
claw contact tau = 0.2955
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 0.0177
shoulder_side pos = -2.8136
elbow pos = 0.0673
wrist pos = 0.5705
claw pos = 11.2495
```

Result:

```text
Perfect full grasp.
This validates the local far-left wrist-up correction around left ~=34.4cm.
Store in v2 JSON with fit=false because this validates a local wrist correction rather than the affine base geometry.
```

## S58 - Far-Left Edge 100% Geometry Failed, Wrist Too Down

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 43.17 --left 36.94 --up -39.57 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 43.17cm
left = 36.94cm
up = -39.57cm
```

Legacy-equivalent target:

```text
arm_x = 0.0020cm
arm_y = 21.0890cm
arm_z = 35.5021cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": 0.053151503163452185,
  "shoulder_side": -2.7879598390936136,
  "elbow": -0.21377122756560063,
  "wrist": 0.17711714680543955
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.0017543798015774437,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.3089864518585965,
  "wrist": 0.2947952360703969
}
```

100% geometry blend with side protection:

```json
{
  "shoulder_front": 0.053151503163452185,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.21377122756560063,
  "wrist": 0.17711714680543955
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 0.0
far_left_side_release = 0.0
```

Result:

```text
Failed to grasp. Wrist was too downward.
Store in v2 JSON with fit=false. This is a wrist correction sample.
```

Adjustment after S58:

```text
Added local far-left-edge geometry wrist-up correction.
At this point wrist target should move from about 0.177rad toward about 0.29rad.
This correction is separate from the left ~=34.4cm wrist-up band and does not affect the successful left ~=32.5cm transition band.
```

## S59 - Far-Left Edge 100% Geometry Success After Wrist-Up Correction

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 43.17 --left 36.94 --up -39.57 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 43.17cm
left = 36.94cm
up = -39.57cm
```

Geometry correction:

```text
geometry wrist far-left edge extra up = 5.7606deg
```

V2 calibrated geometry planned targets after correction:

```json
{
  "shoulder_front": 0.053151503163452185,
  "shoulder_side": -2.7879598390936136,
  "elbow": -0.21377122756560063,
  "wrist": 0.2776579291973552
}
```

100% geometry blend with side protection:

```json
{
  "shoulder_front": 0.053151503163452185,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.21377122756560063,
  "wrist": 0.2776579291973552
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 0.0
far_left_side_release = 0.0
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 10.7986
claw contact tau = 0.2906
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 0.0719
shoulder_side pos = -2.8136
elbow pos = 0.1581
wrist pos = 0.4389
claw pos = 10.7666
```

Result:

```text
Perfect full grasp.
This validates the local far-left-edge wrist-up correction and closes the S58 failure.
Store in v2 JSON with fit=false because this validates a local wrist correction rather than the affine base geometry.
```

## S60 - Far-Front Center 100% Geometry Grasped But Shoulder-Side Too Inward

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 45.95 --left 29.38 --up -39.15 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 45.95cm
left = 29.38cm
up = -39.15cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.09360435097206232,
  "shoulder_side": -3.0242888046477185,
  "elbow": -0.42924597904208384,
  "wrist": 0.43015227056271943
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.06741366626160417,
  "shoulder_side": -2.98587338275029,
  "elbow": -0.33609328328111165,
  "wrist": 0.49658308015974517
}
```

100% geometry blend:

```json
{
  "shoulder_front": -0.09360435097206232,
  "shoulder_side": -3.0200823159499497,
  "elbow": -0.42924597904208384,
  "wrist": 0.43015227056271943
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 0.8905
```

Result:

```text
Object was grasped, but the arm hit the target.
Operator correction: shoulder_side was too inward; elbow and wrist were acceptable.
Store in v2 JSON with fit=false. This is a side collision correction sample.
```

Adjustment after S60:

```text
Added local far-front center shoulder_side geometry cap.
At forward ~=46cm, left ~=29.5cm, legacy_arm_x ~=7.5cm, shoulder_side_geometry_weight is capped toward 0.35.
This moves shoulder_side outward while leaving shoulder_front, elbow, and wrist unchanged.
```

## S61 - Far-Front Center 100% Geometry Success After Shoulder-Side Cap

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 45.99 --left 29.69 --up -39.74 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Shoulder-frame target:

```text
forward = 45.99cm
left = 29.69cm
up = -39.74cm
```

V2 calibrated geometry planned targets:

```json
{
  "shoulder_front": -0.07921454839109177,
  "shoulder_side": -3.0116563713890474,
  "elbow": -0.41605349106161194,
  "wrist": 0.41210804569043846
}
```

Legacy-equivalent planned targets:

```json
{
  "shoulder_front": -0.06200314558042164,
  "shoulder_side": -2.970994450877038,
  "elbow": -0.3465826723641291,
  "wrist": 0.5002937844927543
}
```

100% geometry blend with side cap:

```json
{
  "shoulder_front": -0.07921454839109177,
  "shoulder_side": -2.9899500879009566,
  "elbow": -0.41605349106161194,
  "wrist": 0.41210804569043846
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 0.4662
far_center_side_cap = 0.7491
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 11.2961
claw contact tau = 0.3053
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = -0.0570
shoulder_side pos = -2.9841
elbow pos = -0.0441
wrist pos = 0.5739
claw pos = 11.2644
```

Result:

```text
Perfect full grasp.
This validates the local far-front center shoulder_side cap and closes the S60 collision.
Store in v2 JSON with fit=false because this validates a local side cap rather than affine base geometry.
```

## Control Semantics Update - Geometry Blend Should Apply To All Joints

Decision:

```text
--geometry-execution-blend 1.0 must mean all planned joints use 100% geometry.
shoulder_side must not silently reduce geometry participation except for hard physical/body safety limits.
```

Reason:

```text
The previous shoulder_side_geometry_weight protection mixed two concepts:
1. geometry participation ratio
2. empirical collision/body-safety correction

This made logs confusing, for example geometry_weight=1.0 while shoulder_side_geometry_weight=0.4662.
```

Implementation change:

```text
All four joints now use the requested geometry_execution_blend directly.
Far-front center shoulder_side collision handling is converted from a blend cap into a geometry shoulder_side outward correction.
```

Expected log behavior:

```text
For --geometry-execution-blend 1.0:
geometry_weight = 1.0
shoulder_side_geometry_weight = 1.0
```

The corrected geometry target itself should contain the local side outward adjustment:

```text
geometry shoulder_side far-center outward: deg=...
```

## S62 - Far-Front Right-Center Pure Geometry, Success But Side Can Move Outward

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 46.11 --left 28.57 --up -39.53 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Planned target:

```json
{
  "shoulder_front": -0.09977764087902075,
  "shoulder_side": -3.0276771365667012,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.43991811405997083,
  "arm_roll": 2.0243000984191895,
  "wrist": 0.4285762747632781
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 1.0
far_center_side_outward_deg = 0.3255
```

Result:

```text
Successful grasp and no target collision.
Operator observed shoulder_side could move slightly farther outward.
```

Adjustment:

```text
Added a narrow far-right-center geometry shoulder_side outward correction near:
forward ~= 46.1 cm, left ~= 28.6 cm, legacy_arm_x ~= 8.4 cm.

This preserves geometry_execution_blend semantics:
--geometry-execution-blend 1.0 still means shoulder_side_geometry_weight = 1.0.
The correction changes the geometry target itself instead of lowering geometry participation.
```

## S63 - Far-Front Right-Center Success After Side Outward Correction

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 46.41 --left 28.50 --up -39.56 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Geometry corrections:

```text
geometry shoulder_side far-center outward = 0.1959deg
geometry shoulder_side far-right-center outward = 0.5146deg
```

Planned target:

```json
{
  "shoulder_front": -0.10410125556067673,
  "shoulder_side": -3.0234716054056907,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.4513115351759507,
  "arm_roll": 2.0243000984191895,
  "wrist": 0.4548271313665718
}
```

Execution blend:

```text
geometry_weight = 1.0
shoulder_side_geometry_weight = 1.0
far_center_side_outward_deg = 0.1959
far_right_center_side_outward_deg = 0.5146
```

Final status:

```text
shoulder_front pos = -0.0834
shoulder_side pos = -3.0177
elbow pos = -0.0792
wrist pos = 0.6163
claw pos = 11.4700
```

Result:

```text
Successful grasp.
This validates the far-right-center shoulder_side outward correction while keeping 100% geometry participation.
Store in v2 JSON with fit=false because it validates a local side correction rather than affine base geometry.
```

## S64 - Extreme-Left Skip-Claw Failure, Shoulder-Side Too Outward

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 37.52 --left 40.85 --up -38.73 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry --skip-claw
```

Planned target:

```json
{
  "shoulder_front": 0.13784957587364655,
  "shoulder_side": -2.5178700574019723,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.04368372794426567,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.17834109223902006
}
```

Reference target:

```json
{
  "shoulder_front": 0.050605497758252405,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.1588437412410858,
  "wrist": -0.24995225824054557
}
```

Result:

```text
Failed before claw. Operator interrupted at wrist step.
Shoulder_side was too outward.
```

Adjustment:

```text
Added extreme-left shoulder_side inward correction for left >~38.2cm and legacy_arm_x <~1cm.
At this point the correction pulls shoulder_side from -2.5179 to about -2.7834.
This is about 15.2deg inward from the failed target, while still staying about 1.8deg outward of the legacy reference.
The correction changes the geometry target itself and keeps shoulder_side_geometry_weight = 1.0.
```

## S65 - Extreme-Left Retest, Shoulder-Side Fixed But Wrist Too Down

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 37.52 --left 40.85 --up -38.73 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry --skip-claw
```

Corrections:

```text
geometry shoulder_side extreme-left inward = 15.2151deg
extreme_left_side_inward_fraction = 0.8940
```

Planned target:

```json
{
  "shoulder_front": 0.13784957587364655,
  "shoulder_side": -2.7834234584334485,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.04368372794426567,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.17834109223902006
}
```

Final status:

```text
shoulder_front pos = 0.1555
shoulder_side pos = -2.7819
elbow pos = -0.0563
wrist pos = -0.1680
```

Result:

```text
Failed skip-claw retest. Shoulder-side was corrected, but wrist was too downward.
```

Adjustment:

```text
Added local extreme-left wrist-up correction.
At this point the correction adds about 6.76deg, moving wrist from -0.1783 to about -0.0603.
The correction is localized to the extreme-left edge and does not change shoulder-side geometry participation.
```

## S66 - Extreme-Left Retest, Wrist Tracks But Needs Slightly More Up

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 37.52 --left 40.85 --up -38.73 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry --skip-claw
```

Corrections:

```text
geometry wrist extreme-left extra up = 6.7637deg
geometry shoulder_side extreme-left inward = 15.2151deg
```

Planned target:

```json
{
  "shoulder_front": 0.13784957587364655,
  "shoulder_side": -2.7834234584334485,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.04368372794426567,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.06029231051064432
}
```

Final status:

```text
shoulder_front pos = 0.1616
shoulder_side pos = -2.7819
elbow pos = -0.0589
wrist pos = -0.0593
```

Result:

```text
Wrist tracked the corrected target, but operator observed wrist can move slightly farther up.
```

Adjustment:

```text
Increased GEOMETRY_WRIST_EXTREME_LEFT_EXTRA_UP_MAX_DEG from 7.0deg to 10.0deg.
At this point expected wrist moves from -0.0603 to about -0.0097, an additional +2.90deg.
```

## S67 - Extreme-Left Full Grasp Success After Wrist-Up Increase

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 37.52 --left 40.85 --up -38.73 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Corrections:

```text
geometry wrist extreme-left extra up = 9.6624deg
geometry shoulder_side extreme-left inward = 15.2151deg
extreme_left_side_inward_fraction = 0.8940
```

Planned target:

```json
{
  "shoulder_front": 0.13784957587364655,
  "shoulder_side": -2.7834234584334485,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.04368372794426567,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.009699975484197543
}
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 10.5587
claw contact tau = 0.3004
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 0.1558
shoulder_side pos = -2.7819
elbow pos = 0.3298
wrist pos = 0.1516
claw pos = 10.5243
```

Result:

```text
Successful full grasp.
This validates the combined extreme-left shoulder_side inward correction and wrist-up correction while preserving 100% geometry participation.
Store in v2 JSON with fit=false because this validates local corrections rather than affine base geometry.
```

## Local Correction Fit Policy

Decision:

```text
S60-S67 should remain fit=false for base affine fitting.
They validate local correction functions rather than the global raw+shoulder_xyz affine model.
```

Reason:

```text
Diagnostic fitting with fit:false success samples included increases shoulder_side and wrist affine error.
This means the new samples are better represented as local corrections.
```

Current local correction diagnostics:

```text
Extreme-left wrist-up:
  insufficient below 6.764deg
  validated success at 9.662deg
  keep GEOMETRY_WRIST_EXTREME_LEFT_EXTRA_UP_MAX_DEG ~= 10.0

Extreme-left shoulder_side inward:
  validated fraction 0.894
  keep GEOMETRY_SHOULDER_SIDE_EXTREME_LEFT_INWARD_MAX_FRACTION ~= 0.9

Far-right-center shoulder_side outward:
  validated far-right-center outward 0.515deg
  keep GEOMETRY_SHOULDER_SIDE_FAR_RIGHT_CENTER_OUTWARD_MAX_DEG ~= 0.9
```

Tooling:

```text
scripts/fit_geometry_calibration.py now prints local correction diagnostics after the base affine and hold-wrist sections.
```

## S68 - Extreme-Left Transition Band, Wrist Up And Side Inward Need More Ramp

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 37.76 --left 39.06 --up -38.55 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry --skip-claw
```

Corrections:

```text
geometry wrist extreme-left extra up = 3.2728deg
geometry shoulder_side extreme-left inward = 3.6408deg
extreme_left_side_inward_fraction = 0.3535
```

Planned target:

```json
{
  "shoulder_front": 0.10962633649041287,
  "shoulder_side": -2.698675479643451,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.0736070294102209,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.10843943285602053
}
```

Final status:

```text
shoulder_front pos = 0.1337
shoulder_side pos = -2.7010
elbow pos = -0.0910
wrist pos = -0.1043
```

Result:

```text
Skip-claw test: wrist needs higher, shoulder_side needs more inward.
The joint tracking itself is good, so the local correction ramp is too weak in this transition band.
```

Adjustment:

```text
Moved the extreme-left side/wrist left ramp from 38.2-40.2cm to 37.8-39.0cm.
At this point expected shoulder_side moves from -2.6987 to about -2.7829.
Expected wrist moves from -0.1084 to about -0.0327.
The left=40.85cm success point remains full-strength and should not become more aggressive.
```

## S69 - Extreme-Left Transition Band Success After Ramp Update

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 37.76 --left 39.06 --up -38.55 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry
```

Corrections:

```text
geometry wrist extreme-left extra up = 7.6111deg
geometry shoulder_side extreme-left inward = 8.4671deg
extreme_left_side_inward_fraction = 0.8220
```

Planned target:

```json
{
  "shoulder_front": 0.10962633649041287,
  "shoulder_side": -2.7829090319832024,
  "shoulder_rotate": -1.7408636808395386,
  "elbow": -0.0736070294102209,
  "arm_roll": 2.0243000984191895,
  "wrist": -0.03272123214033362
}
```

Successful path:

```text
Normal safe path, hold_wrist_lift = False.
claw contact detected pos = 10.5636
claw contact tau = 0.2955
safe path step 8: lift grasped object about 5cm
safe path stopped after eight motions. Object should be lifted.
```

Final status:

```text
shoulder_front pos = 0.1284
shoulder_side pos = -2.7819
elbow pos = 0.3000
wrist pos = 0.1303
claw pos = 10.5282
```

Result:

```text
Successful full grasp.
This validates the updated extreme-left side/wrist left ramp in the transition band.
Store in v2 JSON with fit=false because it validates local ramp timing rather than affine base geometry.
```

## S70 - Near-Left Body Safety Clamp, Wrist Too Down

Command:

```bash
sudo python3 left_arm_controller.py target-shoulder --forward 38.25 --left 34.25 --up -38.57 --geometry-execution-blend 1.0 --execute --allow-unverified-geometry --skip-claw
```

Planned geometry before safety clamp:

```json
{
  "shoulder_front": 0.04745470164476817,
  "shoulder_side": -2.9072829122723842,
  "elbow": -0.13432053145127398,
  "wrist": -0.17085816657772135
}
```

Safety clamp:

```text
geometry shoulder_side body safety clamp:
requested = -2.9072829122723842
used = -2.8149096558711855
body_safe_t = 1.0
```

Executed target:

```json
{
  "shoulder_front": 0.04745470164476817,
  "shoulder_side": -2.8149096558711855,
  "elbow": -0.13432053145127398,
  "wrist": -0.17085816657772135
}
```

Result:

```text
Skip-claw test. Wrist was too downward.
This is not a base affine sample because shoulder_side was fully rewritten by the old body-safety clamp.
```

Correction:

```text
The old clamp used legacy_arm_x as a proxy and was too conservative.
Body safety clamp is now disabled by default and can only trigger from an explicit shoulder_front/shoulder_side angle window.
S70 should be retested if we want to evaluate this point for base fitting.
```

## 2026-07-12 Full-Geometry Validation Closeout

After correcting the body-safety clamp logic, the shoulder-frame geometry path was validated with
`--geometry-execution-blend 1.0` across the near-left through right-front tabletop band.

Important clamp correction:

```text
Body safety is angle-gated only.
It must not be inferred from legacy_arm_x.

Measured references:
- shoulder_front forward ~= 50deg: normal grasp posture, side inward has more room.
- shoulder_front forward ~= 90deg: folded/body-risk posture.
- at ~=90deg front, shoulder_side inward safe/auto limit ~=12deg, hard limit ~=15deg, forbidden ~=20deg.
```

Validated base samples added to `geometry_calibration_samples_v2.json`:

```text
S70 forward=38.25 left=34.25 up=-38.57 result=success
    Full grasp succeeded after wrist base target was raised. Planned wrist before claw ~= -0.0866 rad.

S71 forward=38.38 left=33.57 up=-38.60 result=skip_claw_good
    Full geometry posture good.

S72 forward=38.19 left=32.02 up=-38.49 result=skip_claw_good
    Full geometry posture good.

S73 forward=38.30 left=30.46 up=-38.66 result=skip_claw_good
    Full geometry posture good.

S74 forward=39.55 left=28.08 up=-38.73 result=skip_claw_good
    Full geometry posture acceptable.

S75 forward=41.91 left=24.07 up=-39.25 result=success
    Full grasp succeeded.

S76 forward=45.03 left=26.39 up=-39.20 result=success
    Full grasp succeeded.
```

Current base affine status after S76:

```text
Sample count: 29
raw+shoulder_xyz:
shoulder_front mean_abs_err_deg ~= 0.253, max_abs_err_deg ~= 1.373
shoulder_side  mean_abs_err_deg ~= 0.419, max_abs_err_deg ~= 1.457
elbow          mean_abs_err_deg ~= 0.833, max_abs_err_deg ~= 2.607
wrist          mean_abs_err_deg ~= 2.181, max_abs_err_deg ~= 17.569
```

Operational conclusion:

```text
Within the tested tabletop band and current home/camera calibration, full geometry execution is stable enough
to be the default shoulder-mode path.

hp60c_auto_target.py now defaults --geometry-execution-blend to 1.0 in shoulder mode, so auto target
commands generated with --controller-mode shoulder run full calibrated geometry unless explicitly overridden.
```
