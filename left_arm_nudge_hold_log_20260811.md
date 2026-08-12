# Left Arm Nudge-Hold Log - 2026-08-11

## wrist_side nudge at table clearance

Test context:

- Pose: table clearance.
- Command: `nudge-hold wrist_side +5 deg`.
- Observation: `wrist_side` moved without shake, but `shoulder_front` sagged from `1.9686045647` to `1.9613565207` rad, about `0.42 deg`.

Diagnosis:

- The active `wrist_side` motion is stable.
- The remaining issue is load holding on `shoulder_front` during independent nudge.
- This is a hold stiffness/feedforward issue, not a `wrist_side` instability.

Code change:

- Added `NUDGE_HOLD_TAU` for independent `nudge-hold`.
- `shoulder_front` hold torque in `nudge-hold` changed from `2.5` to `3.2`.
- Added `control_dt` to `move_target_with_holds`.
- `nudge-hold` now uses `control_dt=0.01` so hold commands refresh faster.

Expected debug output:

```text
v2.5.3 nudge-hold hold_tau= {"shoulder_front": 3.2} control_dt= 0.01
```

Next test:

- Run table clearance.
- Run `wrist_side` +5 deg again.
- Compare `shoulder_front` start/end. Target is less sag than `0.42 deg` and no shake.

## wrist sag after shoulder/elbow hold fix

Follow-up test:

- `wrist_side` +5 deg: `shoulder_front` stayed fixed, `elbow` stayed fixed.
- New observation: `wrist` sagged from `-1.0034713745` to `-1.0309376717` rad, about `1.57 deg`.
- `wrist` +5 deg: the wrist also started by sagging and finished short of the requested target by about `1.74 deg`.

Diagnosis:

- The shoulder/elbow load hold fix worked.
- The remaining weak point is `wrist` gravity/load compensation during nudge-hold.
- The observed wrist final error is close to the existing clearance wrist fine bias range (`1.5 deg`).

Rejected approach:

- Do not use command target compensation for independent nudge.
- Reason: the wrist misses the target because it sags before/at motion start. A target bias would hide the real startup sag and make a 5 deg command no longer mean a true 5 deg command.

Code change:

- Added `wrist: 0.55` to `NUDGE_HOLD_TAU`, so wrist is supported while other joints move.
- Added `NUDGE_ACTIVE_TAU = {"wrist": 0.35}`, so active wrist nudge has a small torque feedforward during preload, motion, and final hold.
- `nudge-hold` debug output now prints `requested_target` and `target`; for nudge they should be identical.

Expected debug output for wrist nudge:

```text
v2.5.3 nudge-hold hold_tau= {"shoulder_front": 3.8, "elbow": 2.2} control_dt= 0.01
v2.5.3 nudge-hold active_tau= {'wrist': 0.35}
```

Expected debug output when wrist is only held:

```text
v2.5.3 nudge-hold hold_tau= {"shoulder_front": 3.8, "elbow": 2.2, "wrist": 0.55} control_dt= 0.01
```

## Validation after wrist torque update

Test results:

- `wrist_side` +5 deg:
  - `shoulder_front` held at `1.9686045647` rad.
  - `elbow` changed only from `0.2759975493` to `0.2756160796` rad, about `0.02 deg`.
  - `wrist` held from `-1.0034713745` to `-1.0042344332` rad, about `0.04 deg`.
  - Visual result: no visible sag.

- `wrist` +5 deg:
  - `requested_target == target`, no command bias.
  - `wrist` moved from `-1.0042344332` to `-0.9287022352` rad.
  - Requested target was `-0.9169679706` rad, remaining error about `0.67 deg`.
  - Visual result: no visible sag.

Current nudge-hold recommendation:

```python
NUDGE_HOLD_TAU = {
    "shoulder_front": 3.8,
    "elbow": 2.2,
    "wrist": 0.55,
}

NUDGE_ACTIVE_TAU = {
    "wrist": 0.55,
}
```

Keep the no-bias rule: nudge target must remain the exact requested joint angle delta.

## shoulder_front active nudge at table clearance

Test result:

- `shoulder_front` +5 deg from table clearance.
- Start: `1.9686045647` rad.
- Requested target: `2.0558710273` rad.
- Final: `1.9892042875` rad.
- Actual motion: about `+1.18 deg` out of requested `+5 deg`.
- Visual result: sag/weak movement was visible.

Diagnosis:

- In this test `shoulder_front` is the active joint, so `NUDGE_HOLD_TAU["shoulder_front"]` does not apply to it.
- Existing active nudge gains were too soft for the high-load table-clearance pose: `kp=80.0, kd=3.5`.
- The stable clearance coupled phase already uses stronger `shoulder_front` motion gains with feedforward.

Code change:

```python
NUDGE_GAINS["shoulder_front"] = {"kp": 124.0, "kd": 4.8}

NUDGE_ACTIVE_TAU = {
    "shoulder_front": 0.6,
    "wrist": 0.55,
}
```

No target bias is used.

## End-of-day follow-up: web control, claw, home safety, and nudge stability

Date recorded: 2026-08-12 for work done on 2026-08-11.

### Web controller v1.2 updates

- Added independent joint nudge controls to the web controller.
- Available nudge sizes: `2 deg`, `5 deg`, `10 deg`.
- Added `Copy` button for debug output. Fallback copy path is used when `navigator.clipboard.writeText` is unavailable.
- Fixed debug output streaming: web controller now reads stdout/stderr while the command is still running, instead of waiting for the process to exit.
- Arm commands now run as direct unbuffered Python by default:

```text
python3 -u left_arm_v2_5_3.py ...
```

- `sudo` is no longer used by default. It is only used if the web server is started with `--sudo`.
- Long-running commands are started in their own process group so `Emergency Stop` and `Claw Home` can terminate child processes cleanly.

### HP60C / camera geometry

Recorded robot geometry updates:

```text
camera_forward_from_shoulder_cm = 10.5
camera_left_from_shoulder_cm = -13.0
camera_up_from_shoulder_cm = 12.0
camera_pitch_down_deg = 70.0
```

Notes:

- `shoulder` in the camera transform refers to `shoulder_front`.
- The visible shoulder axis where the arm exits the body is `shoulder_side`; `shoulder_front` is inside the body.
- Restored the green ROI/target box and XYZ/depth-style debug values in the video stream.

### Claw integration

Added claw motor support:

```text
claw motor: DM4310
slave_id: 0x28
master_id: 0x18
```

Web buttons:

- `Capture Claw Home`: records current claw position into `scripts/data/left_arm_v2_claw_home.json`.
- `Claw Home`: moves claw back to captured claw home.
- `Claw Close`: closes claw until pressure/stall is detected, then holds the contact position.

Pressure-stop behavior is based on `claw_pressure_stop_test.py`:

```python
CLAW_CLOSE_OFFSET = 6.0
CLAW_KP_MOVE = 28.0
CLAW_KD_MOVE = 1.0
CLAW_KP_OPEN = 22.0
CLAW_KD_OPEN = 0.8
CLAW_KP_HOLD = 14.0
CLAW_KD_HOLD = 0.7
CLAW_TAU_THRESHOLD = 0.30
CLAW_STALL_TAU_THRESHOLD = 0.24
CLAW_VEL_STALL_THRESHOLD = 0.08
CLAW_CONFIRM_COUNT_NEEDED = 5
CLAW_BACKOFF = 0.04
```

Validation:

- `claw-close` detected contact and entered hold.
- Hold debug output now streams once per second:

```text
claw holding status= {"target": ..., "kp": 14.0, "kd": 0.7, "pos": ..., "vel": ..., "tau": ...}
```

Important behavior:

- `Claw Close` intentionally keeps running while holding the object.
- `Claw Home` can interrupt a running `Claw Close` hold, then move back to claw home.

### Home after independent movements

New home safety rule:

- If the arm has been moved independently after table clearance, web `Home Move` first returns joints to table clearance one by one.
- After that pre-home clearance stage completes, it runs the normal home sequence.

Final pre-home clearance order:

```text
wrist -> wrist_side -> arm_roll -> elbow -> shoulder_rotate -> shoulder_side -> shoulder_front
```

Implementation:

- Web `Home Move` calls:

```text
python3 -u left_arm_v2_5_3.py home --deadband-deg 0.5 --max-delta-deg 120 --prehome-clearance --execute
```

### Table clearance / home tuning state

Stable clearance settings at the end of the day:

```python
COUPLED_CLEARANCE_MAX_SECONDS = 12.0
COUPLED_CLEARANCE_PROGRESS_WINDOWS = {
    "arm_roll": (0.35, 1.0),
    "wrist": (0.0, 1.0),
}
COUPLED_CLEARANCE_MOVE_GAINS["wrist_side"] = {"kp": 16.0, "kd": 2.2}
COUPLED_CLEARANCE_MOVE_GAINS["wrist"] = {"kp": 22.0, "kd": 1.4}
COUPLED_CLEARANCE_WRIST_FINE_GAINS = {"kp": 26.0, "kd": 4.7, "seconds": 8.0}
CLEARANCE_WRIST_FINE_ACTIVE_TAU = 0.55
CLEARANCE_WRIST_FINE_CONTROL_DT = 0.01
```

Rejected clearance experiments:

- Skipping wrist fine caused clearance to remain short and/or shake in later tests.
- Slowing clearance globally did not reliably remove light shake.
- Treating elbow as clamped at a hard physical limit was rejected because it caused undesirable side effects.

Current conclusion:

- Keep wrist fine enabled.
- Keep current slower wrist fine and softer wrist-side settings.
- Do not add elbow target clamp.

### Joint direction records

Physical direction mapping recorded from testing:

```text
shoulder_front positive: arm backward
shoulder_side positive: arm outward
elbow positive: arm forward
shoulder_rotate positive: inward rotate in web mapping after correction
arm_roll positive: inward roll
wrist_side positive: wrist outward
wrist positive: wrist forward
```

Correction:

- `shoulder_rotate` web buttons were reversed.
- Final web mapping:

```text
Shoulder Rotate Inward  -> +deg
Shoulder Rotate Outward -> -deg
```

### Independent nudge stability results

#### shoulder_rotate

- Initial independent `shoulder_rotate -5 deg` caused visible shake.
- Shake source was not `shoulder_rotate` itself; it excited `arm_roll`, `wrist_side`, and `wrist`.
- Final stable approach: keep `shoulder_rotate` active gains, but soften distal hold joints.

Current compliant holds for `shoulder_rotate` active:

```python
arm_roll: {"kp": 4.0, "kd": 6.0}
wrist_side: {"kp": 6.0, "kd": 1.2}
wrist: {"kp": 8.0, "kd": 1.4}
```

#### shoulder_side and elbow

- Independent `shoulder_side` and `elbow` movements caused distal chain shake, especially `arm_roll`.
- Added compliant distal holds for both active joints:

```python
arm_roll: {"kp": 2.5, "kd": 7.0}
shoulder_rotate: {"kp": 4.0, "kd": 6.0}
wrist_side: {"kp": 4.0, "kd": 1.6}
wrist: {"kp": 6.0, "kd": 2.0}
```

#### arm_roll

Initial issue:

- `arm_roll` negative direction shook both at clearance and home positions.
- Early hypothesis that `-3.0 rad` was a mechanical or motor limit was rejected:
  - with motor disabled, `arm_roll` could move past that area;
  - later lower-gain tests crossed the same area without shake.

Final conclusion:

- The shake was due to active `arm_roll` nudge gains being too stiff for negative-direction dynamic motion.
- Stable final active gains:

```python
NUDGE_GAINS["arm_roll"] = {"kp": 12.0, "kd": 7.0}
```

Rejected:

- `arm_roll` negative soft limit at `-2.98 rad`; removed after lower-gain validation.
- `arm_roll` 1-degree segmentation; removed after validation showed the low-gain continuous move is stable.

Final state:

- `arm_roll` has no soft clamp.
- `arm_roll` has no internal nudge segmentation.
- `arm_roll -5 deg` at home and clearance was stable with `kp=12`, `kd=7`.

Observed accuracy:

- Home test moved about `4.79 deg` for a requested `5 deg`, with no shake.
- Clearance test crossed past `-3.0 rad` and remained stable.

#### wrist

Stable wrist nudge:

```python
active wrist gains: {"kp": 26.0, "kd": 4.7, "seconds": 8.0}
active wrist tau: 0.55
```

`wrist backward 5 deg` at clearance:

- wrist reached target within about `0.13-0.15 deg`.
- Visual result: no visible drift in later validation.
- Added a small arm-roll hold during wrist movement:

```python
NUDGE_COMPLIANT_HOLD_GAINS_BY_ACTIVE["wrist"]["arm_roll"] = {"kp": 8.0, "kd": 6.0}
```

### Current end-of-day nudge constants

Important final values:

```python
NUDGE_HOLD_TAU = {
    "shoulder_front": 3.8,
    "elbow": 2.2,
    "wrist": 0.55,
}

NUDGE_ACTIVE_TAU = {
    "shoulder_front": 2.4,
    "wrist": 0.55,
}

NUDGE_GAINS = {
    "wrist": {"kp": 18.0, "kd": 1.2},
    "wrist_side": {"kp": 18.0, "kd": 1.2},
    "arm_roll": {"kp": 12.0, "kd": 7.0},
    "elbow": {"kp": 70.0, "kd": 3.0},
    "shoulder_front": {"kp": 140.0, "kd": 5.2},
    "shoulder_side": {"kp": 70.0, "kd": 3.0},
    "shoulder_rotate": {"kp": 60.0, "kd": 3.0},
}
```

Open item:

- `shoulder_front` independent nudge is still the least reliable/high-load case. Do not aggressively increase feedforward; earlier higher torque tests caused shake.

Follow-up tuning:

- `shoulder_front active_tau=1.5` caused shake and still sagged. Rejected.
- `shoulder_front kp=140.0, kd=6.2, active_tau=1.2` also caused shake. Rejected.
- Current rollback point: `kp=140.0, kd=5.2, active_tau=1.2`, no command bias, no segmentation.

Interpretation:

- Single-shot `shoulder_front +5 deg` at table clearance is close to the hardware/control stability boundary.
- Increasing feedforward or damping past the current rollback point causes shake.
- If independent `shoulder_front` control must remain available, safer options are:
  - restrict web nudge size for `shoulder_front` to `2 deg`, or
  - use a coupled micro-move with elbow/shoulder_side instead of pure single-joint motion.

## shoulder_front active torque ramp

Question:

- `shoulder_front` shakes when active torque is greater than about `1.2`.
- Instead of increasing constant torque, optimize the transition that excites the shake.

Change:

```python
NUDGE_ACTIVE_TAU_RAMP_SECONDS = {
    "shoulder_front": 0.8,
}
```

Implementation:

- `move_target_with_holds` now accepts `active_tau_ramp_seconds`.
- During preload, active torque ramps from `0` to the configured `active_tau` using `smoothstep`.
- During motion, the same ramp clock continues, so torque does not step sharply.

Current recommended validation:

- Keep `shoulder_front active_tau=1.2` first.
- Verify the ramp does not introduce shake.
- Only if stable, try a small increase such as `1.3`, not directly `1.5`.

No target bias is used.
