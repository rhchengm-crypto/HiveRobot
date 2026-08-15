# Left Arm Replay / Home Tuning Log - 2026-08-12 to 2026-08-13

## Scope

This note records the two-day tuning pass around `left_arm_v2_5_3.py`, web controller 1.2, saved move replay, home safety, and low-shoulder nudge stability.

Main goals:

- Add web saved-move capture/replay for named poses such as `pawn02` and `pawn03`.
- Make replay reach saved poses without relying on settle correction.
- Keep low-shoulder poses stable while moving `wrist_side`, `wrist`, `shoulder_front`, and `arm_roll`.
- Make `Home Move` safer after independent movements from low table poses.

## Web Controller 1.2

Added web controller 1.2 files:

```text
scripts/left_arm_v2_5_3_web_control_1_2.py
scripts/LEFT_ARM_V2_5_3_WEB_CONTROL_1_2.md
scripts/left_arm_v2_5_3_move_library.py
```

The controller keeps the HP60C RGB-D stream and debug panel, and adds saved move workflow:

```text
Low Torque -> manually pose arm -> Capture New Move -> Replay
```

Saved moves are stored in:

```text
scripts/data/left_arm_v2_5_3_saved_moves.json
```

Replay sequence:

```text
Table Clearance Move
shoulder_front -> shoulder_side -> shoulder_rotate -> elbow -> arm_roll -> wrist_side -> wrist
```

## Replay Findings

`pawn02` initially replayed the newly captured pose, but `arm_roll` and `shoulder_rotate` drifted after `wrist_side` / `wrist` motion.

Important observation:

- Settle could pull the pose closer after the fact, but the correct fix was to hold completed joints during later moves.
- Replay settle is now disabled by default:

```python
REPLAY_SETTLE_ENABLED = False
```

Validated direction:

- `arm_roll` should be held during low-shoulder `wrist_side` and final `wrist` moves.
- `elbow` height is sensitive and must be held with enough static torque.
- `shoulder_rotate` should not be allowed to drift during final wrist motion.

Current replay hold tuning:

```python
REPLAY_COMPLETED_HOLD_GAINS = {
    "shoulder_front": {"kp": 75.0, "kd": 7.0},
    "shoulder_side": {"kp": 65.0, "kd": 4.5},
    "shoulder_rotate": {"kp": 8.0, "kd": 4.0},
    "elbow": {"kp": 78.0, "kd": 7.2},
    "arm_roll": {"kp": 6.0, "kd": 6.0},
    "wrist_side": {"kp": 8.0, "kd": 1.6},
    "wrist": {"kp": 8.0, "kd": 1.4},
}

REPLAY_COMPLETED_HOLD_TAU = {
    "shoulder_front": 2.4,
    "elbow": 5.1,
    "wrist": 0.55,
}
```

During `shoulder_rotate` active replay, `elbow` was visibly sagging before it became a completed joint. Added a pending hold override only for that phase:

```python
REPLAY_PENDING_HOLD_GAINS_BY_ACTIVE = {
    "shoulder_rotate": {
        "elbow": {"kp": 92.0, "kd": 6.2},
    },
}

REPLAY_PENDING_HOLD_TAU_BY_ACTIVE = {
    "shoulder_rotate": {
        "elbow": 3.2,
    },
}
```

`pawn03` validation:

- Final wrist executed instead of skipping.
- All joints landed inside 1 degree.
- `elbow` final error was about `0.31 deg` in one run and later `0.13 deg` after `elbow` completed hold tau was raised to `5.1`.

## Low-Shoulder Nudge Tuning

Low shoulder is detected when:

```python
shoulder_front < 1.8
```

Low-shoulder `wrist_side` tuning:

- `wrist_side` active uses stronger low-shoulder gains.
- Fine pass is allowed when the remaining wrist-side error is outside a small deadband.
- Completed `arm_roll`, `shoulder_rotate`, and `shoulder_side` are held with static torque during `wrist_side` and final `wrist`.

Key low-shoulder constants:

```python
NUDGE_LOW_SHOULDER_WRIST_HOLD_TAU = {
    "arm_roll": -0.55,
    "shoulder_rotate": -0.8,
    "shoulder_side": 0.7,
}
```

`arm_roll` independent nudge issue:

- UI direction and target math were correct.
- Actual `arm_roll` initially moved toward increasing position for both inward and outward commands.
- Cause was low-shoulder coupling and insufficient active compensation, not button sign.

Current low-shoulder `arm_roll` nudge tuning:

```python
NUDGE_LOW_SHOULDER_ARM_ROLL_ACTIVE_TAU = -0.65
NUDGE_LOW_SHOULDER_ARM_ROLL_HOLD_TAU = {
    "elbow": 5.1,
    "shoulder_rotate": -0.8,
    "wrist_side": -0.25,
}
```

When `shoulder_front` is nudged in a low-shoulder pose, `arm_roll` and `shoulder_rotate` used to drift badly. Low-shoulder hold overrides now harden the relevant joints:

```python
NUDGE_LOW_SHOULDER_COMPLIANT_HOLD_GAINS_BY_ACTIVE = {
    "shoulder_front": {
        "arm_roll": {"kp": 12.0, "kd": 7.0},
        "shoulder_rotate": {"kp": 12.0, "kd": 5.0},
        "wrist_side": {"kp": 8.0, "kd": 1.6},
    },
    "arm_roll": {
        "shoulder_rotate": {"kp": 12.0, "kd": 5.0},
        "wrist_side": {"kp": 8.0, "kd": 1.6},
        "wrist": {"kp": 6.0, "kd": 2.0},
    },
}
```

## Home / Pre-Home Tuning

Observed problem from `pawn03 -> home`:

- `Home Move` did not take the coupled home path because `shoulder_side` was inside the `0.5 deg` deadband and was skipped.
- The old coupled trigger required `elbow + shoulder_front + shoulder_side` all to be in targets.
- Result: formal home fell back to sequential per-joint moves, and final `arm_roll` home showed visible shake.

Fix:

- Coupled home now triggers when `elbow + shoulder_front` need home.
- `shoulder_side` is included in the coupled group even if it was inside deadband, so the group remains synchronized.

Pre-home `arm_roll` was also too abrupt:

- It moved about `25 deg` with only `1.2 s`.
- Pre-home clearance now gives `arm_roll` dedicated timing based on delta, up to `20 s`.

Home `arm_roll` fallback was softened:

```python
HOME_GAINS["arm_roll"] = {"kp": 18.0, "kd": 6.0, "seconds": 12.0}
```

## Current Validation Targets

For the next session:

- Re-test `arm_roll outward -2 deg`; target is closer than the previous `1.5 deg` shortfall.
- Re-test `shoulder_front forward -2 deg`; target joint should remain accurate and `arm_roll` should not drift by multiple degrees.
- Re-test `pawn03 -> Home Move`; expected log should include:

```text
v2.5.3 home coupled shoulder/roll/wrist active
```

- Continue watching for high-frequency elbow or arm-roll shake when low-shoulder static torques are active.

## Validation Commands

Code syntax check used during tuning:

```text
python -m py_compile scripts/left_arm_v2_5_3.py scripts/left_arm_v2_5_3_move_library.py
```

## 2026-08-14 v2.6 nudge-hold adaptive offset

Added v2.6 controller/script copies and web-control notes for low-shoulder adaptive hold compensation.

Observed `shoulder_front:-2 deg` from home/low-shoulder posture:

- `arm_roll` initially drifted about `2.5-2.7 deg`.
- Hold-tau learning plateaued around `arm_roll tau=-0.545..-0.565`.
- The first implementation waited for a run without a same-run hold-tau update, so it stayed pending while tau explored around the best value.
- Changed nudge-hold learning so a same-run hold-tau record that reaches `samples>=3` and `plateau/backoff` can immediately enter hold-target offset learning.
- Changed hold-target offset application so an existing offset is applied immediately on matching nudges, even while hold-tau continues exploring.

Current useful learned value from robot-side data:

```text
shoulder_front:negative -> arm_roll hold-target bias ~= +2.2933 deg
arm_roll hold_tau ~= -0.545..-0.565
```

Validation after homing:

- With `arm_roll` hold-target bias `+1.9086 deg`, true `arm_roll` drift dropped to about `1.09 deg`.
- With `arm_roll` hold-target bias `+2.2933 deg`, true `arm_roll` drift was about `0.24-0.46 deg` on later home-near `shoulder_front:-2 deg` tests.

Important interpretation:

- Adaptive records are keyed by active joint and direction, e.g. `shoulder_front:negative -> arm_roll`.
- The `wrist:positive -> arm_roll` case is separate. First observed `wrist:+2 deg` caused about `2.73 deg` `arm_roll` drift and started its own hold-tau learning at sample 1.
- Current low-shoulder detection is a simple absolute shoulder-front threshold: `shoulder_front < 1.8 rad` (`103.1 deg`). Home is about `1.517 rad` (`86.9 deg`), so home is currently treated as low-shoulder.

Follow-up:

- Consider adding posture buckets based on `shoulder_front` offset from home or ranges, because one low-shoulder bucket may be too broad across all chess-piece positions.
- Consider offset-learning hysteresis, for example no additional offset learning for residuals in a `1.0-1.3 deg` gray band, to avoid chasing edge noise.
