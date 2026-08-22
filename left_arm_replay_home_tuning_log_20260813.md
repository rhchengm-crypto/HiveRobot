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

## 2026-08-15 pawn02 replay order investigation

Confirmed current v2.6 replay path:

```text
Table Clearance Move
shoulder_front -> shoulder_side -> shoulder_rotate -> elbow -> arm_roll -> wrist_side -> wrist
```

The first six joints are replayed as one-active-joint steps. During each step, previously completed joints hold their saved targets and pending joints hold their current positions. The final `wrist` step is special-cased and holds every other joint at the saved pose.

Added `describe-replay --name <move-name>` to `left_arm_v2_6_move_library.py` so `pawn02` can be inspected from the saved moves file without moving the arm. The command prints table-clearance, active joint, completed hold targets, pending hold targets, final wrist phase, and whether the pose is detected as low-shoulder.

Changed the v2.6 default replay trial to couple every non-wrist replay joint:

```text
Table Clearance Move
shoulder_front + shoulder_side + shoulder_rotate + elbow + arm_roll + wrist_side
wrist
```

During the coupled non-wrist phase, `wrist` holds its current angle. The final `wrist` phase still uses the prior special final-wrist hold logic. Use `--sequential` on `replay-move` or `describe-replay` to use the previous one-joint-at-a-time path.

Updated `Home Move` pre-home clearance only; formal home is unchanged. New pre-home sequence:

```text
wrist
wrist_side + arm_roll + elbow + shoulder_rotate + shoulder_side + shoulder_front
formal home unchanged
```

The wrist moves to table clearance first while the other joints hold current position. After wrist reaches clearance, the remaining pre-home clearance joints move as one coupled group while wrist holds the clearance target.

After a `capture-clearance` permission failure on `/home/nvidia/hive_robot/DM_Control_Python/data/left_arm_v2_table_clearance.json`, v2.6 web control was changed to force non-sudo command execution. The `--sudo` flag is now accepted only for backward-compatible startup commands and is ignored, so capture/replay/home writes stay owned by the `nvidia` user. Existing root-owned data files still need a one-time ownership fix on the Orin:

```bash
sudo chown -R nvidia:nvidia /home/nvidia/hive_robot/DM_Control_Python/data
```

Added a gated replay correction pass before final wrist. The pass does not run just because an error exists; it first checks whether continued learning looks ineffective. For `shoulder_side` and `elbow`, correction is allowed only when:

```text
abs(before_final_wrist_error) >= 1.5 deg
target-bias samples >= 3
learning_state is plateau or backoff
step_scale <= 0.5
```

When those conditions hold, replay prints `v2.6 replay correction decisions=...` and moves the selected joints as a coupled correction group before the final `wrist` move. An initial trial included `arm_roll`, but correction pulled `arm_roll` from about `+2.08 deg` residual to about `+3.74 deg` and also disturbed `shoulder_rotate`; `arm_roll` was therefore removed from the correction group and left to the existing final-wrist hold-target compensation path.

Follow-up trial with only `shoulder_side + elbow` still disturbed `arm_roll`, pulling it to about `+5.33 deg` after correction. Disabled replay correction by default (`REPLAY_CORRECTION_ENABLED = False`). The decision log remains useful for diagnosis, but replay no longer applies the correction move before final wrist.

## 2026-08-15 pawn02 wrist active softening

Softened `wrist` active motion in both paths that can reach a pawn replay pose:

```text
Replay final wrist
Independent nudge-hold wrist
```

Added `WRIST_ACTIVE_SOFT_GAINS = {"kp": 22.0, "kd": 5.2, "seconds": 10.0}` in `left_arm_v2_6.py`. `left_arm_v2_6_move_library.py` now uses that value from `active_replay_gains()` and `replay_seconds()` for the final replay wrist phase.

Reduced base `wrist` active feed-forward tau from `0.55` to `0.40`, within the suggested `0.35-0.45` range. Clearance wrist fine correction still keeps its separate `CLEARANCE_WRIST_FINE_GAINS` / `COUPLED_CLEARANCE_WRIST_FINE_GAINS` constants so this change does not retune the clearance-specific path.

Follow-up observation: shaking disappeared, but independent `wrist` nudge felt stick-slip / one-step-at-a-time. The 5 degree web nudge was being forced from the web-requested `6.0s` to the replay-oriented `10.0s` minimum, which made the wrist crawl at about `0.5 deg/s`.

Changed independent `nudge-hold` wrist timing to use `WRIST_ACTIVE_NUDGE_MIN_SECONDS = 6.0` while keeping replay final wrist on the slower `WRIST_ACTIVE_SOFT_GAINS` `10.0s` window. This separates small manual wrist nudges from large replay wrist moves without changing gains or tau again.

Follow-up replay still felt stick-slip compared with the pre-softening wrist motion. Kept the softer `kp=22.0` and `10.0-12.0s` replay timing, but moved wrist damping and active feed-forward closer to the previously smooth values: `kd=4.7`, `active_tau=0.45`.

That trial restored smoothness but also brought shaking back. A midpoint damping/feed-forward pair (`kd=5.0`, `active_tau=0.42`) shook even more, so active tau increase is the wrong direction for this posture. Reverted to the non-shaking pair (`kd=5.2`, `active_tau=0.40`) and reduced replay final wrist from a `12.0s` cap to `10.0s` to avoid very slow stick-slip without adding torque.

Follow-up independent `wrist` nudge with `kd=5.2`, `active_tau=0.40`, and `6.0s` no longer shook, but still felt stick-slip. Split manual wrist nudge gains from replay final wrist: replay keeps `WRIST_ACTIVE_SOFT_GAINS` (`kp=22.0`, `kd=5.2`, `10.0s`) for stability, while independent wrist nudge uses `WRIST_ACTIVE_NUDGE_GAINS` (`kp=22.0`, `kd=4.7`) with the same `active_tau=0.40` and `6.0s` minimum.

Independent `wrist` nudge with `kp=22.0`, `kd=4.7`, `active_tau=0.40`, and `6.0s` was observed as smooth with no shaking and no stick-slip. Copied the same data to replay final wrist by changing `WRIST_ACTIVE_SOFT_GAINS` to `kp=22.0`, `kd=4.7`, `seconds=6.0`, `max_seconds=6.0`.

Follow-up replay showed that the independent nudge sweet spot does not transfer directly to the large final wrist move: replay final wrist moves about `44.6 deg`, and `6.0s` brought shaking back. Kept independent nudge at the observed-good `kp=22.0`, `kd=4.7`, `6.0s`, but retuned replay final wrist to `kp=22.0`, `kd=5.2`, `active_tau=0.40`, `seconds=8.0`, `max_seconds=8.0`.

Replay final wrist with `kp=22.0`, `kd=5.2`, `active_tau=0.40`, and `8.0s` removed shaking but still had stick-slip. Rather than raising active tau again, changed only the final-wrist trajectory from pure `smoothstep` to `blend_smootherstep` with `linear_blend=0.35`, so the command does not linger as long in near-zero-velocity start/end regions.

The blended trajectory still had stick-slip, so the remaining likely culprit is replay wrist damping. Changed replay final wrist from `kd=5.2` to the independently smooth `kd=4.7`, while keeping the replay-only slower `8.0s` duration and `active_tau=0.40`. Added a replay final wrist trajectory debug line so future logs show the active trajectory and linear blend.

Follow-up replay with `kp=22.0`, `kd=4.7`, `active_tau=0.40`, `8.0s`, and blended trajectory removed stick-slip but had light shaking. Increased replay-only wrist damping slightly to `kd=4.9`, leaving independent wrist nudge at the observed-good `kd=4.7`.

Follow-up at `kd=4.9` still had light shaking. Increased replay-only wrist damping one more small step to `kd=5.0`, while keeping active tau at `0.40` because earlier tau increases brought shaking back more strongly.

Follow-up at `kd=5.0` still shook, so lowering replay wrist damping is not the right fix for the large final wrist move. Reverted replay final wrist to the last no-shake damping, `kd=5.2`, and shortened replay duration from `8.0s` to `7.0s` to reduce stick-slip by moving through the low-speed friction region faster without adding active tau.

Replay final wrist at `kp=22.0`, `kd=5.2`, `active_tau=0.40`, `7.0s`, and blended trajectory did not shake, but still had stick-slip. Kept the no-shake damping/tau pair and shortened replay final wrist to `6.5s` for the next test.

Replay final wrist at `kp=22.0`, `kd=5.2`, `active_tau=0.40`, `6.5s`, and blended trajectory still did not shake, but stick-slip remained. Since shortening time did not clear it, lowered replay-only wrist stiffness from `kp=22.0` to `kp=20.0` while keeping `kd=5.2`, `active_tau=0.40`, and `6.5s`.

Replay final wrist at `kp=20.0`, `kd=5.2`, `active_tau=0.40`, `6.5s`, and blended trajectory still did not shake, but stick-slip remained. Enabled replay-only wrist velocity feed-forward and updated `move_target_with_holds()` so velocity feed-forward follows the configured `blend_smootherstep` trajectory instead of forcing pure `smootherstep`.

Table clearance wrist motion was observed to be both smooth and no-shake. Switched replay final wrist to mirror the table-clearance coupled wrist profile instead of continuing the high-damping replay search: `kp=22.0`, `kd=1.4`, `seconds=12.0`, `max_seconds=12.0`, `blend_smootherstep`, `linear_blend=0.35`, no replay wrist velocity feed-forward, and replay-only `active_tau=0.0`. The global wrist active tau remains `0.40` so independent wrist nudge keeps its observed-good behavior.

This table-clearance-style replay wrist was smooth and no-shake, but final wrist residual rose to about `1.55 deg`. The log showed `adaptive target bias pending hold-tau convergence` for `wrist`, which is too conservative for the active final wrist joint. Added `force_joints` to `update_adaptive_target_bias()` and force-enabled target-bias learning for `REPLAY_FINAL_JOINT` during replay final error handling, while keeping the hold-tau gate for held joints.

Policy update: adaptive-first for replay final accuracy. Any joint with final residual beyond the target-bias deadband should keep learning target bias; replay final target-bias updates no longer wait for hold-tau convergence. The hold-tau gate remains available for other callers through `require_ready=True`, but replay final calls `update_adaptive_target_bias(..., require_ready=False)`.

## 2026-08-16 pawn02 adaptive replay order

Changed default replay from the fixed all-non-`wrist` coupled trial to an adaptive non-`wrist` plan. Replay now measures current deltas after table clearance, skips joints inside deadband, and chooses either one soft group for small total moves or two supported groups for larger pawn-table moves:

```text
shoulder_front + shoulder_side + elbow
shoulder_rotate + arm_roll + wrist_side
wrist
```

The second group holds the first group's completed targets, so the order is softer than one-joint-at-a-time replay but still keeps load-bearing joints anchored before the distal joints move. Replay prints `v2.6 replay adaptive plan ...` with deltas, groups, and thresholds before executing.

`--sequential` still runs the legacy one-active-joint path. Added `--coupled` to keep the earlier all-non-`wrist` coupled trial available for A/B testing.

Follow-up replay showed the adaptive planner was stable, but final `wrist` residual stayed fixed at about `+1.55 deg` across runs. The log learned `wrist` target bias, but `run_final_wrist_move()` still targeted the raw saved `wrist` angle. Fixed the final wrist path to apply `adaptive_target_bias_for("wrist", ...)` before computing delta and to print `v2.6 replay final wrist adaptive target bias ...` when active.

Follow-up manual `arm_roll` nudge in the low-shoulder pawn posture showed the active joint itself missed badly: a positive `+5 deg` nudge used fixed `active_tau=-0.45`, but `arm_roll` moved slightly farther negative instead of toward target. Existing adaptation learned held-joint drift, target bias, and hold-target bias, but did not learn active feed-forward torque. Added direction-specific adaptive active tau in `scripts/data/left_arm_v2_6_adaptive_active_tau.json`; `nudge_active_tau_for()` now applies learned tau for posture + joint + direction, and both `nudge-hold` and adaptive replay group moves update it from active joint residuals. `arm_roll` active tau limits were widened from `(-0.8, -0.2)` to `(-0.8, 0.2)` so learning can back a wrong negative tau toward zero.

Follow-up `shoulder_side` residual stayed around `+1.07 deg` while target-bias samples increased, because plateau handling reset target-bias updates back to `best_bias_deg` before adding the next small step. Changed only `update_adaptive_target_bias()` plateau behavior: clear worsening still backs off to the best bias, but a non-worsening plateau now continues from the current bias with a reduced step scale. Hold-target-bias learning remains conservative.

Follow-up independent `wrist_side +5 deg` nudge showed `arm_roll` held-joint error increasing while `update_adaptive_hold_target_bias()` was in `backoff`. The cause was that the function reset to `best_bias_deg` on worse samples, then immediately added another same-direction bias step from the current error. Changed hold-target-bias backoff to truly return to `best_bias_deg` without adding a same-direction step; non-worsening samples keep the previous conservative behavior.

Follow-up repeated `wrist_side +5 deg` nudges still held `arm_roll` around `+1.70 deg` error because the sample was classified as plateau rather than worse, and plateau from `best_bias_deg=1.95` still added a step back to `2.10`. Tightened hold-target-bias plateau handling: if a sample does not refine the best error, write back `best_bias_deg` instead of adding another step. This keeps exploratory hold-target bias from staying above the best observed setting.

Policy update for independent nudge held-joint learning: tune `hold_tau` first, then use `hold_target_bias` only when hold-tau learning has no update to make and the existing active/held record is converged. This prevents a single nudge from changing both `arm_roll` hold torque and `arm_roll` hold target bias at the same time.

Follow-up showed hold-tau still wrote `-0.54` after `-0.54` performed worse than `-0.52`, because the worse path reset its base to `best_tau=-0.53` and then applied one more correction-direction step. Changed hold-tau worse/non-best plateau handling to write back `best_tau` directly. The update record now reports `previous_tau` as the actually applied tau and includes `base_tau` separately.

Correction to the previous hold-tau policy: simply writing back `best_tau` stalls exploration. Hold-tau now tracks `successful_direction`; when a sample refines the best error, the direction of the applied tau change is saved. Later worse/plateau samples start again from `best_tau` and try one small step in that successful direction. For example, if `-0.52` is best and `-0.53` is worse, the next tau becomes `-0.51`, not `-0.54`.

Compatibility fix: existing adaptive hold-tau records do not have `successful_direction`. When that field is missing, infer the direction from `best_tau - applied_tau` before falling back to the previous delta. This prevents old records such as `best_tau=-0.53` with applied `-0.54` from recomputing `-0.54` and silently skipping the update.

Follow-up showed `-0.52` improved over the previous `-0.54` run but did not beat the historical best, so it was still pulled back toward `best_tau=-0.53`. Added `recent_improved`: when the current sample improves over the immediately previous sample by the improvement deadband, keep moving in the previous successful direction from the applied tau even if the historical best was not refreshed. This lets `-0.52` lead to a test at `-0.51`.

Follow-up `-0.51` matched the previous `-0.52` error without worsening, but plateau handling still pulled exploration back toward `best_tau`. If `successful_direction` exists and the latest sample is not worse, hold-tau now continues from the applied tau in that direction. This lets `-0.51` lead to a test at `-0.50`.

Follow-up at `arm_roll` hold error about `1.03 deg` showed hold-tau was effectively at the edge of usefulness. Added a near-threshold handoff: when hold-tau step scale is already minimal and both best/current held error are `<=1.1 deg`, `update_adaptive_hold_tau()` skips further tau exploration. Because no hold-tau update is produced, the nudge path can hand off to hold-target-bias learning for the remaining small residual.

Policy correction: handoff to hold-target-bias is now based on whether the current hold-tau attempt improved, not on a stricter convergence label alone. Replay and nudge-hold wait only when the hold-tau update reports `improved`, `recent_improved`, or `refined_best`; `plateau`, `backoff`, or no hold-tau update allow hold-target-bias learning for the remaining residual.

Follow-up replay reached all joints within about `1 deg`, but `shoulder_front` visibly dropped when replay restarted after the table-clearance subprocess. Added a replay entry takeover hold: after the replay process reopens/enables the motors, it holds the current post-clearance pose for `0.5s` before reporting `after_clearance` and starting group-1. Also added replay-only `shoulder_front` descent support tau (`+1.0`, matching the tested clearance support direction) so the first active shoulder-front move does not begin with zero support torque. This support tau is excluded from active-tau learning so it does not pollute direction-specific adaptive tau records.

Follow-up still showed a handoff drop. The log confirmed group-1 was using `shoulder_front` active tau `+1.0`, while replay entry takeover was holding with `+3.0`, so the transition still had a support-torque step. Raised replay-only `shoulder_front` descent support tau to `+3.0` and extended the group-1 preload to `0.8s` whenever `shoulder_front` is active. The next replay should print `v2.6 replay adaptive group shoulder_front preload_seconds= 0.8` before group-1 starts.

Follow-up with `shoulder_front` support tau `+3.0` changed the symptom to drop-then-lift and final `shoulder_front` overshoot, showing constant `+3.0` is too much once the active move is underway. Found the preload bug: `move_targets_with_holds()` used zero feed-forward tau for active joints during preload, so extending preload made the shoulder-front active joint hold current position with `tau=0` before the trajectory started. Fixed group preload to apply active feed-forward tau, then changed replay shoulder-front descent to ramp smoothly from `+3.0` at takeover/preload to `+1.0` over the first `25%` of the move. The support ramp remains excluded from active-tau learning.

## 2026-08-16 final-wrist elbow hold softening

Replay log analysis: the hard elbow feel occurs in the final `wrist` phase, but elbow is not active there. It is being held while wrist moves. Before final wrist, elbow was already inside tolerance at about `-0.68 deg`; final wrist then held it with completed-joint settings (`kp=78.0`, `kd=7.2`, `tau=5.1`). That is accurate, but mechanically stiff enough that wrist motion can transmit as whole-arm vibration even when the elbow motor itself does not visibly shake.

Also noticed a small target handoff: adaptive group replay can complete elbow at its biased target, then final wrist previously rebuilt hold targets from the raw saved pose. In the sample log that snap was only about `0.146 deg`, so it is probably not the main cause, but removing the discontinuity is still the right direction.

Changed final wrist to carry forward completed non-wrist targets from the adaptive/sequential replay phase, so elbow and shoulder holds do not snap back to nominal before wrist starts. Added final-wrist-only elbow soft hold overrides: `kp=45.0`, `kd=5.0`, and tau capped at `4.2`. The next replay should print `v2.6 replay final wrist carried hold targets=...` when a biased completed target is reused, and `v2.6 replay final wrist soft hold overrides ...` when the elbow hold softening is active.

Follow-up replay showed wrist-stage shaking returned. The log revealed the active final `wrist` line had become `kp=45.0`, `kd=5.0`; that was not the intended elbow hold softening. Root cause: the loop that applied soft hold gains used the local variable name `gains`, overwriting the active wrist `gains` dict with the elbow hold override. Renamed that loop variable and added a final-wrist active-gain guard. The intended previous no-shake final wrist active profile remains `kp=22.0`, `kd=1.4`, `seconds=12.0`, active tau `0.0`, and `blend_smootherstep`.

Follow-up confirmed final wrist active returned to `kp=22.0`, `kd=1.4` and wrist shaking disappeared, but the elbow hard feel did not improve. The reason is that final wrist starts after group-2 has already held elbow with completed-hold settings (`kp=78.0`, `kd=7.2`, `tau=5.1`), so softening only the final phase is too late and still asks elbow to correct a small sub-degree residual. Added group-2 soft completed elbow hold when active is `wrist_side` (`kp=45.0`, `kd=5.0`, tau cap `4.2`). Final wrist now uses a much softer elbow hold (`kp=25.0`, `kd=3.0`, tau cap `3.2`) and freezes elbow at its current angle when the pre-final-wrist nominal error is within `1.2 deg`, so the final wrist phase does not try to correct an already acceptable elbow residual.

Follow-up showed elbow felt better, but did not reach position: before final wrist and final residual were both about `+2.06 deg`, and final status had elbow at `0.6113 rad` versus saved `0.6476 rad` / biased `0.6450 rad`. That means the previous softening was too soft during group-2 and let elbow drift before final wrist. Important correction: this should be solved by adaptive learning, not by repeatedly hand-tuning the constants or raising hold stiffness back to a hard feel. For soft elbow holds, keep low-gain/low-tau caps (`kp=25.0`, `kd=3.0`, tau cap `3.2`) and learn hold-target bias instead of hold-tau. Adaptive group and final wrist now apply learned `adaptive_hold_target_bias_for(active, elbow, ...)` before moving, and after the move update `update_adaptive_hold_target_bias(...)` from elbow residuals. Soft elbow holds are excluded from hold-tau learning so the system does not respond to missing position by increasing torque into a hard top.

Follow-up showed hold-target-bias observations for elbow (`~4.3-4.7 deg`) but no update lines and no bias file. Root cause: `update_adaptive_hold_target_bias()` treated an empty `{}` as an existing previous record, set `previous_error` to the current error, then chose `new_bias = best_bias = current_bias = 0`, so the first real sample was skipped. Fixed the first-sample path by adding `has_previous = isinstance(previous, dict) and bool(previous)` and only using the conservative best-bias rollback when a non-empty record exists. The next first elbow hold-target-bias sample should now write an initial bias and print `v2.6 replay adaptive hold target bias updates ...`.

Policy correction: replay should tune holding first, and only use hold-target-bias after holding no longer moves the residual. Reinstated that ordering for soft elbow holds. The soft gain remains gentle (`kp=25.0`, `kd=3.0`), but soft tau caps now only protect the no-record first run; once an adaptive hold-tau record exists, the learned holding tau is allowed through and logged with `soft_cap_skipped`. Both applying existing hold-target-bias and writing new hold-target-bias are now gated by `adaptive_hold_tau_converged_for_active(...)`; before that, replay prints pending/waiting messages and keeps tuning holding.
