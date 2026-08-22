# Left Arm v2.6 Web Control v2.6

Browser control panel 2.6 for `left_arm_v2_6.py` with HP60C RGB-D video, saved move capture, and replay.

## Start

Run on the Orin where HP60C ROS topics and the left-arm CAN serial device are available:

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 left_arm_v2_6_web_control.py --enable-execute
```

Open from another computer:

```text
http://<orin-ip>:8091/
```

The web server defaults to running arm commands directly with unbuffered Python:

```text
python3 -u left_arm_v2_6.py ...
```

The v2.6 web controller always runs arm commands as the current user, even if an old startup command still includes `--sudo`. If the serial device permissions require root, fix the device/group permissions instead of running this controller with sudo.

```bash
sudo chown -R nvidia:nvidia /home/nvidia/hive_robot/DM_Control_Python/data
```

## Video

The main image is `/stream.mjpg`, a side-by-side RGB and depth visualization.

Direct streams:

```text
/stream.mjpg  RGB + depth
/rgb.mjpg     RGB only
/depth.mjpg   depth colormap only
```

Default ROS topics:

```text
/ascamera_hp60c/rgb0/image
/ascamera_hp60c/depth0/image_raw
```

## Buttons

| Button | Command |
| --- | --- |
| Capture New Home | `python3 -u left_arm_v2_6.py capture-home --note web-control` |
| Capture New Clearance | `python3 -u left_arm_v2_6.py capture-clearance --note web-control` |
| Capture Claw Home | `python3 -u left_arm_v2_6.py capture-claw-home --note web-control` |
| Table Clearance Move | `python3 -u left_arm_v2_6.py clearance --deadband-deg 0.5 --max-delta-deg 120 --step-deg 5 --execute` |
| Home Move | `python3 -u left_arm_v2_6.py home --deadband-deg 0.5 --max-delta-deg 120 --prehome-clearance --execute` |
| Claw Home | `python3 -u left_arm_v2_6.py claw-home --execute` |
| Claw Close | `python3 -u left_arm_v2_6.py claw-close --execute` |
| Low Torque | `python3 -u left_arm_v2_6_move_library.py low-torque` |
| Capture New Move | `python3 -u left_arm_v2_6_move_library.py capture-move --name <move-name> --note web-control-2.6` |
| Replay | `python3 -u left_arm_v2_6_move_library.py replay-move --name <move-name>` |
| Describe Replay | `python3 -u left_arm_v2_6_move_library.py describe-replay --name <move-name>` |
| Emergency Stop | terminates the current web-started command, then directly disables all left-arm and claw motors |

## Saved Moves

Saved moves are stored in:

```text
scripts/data/left_arm_v2_6_saved_moves.json
```

To reuse saved moves captured with v2.5.3, merge the legacy file into the v2.6 file:

```bash
python3 -u left_arm_v2_6_move_library.py migrate-moves
```

By default this keeps existing v2.6 moves when names collide. Add `--overwrite` only when you intentionally want the old v2.5.3 record to replace the v2.6 record.

Workflow:

1. Click `Low Torque`.
2. Manually move the arm to the target grasp pose.
3. Type a move name.
4. Click `Capture New Move`.
5. Choose the saved move from the dropdown.
6. Click `Replay`.

`Capture New Move`, `Replay`, and joint nudge commands can interrupt a running `Low Torque` command before starting.

Replay safety sequence:

1. Replay always runs `Table Clearance Move` first.
2. After table clearance is reached, replay builds an adaptive non-`wrist` plan from the current deltas. Small moves can run as one soft group; larger pawn-table moves split into supported groups so completed joints keep holding their saved targets:

```text
shoulder_front + shoulder_side + elbow
shoulder_rotate + arm_roll + wrist_side
```

3. Replay then runs the final `wrist` move while holding the other joints at the saved pose.

Use `describe-replay --name pawn02` to print the replay phases for a saved move without moving the arm. Add `--sequential` to describe or run the previous one-joint-at-a-time path, or `--coupled` to use the earlier all-non-`wrist` coupled trial path.
Replay uses the same anti-shake nudge-hold gains and adaptive posture rules used by independent joint control.
The `wrist` active phase is softened for both replay and independent `nudge-hold`. Replay final wrist now mirrors the smooth table-clearance wrist profile: `kp=22.0`, `kd=1.4`, `12.0s`, no wrist active tau, `blend_smootherstep` with `0.35` linear blend, and no wrist velocity feed-forward. Independent wrist nudge keeps the observed-good `kp=22.0`, `kd=4.7`, `6.0s`, and active tau `0.40`.
For low-shoulder saved moves such as pawn table poses, replay applies the v2.6 low-shoulder adaptive hold-gain table used by `nudge-hold`; the debug output prints `v2.6 replay low shoulder adaptive hold gains=` when those rules are active.
Replay also observes low-shoulder hold-joint errors and micro-adjusts persistent hold-tau overrides in `scripts/data/left_arm_v2_6_adaptive_hold_tau.json`; it records the actual applied tau with best tau/error, accepts small best-error refinements, steps from the best observed value once samples plateau or get worse, reverses search direction after non-improving samples, keeps a minimum exploration step so plateau searches are not too timid, and does not run an extra settle motion.
If a final residual of at least 1.0 degree remains, replay learns a feed-forward target bias in `scripts/data/left_arm_v2_6_adaptive_target_bias.json`; the bias is applied next time when that joint reaches its own active replay step. Replay final target-bias learning is adaptive-first: any joint that is not at target keeps learning from final residuals without waiting for hold-tau convergence.
Before the final wrist move, replay checks `shoulder_side` and `elbow` for plateaued residuals. The correction pass is currently disabled by default after coupled correction trials disturbed `arm_roll`; replay still prints `v2.6 replay correction decisions=...` for diagnosis, then continues to the final `wrist` move without applying correction.
Replay skips target-bias learning for joints that were already within that threshold before the final wrist motion but were pulled out of tolerance by the wrist move; those cases are logged as `v2.6 replay adaptive target bias skipped final-wrist induced errors` and should be handled by final-wrist hold compensation instead.
For those final-step induced errors, replay also learns a bounded per-active/per-held-joint hold-target offset in `scripts/data/left_arm_v2_6_adaptive_hold_target_bias.json`; when present, the next matching active move prints `v2.6 replay final wrist adaptive hold target bias=` before applying the adjusted hold target.
The final active `wrist` joint follows the same adaptive-first rule and learns its own replay target bias immediately from final residual error.
`nudge-hold` first learns adaptive hold-tau for low-shoulder manual nudge induced hold drift using the target-bias threshold as its nudge-induced deadband; it learns a bounded hold-target offset once that active/held-joint hold-tau path has enough samples and reaches plateau/backoff, including the run that first reports the converged hold-tau state. Once a hold-target offset exists, later matching nudges apply it immediately while hold-tau can continue exploring.
Manual nudge adaptation is direction-specific: positive and negative nudges use separate active keys such as `wrist_side:positive` and `wrist_side:negative`, so compensation learned in one direction is not reused in the opposite direction.
Low-shoulder active feed-forward torque also learns direction-specific corrections in `scripts/data/left_arm_v2_6_adaptive_active_tau.json`; `nudge-hold` and adaptive replay group moves print `adaptive active tau updates` when an active joint misses its own target by at least 1 degree.

Replay investigation output:

- `v2.6 replay target pose=` prints the saved joint targets.
- `v2.6 replay after_clearance errors_deg=` prints the error from table clearance to the saved move before replay starts.
- `v2.6 replay final errors_deg=` prints final per-joint replay error after the wrist move.
- `v2.6 replay adaptive hold tau updates` prints any persistent hold-tau table changes learned from that run.
- `v2.6 replay adaptive target bias updates` prints any feed-forward target bias learned from final residual error.
- `v2.6 replay adaptive hold target bias updates` prints final-step induced hold-target offset updates.
- `v2.6 nudge-hold adaptive hold tau updates` prints manual nudge induced hold-tau updates.
- `v2.6 nudge-hold adaptive hold target bias updates` prints manual nudge induced hold-target offset updates after hold-tau convergence.

## Debug Panel

The right-side debug panel shows:

- `Debug Command`: exact command triggered by the button.
- `Debug Output`: return code, duration, stdout, stderr, and emergency-stop cancellation details.

Run records are appended to:

```text
scripts/data/left_arm_v2_6_web_runs.jsonl
```

## Safety Notes

- Start without `--enable-execute` for a dry UI check. Buttons will show the command but refuse to execute.
- Emergency Stop is software-level motor disable for the left arm; keep hardware power safety available.
- Home Move first returns `wrist` to table clearance by itself, then moves `wrist_side`, `arm_roll`, `elbow`, `shoulder_rotate`, `shoulder_side`, and `shoulder_front` to table clearance as one coupled group. It then runs the normal home move.
- Replay Move always returns to table clearance before moving to the selected saved move.
- Claw Close stops on pressure/stall detection and keeps holding the contact position until interrupted. During hold it prints claw status once per second.
- Claw Home can interrupt a running Claw Close hold, then moves the claw back to the captured claw home position.
- Long-running commands are launched in their own process group so Claw Home/Emergency Stop can terminate child processes cleanly.
- The web controller streams command stdout/stderr while a command is running, so long holding actions still update the debug output.
- Only one non-emergency action runs at a time. Emergency Stop can interrupt a web-started action.
