# Left Arm v3.1 Identified-Dynamics Controller

`left_arm_v3_1_model_control.py` is a separate experimental controller. It
does not modify the v2.6, v2.8, or v3.0 motion flows.

It schedules gains from identified effective inertia:

```text
kp = omega_n^2 * M_ii(q)
kd = 2 * zeta * omega_n * M_ii(q)
tau_ff = G_i(q) + friction_i(q_dot)
```

All actual motor commands require `--execute`. Model holding also refuses to
run until MIT gain/torque units have been explicitly marked as verified and
every selected joint has enough nearby gravity and inertia data.

## 1. Create the model

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 left_arm_v3_1_model_control.py init-model
```

## 2. Capture gravity samples

Place the arm at a stable pose. Keep it supported and stationary, then record
the motor-reported holding torque. This command reads data and does not command
motion:

```bash
python3 left_arm_v3_1_model_control.py capture-gravity \
  --joints shoulder_front,shoulder_side,elbow \
  --direction stationary
```

Repeat throughout the safe joint range and at folded, middle, and extended arm
postures. Capture both travel directions where possible so friction and
backlash can be separated later. Each record stores all seven joint angles,
not only the sampled joint angle.

The reported torque is useful only while the motor is actively holding the
pose. Stop the Web controller so this process has exclusive access to the CAN
serial port, but first leave the motor in its stable holding state and support
the arm physically during takeover. Do not collect an unpowered, manually
supported pose as a gravity sample.

## 3. Add effective inertia measurements

After obtaining natural frequency from a small, supervised step test, add the
sample with the test `kp`. The script calculates `M = kp / omega_n^2`:

```bash
python3 left_arm_v3_1_model_control.py add-inertia \
  --joint elbow --from-arm --test-kp 20 \
  --measured-omega-n 3.2 --label middle
```

You may instead supply a directly estimated value:

```bash
python3 left_arm_v3_1_model_control.py add-inertia \
  --joint elbow --from-arm --effective-inertia 1.95 --label middle
```

Repeat at folded, middle, and extended poses for the load-bearing joints.

If symmetric low-speed tests have identified Coulomb and viscous friction,
store their non-negative magnitudes:

```bash
python3 left_arm_v3_1_model_control.py configure-friction \
  --joint elbow --coulomb 0.08 --viscous 0.03
```

## 4. Configure and preview

Start with natural frequency well below the lowest measured structural
resonance and damping ratio between 0.7 and 1.0:

```bash
python3 left_arm_v3_1_model_control.py configure \
  --joint elbow --omega-n 2.5 --zeta 0.9

python3 left_arm_v3_1_model_control.py preview \
  --joints shoulder_front,shoulder_side,elbow
```

`preview` opens the CAN device only to read the current pose. It never commands
a motor.

## 5. Verify units and perform the first guarded hold

Only after verifying the joint-side units of the DM MIT `kp`, `kd`, and torque
fields:

```bash
python3 left_arm_v3_1_model_control.py set-units-verified
```

Support the arm physically during the first test. Start with one joint and a
short duration:

```bash
python3 left_arm_v3_1_model_control.py model-hold \
  --joints elbow --seconds 2 --execute
```

The controller starts with compliant takeover gains and the currently measured
load, then rate-limits `kp`, `kd`, and model feed-forward toward the scheduled
values. It captures `load_support_tau` as the difference between measured
output torque and the model plus static-friction prediction. That residual
stays fixed for this current-pose hold; it is not decayed, confused with the
Coulomb estimate, or written back to the model. First-frame feed-forward also
compensates the new damping term so net torque matches the observed load.

Model `tau_limits` continue to constrain the predicted feed-forward component.
The per-hold command envelope additionally includes the measured support and
its first-frame damping compensation, within the MIT protocol torque range.
For example, a preceding shoulder_rotate output of -2.43 Nm must not be
truncated to the model's -1 Nm feed-forward limit. The first frame and the
whole hold retain the additional load support while gains change smoothly.
`preview` shows `load_support_tau`, `hold_tau_limits`, and the supported hold
torque; the run result records the same data in `load_support`. A
velocity or position-error watchdog aborts the run. Logs are written to:

```text
data/left_arm_v3_1_model_runs.jsonl
```

## Web 3.0

### Motion-stage dynamics (opt-in)

The synchronized model used for this implementation has `updated_at` equal to
`2026-10-20 16:49:22`, 73 gravity samples and 29 inertia samples. All seven
joints have identified friction and verified MIT units. Some work poses lack
local shoulder_rotate inertia and use its configured minimum-gains fallback.

`left_arm_v3_1_motion_dynamics.py` integrates bounded corrections into the four
continuous trajectory methods in `left_arm_v2_6.py`. Home, Clearance and the
v2.8/v3.0 replay/multi-step flows inherit these methods. The original target,
trajectory, velocity field, torque ramps, active/held joint selection, and
adaptive-learning calls remain in place. Ordinary Clearance still skips
shoulder_rotate; reading its coordinate for a full-pose prediction does not
enable or command it. Claw commands are not modified.

The existing trained controller remains the baseline. The added torque is the
change in the model's limited gravity/friction feed-forward relative to the
segment entry pose; this preserves existing learned torque and loaded support
instead of replacing them with a smaller model limit. Friction uses the
reference trajectory velocity, including trajectories whose MIT `dq` field
deliberately remains zero. Local effective inertia schedules relative changes
in the trained gains, bounded to 15%, at most 5 in kp and 0.3 in kd. No gain
increases past the model maximum are allowed when the trained baseline is
already above it. Torque corrections are at most 0.5 Nm and at most one quarter
of that joint's model torque range. Corrections use the model's slew limits;
their caps shrink further when needed to release within the existing terminal
hold. The original baseline continues to be subject to MIT protocol limits.

Corrections fade in/out over the first/last 15% of each segment and release to
zero during the existing 0.5-second terminal hold, before terminal checks and
adaptive updates. Logs report peak corrections, coverage fallback frame counts,
and `terminal_correction_zero`. No new learned values or model values are
written by this layer. Model Hold and its measured-load bridge are unchanged.
This is incremental gravity/friction compensation and relative gain scheduling;
it is not a full inverse-dynamics controller with identified coupling,
Coriolis terms or acceleration feed-forward.

Model queries run at 20 Hz inside the existing command loop. Each query uses
the current full reference pose, with uncommanded coordinates retained from
the segment's measured entry pose. Outside the existing local sample limits,
the correction slews toward zero and the trained controller continues. A joint
without valid entry coverage stays on the original controller for that segment;
missing inertia is never fabricated or admitted from a remote pose. Malformed
models fail validation before opening the serial port.

The feature is **off by default**, independently of automatic post-motion
Model Hold. After syncing the module, `left_arm_v2_6.py`, and
`chessboard_vision_v3_0_web_control.py`, add this option to the existing Web
startup command to opt in:

```text
--motion-dynamics --dynamics-model /home/nvidia/hive_robot/DM_Control_Python/data/left_arm_v3_1_dynamics.json
```

The Web runner supplies `HIVE_ROBOT_MOTION_MODEL` only to executable operational
motion commands. It explicitly removes that variable for Model Hold,
identification, emergency stop and dry-run commands. The Dynamics status
reports `motion_dynamics_enabled`. Existing automatic Model Hold remains an
independent checkbox and retains its five-second handoff.

For a standalone operational command, set the variable only for that process:

```bash
HIVE_ROBOT_MOTION_MODEL=/home/nvidia/hive_robot/DM_Control_Python/data/left_arm_v3_1_dynamics.json \
  python3 -u left_arm_v2_8.py home --deadband-deg 0.5 --max-delta-deg 120 --prehome-clearance --execute
```

This is an activation example, not evidence of a new physical test. Offline
regressions verify the actual four trajectory methods with simulated CAN/time,
including endpoint equivalence, claw passthrough, skipped joints, slew limits,
coverage loss and learning/model immutability. Physical movement with this
new layer still requires validation; the earlier successful Model Hold tests
validated the existing handoff controller.

An offline reference preview does not import the motor stack:

```bash
python3 left_arm_v3_1_motion_dynamics.py \
  --model left_arm_v3_1_dynamics.json --preview-plan reference_plan.json
```

The plan contains `start_pose_rad` (all seven coordinates), `targets_rad`
(only intended commanded joints), `seconds`, and `commands` mapping each
commanded joint to `kp`, `kd`, and `tau`. Output samples show baseline plus
corrections along that supplied smoothstep reference and the terminal release.
It does not simulate an actual saved replay's staged paths or prove stability.

Web 3.0 exposes this controller on a separate **Dynamics Control** tab. Start
the normal Web 3.0 server:

```bash
python3 chessboard_vision_v3_0_web_control.py --enable-execute
```

Then open Web 3.0 and select **Dynamics Control**. The page can initialize and
inspect the model, collect gravity samples, add inertia data, configure the
scheduler and friction, preview scheduled terms, and run a guarded Model Hold.

The Web 3.0 shell loads only the selected panel. Switching tabs unloads the
previous document, releasing its MJPEG streams and polling requests. Unsaved
form inputs reset when that panel is opened again; already-started server
commands continue independently. Dynamics status and saved-move polling each
allow one outstanding request and abort read-only queries after five seconds.
Motion submissions are not automatically retried. The standalone `/dynamics`
page remains available for identification without video streams.

White Rook Place_A1 inherits the complete Clearance terminal controller before
replay starts. Each non-wrist motion group also refreshes pending arm joints
that the legacy adaptive replay group omitted, and ramps active feed-forward
from the measured entry load (with damping compensation) to the existing
motion torque over at least the first quarter of the trajectory. Logs named
`inherited clearance controller` and `continuous group entry` expose the
retained controls, pending joints and torque ramps. This is specific to the
rook A1 motion stage; the destination poses, grouping and final Model Hold
controller remain the same.

The standalone **Enable actual motor commands** checkbox is clear by default.
Standalone Model Hold starts only when both that checkbox is selected and the
server was started with `--enable-execute`.

The Robot Arm and Dynamics pages also expose **Automatic Model Hold After
Motion**, which is enabled when Web 3.0 starts. Home, Table Clearance, Replay,
and the existing multi-step chess flows still run their complete v2.8 motion
and adaptive-learning logic first. Placement1 now converts its terminal
position-error support into an equivalent measured-position plus
measured-torque hold for `0.5 s` before it exits. The learned offsets remain
available for the next move, while the terminal controller no longer depends
on a hidden multi-degree PD error; its log reports this conversion for every
joint. Only after the motion command exits successfully does Web 3.0 begin the
handoff. Web 3.0 reads the preceding motion's complete
log and, only when it finds an exact enable event for every selected joint,
reuses that physical motor state instead of sending duplicate enable frames.
The automatic reuse path also skips the read-only settling window: it immediately
captures position, velocity, and measured torque and sends a matching low-gain
MIT bridge frame with inherited load support. The 2026-10-20 Placement1 logs
confirmed real shoulder_rotate motion after the old bridge clipped about
-2.43 Nm to -1 Nm. Keeping the load residual throughout the hold addresses
that observed unloading. Skipping the read-only window also reduces the
cross-process command gap, but does not establish a continuous single-process
command stream. If
the log cannot prove that all selected motors were enabled, Model Hold keeps
the normal explicit-enable path and repeats the stability gate after enabling.
That standalone/explicit-enable path waits until all seven joints remain below
`0.25 rad/s` and drift no more than `0.25 degrees` over a `0.5 s` window, with
a `6 s` timeout.
It then hands all seven joints to the v3.1 controller for a five-second guarded
hold. These handoff gates do not relax the active hold's `0.3 rad/s` velocity
watchdog. Failed,
cancelled, or emergency-stopped motion never triggers the handoff. The final
Web run record includes the preceding motion result so both stages remain
visible. Turning the checkbox off affects future motions.

During the active hold, a velocity sample above `0.3 rad/s` is immediately
read again and checked against encoder displacement. Two high velocity frames,
or more than `0.1 degrees` of encoder motion from the last accepted frame,
still stop the controller. One high velocity field followed by a normal frame
with no corresponding position motion is recorded in
`rejected_velocity_spikes` and treated as an isolated feedback sample.

Validate the identified model throughout the required workspace before using
Model Hold without physical support.
