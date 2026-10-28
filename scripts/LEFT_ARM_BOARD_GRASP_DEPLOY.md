# Board grasp v21: restore endpoint settling by operator request

The operator explicitly requested restoration of the former `endpoint_settle`
after reviewing its historical behavior. Board Above/Touch/Approach now re-anchor
the measured load at the final geometric IK target, up to three attempts, stopping
when all joint errors are within 0.35 degrees. Each correction retains the current
motion gains, model controller, timing and watchdog, and checks the transit edge
before moving. It may change both XY and height; it is not a pure D-direction move.
Isolated direction probes retain no endpoint chase.

The proposed explicit 8 mm D descent bias was withdrawn before any biased descent
was executed. No fixed board offset is applied. Shoulder_rotate Kp cap remains 36,
Kd remains 4.5, and the same-session Model Hold inherits the final MIT commands
after settling. Existing adaptive learning, physical emergency stop and ordinary
Clearance's shoulder_rotate skip remain unchanged. Historical v8 removal and v9
gain-handoff notes below explain why settling had previously been removed.

This restoration is a requested behavior change, not proof of repaired physical
board registration. Validate subsequent physical endpoint movement on the robot.

## Historical v9: retain load-bearing gains during Model Hold takeover

Build: seven-axis-board-grasp-v9-stiffness-handoff. Velocity and position-tracking diagnostics are record-only by explicit operator instruction. They do not abort motion at any measured speed or tracking error. Reference speed and acceleration remain unchanged.

## v9 fixed-pose stiffness handoff

In arm_run_1793144737715_55120.txt, the motion completed without endpoint chasing, but subsequent Model Hold lowered shoulder_front gains from 90/4 to 2/0.4 while preserving approximately -7.35 Nm. A later speed check observed -0.315 rad/s and 1.071 degrees of real motion. The operator reported unnecessary upward motion during this final stage.

Board motion now reports its actual final MIT gains. Its post-move Model Hold inherits those gains as fixed-pose minimum gains, seeds the takeover filter with them and uses the corresponding damping-cancellation torque bridge. The target is the measured endpoint, never the nominal IK goal. This preserves the preceding local torque/stiffness law at takeover rather than releasing a loaded joint through a gain reduction. Existing model torque scheduling and load residuals remain active. No new upward waypoint or endpoint correction is added. Other workflows retain their existing Model Hold defaults.

The new --motion-gain-floor-json option is internal to board completion and is validated before opening hardware. It is not persisted to the learned model. Software motion velocity/tracking diagnostics remain record-only, Home retains its 180-degree entry allowance, and ordinary Clearance/adaptive learning are unchanged. Normal Model Hold's own existing checks remain.

Retest Above Only at X=132.5,Y=192.5 and observe the five-second hold specifically: it should keep the measured hover without the extra upward shift. Physical retest remains pending; simulation and offline validation do not establish hardware stability.

## Previous v8 endpoint correction removal

The log arm_run_1793144094123_54424.txt shows the source-above motion ending at model TCP [110.141,189.571,153.046] mm. A separate endpoint_settle movement then moved it approximately [+20.998,+3.596,+6.072] mm. The operator confirmed the gripper was basically centred on C4 before this extra movement and was pushed toward D by it. Model Hold afterward remained stationary.

Board execution now completes each planned trajectory once and reports measured endpoint residuals. It does not launch a second endpoint-chasing movement. Normal Model Hold takes over at the measured pose. The endpoint report separates trajectory completion from nominal_joint_target_reached and never claims physical_board_target_verified. The visual C4 correspondence is retained separately as approximate XY calibration evidence; board registration, CAD dimensions and dynamics/learning JSON are unchanged.

Retest from the same Home start with X=132.5, Y=192.5, white_knight, Above Only. Confirm the previous extra final movement is absent and the final hover still aligns physically. Resolve global geometry registration using the recorded evidence before descending; this change does not establish arbitrary-point accuracy.

## Home from general board targets

Web Home and explicit same-session Home recovery now request --max-delta-deg 180. The observed Above Only wrist posture required a 131.60 degree current-to-Clearance transition and was refused by the old 120/122.02 degree entry threshold. Home's staged path, gains, timing and post-move Model Hold remain unchanged. Ordinary Clearance is unchanged. For direct CLI use, request 180 explicitly.

## Active v7 policy

Board motion and newly started same-session Home recovery use record_only=true. The previous 0.2 rad/s and 5 degree values are logging thresholds only, not stopping thresholds. Continuous tracking audit records contain measured position/velocity/torque, reference position, signed error and sent MIT commands at approximately 0.5 second intervals and on threshold-state changes. Logging failures do not interrupt motion. Segment-entry velocity is also logged without triggering support. Endpoint tracking error is no longer a reason to enter protective support.

Normal Model Hold, adaptive learning, ordinary Clearance's shoulder_rotate skip, physical/Web emergency stop and motor/driver protection remain. Invalid feedback, transport exceptions, invalid model results and planning/collision failures remain real execution failures: if a movement is already active, the existing load-support path still applies. No automatic trajectory continuation is introduced.

The previous guarded v6 policy below is historical. A current v6 process stays on its loaded code; newly launched board tasks load v7 without restarting the Web server. Use the existing Home handoff to finish a previously stopped task before starting a new Above Only test. Do not kill its load-support process just to apply an update.

## Previous v6 correction and recovery

The v5 log arm_run_1793141765154_51433.txt shows a separate tracking failure: elbow feedback lagged by 5.039 degrees while carrying +5.368 Nm. Its commanded feedforward still used the Home baseline -1.115 Nm plus at most 0.5 Nm of model correction, fading that correction at the endpoint. This could not supply the large gravity change over the continuous transit.

For board motion only, v6 separates pose-relative gravity from inertial/friction correction. Gravity is added to the measured entry-load baseline with torque limits and a 1.5 Nm/s slew limit, and remains at the endpoint. The residual dynamic correction still uses the existing 0.5 Nm bound and terminal fade. Gravity is not counted twice. Missing fresh gravity predictions for more than 250 ms stops the trajectory into load support. Existing normal flows keep their current dynamics policy.

During protective support, status reports running=false, motion_running=false, support_running=true and process_alive=true. Home Move requests recovery inside that same process and serial connection; the support loop hands over to the existing v2.8 Home controller. No second serial owner, disable command or automatic trajectory retry is introduced. A Home failure re-latches support at fresh feedback. Emergency Stop remains an explicit disable-all action. Other motion commands remain blocked until recovery completes.

Only a support process advertising home_handoff_available=true may receive the Home signal. An already-running v5 process cannot be upgraded in memory; support the arm mechanically before ending that old task and restarting Web. Do not send SIGUSR1 manually.

## What the supplied log establishes

Source: data/left_arm_v2_8_full_runs/arm_run_1793140179675_43265.txt.
The C4 Above Only run started at the reported Home pose. It had 52 planner samples. At original sample 44, wrist velocity feedback was -0.212454 rad/s against the unchanged 0.2 rad/s threshold. Tracking error was 0.151211 degrees. The first post-stop read was -0.080586 rad/s. There is no continuous pre-trigger feedback trace, so this log cannot distinguish a one-frame feedback excursion from actual brief acceleration.

A definite structural defect was found: geometry samples were each treated as a new stop/start movement, recapturing measured total torque and restarting the dynamics session. The two checked straight joint-space legs were broken into 52 separate movements. v5 executes exactly collinear transit samples as two continuous movements, retaining the geometric path and all collision checks. It does not shortcut curved IK edges. Measured-load anchoring happens at meaningful movement boundaries; endpoint residuals are settled using the existing measured-load bridge. Velocity/acceleration reference budgets and the feedback threshold are unchanged. This fixes the repeated-reset defect, not a proven hardware velocity-feedback defect. Do not claim overspeed is physically eliminated before retesting.

The fault support loop remained active for over 390 seconds in the provided log, with no reported I/O failures; recorded load torques included shoulder_front -4.42 Nm and elbow +3.92 Nm. Maximum reported drift stabilized at wrist 0.852 degrees. The operator reported no vibration. This is an observed loaded protective stop, not comprehensive fault validation.

## Preserved behavior

- Trajectory errors retain same-process load-support command refresh; no automatic disable or trajectory restart.
- Physical and Web Emergency Stop remain effective. Explicit process termination ends support.
- Successful completion retains normal Model Hold takeover.
- Existing adaptive learning and its data files are unchanged.
- Ordinary Clearance continues skipping shoulder_rotate as an active target.
- General dynamics model, gain limits, board registration and joint-envelope data are unchanged.

The protective state is now named support-commanding, not holding: current DM_CAN integration does not verify the driver's enable state. Zero I/O errors do not prove a motor is enabled or carrying load. First-fault logs include one board watchdog trace with the last 32 checked feedback frames and actual MIT command parameters; it is not repeated in every heartbeat.

## Transfer

Runtime files under /home/nvidia/hive_robot/DM_Control_Python/:

- left_arm_board_grasp.py
- left_arm_board_home_handoff.py
- left_arm_fault_support.py
- left_arm_general_motion.py
- left_arm_dynamics_worker.py
- left_arm_v3_1_motion_dynamics.py
- chessboard_vision_v3_0_web_control.py

No learning JSON or model JSON needs replacing for this repair. The complete bundle also includes unchanged board kinematics and registration files. Use the newly authorized SSH key for atomic code upload and checksum verification. Back up the prior code first. Never restart the server or kill a support process while the arm is unsupported. Uploading code does not update an already-running Python process.

## Web start

With reliable mechanical support and no active motion/support task:

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
source /opt/ros/noetic/setup.bash
source /home/nvidia/ascam_ws/devel/setup.bash
python3 -u chessboard_vision_v3_0_web_control.py \
  --host 0.0.0.0 --port 8100 --python-bin python3 \
  --arm-script left_arm_v2_8.py \
  --move-script left_arm_v3_0_move_library.py \
  --enable-execute \
  --dynamics-model data/left_arm_v3_1_dynamics.json \
  --motion-dynamics \
  --rigid-body-model data/left_arm_general_dynamics.json \
  --rigid-body-mode apply
```

## Test 1: finish Above Only normally

Do not repeat the cancelled protective-injection test. Keep suitable mechanical fall protection, a clear swept path and an operator beside the physical emergency stop. Motors must be normally enabled and stationary, not red/faulted. Do not run two serial-owning programs.

At /dynamics, choose X=137.5, Y=192.5, white_knight, Above Only. Preview first, then Execute Selected Stage. Expect board execution groups with planner_samples 52 and execution_groups 2 when starting at the recorded Home pose. A different actual start may produce a different sample count; only exactly collinear samples are merged. Model Hold should complete normally after movement. A protective-hold result is an aborted movement, not success: its support process stays alive while motion running is false. On v6, use Home Move for explicit same-session recovery. If recovery itself fails, support continues at the new stop pose; do not force another serial process. Support the arm mechanically before an explicit disable or server restart.

## Test 2: vertical-only Touch 160 -> 140 mm

Only after Above Only actually completes and its post-move Model Hold succeeds without abnormal motion: retain X/Y and profile, choose Touch Position (no close), and set Touch target height (mm) to 140. Preview first. It should contain contact stages only, preserve XY and current wrist orientation, descend about 20 mm, and omit a free transit to clearance or a claw command. Then Execute Selected Stage.

The touch planner checks the measured start through FK. XY must be within 5 mm of the requested point and each test must descend 1..25 mm. Otherwise it returns touch_start_mismatch without moving. Do not loosen that check or guess a replacement start pose.

After this succeeds, the next staged test is 140 -> 120 mm with target height 120. Do not jump directly from 160 to the 55 mm profile grip height. Do not place a hand under the gripper. Stop on unexpected direction or approaching collision.

Grasp/automatic close-and-lift execution remains unreleased until descent and the physical contact/grip height are validated. Existing named chess-piece flows are not altered. Nominal board/TCP registration residuals and incomplete workspace coverage are unchanged; this is not a claim of full-board grasp validation.
# V12 withdrawn after physical test

The operator reported approximately half a square of lateral movement during
the V12 test. Its encoder-derived 1 mm displacement is inconsistent with that
observation. V12 tracking trim is withdrawn; the production board executor is
restored byte-for-byte to V11. V12 source/tests are retained under
outputs/left_arm_general_model/board_v12_withdrawn_*.py for diagnosis only.
This rollback issues no robot movement commands. The arm remained
stable; operator estimates about 30 mm of remaining downward space.
Do not treat the current CAD/encoder/board/TCP mapping as validated Cartesian
metrology, or extrapolate descent targets from it to complete a grasp.

# V12 historical implementation (withdrawn)

Touch only now integrates fresh joint position tracking errors into a temporary
load trim (one-second integral time, 0.5 Nm per-joint cap, 1 Nm/s slew). Targets
and existing MIT gains are unchanged. Feedback is checked every two frames.
Trim is subtracted from the next segment's measured-load baseline to avoid
double counting, and the actual final total command passes unchanged to the
same-session hold. Fault support also receives total output without adding trim
again. No model/adaptive data is persisted; Above, Home and Clearance retain
their previous control paths.

Hardware test log arm_run_1793149694827_64243.txt completed Touch and hold.
Model-derived descent was 7.22 mm for a nominal 5 mm request; lateral movement
was about 1.0 mm during motion and 0.15 mm during hold. The physical lateral
outcome still requires operator observation. This is not evidence of accurate
depth control or a fully corrected physical Cartesian model.

# V11: keep board motion and Model Hold on the same serial session

The operator observed another physical shift toward B after a V10 descent, even
though encoder-derived hold displacement was zero. That observation remains an
unresolved physical measurement, not proof of a correct Cartesian path.

Code review identified a remaining handoff gap: the board process closed serial,
then Web spawned standalone Model Hold; the Jetson DM_CAN initializer itself
closes and reopens an already opened serial device. V11 removes this lifecycle
transition for board automatic Model Hold. The same arm/controller instance runs
the existing five-second hold before closing; Web must not start a second hold.
The complete terminal MIT command from V10 is retained. Other Web workflows,
learning files and ordinary Clearance joint selection are unchanged.

`board same-session hold begin` logs `terminal_to_first_refresh_ms`, and
`board same-session hold complete` contains the hold result. This eliminates a
confirmed software gap, but the physical cause of the reported shift still needs
an observed hardware test. From the last reported position only, preview a 5 mm
Touch with X=107.0 mm, Y=184.9 mm, target height=91.6 mm; no claw close.
Observe the end of descent, the five-second hold and the end of hold separately.

# V10: continue the terminal motor command into board Model Hold

Board completion now includes the captured final MIT `q`, `dq`, `kp`, `kd`,
and `tau_ff` in the existing per-joint handoff payload. For a complete stationary
payload, Model Hold continues that command for the fixed-pose hold rather than
reanchoring the position and reconstructing feed-forward from measured torque.
The log mode is `continued-terminal-motion-command`. Legacy gain-only payloads
and non-board workflows retain their previous behavior. Learning/model JSON and
ordinary Clearance selection are unchanged.

This eliminates a command discontinuity; it does not establish that the reported
5 mm physical offset was caused by it. Hardware confirmation remains necessary.
After deployment, from the last reported pose only, preview a 2 mm Touch using
X=113.2 mm, Y=191.0 mm, height=48.6 mm, with automatic Model Hold enabled.
Do not close the claw during this check. Observe descent completion and the
following hold separately; inspect the terminal command and bridge log fields.

