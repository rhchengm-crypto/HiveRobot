# A7 v29 — rejected physical trial, recovered to Home

## Operator result

> 这轮也不可接受，过程中已经抬起，但是夹爪最后触碰棋盘，需要再抬高10mm，且整体需要再往a方向10mm

> 稳定，我已让机械臂home

The raised approach improved. The terminal touch makes this trial **rejected**. The requested next physical correction is up approximately 10 mm and toward file A approximately 10 mm. These are operator observations, not calibrated Cartesian measurements. This record is diagnostic evidence only; it is not an accepted reference path or fitting target.

## Recovery verified

The completed trace records `explicit_home_handoff`, followed by `trace_closed`. Web reports the process finished, return code 0, and the same-serial 5-second hold complete. The final shoulder-front, elbow and wrist feedback is respectively 0.8291371, -1.2815671 and -2.0708401 rad. No board-grasp process remained when checked. The operator executed this Home; this follow-up issued no motor command.

The successful Home/hold must not be confused with successful completion of A7. A7 stopped before waypoint 41/41. Its ordinary terminal correction and hold were not reached; it entered continuous support and subsequently accepted the operator's Home handoff.

## Why nominal clearance did not prevent this touch

The shared full-gripper CAD envelope fixed the low approach identified in the first A7 trial. The prepared nominal route had about 41.8 mm minimum sampled approach clearance and 23.6 mm nominal final clearance. Those numbers assume joint tracking.

The last completed nominal target was `[27.5, 357.5, 48.2]` mm; the requested `[27.5, 357.5, 40.2]` mm final point was never executed. At support entry, feedback-based FK gave TCP height 17.47 mm and minimum gripper vertex height 2.08 mm. The operator independently reported physical contact; FK is not an external measurement and the 2.08 mm prediction does not contradict or override that report.

Shoulder-front tracking error was +3.62449 degrees. Substituting just that joint's measured angle into the previous nominal configuration lowered the modeled gripper envelope by 36.44 mm. Elbow and other deviations partly offset this; the isolated contributions are not additive. Thus commanded target height alone does not establish physical height.

## Load tracking evidence

| Shoulder-front quantity | Above-entry end | Last descent end |
|---|---:|---:|
| Position error, degrees | +4.4644 | +3.6245 |
| Sent feed-forward, N m | -4.8000 | -4.8000 |
| Position-feedback contribution, N m | -7.0127 | -5.6933 |
| Reconstructed total command, N m | -11.8049 | -10.4855 |
| Measured torque, N m | -11.8906 | -10.5641 |

The single-entry gravity baseline was clamped on 18 of 19 calls. Its final raw prediction was -8.4423 N m, truncated to -4.8 N m. Therefore about 3.6423 N m of modeled load support was discarded. The remainder between this raw prediction and terminal measured torque is about 2.12 N m. Removing that clipping alone cannot be claimed to fix all position error. Neither value establishes a motor continuous-torque rating or authorizes treating the MIT encoding range as a safe mechanical rating.

Relevant code paths:

- `scripts/left_arm_board_grasp.py`, `TrajectorySupportBaseline.command`: clamps entry load plus pose-dependent gravity to the static model/entry envelope.
- `scripts/left_arm_general_motion.py`, `GeneralMotionSession.transform`: applies the board torque envelope again to continuous gravity updates.
- Runtime collision checks use measured joint feedback, which eventually stopped this trajectory before its final point. The successful nominal-path audit did not account for the observed load-induced tracking deviation.

The next correction must address the common motion load/tracking calculation and actual gripper clearance. It must not silently remove motor limits, reuse later PD feedback as repeated gravity learning, or add a per-square stored joint replay. The operator's +10 mm request must not be described as a verified physical +10 mm merely by changing the unexecuted final request from 40.2 to 50.2 mm. No corrected physical trial has been performed yet.

## Evidence and synchronization

- `load_tracking_diagnosis.json`: phase-by-phase force/position decomposition and clamp audit.
- `support_snapshot_audit.json`: nominal/measured geometric comparison at stop.
- `control_evidence.zip`: source/model snapshots and active-support trace prefix.
- `rgbd_evidence.zip`: 444 RGB and 444 depth frames already downloaded and SHA-256 verified.
- `control_final.zip`: complete trace including Home and its completed hold, SHA-256 `4f42d1bf55024db4730e59b01ce0b47ca31ee809e4b76e34dd5d528578b31f6c`.
- `control_final/data/board_motion_traces/trace_1794102248464038593.jsonl`: complete 42,064,424-byte trace; SHA-256 `ac5e55efef996e3806b0795ee40dc7858945d59a443b7be6b6e2712bf78f9ab2`.
- `scripts/data/board_workspace_trials/validation_20261107_a7_v29_height40.json`: rejected result and Home recovery, synchronized to Jetson.

Live runtime remains the deployed v29. This follow-up changed records only; it did not change gains, torque limits, Model Hold, adaptive learning, Clearance behavior, model geometry, or the accepted path library. No new movement or GitHub upload was issued.
