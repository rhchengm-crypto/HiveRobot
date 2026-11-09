# A8 height follow-up after Home — 2026-11-08

User instruction: `先home，再核对修改`. Home was dispatched once through the normal Web endpoint and completed before source changes. No further approach, descent, or gripper closure is authorised by this record.

## Home evidence

- Motion: `arm_run_1794188641610_37144.txt`; workflow `home/fcb98741abf1ba16`.
- Following Model Hold: `arm_run_1794188707909_37371.txt`; five seconds, existing motor enables reused, return code 0.
- Frozen evidence and checksums: `../a8_v33_home_20261108/download_verification.json`.
- 419 RGB/depth pairs and both controller logs were downloaded and verified. All five persistent-learning files match the previous v33 after-state and two remote checks.
- Physical Home vibration/contact observations are pending. Return code 0 is not proof of physically vibration-free motion.

## Separate the two height issues

The operator reported that v33 solved lateral drift but left the gripping region at least approximately 25 mm too high. This is visual feedback, not calibrated TCP or contact-plane metrology; the whole trial remains unaccepted for the successful-path library and full-pose fitting.

Encoder FK predicts that the new motion endpoint is 18.8925 mm higher than the previous v32 endpoint. The following same-serial hold changes predicted height by only about 0.4433 mm. Improved static load support therefore exposed a task-pose/height problem; removing that support would restore tracking error, not solve the Cartesian task.

The original A8 reference-endpoint request was model XYZ `[11.7980505926, 405.6006774367, 84.6226744575]` mm, derived from FK of the operator-demonstrated `A8_take` pose. It is not the nominal A8-centre request `[27.5, 412.5, 50.2]`. A saved encoder pose does not record physical grip height, contact, timestamped packet freshness, or the stiffness/load used when teaching it.

The follow-up diagnostic target is the explicit lower-height task `[11.7980505926, 405.6006774367, 59.6226744575]` mm. No board transform, TCP, per-square offset, or motor-angle replay is used to reach it.

## Common search fix

`scripts/left_arm_board_task_pose.py` previously used the axis-cone objective for almost every candidate, despite a terminal full-frame tolerance of about 0.286 degrees. Candidates could satisfy the reduced task and then be discarded for wrist yaw without being solved against the actual full-frame task.

The fix scores global initial guesses against the complete wrist frame when that constraint is required, solves every common seed against that frame, and refines axis-only candidates against the same full-frame requirement. The preferred-profile seed also obeys the configured terminal tolerance. Original XYZ, joint bounds, collision scene, final acceptance tolerances, and whole-path audits remain in force. Loose 180-degree policy retains its previous axis-only behaviour.

A genuine reachable FK regression reproduces a 0.30085 mm axis-cone solution with 10.1466 degrees full-frame error. The production refinement reaches 0.06075 mm and 0.006162 degrees under the same original bounds and scene. This proves a search defect was fixed; it does not prove the separate lower A8 target is reachable.

Local regression: 157 tests passed in 34.797 seconds; see `regression_tests.log`. Four new real-model tests cover this defect, a tighter terminal tolerance, the unchanged loose policy, and independently recomputed global scores.

## Reach diagnosis and remaining work

`diagnose_height_reach.py` and `height_reach_ablation.json` independently compare current software bounds, selected expanded mathematical bounds, fixed original orientation, and free orientation using vectorised global search plus bounded least squares. Scalar FK and numerical Jacobians check the independent implementation. `bounded_frame_axis_selection.json` separately checks explicit full-frame and tool-axis constraints.

Within the current bounds, the best demonstrated free-orientation position residual is approximately 6.1268 mm. Thus the search fix alone has not produced the requested lower endpoint. Expanding only shoulder_front and selecting a roughly 12.53-degree different frame can satisfy XYZ and the common scene, but that candidate changes the tool axis by roughly 12.53 degrees and fails the existing 10-degree axis constraint. It is not an executable accepted solution.

Finite optimisation does not prove global infeasibility or establish mechanical joint limits. Mathematical bound extensions remain offline diagnostics; none are installed into the active common configuration. A lower target requires a complete newly solved and checked branch, then physical validation. No additional motion has been executed.

## Separate new-frame proposal

`proposal/proposal_report.json` contains a complete **inactive** geometric proposal for the same explicit lower-height XYZ. It selects a new complete wrist frame 12.5335 degrees from the old reference (tool-axis change 12.5296 degrees), then holds that new frame fixed throughout descent. This is an explicitly different requested orientation, not a relaxed trajectory-tracking tolerance and not a claim to satisfy the old 10-degree selection policy.

Its candidate common software envelope extends only the shoulder_front lower bound by 12 degrees; the computed endpoint uses 9.4023 degrees beyond the old bound. Other six bounds and the complete collision geometry stay unchanged. These extensions are proposed mathematical domains, not measured allowable joint travel. The active configuration is untouched.

The proposed path has 47 newly generated waypoints, including a checked Clearance-to-hover transit and the full vertical descent. Dense checks cover 393 descent samples: maximum model XY error 0.24658 mm, maximum full-frame tracking error 0.009316 degrees, with monotonic requested descent. Raw and grouped collision/gripper checks pass. These are offline model results, not physical observations. No prepared execution stamp, approved-reference registration, or motion command was produced.

The proposal still requires current measured entry/load checks and physical validation before its height or orientation can be accepted. It must not be treated as a qualified path or fitting sample.

## Preserved control behaviour

- v33 session-local quasi-static load support, including fresh decoded-packet checks.
- Existing same-serial Model Hold takeover and persistent adaptive learning.
- Ordinary fixed Clearance before approach, skipping shoulder_rotate as an active Clearance target.
- shoulder_rotate Kp cap 36 and Kd 4.5, fixed descent gains, collision checks and physical emergency stop.
- Local precomputation before any new robot motion.

Runtime geometry/model JSON and persistent-learning data are unchanged by this fix. The code deployment receipt lives under `deployment/`; hardware-free remote tests and checksums must pass before that receipt reports installation complete.

Deployment completed with 27 guarded, hardware-free remote tests passing. Only the task-pose source and its test file were installed with backups. The Web server was not restarted; no motor or serial operation occurred during deployment. Old prepared-approach fingerprints are invalidated by the task-pose source hash change.
