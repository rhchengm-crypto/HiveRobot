# A8 accepted-reference offline fitting — 2026-11-08

The operator accepted the last A8 lower25 run as a useful grasping reference for all chess pieces except pawns. This is a qualitative empty-load acceptance: there was no closed-gripper loaded grasp in this trial. The operator's visual reach-limit observation is recorded, not converted into a measured mechanical hard limit.

The A8 record is now included in an **offline fitted candidate** together with the previously curated C4, C5 and B3 torque-proxy groups. The candidate is **not promoted to the active runtime model**. It does not change the board transform, TCP, joint limits, trajectories, gains, adaptive-learning state or Model Hold logic.

## Exact evidence and sample curation

- Motion: `arm_run_1794198443568_49465.txt`.
- Trace: `outputs/a8_lower25_execution_20261108/control_evidence/data/board_motion_traces/trace_1794198451214228486.jsonl`.
- The archived execution audit's 23 checks are all true.
- Last same-serial Model Hold: 126 complete motor frames; the initial 0.75 s is excluded.
- Retained window: 108 complete frames over 4.394845 s, with 108 distinct decoded CAN reply sequences per joint.
- Packet sequence and receive times never regress. Maximum retained packet age is 0.0108302 s.
- Peak absolute feedback velocity is 0.0219780 rad/s; maximum joint-position span is 0.0218703°.
- Per-joint median encoder angle and feedback torque form **one seven-axis pose group**, yielding seven torque-proxy records. Control frames and individual joints are not counted as independent poses.
- Per-joint position spans, velocity peaks, torque MAD/standard deviation/spans, and freshness statistics are in `a8_quasistatic_pose_group.json`.

These motor feedback torques are quasi-static proxies, not independently calibrated gravity or external-force measurements. Static friction, approach history, drive calibration and support-control effects have not been separately identified.

## Fit and entire-pose-group holdout

The original STEP geometry, inspection, identification history and fitting implementation are copied into `source_snapshot/`. Their hashes are recorded in `fit_summary.json`. The deployed nominal model is reproduced exactly: maximum difference over masses, centers of mass, inertial tensors and reflected inertias is **0.0**.

The shared candidate uses the original **73 gravity records and 29 unique closed-loop effective-inertia records**, plus **28 torque proxies from four independent held poses** (C4, C5, B3, A8). Total fit gravity/proxy records: **101**. This run adds only one group and seven proxies.

Every cross-validation fold excludes its complete held-out pose group and all original-history seven-axis poses within a 5° maximum-axis neighborhood. Nuisance offsets are learned only from training observations, never from the held-out sample, and are not deployed torque offsets.

| Held-out group | Original-history baseline RMSE (Nm) | Prior C4/C5/B3 comparator RMSE (Nm) | Four-group candidate procedure RMSE (Nm) |
|---|---:|---:|---:|
| C4 | 0.782758 | 0.845106 | 0.702726 |
| C5 | 0.245035 | 0.311267 | 0.565526 |
| B3 | 0.442321 | 0.493865 | 0.628698 |
| A8 | 1.995329 | 1.926515 | 1.926515 |
| Combined | 1.101106 | 1.091611 | 1.109095 |

For the A8 held-out fold, both proxy procedures train on C4/C5/B3 only: their identical A8 result is expected. The fit containing A8 is never used to evaluate A8 generalization. The A8 fold's complete seven-axis predictions and residuals are in `fit_summary.json`; the current model's raw A8 torque-proxy RMSE without nuisance offsets is 1.906283 Nm.

Adding A8 improves the C4 fold but worsens C5 and B3; combined grouped held-out RMSE rises **1.601686%** relative to the prior three-group procedure. Therefore the accepted reference and curated fitting dataset are retained, while the fitted parameter candidate remains an offline research artifact. Acceptance of a motion/reference and promotion of a new parameter fit are separate recorded decisions.

## What is not fitted

`geometry_target_labels=false`. The requested XYZ, encoder-based FK and visual height acceptance are not independent measured TCP coordinates. No TCP/board transform, per-square offset, geometric correction, new inertia measurement, pawn-height claim or true physical workspace boundary is inferred from this sample. No angle replay is introduced.

## Reproduction and integrity

From the repository root:

```powershell
python -X utf8 outputs/a8_accepted_reference_20261108/fitting/reproduce_fit.py
```

The script has no network calls, motor SDK imports or active-file writes. It snapshots and verifies 121 active source/model/config/learning hashes before and after the study. Each fit independently validates finite parameters, physical inertial representation and positive-definite mass matrices at 30 deterministic algebraic configurations. All curation, freshness, baseline-reproduction and holdout-leakage assertions passed in the completed study.

Files:

- `a8_quasistatic_pose_group.json`: one accepted A8 pose group and dense-window quality statistics.
- `hold_proxy_augmented_history.json`: complete four-group augmented fit dataset.
- `left_arm_general_dynamics_hold_proxy_candidate.json`: fully refitted shared dynamics candidate, not activated.
- `fit_summary.json`: source hashes, protection hashes, fit reports and entire-group holdout results.
- `reproduce_fit.py`: reproducible offline curation and fitting implementation.
- `source_snapshot/`: exact geometry, inspection, original/three-group histories, original deployed model, fitter and rigid-body implementation used in this study.
- `artifact_manifest.json`: output file sizes and SHA-256 values.

Raw trace and RGB-D evidence remain in the original lower25 evidence package; they are referenced rather than duplicated into this small fitting package.
