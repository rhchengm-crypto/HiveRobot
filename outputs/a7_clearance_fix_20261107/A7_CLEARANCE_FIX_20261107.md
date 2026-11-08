# A7 full-gripper clearance revision — 2026-11-07

## Current state

The operator reported the arm has returned to Home. Latest read-only Web record: Home hold finished at 2026-11-07 17:30:02. This revision issued no motor command. It is an offline candidate, not a physically accepted trial. The active Jetson program/model is not replaced by this staging package.

## Rejected trial and evidence

- Old request: A7, X27.5 / Y357.5 / height30.2 mm, White Knight, empty approach, no closure.
- Operator: final XY/orientation acceptable, fingers touched the board, approach passed almost against the board, requested +10 mm height and comparison with A6.
- Main log: `arm_run_1794101005121_29275.txt`; trace: `trace_1794101006747544649.jsonl`.
- Control and RGB-D archives: `../a7_execution_20261107/control_evidence.zip`, `../a7_execution_20261107/rgbd_evidence.zip`.
- Failure record: `scripts/data/board_workspace_trials/validation_20261107_a7_expanded_angle_height30.json`. Not eligible for fitting or the accepted path library.

## Cause established by the comparison

The old collision model covered link axes and wrist-to-TCP segments. It omitted the full CAD gripper outline. A7's configuration-space detour dipped before rising to its hover pose; old checks passed despite fingers extending below the registered plane. Command completion and zero oscillation events did not establish physical clearance.

| Route | Lowest nominal gripper vertex in approach | Nominal endpoint vertex height | Feedback-FK endpoint vertex height |
|---|---:|---:|---:|
| Accepted A6 | 21.0 mm | 13.2 mm | about 6.7 mm |
| Rejected A7 | -11.0 mm | 13.6 mm | about -3.6 mm |
| Revised A7, offline | 41.8 mm | 23.6 mm | Not yet measured |

Values use the same board frame and revised STEP geometry. They are model calculations, not independently observed clearances. The global support-plane test also covers portions outside the finite chessboard. Negative height is not proof of contact at a particular board XY. The A7 terminal joint tracking error reached about 3.32 degrees, so nominal endpoint geometry alone did not predict the physical touch.

## Common algorithm change

- `left_arm_board_collision.py`: add the wrist-frame convex hull of the full CAD gripper; evaluate its lowest vertex against the registered plane; provide its analytic gradient to IK; reject invalid/disabled geometry configurations.
- `left_arm_board_grasp.py`: apply one travel floor to direct edges, reference-assisted edges and RRT edges; include the gripper radius in the continuous swept-edge bound; audit every planned edge again before enabling any motor.
- `left_arm_gripper_vertex_envelope.json`: 161 hull vertices derived from 7877 vertices of revised STEP part7 (OmniPicker), with source hashes. Plane minima over the hull agree with the complete source vertex set; checked against 200 random normals.
- Shared nominal travel floor20 mm, task floor5 mm. The 20 mm rule is informed by accepted A6 geometry and applies to every target. It is not an A7 offset or pose replay.
- Geometry covers the approved STEP finger opening. Other finger openings and external mechanical deformation are not certified by this envelope.

## Revised request and verification

X27.5 / Y357.5 / height40.2 mm. This is the requested target raised10 mm, with no per-square geometry offset. The plan has42 waypoints and uses the accepted A6 reference only as an IK seed/orientation reference. Each target and complete connecting edge is recomputed by the common model. Maximum planned Cartesian residual:0.4344 mm. The complete transit floor is certified by interval checks; 41.8 mm is the minimum from an additional dense quarter-degree audit, not an exact swept minimum claim.

All 107 existing board-related tests and 7 new gripper-envelope tests passed. New tests cover analytic gradients, cache separation of travel/task floors, invalid geometry, the recorded A7 miss, accepted A6 clearance, and rejecting the old low route before any motor access. `final_verification.json` binds final source hashes and the complete-edge recheck.

The geometric model, board registration, operating ranges, runtime gains, torque behavior, adaptive learning, same-serial Model Hold and ordinary Clearance skipping shoulder_rotate are preserved by this revision. Changes are confined to planner collision/clearance handling and an added gripper envelope. Previous board-task development in the worktree is retained.

## Next physical validation

From the current Home: use fixed Clearance, then the fully precomputed revised A7 approach and descent, endpoint handling and5 s same-serial hold, empty and no closure. Do not repeat the rejected30.2 mm plan. Before deployment/execution, bind the plan to the final runtime config/source hashes and verify the start forecast against the latest Home feedback. Record full control frames and RGB-D as before. Observe approach finger clearance, endpoint height and any drift/oscillation. Physical acceptance remains pending; do not add this offline plan to the accepted reference library.
