# A8 lower-height, new-frame trial — 2026-11-08

## Authorisation and result

The user requested `现在实测` and explicitly confirmed being onsite with emergency-stop access, an empty A8/channel, and unchanged board/base/table height. The disclosed change was a new wrist frame about 12.5 degrees from the previous reference and about 9.4 degrees additional shoulder_front travel. Empty load; no claw closure.

The operator reported:

> 夹持区域有改善，但还是偏高10mm
>
> 无横移、抖动或接触，朝向和保持正常

This accepts the observed motion stability and orientation, **not the grasp height**. The remaining approximately 10 mm is visual feedback, not calibrated TCP metrology. No full-pose fitting sample or successful grasp-path-library entry was created.

## Actual control sequence

1. Read and freeze the current Home feedback, 30 protected source/data files and five learning files without a motion command.
2. Recheck the complete local geometric plan against the fresh preserved shoulder_rotate entry angle.
3. Temporarily activate the explicitly authorised shared trial envelope/new requested frame at the existing Web model path; back up the exact original bytes.
4. Seal all 13 prepared source/model fingerprints and the complete plan before any motion.
5. Execute ordinary fixed Clearance through the Web, preserving its shoulder_rotate skip, then its normal five-second hold.
6. Verify actual Clearance entry and the complete planned load-support envelope before executing the prepared approach.
7. Execute all 47 raw geometric waypoints, represented by 24 execution groups, including every one of 21 contact/descent points.
8. Complete five-second Model Hold on the same serial session, without closing the gripper.
9. Restore the pre-trial shared configuration after the serial-owning command finishes. This performs no motor command or server restart.

No additional descent or Home was performed after this trial. The arm remains at the tested endpoint.

## Exact identities and models

| Component | Identity |
| --- | --- |
| Clearance motion | `arm_run_1794198389715_49164.txt` |
| Clearance post-hold | `arm_run_1794198415914_49291.txt` |
| Board motion and inline hold | `arm_run_1794198443568_49465.txt` |
| Continuous motor trace | `trace_1794198451214228486.jsonl` |
| Prepared path | `approach_a8_lower25_20261108.json` |
| Web output path | `approach_fb305d910c1ceed9.json` |
| Rigid-body model SHA256 | `83f5e4b240d48ce4aae1c8037922696c5f2f1ed120476c8ae7da9e6ca00d6fd7` |
| Hold model SHA256 | `aa91af203c5888c4d21db3d1507ea22479b63913824f8bf85145c8924b4f342f` |
| Actual trial board-model SHA256 | `b64370a7c0e4264c65b085b6b0db15fbe6f1ce1be66a489260e8640a988c1301` |
| Original/restored board-model SHA256 | `ff4d75df0ef85964ba153d267c5547ba036f63d075409ca0582174c6e836164d` |
| v33 controller SHA256 | `9d88000d0463b8caafa4616c6ce9d25f0b7dd81b7111f20b56f613df5291bf77` |
| Static-support module SHA256 | `ce4d53088a9818a7c229f822c9d9d2bb5f826328982b2a859a284e04bbe3203d` |
| Full-frame search module SHA256 | `856e96adba0da0f16069fa9b83ef55bc4d5daf7f27c873a39c4e1ba97b557cae` |

The exact request was model-board XYZ **[11.7980505926, 405.6006774367, 59.6226744575] mm**. It is not the nominal A8-centre [27.5, 412.5] request and must not be relabelled as that coordinate-registration validation.

The shared trial configuration changed only the shoulder_front lower software domain by 12 degrees, the explicit requested white-knight frame, a 15-degree frame-selection budget, and trial provenance. Other six joint domains, rigid geometry, TCP, board transform, collision geometry, numerical acceptance tolerances, gains and adaptive-learning policy were preserved. The final new orientation is 12.5335 degrees from the old reference; the tool axis changes by 12.5296 degrees. This is a different requested frame, not a claim to satisfy the old 10-degree selection policy. The complete new frame remains fixed over descent with a 0.005-radian tracking tolerance.

## Independent execution audit

See `execution_audit.json` and its self-contained reproducibility helpers.

- 23/23 checks pass; all 113 control-archive members match their hashes.
- Primary return code 0; total 124.782 seconds; inline same-serial hold 5.0328 seconds.
- 5,622 motor frames and 7,440 event records; no recording loss, writer error, confirmed encoder oscillation, or fault-support frames.
- All 21 planned descent points completed; all seven descent gain pairs stayed fixed, including shoulder_rotate 36/4.5.
- All 22 accepted static-load windows independently reproduce from fresh decoded feedback packets. Packet ages are verified; no sequence regression occurred.
- The final maximum joint error is 0.34659 degrees. Endpoint settle did not execute an additional correction retry; the two settle frames are bookkeeping, not an extra trajectory.
- The final observed static-bias estimate is distinguished from the last bias actually applied. The pending last observation is not falsely counted as applied.

## Height interpretation

The following are **model predictions from encoders/CAD**, not measured physical heights:

| Quantity | mm |
| --- | ---: |
| Requested TCP height | 59.6227 |
| Motion-endpoint TCP height | 62.1770 |
| After-hold TCP height | 62.1752 |
| Change from previous v33 after-hold TCP height | -22.7149 |
| Old saved-reference lowest CAD gripper vertex | 58.1425 |
| New planned lowest CAD gripper vertex | 39.7659 |
| Actual-encoder motion-endpoint lowest CAD vertex | 42.1845 |
| After-hold lowest CAD vertex | 42.1839 |

The changed wrist frame raises the lowest CAD vertex relative to the TCP by about 6.6234 mm. Thus a planned TCP reduction of 25 mm corresponds to about 18.3766 mm reduction of that lowest vertex, even with perfect tracking. The fixed STEP gripper opening and lowest vertex do not establish the actual finger-contact region. Do not infer a measured 15 mm physical descent by subtracting the previous visual lower bound of at least 25 mm from the current approximate 10 mm gap.

## Follow-up diagnosis, without further motion

`next10_diagnostic/diagnostic.json` tests the explicit next target [11.7981, 405.6007, 49.6227] mm with the same new complete frame and the same trial bounds/scene. Twelve numerical seeds did not meet the unchanged 0.5 mm position tolerance. The best residual is 4.1591 mm, including about -3.7284 mm in model board Y; orientation error is only 0.0803 degrees. No complete next path was produced or executed. This finite search does not prove physical/global unreachability; it does show that blindly extending this fixed-frame path is not yet a valid model solution.

That first diagnostic subtracts 10 mm from the **previous request**, not from the actual held endpoint. `feedback_relative10_diagnostic/diagnostic.json` additionally uses the actual final held encoder pose and its full FK orientation. It preserves its model XY [9.6397, 406.6506] mm and changes model Z from 62.1752 to 52.1752 mm. Twelve seeds again do not meet 0.5 mm: the best position residual is 4.0941 mm, with orientation error 0.0841 degrees; arm_roll reaches the trial upper software bound. No short descent was produced or executed. This second target is also a model calculation, not a measured board/contact datum. Re-selecting a valid lower pose remains necessary; no error or joint-bound gate was waived.

## Learning, camera and preservation

All five learning files were captured before/after and rechecked against the remote files twice. Only ordinary Clearance adaptation changed `left_arm_v2_8_clearance_bias.json`, from `d03c511c...` to `52f1b83bc1a267e673cb1abcfa6981b25465111200daadade69b7ee7f31acd68`; the updated bytes were synced to the local `scripts/data` directory. The other four files did not change.

Actual RGB-D recorder PID 48772 used `a8_lower25_execution_20261108_1794198293940875630`; 674 complete RGB/depth pairs were collected. A first pre-motion recorder invocation rejected a 900-second request because its unchanged limit is 600 seconds; it exited without frames or robot movement. That failed attempt remains documented separately, and only the identified successful recorder was stopped after the trial.

The exact executed trial configuration is retained as `board_workspace_trial.json` in the frozen robot context, while `config_restore_receipt.json` records restoration of the original shared configuration. This restoration leaves firmware holding commands unchanged. Same-serial Model Hold, v33 support, persistent adaptation, ordinary Clearance's shoulder_rotate skip, and physical emergency-stop behaviour remain intact.
