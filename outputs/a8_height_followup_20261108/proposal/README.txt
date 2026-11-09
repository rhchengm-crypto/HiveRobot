INACTIVE OFFLINE LOWER-25mm FULL-POSE PROPOSAL

Status: complete geometric proposal only. Executable=false; candidate config
planning_only=true provides the existing runtime planning guard. No approved task
reference, prepared stamp, motor command, runtime activation, or config write.
Actual physical acceptance/contact height is unknown. Mechanical qualification
of the proposed additional front travel remains pending.

Target board XYZ remains [11.798050592563328,405.6006774366655,
59.622674457482844]mm. No per-square XYZ offset or geometry change was used.
The requested complete wrist frame is NEW:12.533472641deg from the original
saved-reference frame; its configured tool axis changes12.529580423deg.
This proposal explicitly permits frame and axis selection up to15deg. It does
not satisfy the historical10deg axis policy, and does not alter that active
policy. Once selected, the new frame remains fixed throughout the descent;
strict pose acceptance remains position0.5mm/full-frame0.005rad.

The candidate config is an unapproved COPY of the immutable v33 geometry and
software envelope. Only the front lower software bound is extended12deg in
the counterfactual domain; the other six stored limits and all scene geometry
remain unchanged. The endpoint uses9.402317847deg additional front travel.
Copied historical approved references are not used as this proposal's desired
frame; no new approved reference was added. Both plan and report explicitly
record requested_frame and historical_global_selection_policy_passed=false;
the inherited terminal_reference_tolerance_deg0.286478898 stays unchanged.
Proposal metadata is not a claim
that this inactive config is qualified for deployment.

Path:22 fresh reverse Cartesian IK samples, targetZ59.622674457 to requested
hoverZ140mm; fresh ordinary-Clearance transit has2 geometric edges. Forecast
Clearance preserves shoulder_rotate2.366483449935913 from the completed Home
hold log arm_run_1794188707909_37371.txt. Ordinary Clearance has not been
executed for this proposal and must be re-established with fresh feedback.

47 raw waypoints,23 controller groups. Full body/board/gripper swept-edge
checks passed on both raw and grouped paths.393 dense descent samples give
maximum XY line error0.246578233mm and full new-frame error0.009316185deg
(tolerance0.286478898deg); descent is monotonic. Maximum raw joint increment
is7.468921492deg, below15deg. Achieved modeled hoverZ139.622553586mm is within
the0.5mm position tolerance of its140mm request. No old trajectory was read.

The entire planned gravity envelope is recorded. Front model gravity spans
[-9.102864235,2.137335459]Nm. This sampled model budget is not a mechanical
rating or completed runtime load-support qualification. Fresh entry loading,
hard torque headroom, current source verification, mechanical travel evidence,
and separate movement authorization remain unresolved.

Files
  proposal_report.json: complete audits, exact endpoint/frame, provenance,
    Home forecast source, candidate policy, hashes and limitations.
  local_plan_UNAPPROVED.json: inactive geometric path; no execution stamp.
  candidate_board_config_UNAPPROVED.json: inactive counterfactual config copy.
  forecast_clearance.json: planned entry forecast, not a live status response.
  prepare_offline_lower25_proposal.py: reproducible local-only preparation.

Reproduce from workspace:
  python outputs/a8_height_followup_20261108/proposal/prepare_offline_lower25_proposal.py
The script imports the immutable v33 source snapshot and sets BLAS threads1.
The active config SHA is checked before and after and remains unchanged.
CAD feasibility cannot certify physical reach, real contact height, alignment,
orientation suitability for a white knight, or grasp success.
