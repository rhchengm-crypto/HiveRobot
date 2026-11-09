NEXT10 HEIGHT DIAGNOSTIC — OFFLINE ONLY

Endpoint failed; no subsequent reverse branch, Clearance transit, executable
path, execution stamp, motor access, network, or active config/source write.

Request board XYZ: [11.798050592563328,405.6006774366655,
49.622674457482844]mm. This is10mm below the completed lower25 trial request,
with the trial's NEW complete wrist frame frozen exactly as requested. Keep
position tolerance0.5mm, full-frame tolerance0.005rad, all existing scoped trial
software bounds (front lower extension12deg only) and all scene/gripper checks.
No further orientation selection, bound expansion, geometry or gain change.

12 deterministic IK initial values all failed Cartesian acceptance. Best
position residual4.159067363mm, full-frame error0.080310998deg (orientation
passes). Predicted board XYZ [11.949640954,401.872319607,51.459576630]mm, so
Cartesian residual vector is approximately [+0.151590,-3.728358,+1.836902]mm.
This residual is not a pure4.159mm height deficit. The common scene/gripper
check passes at the numerical candidate, but the candidate misses the target.
arm_roll is close to its upper software bound; shoulder_front is not at its
lower bound. The coupling and finite local search prevent attributing failure
to a simple missing front angle or asserting global/model/physical reachability.

No next test is prepared. The operator's approximately10mm remaining physical
gap is useful task feedback, not calibrated model TCP/contact height. A future
authorized trial would require Home and fresh ordinary Clearance; this script
does not propose descending directly from the currently held endpoint.

diagnostic.json records all12 numerical results, exact best q, limits, chosen
frame, scene checks, source/config hashes, failure reason and limitations.
Reproduce using:
  python outputs/a8_lower25_execution_20261108/next10_diagnostic/diagnose_next10.py
The script imports the completed trial's frozen current source snapshot and
checks config and raw plan hashes against that trial's preparation.json.
