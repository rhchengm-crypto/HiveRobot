FEEDBACK-RELATIVE10mm — SEPARATE OFFLINE ENDPOINT DIAGNOSTIC

Use actual final same-serial inline-hold encoder feedback and its FK wrist R.
Starting model board XYZ [9.639697000830376,406.65055148807653,
62.17518852976162]mm; new target preserves actual model XY and subtracts10mm:
[9.639697000830376,406.65055148807653,52.17518852976162]mm.
The complete desired wrist frame is frozen from that SAME actual feedback.
This equation defines a model target, not a measured board/contact datum.

Same scoped trial bounds (front lower extension12deg, other axes unchanged),
same geometry and common scene/gripper, strict position0.5mm/full-frame0.005rad.
Actual q was the first seed.12 deterministic starts all failed position
acceptance; best residual4.094108728mm and full-frame error0.084113838deg.
No short vertical path was attempted after endpoint failure. Finite failures
do not prove global or physical unreachability. No new frame/bound relaxation,
activation, stamp, hardware/network access or movement occurred.

The earlier request-relative target [11.798050592563328,405.6006774366655,
49.622674457482844]mm and its diagnostic were left unchanged in their separate
directory. That target subtracts10mm from the old REQUEST59.622674457mm; it is
not the feedback-relative model operation requested here.

diagnostic.json contains exact held q/frame, all12 solves, source SHA256s,
residual vectors, joint margins and limitations. Reproduce with:
  python outputs/a8_lower25_execution_20261108/feedback_relative10_diagnostic/diagnose_feedback_relative10.py
