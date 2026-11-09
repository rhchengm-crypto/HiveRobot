OFFLINE A8 LOWER-25mm HEIGHT DIAGNOSTIC — 2026-11-08

No motor commands, network access, model/config writes, or runtime activation.
The target is [11.798050592563328,405.6006774366655,59.622674457482844]mm
in the existing board frame. It is a new diagnostic target 25mm below the
saved-reference FK endpoint, not a calibrated physical contact datum.

Main conclusion
The present stored software envelope did not yield this target even with
position-only IK (best finite search residual 6.1268mm). Thus merely allowing
the planner to select a different wrist frame within the existing bounds is
not established as a sufficient height fix. The independently tested strict
original frame is also numerically short. None of these finite failures is a
global proof of infeasibility or a claim about physical reachability.

Front-only mathematical extension
With the front lower bound extended only in memory, exact target position and
common CAD scene can be satisfied with full-frame change 12.5335deg and front
additional lower travel 9.4023deg. However its tool axis changes 12.5296deg,
which violates the existing 10deg axis cone. It is not an eligible candidate.

Explicit full-frame AND tool-axis constraints
Three seeds from independently found exact-position branches were tested.
No endpoint passed full-frame<=15deg AND tool-axis<=10deg with only the front
software lower bound relaxed (other six stored bounds unchanged). An
independent minimum-axis constrained solve at full-frame<=15deg terminated
successfully at axis11.157934deg, front additional9.969778deg, arm_roll upper
bound active, position residual8.1e-12mm and scene margin12.2663mm. Under
full-frame<=20deg the best corresponding local axis minimum was11.095963deg;
rotate lower and roll upper were active. These are local constrained minima,
not global lower bounds on tool-axis change or required travel.

One bounded coupled-front/roll diagnostic
Allowing up to30deg mathematical front-lower and roll-upper extension, while
holding the other five limits and common scene fixed, a single constrained
local solve still failed the requested budgets. Its residual0.0311mm was
small, but full-frame15.019575deg and tool-axis10.889997deg both exceeded the
limits; front extra9.799547deg and roll extra26.313330deg were unapproved
mathematical-domain changes. This is a failed diagnostic, not a valid pose.

Practical implication
A generic explicit endpoint-frame selection step can improve search policy,
but does not by itself solve this lower target under the current envelope.
The demonstrated bottleneck couples position, tool-axis policy, and front/
roll/rotate software bounds. Any revision to actual allowable travel needs
separate evidence; solver output cannot establish safe mechanical limits.
Keep the current active config unchanged. No complete reverse vertical branch
or checked Clearance transit was claimed because no endpoint passed the
requested constraints. No executable trajectory was produced.

Reproduction
Use the pre-existing local Python environment (no packages installed):
  python outputs/a8_height_followup_20261108/diagnose_height_reach.py
  python outputs/a8_height_followup_20261108/select_bounded_frame_axis.py
Both set BLAS threads1 and import the immutable v33 source/model snapshot.
The JSON files record source SHA256 hashes, solver flags, exact q and selected
frames, residuals, common-scene checks, domain changes, and finite-difference
Jacobian checks. Selected full frames are new desired orientations, never a
relaxation of strict descent rotation acceptance. The original nominal A8
geometry/contact-height consistency remains unresolved.
