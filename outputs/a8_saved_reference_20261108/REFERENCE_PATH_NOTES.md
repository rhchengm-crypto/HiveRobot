# A8_take endpoint inverse-path evidence — 2026-11-08

The user confirms that `A8_take` can grasp a white knight near the A8 centre.
This is an accepted physical endpoint, not a previously accepted full motion.

- Exact Jetson Saved Moves source: `source_snapshot/left_arm_v2_6_saved_moves.json`.
- Nominal physical square centre: X27.5, Y412.5 mm; physical TCP Z was not measured.
- Current model FK at that endpoint: X11.798, Y405.601, Z84.623 mm.
- Shoulder-front is 2.052° below the previous software operating envelope.
- The earlier Z50.2 request was an assumption, not a measurement of this grasp.

`reference_inverse_plan_hover140.0.json` reconstructs 33 new geometric waypoints
from fixed ordinary Clearance to a 140 mm model hover and vertically to the
approved saved endpoint. Its intermediate poses are IK solutions and geometric
transit points, with full-frame descent and swept tool/body checks. The endpoint
is deliberately an explicit boundary condition. This is not a passing solution
for the old nominal `[27.5,412.5,50.2]` request, and is not an executable release.
The diagnostic envelope admits exactly the demonstrated shoulder-front angle;
it does not invent additional mechanical travel.

`shared_reference_candidate.json` adds the endpoint as a general numerical
reference for every board XY, with no per-square compensation or replay.
It is not installed as the runtime board model. Existing model hold, learning,
gain policy and ordinary Clearance shoulder-rotate skip are unchanged.

`common_geometry_fit_audit.json` checks approximate known square centres without
fitting their old requested Z values. The data do not yet support a robust new
shared TCP/board fit. In particular, shifting the common plane to force this
endpoint to Z50.2 contradicts existing non-contact trials. These approximate
observations must not be promoted to exact TCP metrology.

No A8 motion, motor enable, claw closure or runtime deployment occurred in this
inverse-path diagnostic. Read fresh actual state before any later execution;
the latest Web action when data were collected was `capture-move:A8_take`.
