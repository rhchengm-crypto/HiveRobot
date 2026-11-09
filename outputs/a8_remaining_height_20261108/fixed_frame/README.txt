FIXED ACTUAL-HELD FRAME: DEMONSTRATED REMAINING DESCENT

No hardware/network, movement, activation, execution stamp, source/model edits,
or physical-height certification. All files here are diagnostic and explicitly
not executable. The approximate physical10mm gap is not a calibrated datum.

Starting actual held FK board XYZ is [9.639697000830376,406.65055148807653,
62.17518852976162]mm, with its own actual held full wrist R fixed as desired.
Trial software bounds, including only the already-used front lower12deg
extension, remain unchanged. Body/board/gripper checks remain unchanged.
Acceptance remains0.5mm total position error and0.005rad full-frame error.

Conservative demonstrated continuous result
Request1.5mm downward, same model XY/desired R: complete short branch from the
actual held encoder feedback passes common swept body/board/gripper checks.
415 dense samples: maximum XY error0.390535978mm; maximum full-frame error
0.032426322deg; minimum scene margin11.983664341mm; max joint edge13.940380171deg;
model height decreases monotonically. Its final FK drops1.316289833mm, not an
exact1.5mm. Thus a conservative value rounded down to a0.5mm grid of ACTUAL
MODEL displacement is about1.0mm. This does not promise any physical descent.
The0.5mm Cartesian/.005rad rotation acceptance budgets are partly consumed.

Failure and numerical range evidence
Coarse0.5mm endpoint continuation first fails at2.0mm requested descent:
position error0.644278670mm exceeds0.5mm; R error0.009471862deg passes. The
arm_roll upper software bound is active; front retains about1.44deg lower
margin. Default-DLS endpoint bisection brackets its local acceptance frontier
between1.65625 and1.6640625mm REQUEST, not a global/physical maximum.

Two independent SLSQP exact-XY/exact-R minimum-height optimizations terminate
at0.482859325mm actual model descent with arm_roll upper active. This stronger
equality diagnostic explains why the larger accepted branch necessarily uses
some allowed position/orientation error; it is not a proof of a global bound.

Independent optimization over the ORIGINAL acceptance balls
Two further SLSQP solves maximize requested descent subject to the unchanged
0.5mm position sphere, same desiredR within0.005rad, bounds and common scene.
Both terminate near1.718270091mm requested /1.506114mm actual model descent,
with BOTH pose tolerances fully active and arm_roll upper active. That endpoint
does not establish an executable continuation. The supplementary local branch
optimizer attempted a19deg jump at0.5mm, so its branch was rejected under the
15deg edge rule. It does not replace or invalidate the separately validated
1.5mm-request continuous DLS branch above. No larger full branch was accepted.

Practical interpretation
Under this frozen desired frame/current trial domain, this work demonstrates
only roughly1mm additional model descent. It has not solved the operator's
remaining approximate10mm. Finite solvers and local constrained extrema do
not prove that larger descent is globally or physically impossible. The
separate frame-selection study belongs to its own directory and is not part
of this fixed-frame result.

Files and reproduction
  report.json: sweep, bisection, exact-equality local extrema and original
    complete continuous branch audit with source fingerprints.
  short_vertical_DIAGNOSTIC_NOT_EXECUTABLE.json: the passed1.5mm request branch.
  independent_acceptance_boundary_polish.json: stronger local endpoint search
    and its rejected supplementary branch attempt.
  diagnose_fixed_frame_range.py and polish_acceptance_boundary.py: reproducers.

Run from workspace in that order. Both import frozen completed-trial sources,
use pre-existing local NumPy/SciPy and set BLAS threads1. No packages installed.
