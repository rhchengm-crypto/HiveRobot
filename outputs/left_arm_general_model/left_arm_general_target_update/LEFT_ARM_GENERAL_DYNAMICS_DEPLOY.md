# Seven-axis general motion dynamics: deployment and first test

This connects the approved revised STEP geometry and calibrated seven axes to
the four existing continuous trajectory methods. No chess-action-specific
parameters were added. Existing Model Hold takeover, adaptive updates, and
ordinary Clearance's shoulder_rotate skip remain in place.

## Unnamed joint-space targets

`left_arm_general_move.py` accepts any target expressed as absolute seven-axis
encoder radians, without a saved move name or nearby identification sample.
Supply all seven coordinates or a subset; omitted axes hold their entry pose.
It uses the same common trajectory loops, prepared dynamics worker, 0.5 Nm
bounded correction and terminal release as the tested routes. Per-joint gains
are reused across points, not fitted per target. The entry measured load is the
baseline for a new point. Existing route adaptive-learning files remain in use
by those routes and are not rewritten by this new entry point.

This is joint-space motion, not Cartesian XYZ inverse kinematics, collision
planning, or verification of mechanical joint limits. MIT position encoding
bounds, a current-to-target delta limit, duration/velocity planning and feedback
watchdogs are enforced. No new target has been physically validated by the
offline tests.

In Web's **Dynamics Control → General Joint Target**, enter a target JSON,
select Preview Target, then use Move to Target with actual commands enabled.
For example `{"wrist": -2.0}` is an absolute encoder target, not a relative jog.
Only use a target/path whose clearance has been checked. Preview reads the
current pose but does not enable motors. The default max change is 15 degrees;
it is a guard, not a list of supported target points. Duration grows when needed
to keep the quintic reference below 75% of the feedback velocity limit.

The existing automatic Model Hold checkbox controls Web takeover. Standalone
execution takes over for five seconds by default:

```bash
python3 left_arm_general_move.py \
  --model data/left_arm_general_dynamics.json \
  --targets-json '{"wrist": -2.0}'
# Preview only above; add --execute to issue motion after reviewing the target.
```

For new points outside historical hold coverage, only the missing gravity or
inertia prior is supplied in memory by the general model at the current pose.
The existing Model Hold current-state bridge, measured-load support, filters
and watchdogs are retained. No synthetic identification samples are saved.
Existing named-route hold behavior is unchanged.

Ordinary Web nudge/nudge-hold and all saved replay names now receive the model
configuration by their motion adapter and command verb, rather than a point
name allowlist. Identification, emergency actions and Model Hold do not inherit
the movement correction environment. CLI users of legacy controllers can also
set `HIVE_ROBOT_RIGID_BODY_MODEL` and `HIVE_ROBOT_RIGID_BODY_MODE`; the standalone
generic mover sets these itself.

Incremental update: copy these four Python files to the Jetson code root:
`left_arm_general_move.py`, `left_arm_v2_6.py`,
`left_arm_v3_1_model_control.py`, `chessboard_vision_v3_0_web_control.py`.
Keep the existing model JSON and learning data. Restart Web with the same
`--rigid-body-model data/left_arm_general_dynamics.json --rigid-body-mode apply`
options. The generic mover reuses the already deployed rigid-body modules.

Filtered log summary for the Web generic target:

```bash
python3 left_arm_general_motion.py --summary --action dynamics:move-target --last 4
```

The nominal model uses all 73 existing gravity samples and 29 unique effective
inertia samples in `scripts/left_arm_v3_1_dynamics.json`. It uses nonnegative
mixtures of CAD-segment cylinder shapes and fitted diagonal residual inertias.
These are effective lumped estimates, not weighed link masses. Some individual
link masses are at the numerical floor because these data do not uniquely
identify the mass distribution. The 28-column fit has rank 20. Static offset
and approach-history terms are nuisance variables and are not deployed as an
extra torque. Scalar closed-loop inertia is only a soft constraint on M_ii;
the coupled off-diagonal terms come from the nominal geometry, not independent
measurements. Model metadata records these assumptions and the source hashes.

The nominal CAD-frame gravity is `[0, -1.7029069047, -9.6576649505] m/s²`.
No new manual dimensions, angles, or inertia tests are required for this build.

## Files to sync

Copy these six files from local `scripts/` to
`/home/nvidia/hive_robot/DM_Control_Python/`:

* `left_arm_v2_6.py`
* `left_arm_v3_1_motion_dynamics.py`
* `left_arm_rigid_body.py`
* `left_arm_general_motion.py`
* `left_arm_dynamics_worker.py`
* `chessboard_vision_v3_0_web_control.py`

Copy local
`outputs/left_arm_general_model/left_arm_general_dynamics.json` to
`/home/nvidia/hive_robot/DM_Control_Python/data/left_arm_general_dynamics.json`.

Do not replace the Jetson's existing `left_arm_v3_1_dynamics.json`, saved poses,
move library, adaptive JSONs, or log files with this deployment. Existing
`left_arm_v3_1_model_control.py` and the v2.8/v3.0 move wrappers are dependencies
already installed by the earlier work; they were not changed by this update.
The offline fitting/build/test scripts and STEP file are not runtime dependencies.

## Configuration check: no serial access or motor commands

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 left_arm_general_motion.py \
  --model data/left_arm_general_dynamics.json --mode apply
```

Expected fields: `ok: true`, `motion_issued: false`,
`dynamics_parameters_available: true`, `missing_inertials: []`, and record
counts 73/29. `parameters_independently_verified: false` is intentional: the
parameters are estimates from existing tests, not newly measured link masses.
Invalid geometry, friction, gravity or inertials fail before serial opens.

## Web startup

Stop the old Web process through its normal terminal before starting a new one.
Use the same Jetson Python/ROS/camera environment as the working Web server.
Retain any site-specific camera/topic/calibration options previously in use.

First, the optional shadow mode computes the full model during motion while
preserving the previous motion command output, including the existing sparse
motion layer if `--motion-dynamics` is set:

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 -u chessboard_vision_v3_0_web_control.py \
  --host 0.0.0.0 --port 8100 --enable-execute \
  --dynamics-model data/left_arm_v3_1_dynamics.json \
  --motion-dynamics \
  --rigid-body-model data/left_arm_general_dynamics.json \
  --rigid-body-mode shadow
```

For actual general dynamics compensation, restart with `--rigid-body-mode apply`
instead of `shadow`; all other arguments are the same. In apply mode the new
general layer replaces the old sparse correction layer, so the two models do
not add gravity/friction twice. The original trained/adaptive control baseline
still runs. No per-action gain changes are required.

## First on-robot test

1. Begin from the already validated Home pose. If currently elsewhere, use the
   existing validated procedure to return; this is not a request to repeat the
   previous Home acceptance/identification tests.
2. Keep **Automatic Model Hold After Motion** enabled in Web. Check that the
   Dynamics status displays `general dynamics shadow` or `apply`, as selected.
3. Run the existing **Table Clearance Move** once. Observe the motion and its
   automatic five-second Model Hold. In apply mode this is the first physical
   test of the newly fitted model, not a stability guarantee from offline tests.
4. The motion log should contain `general motion begin=` and
   `general motion complete=`. The summary should show
   `prediction_status: available`, `terminal_correction_zero: true`, and the
   requested mode. Model Hold should then complete with its original logic.
5. Send back the complete Web result and the general-model trace:

```bash
tail -n 4 /home/nvidia/hive_robot/DM_Control_Python/data/left_arm_general_motion_runs.jsonl
```

Prefer the compact summary instead of copying the full reference arrays:

```bash
python3 left_arm_general_motion.py --summary --last 2
```

Build `seven-axis-rigid-body-motion-v2-async` starts and warms a dedicated
numerical process before serial opens. The command loop publishes the latest
reference and reads predictions with nonblocking mailbox try-locks. There is
no queue of outdated references. Results from a previous trajectory, failed
worker, or references older than 100 ms are not used; the correction slews
toward zero under the existing rate limit. The worker never owns motor or
learning state. Full inverse dynamics and entry gravity run only in that
process. Closing the arm terminates its worker without waiting for a numerical
result. Model Hold does not receive the worker opt-in.

`peak_dynamics_compute_ms` now measures the separate numerical process, not
time spent running inverse dynamics inside the command loop. To assess the
actual command-side impact, inspect `peak_prepare_frame_ms` and
`frame_interval_ms` (p50/p99/max), along with accepted/stale/error counts. These
metrics include the existing sparse shadow layer where enabled; this update
does not establish a hard 10 ms end-to-end deadline including serial traffic
and OS scheduling. First validate this build in shadow on the same existing
Clearance path; no new inertia identification is needed.

Build `seven-axis-rigid-body-motion-v3-prepared` additionally caches validated
physical parameters, fixed transforms, and origin inertia tensors at worker
startup. Per-reference RNEA uses direct 3-vector cross products and the known
local +Z joint rotations. It has been compared against the general algorithm;
it does not change the fitted model or its gravity convention. Summary fields
`dynamics_compute_ms` and `dynamics_cpu_ms` report p50/p99 wall time and process
CPU time separately, to help distinguish arithmetic cost from scheduling wait.
The 100 ms freshness limit and bounded fallback policy remain unchanged.

Shadow mode must have zero `peak_applied_delta_tau_Nm`. Apply mode limits each
increment to 0.5 Nm, with a 1.5 Nm/s rate limit. It uses
`ID(q_ref,dq_ref,ddq_ref) + friction(dq_ref) - gravity(q_entry)`, converts through
the confirmed encoder signs, fades at trajectory boundaries and releases
during the existing terminal hold before learning. q/dq/kp/kd and joint masks
remain the existing values. A skipped shoulder_rotate may be read as a model
coordinate, but this adapter never enables or sends a command to it.

Operational Home/Clearance/replay/placement actions get the model through the
Web child-process environment. Identification, standalone nudges, emergency
stop, and post-motion Model Hold do not inherit the general-motion opt-in.

Replay execution contract: saved-move adapters execute `replay-move` without
an outer `--execute` argument. Web recognizes that command only when
`--enable-execute` is enabled and the script is the configured move adapter.
This also covers the multi-stage Bishop/Knight/Rook/Placement entry points;
do not add an unsupported `--execute` flag to the saved-move command.
An earlier Web environment filter incorrectly required that outer flag, so
successful placement/hold output from before this fix is not evidence of
general-motion compensation on those flows.

New general-motion records include `workflow_action`, `workflow_run_id`, and
`session_started_at_utc`. Nested move subprocesses inherit the same workflow
context. Automatic Model Hold explicitly removes all motion-model/context
environment variables while retaining its existing bridge, gains and guards.
For the newest Bishop run, obtain only that run's last four stage summaries:

```bash
python3 left_arm_general_motion.py --summary --action white-bishop-placement --last 4
```

If there are no matching new-format records, the command prints
`summary_status: no-matching-motion-records`; it does not substitute old Home
or Clearance traces. An interrupted run that never finished a stage may also
have no trace entry; retain its Web/full-run output for diagnosis.

An interrupted movement may not have a completed trace line; its Web/full-run
log remains the source for the interruption. If a new vibration appears, use
the existing emergency stop and retain that full log.

To return to the previous motion implementation, restart Web with the existing
working command and omit both `--rigid-body-model` and `--rigid-body-mode`.
Keep `--motion-dynamics` if that was previously enabled. Learned files are
retained in both modes.

## Reproduce the nominal fit locally

```bash
python scripts/fit_left_arm_general_dynamics.py
```

The fitter reads the approved geometry, revised STEP inspection and historical
identification JSON; it writes a separate general-model JSON. It does not open
serial or alter either the identification source or adaptive-learning files.
