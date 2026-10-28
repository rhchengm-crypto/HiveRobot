# Seven-axis general motion dynamics: deployment and first test

This connects the approved revised STEP geometry and calibrated seven axes to
the four existing continuous trajectory methods. No chess-action-specific
parameters were added. Existing Model Hold takeover, adaptive updates, and
ordinary Clearance's shoulder_rotate skip remain in place.

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

Copy these five files from local `scripts/` to
`/home/nvidia/hive_robot/DM_Control_Python/`:

* `left_arm_v2_6.py`
* `left_arm_v3_1_motion_dynamics.py`
* `left_arm_rigid_body.py`
* `left_arm_general_motion.py`
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
