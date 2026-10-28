#!/usr/bin/env python3
"""Unnamed joint-space targets through the common rigid-body motion adapter.

Angles are absolute encoder radians. This is not an XYZ/IK or collision planner.
Hardware is imported only after configuration and request validation.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import sys

from left_arm_v3_1_model_control import JOINTS, MIT_TAU_MAX

SCRIPT_DIR = Path(__file__).resolve().parent


def clean_targets(value):
    if not isinstance(value, dict) or not value:
        raise ValueError('targets_rad must be a non-empty object of encoder radians')
    result = {}
    for name, angle in value.items():
        if name not in JOINTS or isinstance(angle, bool):
            raise ValueError(f'invalid target joint/value: {name}')
        angle = float(angle)
        if not math.isfinite(angle) or not -12.5 <= angle <= 12.5:
            raise ValueError(f'{name}: target must be finite and within MIT position range')
        result[name] = angle
    return result


def positive(value, name, maximum):
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= maximum:
        raise ValueError(f'{name} must be positive and <= {maximum}')
    return value


def plan_move(start, targets, seconds=8., max_delta_deg=15., max_velocity=.3):
    """No saved pose, workflow name or nearest-sample coverage lookup."""
    start = clean_targets(start)
    if set(start) != set(JOINTS):
        raise ValueError('start pose must contain all seven joints')
    targets = clean_targets(targets)
    seconds = positive(seconds, 'seconds', 120.)
    max_delta_deg = positive(max_delta_deg, 'max_delta_deg', 180.)
    max_velocity = positive(max_velocity, 'max_velocity', 2.)
    full_target = {**start, **targets}
    delta = {n: full_target[n] - start[n] for n in JOINTS}
    if max(abs(math.degrees(v)) for v in delta.values()) > max_delta_deg:
        raise ValueError('target exceeds max_delta_deg from current pose')
    # Quintic smootherstep has peak derivative 1.875. Leave velocity margin.
    duration = max(seconds, 1.875 * max(map(abs, delta.values())) / (.75 * max_velocity))
    if duration > 120.:
        raise ValueError('requested move requires more than 120 seconds at velocity limit')
    return {'start_rad': start, 'targets_rad': full_target,
            'active_joints': list(targets), 'held_joints': [n for n in JOINTS if n not in targets],
            'delta_deg': {n: math.degrees(v) for n, v in delta.items()},
            'seconds': duration, 'trajectory': 'smootherstep',
            'reference_peak_velocity_rad_s': 1.875 * max(map(abs, delta.values())) / duration,
            'path_validation': 'joint-interpolation-only; no collision or mechanical-limit validation'}


class TrackingWatchdog:
    def __init__(self, arm, max_velocity, max_error_deg):
        self.arm = arm
        self.max_velocity = max_velocity
        self.max_error_deg = max_error_deg
        self.calls = 0

    def __call__(self, reference):
        self.calls += 1
        if self.calls % 8 != 1:
            return
        for name, row in self.arm.read_status(JOINTS).items():
            pos, vel = float(row['pos']), float(row['vel'])
            if not math.isfinite(pos) or not math.isfinite(vel):
                raise RuntimeError(f'general move invalid feedback: {name}')
            if abs(vel) > self.max_velocity:
                raise RuntimeError(f'general move velocity watchdog: {name} {vel:.3f}rad/s')
            error = abs(math.degrees(pos - reference[name]))
            if error > self.max_error_deg:
                raise RuntimeError(f'general move tracking watchdog: {name} {error:.2f}deg')


def execute_plan(arm, legacy, plan, status, max_velocity, max_error_deg):
    # Carry measured entry load into the common baseline; don't substitute
    # action-specific gravity tables for a new point. Existing route learning
    # remains owned by the legacy routes and its files are not rewritten here.
    baseline = {}
    for name in JOINTS:
        tau = float(status[name]['tau'])
        vel = float(status[name]['vel'])
        if not math.isfinite(tau) or abs(tau) > MIT_TAU_MAX[name] or not math.isfinite(vel):
            raise ValueError(f'invalid entry load/velocity for {name}')
        if abs(vel) > .1:
            raise ValueError(f'general move requires a stationary start: {name}')
        baseline[name] = tau
    gains = {n: dict(legacy.HOME_GAINS[n]) for n in JOINTS}
    active = {n: plan['targets_rad'][n] for n in plan['active_joints']}
    held = {n: plan['start_rad'][n] for n in plan['held_joints']}
    watchdog = TrackingWatchdog(arm, max_velocity, max_error_deg)
    arm._general_move_enabled = True
    arm.enable(JOINTS)
    arm.move_targets_with_holds(
        active, seconds_per_step=plan['seconds'], move_gains=gains,
        hold_targets=held, hold_gains=gains, fallback_kp=2., fallback_kd=.4,
        hold_tau=baseline, move_tau_ff=baseline, preload_seconds=.2,
        control_dt=.01, trajectory='smootherstep', watchdog=watchdog)
    return arm.read_status(JOINTS)


def hold_command(model):
    return [sys.executable, '-u', str(SCRIPT_DIR/'left_arm_v3_1_model_control.py'),
            '--model', str(Path(model).resolve()), 'model-hold', '--joints', ','.join(JOINTS),
            '--seconds', '5', '--control-dt', '.01', '--takeover-kp', '2',
            '--takeover-kd', '.4', '--max-velocity', '.3', '--max-error-deg', '2.5',
            '--velocity-spike-max-drift-deg', '.1', '--motors-already-enabled', '--execute']


def supply_hold_coverage(model, rigid, pose, joints):
    """In-memory current-pose priors only; retain the existing takeover code.

    Historical samples and files are never modified or extrapolated by relaxing
    coverage thresholds. Only absent local gravity/inertia comes from RNEA/M.
    """
    from left_arm_v3_1_model_control import gravity_prediction, inertia_prediction
    missing = []
    for name in joints:
        for kind, predictor in (('gravity', gravity_prediction), ('inertia', inertia_prediction)):
            try:
                predictor(model, name, pose)
            except ValueError as exc:
                if not str(exc).startswith(('nearest tau sample is ', 'no usable tau samples',
                                             'nearest effective_inertia sample is ',
                                             'no usable effective_inertia samples')):
                    raise
                missing.append((name, kind))
    if not missing:
        return model, []
    q = rigid.encoder_to_model([pose[n] for n in rigid.names])
    gravity = rigid.signs * rigid.gravity_torques(q)
    mass = rigid.mass_matrix(q)
    result = copy.deepcopy(model)
    for name, kind in missing:
        index = rigid.names.index(name)
        sample = {'pose_rad': dict(pose), 'source': 'general-model-current-hold-only'}
        if kind == 'gravity':
            sample.update(tau=float(gravity[index]), direction='stationary')
            result['joints'][name]['gravity_samples'].append(sample)
        else:
            value = float(mass[index, index])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'invalid general-model inertia for {name}')
            sample['effective_inertia'] = value
            result['joints'][name]['inertia_samples'].append(sample)
    return result, [{'joint': name, 'quantity': kind} for name, kind in missing]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', default=str(SCRIPT_DIR/'data/left_arm_general_dynamics.json'))
    p.add_argument('--hold-model', default=str(SCRIPT_DIR/'data/left_arm_v3_1_dynamics.json'))
    p.add_argument('--mode', choices=('shadow', 'apply'), default='apply')
    p.add_argument('--targets-json', required=True)
    p.add_argument('--start-json', help='offline planning only; no serial or motion')
    p.add_argument('--seconds', type=float, default=8.)
    p.add_argument('--max-delta-deg', type=float, default=15.)
    p.add_argument('--max-velocity', type=float, default=.3)
    p.add_argument('--max-error-deg', type=float, default=5.)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--no-model-hold', action='store_true', help='Web owns the existing automatic takeover')
    args = p.parse_args(argv)
    targets = clean_targets(json.loads(args.targets_json))
    positive(args.seconds, 'seconds', 120.)
    positive(args.max_delta_deg, 'max_delta_deg', 180.)
    positive(args.max_velocity, 'max_velocity', 2.)
    positive(args.max_error_deg, 'max_error_deg', 10.)
    if args.start_json and args.execute:
        p.error('--start-json is offline only and cannot be combined with --execute')
    from left_arm_general_motion import load_configuration
    load_configuration(args.model, args.mode)  # Validate before any serial open.
    if args.start_json:
        plan = plan_move(json.loads(args.start_json), targets, args.seconds,
                         args.max_delta_deg, args.max_velocity)
        print(json.dumps({'ok': True, 'motion_issued': False, 'plan': plan}))
        return
    os.environ['HIVE_ROBOT_RIGID_BODY_MODEL'] = str(Path(args.model).resolve())
    os.environ['HIVE_ROBOT_RIGID_BODY_MODE'] = args.mode
    os.environ.setdefault('HIVE_ROBOT_MOTION_ACTION', 'general-move')
    os.environ.setdefault('HIVE_ROBOT_MOTION_RUN_ID', secrets.token_hex(8))
    import left_arm_v2_6 as legacy
    arm = legacy.LeftArmV2()
    try:
        status = arm.read_status(JOINTS)
        start = {n: float(status[n]['pos']) for n in JOINTS}
        plan = plan_move(start, targets, args.seconds, args.max_delta_deg, args.max_velocity)
        print('general target plan=', json.dumps(plan), flush=True)
        if not args.execute:
            print(json.dumps({'ok': True, 'motion_issued': False, 'plan': plan}))
            return
        # Fail before enable if the subsequent standalone takeover cannot load.
        if not args.no_model_hold:
            from left_arm_v3_1_model_control import load_model
            load_model(Path(args.hold_model))
        final = execute_plan(arm, legacy, plan, status, args.max_velocity, args.max_error_deg)
        print(json.dumps({'ok': True, 'motion_issued': True, 'final': final}), flush=True)
    except BaseException:
        if getattr(arm, '_general_move_enabled', False):
            arm.disable(JOINTS)
        raise
    finally:
        arm.close()
    if not args.no_model_hold:
        env = {k: v for k, v in os.environ.items() if k not in {
            'HIVE_ROBOT_RIGID_BODY_MODEL', 'HIVE_ROBOT_RIGID_BODY_MODE',
            'HIVE_ROBOT_MOTION_MODEL', 'HIVE_ROBOT_MOTION_ACTION', 'HIVE_ROBOT_MOTION_RUN_ID'}}
        subprocess.run(hold_command(args.hold_model) + ['--general-fallback-model', args.model],
                       env=env, check=True)


if __name__ == '__main__':
    main()
