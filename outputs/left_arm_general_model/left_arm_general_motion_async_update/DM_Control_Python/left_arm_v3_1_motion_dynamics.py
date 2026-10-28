#!/usr/bin/env python3
"""Bounded dynamics corrections inside existing continuous arm trajectories.

No CAN imports, motion entry point, model writes, or learning writes. The
legacy command remains the baseline. Corrections vanish before terminal
validation/learning and the existing post-motion Model Hold takes over as usual.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import time
from pathlib import Path

from left_arm_v3_1_model_control import (
    JOINTS, MIT_TAU_MAX, clamp, load_model, scheduled_terms, validate_ready,
)

MODEL_ENV = 'HIVE_ROBOT_MOTION_MODEL'
RIGID_MODEL_ENV = 'HIVE_ROBOT_RIGID_BODY_MODEL'
RIGID_MODE_ENV = 'HIVE_ROBOT_RIGID_BODY_MODE'
BUILD = 'v3.1-bounded-motion-dynamics-v1'
MAX_GAIN_FRACTION = 0.15
MAX_KP_CORRECTION = 5.0
MAX_KD_CORRECTION = 0.3
MAX_TAU_CORRECTION = 0.5


def load_motion_model(path):
    model = load_model(Path(path))
    validate_ready(model, JOINTS)

    def finite_numbers(value):
        if isinstance(value, dict):
            for item in value.values():
                finite_numbers(item)
        elif isinstance(value, list):
            for item in value:
                finite_numbers(item)
        elif isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError('motion model contains a non-finite number')
    finite_numbers(model)
    for key in ('max_gravity_distance_deg', 'max_inertia_distance_deg',
                'gravity_neighbors', 'inertia_neighbors'):
        if float(model['settings'][key]) <= 0:
            raise ValueError('motion model requires positive ' + key)
    for key in ('kp_slew_per_second', 'kd_slew_per_second', 'tau_slew_per_second'):
        if float(model['settings'][key]) <= 0:
            raise ValueError('motion model requires positive ' + key)
    for name in JOINTS:
        joint = model['joints'][name]
        low, high = map(float, joint['tau_limits'])
        if not -MIT_TAU_MAX[name] <= low < high <= MIT_TAU_MAX[name]:
            raise ValueError('invalid motion torque limits for ' + name)
        for key in ('coulomb', 'viscous'):
            value = float(joint.get('friction', {}).get(key, 0.0))
            if not math.isfinite(value) or value < 0:
                raise ValueError('invalid motion friction for ' + name)
        for key in ('gravity_samples', 'inertia_samples'):
            for sample in joint[key]:
                if any(n not in sample.get('pose_rad', {}) for n in JOINTS):
                    raise ValueError('incomplete motion sample pose for ' + name)
                value = float(sample['tau' if key == 'gravity_samples' else 'effective_inertia'])
                if (not math.isfinite(value)
                        or any(not math.isfinite(float(sample['pose_rad'][n])) for n in JOINTS)):
                    raise ValueError('non-finite motion sample for ' + name)
        for sample in joint['inertia_samples']:
            if float(sample['effective_inertia']) <= 0:
                raise ValueError('non-positive motion inertia for ' + name)
        # Validate configured envelopes using an actual sample, allowing only
        # existing coverage failures (not invalid scheduler/model values).
        try:
            scheduled_terms(model, name, joint['gravity_samples'][0]['pose_rad'])
        except ValueError as exc:
            if not coverage_error(exc):
                raise
    return model


def coverage_error(exc):
    return str(exc).startswith((
        'nearest tau sample is ', 'nearest effective_inertia sample is ',
        'no usable tau samples', 'no usable effective_inertia samples',
    ))


def configured_model():
    """Called before opening serial. Absence of the opt-in preserves v2.8."""
    path = os.environ.get(MODEL_ENV)
    rigid_path = os.environ.get(RIGID_MODEL_ENV)
    if rigid_path:
        from left_arm_general_motion import load_configuration, start_runtime_worker
        config = load_configuration(rigid_path, os.environ.get(RIGID_MODE_ENV, 'shadow'))
        # Shadow preserves the existing sparse layer. Apply replaces it to
        # avoid adding two models' gravity/friction to the same command.
        config['legacy'] = load_motion_model(path) if path and config['mode'] == 'shadow' else None
        start_runtime_worker(config)
        return {'rigid_body_configuration': config}
    if not path:
        return None
    model = load_motion_model(path)
    print('v3.1 motion dynamics model=', json.dumps({
        'build': BUILD, 'path': str(Path(path).resolve()),
        'updated_at': model.get('updated_at'),
        'policy': 'legacy-baseline-plus-bounded-pose-relative-corrections',
    }), flush=True)
    return model


def begin_motion(arm, commanded_names):
    model = getattr(arm, 'motion_dynamics_model', None)
    if model is None:
        return None
    names = [name for name in commanded_names if name in JOINTS]
    if not names:
        return None
    # All seven coordinates are needed to query a pose-dependent model. This
    # read does not enable or command any joint, including skipped rotate.
    pose = arm.positions(JOINTS)
    if 'rigid_body_configuration' in model:
        from left_arm_general_motion import GeneralMotionSession
        config = model['rigid_body_configuration']
        legacy = MotionSession(config['legacy'], pose, names) if config['legacy'] is not None else None
        return GeneralMotionSession(config, pose, names, legacy)
    return MotionSession(model, pose, names)


def prepare_frame(session, targets, progress, velocities=None, accelerations=None):
    if session is not None:
        if getattr(session, 'uses_rigid_body_reference', False):
            session.prepare(targets, progress, time.monotonic(), velocities, accelerations)
        else:
            session.prepare(targets, progress, time.monotonic())


def reference_derivatives(kind, t, seconds, linear_blend=0.0, window=None):
    """Analytic derivatives of the EXISTING trajectory, without changing q."""
    if not all(math.isfinite(float(x)) for x in (t, seconds, linear_blend)) or seconds <= 0:
        raise ValueError('invalid trajectory reference time')
    x = clamp(t, 0.0, 1.0)
    if kind == 'linear':
        s, ds, dds = x, 1.0, 0.0
    elif kind in ('smootherstep', 'blend_smootherstep'):
        s = x*x*x*(10+x*(-15+6*x))
        ds, dds = 30*x*x*(1-x)**2, 60*x*(1-x)*(1-2*x)
        b = clamp(linear_blend, 0.0, 1.0) if kind == 'blend_smootherstep' else 0.0
        s, ds, dds = (1-b)*s+b*x, (1-b)*ds+b, (1-b)*dds
    else:
        s, ds, dds = x*x*(3-2*x), 6*x*(1-x), 6-12*x
    if window is not None:
        a, b = window
        width = max(b-a, 1e-6)
        u = (s-a)/width
        if u <= 0 or u >= 1:
            ds, dds = 0.0, 0.0
        else:
            ds, dds = 6*u*(1-u)*ds/width, (6-12*u)*(ds/width)**2+6*u*(1-u)*dds/width
    if t <= 0 or t >= 1:
        return 0.0, 0.0
    return ds/seconds, dds/(seconds*seconds)


def send_command(arm, session, name, kp, kd, q, dq, tau):
    if session is not None and name in session.names:
        kp, kd, tau = session.transform(name, kp, kd, tau)
    arm.ctrl.controlMIT(arm.motors[name], kp, kd, q, dq, tau)


def finish_motion(session):
    if session is not None:
        session.finish()


class MotionSession:
    def __init__(self, model, pose, names):
        self.model = model
        self.pose = {name: float(pose[name]) for name in JOINTS}
        if not all(math.isfinite(value) for value in self.pose.values()):
            raise ValueError('non-finite motion entry pose')
        self.names = tuple(dict.fromkeys(names))
        self.entry = {name: self.terms(name, self.pose, 0.0) for name in self.names}
        self.desired = {}
        self.correction = {name: {'kp': 0.0, 'kd': 0.0, 'tau': 0.0} for name in self.names}
        self.peak = {name: {'kp': 0.0, 'kd': 0.0, 'tau': 0.0} for name in self.names}
        self.fallback = {}
        self.last_targets = dict(self.pose)
        self.last_time = None
        self.last_evaluation = None
        self.dt = 0.0
        self.frames = 0
        print('v3.1 motion dynamics begin=', json.dumps({
            'build': BUILD, 'commanded_joints': self.names,
            'entry_coverage': {name: value is not None for name, value in self.entry.items()},
        }), flush=True)

    def terms(self, name, pose, velocity):
        try:
            return scheduled_terms(self.model, name, pose, velocity)
        except ValueError as exc:
            if not coverage_error(exc):
                raise
            # Invalid coverage is not a reason to interrupt a stable legacy
            # trajectory. Corrections slew back to zero; no remote samples
            # are admitted by relaxing the model's distance limits.
            return None

    def prepare(self, targets, progress, now):
        if not math.isfinite(now) or not math.isfinite(progress):
            raise ValueError('non-finite motion frame')
        pose = dict(self.pose)
        pose.update({name: float(q) for name, q in targets.items() if name in JOINTS})
        if not all(math.isfinite(q) for q in pose.values()):
            raise ValueError('non-finite motion reference')
        self.dt = 0.0 if self.last_time is None else clamp(now - self.last_time, 0.0, 0.05)
        # Differentiate the actual reference even when v2.8 deliberately sends
        # dq=0. Keep the original MIT dq field unchanged. Discard timing gaps.
        elapsed = 0.0 if self.last_time is None else now - self.last_time
        velocities = {
            name: clamp((pose[name] - self.last_targets[name]) / elapsed, -1.0, 1.0)
            if 1e-5 <= elapsed <= 0.1 else 0.0 for name in self.names
        }
        progress = clamp(progress, 0.0, 1.0)
        edge = min(1.0, progress / 0.15, (1.0 - progress) / 0.15)
        self.weight = edge * edge * (3.0 - 2.0 * edge)
        # Evaluate the sparse model at 20Hz, keeping the normal CAN loop rate.
        # Endpoint frames always request zero to release before learning.
        if self.last_evaluation is None or now - self.last_evaluation >= 0.05:
            self.desired = {name: self.terms(name, pose, velocities[name]) for name in self.names}
            self.last_evaluation = now
        for name in self.names:
            reason = ('no-entry-coverage' if self.entry[name] is None
                      else 'outside-local-coverage' if self.desired[name] is None else None)
            if reason:
                self.fallback[name] = self.fallback.get(name, 0) + 1
                if self.fallback[name] == 1:
                    print('v3.1 motion dynamics fallback=', json.dumps({'joint': name, 'reason': reason}), flush=True)
        self.last_time = now
        self.last_targets = pose
        self.frames += 1

    def transform(self, name, kp, kd, tau):
        values = (float(kp), float(kd), float(tau))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('non-finite legacy motion command for ' + name)
        kp, kd, tau = values
        goal = {'kp': 0.0, 'kd': 0.0, 'tau': 0.0}
        entry, desired = self.entry[name], self.desired[name]
        if entry is not None and desired is not None:
            low, high = self.model['joints'][name]['tau_limits']
            cap = min(MAX_TAU_CORRECTION, 0.25 * (high - low),
                      0.4 * self.model['settings']['tau_slew_per_second'])
            goal['tau'] = self.weight * clamp(desired['tau_ff'] - entry['tau_ff'], -cap, cap)
            # Local effective inertia schedules relative to the trained gains.
            # A minimum-gains fallback is not an identified inertia estimate.
            if entry['effective_inertia'] is not None and desired['effective_inertia'] is not None:
                ratio = clamp(desired['effective_inertia'] / entry['effective_inertia'] - 1.0,
                              -MAX_GAIN_FRACTION, MAX_GAIN_FRACTION)
                kp_cap = min(MAX_KP_CORRECTION, 0.4 * self.model['settings']['kp_slew_per_second'])
                kd_cap = min(MAX_KD_CORRECTION, 0.4 * self.model['settings']['kd_slew_per_second'])
                goal['kp'] = self.weight * clamp(kp * ratio, -kp_cap, kp_cap)
                goal['kd'] = self.weight * clamp(kd * ratio, -kd_cap, kd_cap)
        rates = self.model['settings']
        for key, setting in (('kp', 'kp_slew_per_second'), ('kd', 'kd_slew_per_second'),
                             ('tau', 'tau_slew_per_second')):
            current = self.correction[name][key]
            limit = float(rates[setting]) * self.dt
            self.correction[name][key] += clamp(goal[key] - current, -limit, limit)
            self.peak[name][key] = max(self.peak[name][key], abs(self.correction[name][key]))
        correction = self.correction[name]
        scheduler = self.model['joints'][name]['scheduler']
        # Preserve a legacy gain above the model's configured maximum, but do
        # not increase it further. Hardware limits still apply to every frame.
        return (
            clamp(kp + correction['kp'], 0.0, min(500.0, max(kp, scheduler['kp_max']))),
            clamp(kd + correction['kd'], 0.0, min(5.0, max(kd, scheduler['kd_max']))),
            clamp(tau + correction['tau'], -MIT_TAU_MAX[name], MIT_TAU_MAX[name]),
        )

    def finish(self):
        remaining = {name: values for name, values in self.correction.items()
                     if any(abs(value) > 1e-8 for value in values.values())}
        # The legacy 0.5s terminal hold normally releases the bounded layer.
        # Never silently hand an unfinished correction to terminal learning.
        if remaining:
            raise RuntimeError('motion dynamics correction did not release before learning: ' + json.dumps(remaining))
        print('v3.1 motion dynamics complete=', json.dumps({
            'frames': self.frames, 'peak_correction': self.peak,
            'legacy_fallback_frames': self.fallback,
            'terminal_correction_zero': True,
        }), flush=True)


def preview_reference_plan(model, plan):
    """Offline preview of a supplied smoothstep reference, not a saved replay.

    start_pose_rad has all seven observed angles; targets_rad lists only joints
    the caller intends to command. A stationary held joint can be listed with
    its current target. This function never opens serial or writes learning.
    """
    pose = {name: float(plan['start_pose_rad'][name]) for name in JOINTS}
    targets = {name: float(q) for name, q in plan['targets_rad'].items()}
    if not targets or set(targets) - set(JOINTS):
        raise ValueError('preview targets must be a non-empty subset of arm joints')
    seconds = float(plan['seconds'])
    if not math.isfinite(seconds) or not 1 <= seconds <= 120:
        raise ValueError('preview seconds must be between 1 and 120')
    commands = {}
    for name in targets:
        row = plan['commands'][name]
        kp, kd, tau = (float(row[key]) for key in ('kp', 'kd', 'tau'))
        if not (0 <= kp <= 500 and 0 <= kd <= 5 and abs(tau) <= MIT_TAU_MAX[name]):
            raise ValueError('invalid preview command for ' + name)
        commands[name] = (kp, kd, tau)
    session = MotionSession(model, pose, targets)
    samples = []
    count = math.ceil(seconds / 0.01)
    for index in range(count + 51):
        progress = min(1.0, index / count)
        shape = progress * progress * (3 - 2 * progress)
        frame = {name: pose[name] + (targets[name] - pose[name]) * shape for name in targets}
        session.prepare(frame, progress, index * 0.01)
        applied = {name: session.transform(name, *commands[name]) for name in targets}
        if index % 25 == 0 or index == count + 50:
            samples.append({'time_s': index * 0.01, 'progress': progress,
                            'targets_rad': frame,
                            'commands': {name: dict(zip(('kp', 'kd', 'tau'), row))
                                         for name, row in applied.items()}})
    session.finish()
    return {'motion_issued': False, 'preview_kind': 'supplied-smoothstep-reference',
            'model_updated_at': model.get('updated_at'), 'commanded_joints': list(targets),
            'peak_correction': session.peak, 'legacy_fallback_frames': session.fallback,
            'terminal_correction_zero': True, 'samples': samples}


def main():
    parser = argparse.ArgumentParser(description='Offline motion dynamics reference preview (no motor commands)')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--preview-plan', type=Path, required=True)
    args = parser.parse_args()
    model = load_motion_model(args.model)
    plan = json.loads(args.preview_plan.read_text(encoding='utf-8'))
    # Keep stdout as one parseable JSON result; coverage diagnostics go to stderr.
    import sys
    with contextlib.redirect_stdout(sys.stderr):
        result = preview_reference_plan(model, plan)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
