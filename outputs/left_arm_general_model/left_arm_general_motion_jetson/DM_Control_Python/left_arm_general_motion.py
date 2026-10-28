#!/usr/bin/env python3
"""Seven-axis rigid-body adapter for existing operational trajectories.

No hardware imports or learning writes. Shadow records references without
changing commands. Apply requires physical inertials and replaces the sparse
correction layer; it never replaces the legacy/adaptive baseline itself.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path

import numpy as np

from left_arm_rigid_body import RigidBodyModel, inertial_parameters, vector
from left_arm_v3_1_model_control import JOINTS, MIT_TAU_MAX

BUILD = 'seven-axis-rigid-body-motion-v1'
LOG_ENV = 'HIVE_ROBOT_RIGID_BODY_LOG'
MAX_TAU_CORRECTION = 0.5
TAU_SLEW_PER_SECOND = 1.5


def load_configuration(path, mode='shadow'):
    if mode not in ('shadow', 'apply'):
        raise ValueError('rigid body mode must be shadow or apply')
    path = Path(path).resolve()
    model = RigidBodyModel.load(path)
    if tuple(model.names) != tuple(JOINTS):
        raise ValueError('rigid body model must use the complete calibrated seven-axis order')
    gravity = model.configured_gravity()
    if not 9.0 <= np.linalg.norm(gravity) <= 10.5:
        raise ValueError('configured installation gravity magnitude must be between 9 and 10.5 m/s2')
    missing = []
    for name, body in zip(model.names, model.inertials):
        if any(body.get(key) is None for key in ('mass_kg', 'com_m', 'inertia_com_kg_m2')):
            missing.append(name)
        else:
            inertial_parameters([body])
    # Require valid friction priors before serial opens, even for shadow.
    model.friction(np.zeros(model.n))
    model.rotor_inertias()
    if mode == 'apply' and missing:
        raise ValueError('rigid body apply requires mass/COM/inertia for: ' + ','.join(missing)
                         + '; use shadow to validate the motion connection without invented parameters')
    log_path = Path(os.environ.get(LOG_ENV, Path(__file__).parent/'data'/'left_arm_general_motion_runs.jsonl')).resolve()
    result = {'model': model, 'mode': mode, 'missing_inertials': missing,
              'model_path': str(path), 'model_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'log_path': str(log_path), 'legacy': None}
    print('general motion configuration=', json.dumps(configuration_report(result)), flush=True)
    return result


def configuration_report(config):
    model = config['model']
    return {'build': BUILD, 'mode': config['mode'], 'model': config['model_path'],
            'model_sha256': config['model_sha256'],
            'gravity_in_cad_frame_m_s2': model.configured_gravity().tolist(),
            'gravity_configuration': model.data.get('gravity_configuration'),
            'dynamics_parameters_available': not config['missing_inertials'],
            'parameter_status': model.data.get('parameter_status', 'supplied-inertials'),
            'identification_records': {key: model.data.get('identification', {}).get(key)
                                       for key in ('gravity_records_used', 'unique_inertia_records_used')},
            'parameters_independently_verified': model.data.get('identification', {}).get('physical_parameters_independently_verified', False),
            'missing_inertials': config['missing_inertials'],
            'torque_policy': 'bounded-pose-relative-ID-plus-friction; legacy/adaptive baseline retained',
            'torque_cap_Nm': MAX_TAU_CORRECTION, 'log': config['log_path']}


class GeneralMotionSession:
    uses_rigid_body_reference = True

    def __init__(self, config, pose, names, legacy=None):
        self.config = config
        self.model = config['model']
        self.names = tuple(dict.fromkeys(names))
        self.legacy = legacy
        self.entry_encoder = vector([pose[n] for n in self.model.names], self.model.n)
        self.entry_q = self.model.encoder_to_model(self.entry_encoder)
        self.entry_gravity = (self.model.gravity_torques(self.entry_q)
                              if not config['missing_inertials'] else None)
        self.corrections = dict.fromkeys(self.names, 0.0)
        self.peak = dict.fromkeys(self.names, 0.0)
        self.goal = dict.fromkeys(self.names, 0.0)
        self.last_time = None
        self.start_time = None
        self.last_evaluation = None
        self.last_log_time = None
        self.frames = 0
        self.records = []
        self.dropped_records = 0
        self.prediction = None
        self.compute_peak_ms = 0.0
        print('general motion begin=', json.dumps({**configuration_report(config),
              'commanded_joints': self.names, 'legacy_sparse_layer_preserved': legacy is not None}), flush=True)

    def prepare(self, targets, progress, now, velocities=None, accelerations=None):
        if not math.isfinite(now) or not math.isfinite(progress):
            raise ValueError('nonfinite rigid body motion frame')
        if velocities is None or accelerations is None:
            raise ValueError('rigid body motion needs analytic reference velocity and acceleration')
        encoder = self.entry_encoder.copy()
        dq_encoder = np.zeros(self.model.n)
        ddq_encoder = np.zeros(self.model.n)
        for i, name in enumerate(self.model.names):
            if name in self.names:
                if name not in targets or name not in velocities or name not in accelerations:
                    raise ValueError('incomplete rigid body reference for ' + name)
                encoder[i], dq_encoder[i], ddq_encoder[i] = targets[name], velocities[name], accelerations[name]
        if not all(np.isfinite(x).all() for x in (encoder, dq_encoder, ddq_encoder)):
            raise ValueError('nonfinite rigid body reference')
        self.dt = 0.0 if self.last_time is None else min(max(now-self.last_time, 0.0), .05)
        self.last_time = now
        if self.start_time is None:
            self.start_time = now
        q = self.model.encoder_to_model(encoder)
        dq, ddq = self.model.signs*dq_encoder, self.model.signs*ddq_encoder
        p = min(max(progress, 0.0), 1.0)
        edge = min(1.0, p/.15, (1-p)/.15)
        weight = edge*edge*(3-2*edge)
        if self.legacy is not None:
            self.legacy.prepare(targets, progress, now)
        if self.last_evaluation is None or now-self.last_evaluation >= .05:
            if self.entry_gravity is not None:
                compute_start = time.perf_counter()
                # Full coupled inverse dynamics; this includes M*ddq+c+g.
                predicted = self.model.inverse_dynamics(q, dq, ddq) + self.model.friction(dq)
                self.prediction = self.model.signs*(predicted-self.entry_gravity)
                if not np.isfinite(self.prediction).all():
                    raise ValueError('nonfinite general dynamics prediction')
                self.compute_peak_ms = max(self.compute_peak_ms, (time.perf_counter()-compute_start)*1000)
            self.last_evaluation = now
        for name in self.names:
            i = self.model.names.index(name)
            raw = 0.0 if self.prediction is None else float(self.prediction[i])
            self.goal[name] = weight*min(max(raw, -MAX_TAU_CORRECTION), MAX_TAU_CORRECTION)
        if self.last_log_time is None or now-self.last_log_time >= .1:
            record = {'elapsed_s': now-self.start_time, 'progress': p,
                      'q_encoder_ref': encoder.tolist(), 'q_model_ref': q.tolist(),
                      'dq_model_ref': dq.tolist(), 'ddq_model_ref': ddq.tolist(),
                      'predicted_delta_tau_encoder_Nm': None if self.prediction is None else self.prediction.tolist(),
                      'desired_bounded_delta_tau_Nm': dict(self.goal)}
            if len(self.records) < 2000:
                self.records.append(record)
            else:
                self.dropped_records += 1
            self.last_log_time = now
        self.frames += 1

    def transform(self, name, kp, kd, tau):
        if not all(math.isfinite(float(v)) for v in (kp, kd, tau)):
            raise ValueError('nonfinite baseline command')
        if self.config['mode'] == 'shadow':
            return self.legacy.transform(name, kp, kd, tau) if self.legacy is not None else (kp, kd, tau)
        current = self.corrections[name]
        limit = TAU_SLEW_PER_SECOND*self.dt
        current += min(max(self.goal[name]-current, -limit), limit)
        self.corrections[name] = current
        self.peak[name] = max(self.peak[name], abs(current))
        # Same fixed limits as the existing MIT adapter; never change q/dq/kp/kd.
        return kp, kd, min(max(tau+current, -MIT_TAU_MAX[name]), MIT_TAU_MAX[name])

    def finish(self):
        if self.legacy is not None:
            self.legacy.finish()
        if any(abs(x) > 1e-8 for x in self.corrections.values()):
            raise RuntimeError('rigid body correction did not release before terminal learning')
        result = {**configuration_report(self.config), 'commanded_joints': self.names,
                  'frames': self.frames, 'peak_applied_delta_tau_Nm': self.peak,
                  'peak_dynamics_compute_ms': self.compute_peak_ms,
                  'terminal_correction_zero': True, 'reference_samples': self.records,
                  'dropped_reference_records': self.dropped_records,
                  'prediction_status': 'available' if self.entry_gravity is not None else 'missing-physical-inertials'}
        # Disk I/O only AFTER terminal release, never inside the CAN loop.
        try:
            path = Path(self.config['log_path'])
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('a', encoding='utf8') as handle:
                handle.write(json.dumps(result, allow_nan=False)+'\n')
        except OSError as exc:
            print('general motion log error=', str(exc), flush=True)
        print('general motion complete=', json.dumps({k: v for k, v in result.items()
              if k != 'reference_samples'}), flush=True)


def main():
    parser = argparse.ArgumentParser(description='Validate general motion configuration without serial or motion')
    parser.add_argument('--model', required=True)
    parser.add_argument('--mode', choices=('shadow', 'apply'), default='shadow')
    args = parser.parse_args()
    config = load_configuration(args.model, args.mode)
    print(json.dumps({'ok': True, 'motion_issued': False, **configuration_report(config)}, indent=2))


if __name__ == '__main__':
    main()
