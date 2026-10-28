#!/usr/bin/env python3
"""Seven-axis rigid-body adapter for existing operational trajectories.

No hardware imports or learning writes. Shadow records references without
changing commands. Apply requires physical inertials and replaces the sparse
correction layer; it never replaces the legacy/adaptive baseline itself.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import time
from pathlib import Path

import numpy as np

from left_arm_rigid_body import RigidBodyModel, inertial_parameters, vector
from left_arm_v3_1_model_control import JOINTS, MIT_TAU_MAX

BUILD = 'seven-axis-rigid-body-motion-v3-prepared'
LOG_ENV = 'HIVE_ROBOT_RIGID_BODY_LOG'
MAX_TAU_CORRECTION = 0.5
TAU_SLEW_PER_SECOND = 1.5
MAX_RESULT_AGE_SECONDS = .1


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
              'log_path': str(log_path), 'legacy': None,
              'workflow_action': os.environ.get('HIVE_ROBOT_MOTION_ACTION'),
              'workflow_run_id': os.environ.get('HIVE_ROBOT_MOTION_RUN_ID')}
    print('general motion configuration=', json.dumps(configuration_report(result)), flush=True)
    return result


def start_runtime_worker(config):
    """Called by configured_model BEFORE the arm opens serial."""
    if not config['missing_inertials'] and config.get('worker') is None:
        from left_arm_dynamics_worker import DynamicsWorker
        config['worker'] = DynamicsWorker(config['model'].data)
    return config


def close_runtime_configuration(config):
    worker = config.get('worker')
    if worker is not None:
        worker.close(wait=False)


def configuration_report(config):
    model = config['model']
    return {'build': BUILD, 'mode': config['mode'], 'model': config['model_path'],
            'workflow_action': config.get('workflow_action'),
            'workflow_run_id': config.get('workflow_run_id'),
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
            'compute_backend': 'independent-process-latest-reference-mailbox',
            'max_result_age_ms': MAX_RESULT_AGE_SECONDS*1000,
            'torque_cap_Nm': MAX_TAU_CORRECTION, 'log': config['log_path']}


class GeneralMotionSession:
    uses_rigid_body_reference = True

    def __init__(self, config, pose, names, legacy=None):
        self.config = config
        self.session_started_at_utc = datetime.now(timezone.utc).isoformat()
        self.model = config['model']
        self.names = tuple(dict.fromkeys(names))
        self.legacy = legacy
        self.entry_encoder = vector([pose[n] for n in self.model.names], self.model.n)
        self.entry_q = self.model.encoder_to_model(self.entry_encoder)
        self.worker = config.get('worker')
        if not config['missing_inertials'] and self.worker is None:
            raise RuntimeError('dynamics worker must be started before serial opens')
        self.session_id = self.worker.new_session() if self.worker is not None else None
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
        self.frame_peak_ms = 0.0
        self.frame_intervals = []
        self.accepted_results = 0
        self.pending_frames = 0
        self.stale_frames = 0
        self.worker_error_frames = 0
        self.submission_skips = 0
        self.peak_result_age_ms = 0.0
        self.last_result = None
        self.last_accepted_sequence = None
        self.last_seen_worker_sequence = None
        self.compute_samples_ms = []
        self.cpu_samples_ms = []
        print('general motion begin=', json.dumps({**configuration_report(config),
              'commanded_joints': self.names, 'legacy_sparse_layer_preserved': legacy is not None}), flush=True)

    def prepare(self, targets, progress, now, velocities=None, accelerations=None):
        frame_start = time.perf_counter()
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
        if self.last_time is not None and len(self.frame_intervals) < 20000:
            self.frame_intervals.append(max(now-self.last_time, 0.0)*1000)
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
            if self.worker is not None:
                if not self.worker.submit(self.session_id, now, self.entry_q, q, dq, ddq):
                    self.submission_skips += 1
            self.last_evaluation = now
        self.prediction = None
        if self.worker is not None:
            result = self.worker.poll()
            if result is not None and result['session'] == self.session_id:
                self.last_result = result
                self.compute_peak_ms = max(self.compute_peak_ms, result['compute_ms'])
                if result['sequence'] != self.last_seen_worker_sequence:
                    if len(self.compute_samples_ms) < 20000:
                        self.compute_samples_ms.append(result['compute_ms'])
                        self.cpu_samples_ms.append(result.get('cpu_ms',result['compute_ms']))
                    self.last_seen_worker_sequence = result['sequence']
            result = self.last_result
            if not self.worker.is_alive():
                self.worker_error_frames += 1
            elif result is None:
                self.pending_frames += 1
            elif not result['ok'] or not np.isfinite(result['prediction']).all():
                self.worker_error_frames += 1
            else:
                age = now-result['submitted']
                self.peak_result_age_ms = max(self.peak_result_age_ms, max(age, 0.)*1000)
                if not 0 <= age <= MAX_RESULT_AGE_SECONDS:
                    self.stale_frames += 1
                else:
                    self.prediction = result['prediction']
                    if result['sequence'] != self.last_accepted_sequence:
                        self.accepted_results += 1
                        self.last_accepted_sequence = result['sequence']
        for name in self.names:
            i = self.model.names.index(name)
            raw = 0.0 if self.prediction is None else float(self.prediction[i])
            self.goal[name] = weight*min(max(raw, -MAX_TAU_CORRECTION), MAX_TAU_CORRECTION)
        if self.last_log_time is None or now-self.last_log_time >= .1:
            record = {'elapsed_s': now-self.start_time, 'progress': p,
                      'q_encoder_ref': encoder.tolist(), 'q_model_ref': q.tolist(),
                      'dq_model_ref': dq.tolist(), 'ddq_model_ref': ddq.tolist(),
                      'predicted_delta_tau_encoder_Nm': None if self.prediction is None else self.prediction.tolist(),
                      'fresh_result_available': self.prediction is not None,
                      'desired_bounded_delta_tau_Nm': dict(self.goal)}
            if len(self.records) < 2000:
                self.records.append(record)
            else:
                self.dropped_records += 1
            self.last_log_time = now
        self.frames += 1
        self.frame_peak_ms = max(self.frame_peak_ms, (time.perf_counter()-frame_start)*1000)

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
                  'session_started_at_utc': self.session_started_at_utc,
                  'frames': self.frames, 'peak_applied_delta_tau_Nm': self.peak,
                  'peak_dynamics_compute_ms': self.compute_peak_ms,
                  'peak_prepare_frame_ms': self.frame_peak_ms,
                  'dynamics_compute_ms': {'p50': float(np.percentile(self.compute_samples_ms,50)) if self.compute_samples_ms else None,
                                          'p99': float(np.percentile(self.compute_samples_ms,99)) if self.compute_samples_ms else None},
                  'dynamics_cpu_ms': {'p50': float(np.percentile(self.cpu_samples_ms,50)) if self.cpu_samples_ms else None,
                                      'p99': float(np.percentile(self.cpu_samples_ms,99)) if self.cpu_samples_ms else None},
                  'frame_interval_ms': {'p50': float(np.percentile(self.frame_intervals,50)) if self.frame_intervals else None,
                                        'p99': float(np.percentile(self.frame_intervals,99)) if self.frame_intervals else None,
                                        'max': max(self.frame_intervals) if self.frame_intervals else None},
                  'accepted_worker_results': self.accepted_results,
                  'pending_worker_frames': self.pending_frames,
                  'stale_worker_frames': self.stale_frames,
                  'worker_error_frames': self.worker_error_frames,
                  'worker_submission_skips': self.submission_skips,
                  'peak_result_age_ms': self.peak_result_age_ms,
                  'terminal_correction_zero': True, 'reference_samples': self.records,
                  'dropped_reference_records': self.dropped_records,
                  'prediction_status': ('missing-physical-inertials' if self.config['missing_inertials']
                                        else 'available' if self.accepted_results else 'no-fresh-worker-result')}
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
    parser.add_argument('--model')
    parser.add_argument('--mode', choices=('shadow', 'apply'), default='shadow')
    parser.add_argument('--summary', action='store_true', help='print compact recent log summaries without loading a model')
    parser.add_argument('--last', type=int, default=2)
    parser.add_argument('--action', help='summarize only the newest recorded run of this workflow action')
    parser.add_argument('--log', default=str(Path(__file__).parent/'data'/'left_arm_general_motion_runs.jsonl'))
    args = parser.parse_args()
    if args.summary:
        if not 1 <= args.last <= 20:
            parser.error('--last must be between 1 and 20')
        keys=('build','mode','workflow_action','workflow_run_id','session_started_at_utc',
              'frames','commanded_joints','prediction_status',
              'peak_dynamics_compute_ms','peak_prepare_frame_ms','frame_interval_ms',
              'dynamics_compute_ms','dynamics_cpu_ms',
              'accepted_worker_results','pending_worker_frames','stale_worker_frames',
              'worker_error_frames','worker_submission_skips','peak_result_age_ms',
              'terminal_correction_zero','peak_applied_delta_tau_Nm')
        with Path(args.log).open(encoding='utf8') as handle:
            if args.action:
                lines=deque(maxlen=args.last)
                latest_run=None
                for line in handle:
                    try:
                        row=json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if row.get('workflow_action') != args.action:
                        continue
                    run_id=row.get('workflow_run_id')
                    if not run_id:
                        continue
                    if run_id != latest_run:
                        lines.clear()
                        latest_run=run_id
                    # Keep compact records, not large reference arrays, in memory.
                    lines.append(json.dumps({k:row.get(k) for k in keys}))
            else:
                lines=deque(handle,maxlen=args.last)
        if not lines:
            print(json.dumps({'summary_status':'no-matching-motion-records','workflow_action':args.action}))
        for index,line in enumerate(lines,1):
            try:
                row=json.loads(line)
                print(json.dumps({'segment':index,**{k:row.get(k) for k in keys}}))
            except json.JSONDecodeError:
                print(json.dumps({'segment':index,'error':'incomplete log line'}))
        return
    if not args.model:
        parser.error('--model is required unless --summary is used')
    config = load_configuration(args.model, args.mode)
    print(json.dumps({'ok': True, 'motion_issued': False, **configuration_report(config)}, indent=2))


if __name__ == '__main__':
    main()
