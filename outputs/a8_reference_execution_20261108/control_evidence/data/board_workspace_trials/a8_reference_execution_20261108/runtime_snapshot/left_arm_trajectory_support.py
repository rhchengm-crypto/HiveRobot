"""Precompute a finite load-support envelope for a geometric joint path.

The static identification envelope is not a bound on total load at every pose.
No later measured PD torque, integral learning, or per-square correction is used.
The result is a planning budget, not a certified motor continuous-torque rating.
"""
import hashlib
import json
import math

import numpy as np


def path_digest(trajectory):
    geometry = {'start_rad': trajectory['start_rad'],
                'joints': [row['encoder_rad'] for row in trajectory['waypoints']]}
    return hashlib.sha256(json.dumps(geometry, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def gravity_envelope(model, trajectory):
    """Sample the complete interpolated path before physical motion starts."""
    names = model.names
    previous = np.asarray([trajectory['start_rad'][n] for n in names], dtype=float)
    minimum = np.full(model.n, np.inf)
    maximum = np.full(model.n, -np.inf)
    samples = 0
    for row in trajectory['waypoints']:
        end = np.asarray([row['encoder_rad'][n] for n in names], dtype=float)
        if not np.isfinite([previous, end]).all():
            raise ValueError('nonfinite planned support coordinates')
        count = max(1, math.ceil(np.max(np.abs(end-previous))/math.radians(.5)))
        for alpha in np.linspace(0., 1., count+1):
            q = previous+alpha*(end-previous)
            tau = model.signs*model.gravity_torques(model.encoder_to_model(q))
            if not np.isfinite(tau).all():
                raise ValueError('nonfinite planned gravity')
            minimum = np.minimum(minimum, tau)
            maximum = np.maximum(maximum, tau)
            samples += 1
        previous = end
    if samples == 0:
        raise ValueError('empty support path')
    return {'version': 1, 'path_sha256': path_digest(trajectory), 'samples': samples,
            'max_sample_step_deg': .5, 'headroom_Nm': .5,
            'gravity_encoder_Nm': {n: [float(a), float(b)]
                                   for n, a, b in zip(names, minimum, maximum)},
            'physical_torque_rating_verified': False,
            'scope': 'sampled gravity range plus bounded residual headroom; runtime clipping retained'}


def support_limits(envelope, trajectory, anchor, initial_gravity, model_limits, hardware_limits):
    if (envelope.get('version') != 1 or envelope.get('path_sha256') != path_digest(trajectory)
            or envelope.get('headroom_Nm') != .5):
        raise ValueError('planned support envelope differs from trajectory')
    result = {}
    for n in anchor:
        lo, hi = map(float, envelope['gravity_encoder_Nm'][n])
        old_lo, old_hi = map(float, model_limits[n])
        observed, gravity = float(anchor[n]), float(initial_gravity[n])
        maximum = float(hardware_limits[n])
        if (not all(math.isfinite(v) for v in (lo, hi, old_lo, old_hi, observed, gravity, maximum))
                or lo > hi or old_lo >= old_hi or maximum <= 0 or abs(observed) > maximum):
            raise ValueError('invalid trajectory support envelope for '+n)
        # Translate model gravity to the ONE observed entry load. Include the
        # existing static envelope and residual's existing 0.5 N m allowance.
        low = min(old_lo, observed, observed+lo-gravity-.5)
        high = max(old_hi, observed, observed+hi-gravity+.5)
        if low < -maximum or high > maximum:
            raise ValueError('planned load support exceeds MIT range for '+n)
        result[n] = (low, high)
    return result


def gripper_feedback_clearance(model, cfg, feedback):
    """Cheap observed-pose check; never reinterpret FK as visual ground truth."""
    envelope = cfg.get('gripper_vertex_envelope')
    if not envelope:
        return None
    q = [feedback[n]['pos'] for n in model.names]
    frame = model.fk(model.encoder_to_model(q))[-1]
    vertices = np.asarray(envelope['vertices_in_wrist_frame_m'])
    normal = np.asarray(cfg['board_to_cad_rotation'])[:, 2]
    origin = np.asarray(cfg['board_to_cad_translation_m'])
    minimum = float(np.min((vertices@frame[:3, :3].T+frame[:3, 3]-origin)@normal)*1000)
    floor = float(cfg.get('_gripper_min_height_mm', envelope.get('minimum_height_mm', 5.)))
    if not np.isfinite([minimum, floor]).all():
        raise ValueError('nonfinite gripper feedback geometry')
    return {'lowest_vertex_height_mm': minimum, 'minimum_height_mm': floor,
            'clearance_mm': minimum-floor, 'external_measurement': False}
