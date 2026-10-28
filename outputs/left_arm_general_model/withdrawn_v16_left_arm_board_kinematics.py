"""Offline board registration audit and bounded seven-axis Cartesian IK.

No serial, Web requests, motor enables, or controller/learning file writes.
All spatial computations use metres in the approved CAD base frame. The
terminal wrist frame is not assumed to be the centre between the fingers.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from left_arm_rigid_body import RigidBodyModel, axis_rotation, vector


def rotation(value):
    r = np.asarray(value, dtype=float)
    if (r.shape != (3, 3) or not np.isfinite(r).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-8)):
        raise ValueError('rotation must be a proper finite 3x3 matrix')
    return r


def rotation_error(target, current):
    """Spatial SO(3) logarithm, including rotations near pi."""
    r = rotation(target) @ rotation(current).T
    angle = np.arccos(np.clip((np.trace(r) - 1) / 2, -1, 1))
    vee = np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])
    if angle < 1e-7:
        return .5 * vee
    if np.pi - angle < 1e-5:
        _, vectors = np.linalg.eigh((r + r.T) / 2)
        axis = vectors[:, -1]
        if axis @ vee < 0:
            axis = -axis
        return angle * axis
    return angle / (2 * np.sin(angle)) * vee


def tcp_pose(model, encoder, tcp_local_m):
    q = model.encoder_to_model(encoder)
    frame = model.fk(q)[-1].copy()
    frame[:3, 3] += frame[:3, :3] @ vector(tcp_local_m, 3, 'wrist-local TCP')
    return frame


def board_point_to_cad(point_mm, board_to_cad_rotation, board_to_cad_translation_m):
    point = vector(point_mm, 3, 'board point mm')
    if np.any(point[:2] < 0) or np.any(point[:2] > 440) or not 0 <= point[2] <= 400:
        raise ValueError('point outside 440mm board or 0..400mm height')
    return (rotation(board_to_cad_rotation) @ (point / 1000)
            + vector(board_to_cad_translation_m, 3, 'board origin in CAD'))


def plan_board_approach(model, point_mm, seed_encoder, config, clearance_mm=120., spacing_mm=5.):
    """Above/contact/lift preview with sampled Cartesian IK, never execution.

    A fixed wrist rotation keeps the taught grasp orientation. A future
    executor must additionally check swept link/finger obstacles and follow
    the waypoints rather than interpolate directly to the contact endpoint.
    """
    point = vector(point_mm, 3, 'board contact point')
    clearance = float(clearance_mm)
    spacing = float(spacing_mm)
    if not np.isfinite([clearance, spacing]).all() or not point[2] < clearance <= 400 or not 0 < spacing <= 20:
        raise ValueError('clearance must exceed contact height; spacing must be in (0,20] mm')
    # Validate both transforms, endpoints and the full orientation before IK.
    def convert(p):
        return board_point_to_cad(p, config['board_to_cad_rotation'], config['board_to_cad_translation_m'])
    target = convert(point)
    above = convert([point[0], point[1], clearance])
    desired_r = rotation(config['grasp_wrist_rotation_in_cad'])
    tcp = vector(config['tcp_in_wrist_frame_m'], 3, 'TCP')
    limits = config['joint_limits_encoder_rad']
    q = vector(seed_encoder, model.n, 'seed')
    current = tcp_pose(model, q, tcp)[:3, 3]
    records = []
    for stage, end in [('source_above', above), ('contact', target), ('lift', above)]:
        begin = current.copy()
        count = max(1, int(np.ceil(np.linalg.norm(end-begin)*1000/spacing)))
        for index in range(1, count+1):
            position = begin + (end-begin)*index/count
            solution = solve_ik(model, position, q, tcp, limits, desired_r)
            records.append({'stage': stage, 'target_cad_m': position.tolist(), **solution})
            if not solution['success']:
                return {'success': False, 'failed_stage': stage, 'waypoints': records,
                        'motion_issued': False, 'executable': False}
            q = np.array([solution['encoder_rad'][n] for n in model.names])
        current = end
    return {'success': True, 'waypoints': records, 'motion_issued': False,
            'executable': False, 'collision_checked': False,
            'note': 'Geometry preview only; gripper close/contact sensing is not executed.'}


def solve_ik(model, target_m, seed_encoder, tcp_local_m, limits_encoder,
             target_wrist_rotation=None, max_iterations=300,
             position_tolerance_m=.001, rotation_tolerance_rad=.01,
             fixed_encoder=None):
    """Damped Jacobian IK; no angle wrapping, invented limits, or execution.

    Supply a wrist rotation to constrain grasp orientation. Position-only IK
    is useful for geometry tests but does not define a usable grasp pose.
    """
    target = vector(target_m, 3, 'target')
    tcp = vector(tcp_local_m, 3, 'TCP')
    q = vector(seed_encoder, model.n, 'seed').copy()
    limits = np.asarray(limits_encoder, dtype=float)
    if (limits.shape != (model.n, 2) or not np.isfinite(limits).all()
            or np.any(limits[:, 0] >= limits[:, 1])):
        raise ValueError('explicit ordered encoder joint limits required')
    if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
        raise ValueError('seed outside joint limits')
    fixed = {} if fixed_encoder is None else dict(fixed_encoder)
    fixed_indices = []
    for name, value in fixed.items():
        if name not in model.names or not np.isfinite(float(value)):
            raise ValueError('invalid fixed encoder constraint')
        index = model.names.index(name)
        if not limits[index, 0] <= float(value) <= limits[index, 1]:
            raise ValueError('fixed encoder constraint outside joint limits')
        q[index] = float(value)
        fixed_indices.append(index)
    if not (position_tolerance_m > 0 and rotation_tolerance_rad > 0):
        raise ValueError('positive tolerances required')
    desired_r = None if target_wrist_rotation is None else rotation(target_wrist_rotation)

    def residual(angles):
        frame = tcp_pose(model, angles, tcp)
        linear = target - frame[:3, 3]
        angular = np.zeros(3) if desired_r is None else rotation_error(desired_r, frame[:3, :3])
        error = linear if desired_r is None else np.r_[linear, .15 * angular]
        return error, np.linalg.norm(linear), np.linalg.norm(angular)

    for iteration in range(max_iterations + 1):
        error, position_error, angular_error = residual(q)
        if position_error <= position_tolerance_m and angular_error <= rotation_tolerance_rad:
            break
        if iteration == max_iterations:
            break
        jv, jw = model.point_jacobian(model.encoder_to_model(q), model.n - 1, tcp)
        jac = jv if desired_r is None else np.vstack([jv, .15 * jw])
        jac *= model.signs[None, :]
        jac[:, fixed_indices] = 0.
        delta = jac.T @ np.linalg.solve(jac @ jac.T + .003**2 * np.eye(len(error)), error)
        delta *= min(1., .08 / max(np.max(np.abs(delta)), 1e-12))
        accepted = False
        for scale in (1., .5, .25, .125, .0625):
            trial = np.clip(q + scale * delta, limits[:, 0], limits[:, 1])
            if np.linalg.norm(residual(trial)[0]) < np.linalg.norm(error) - 1e-12:
                q = trial
                accepted = True
                break
        if not accepted:
            break
    success = bool(position_error <= position_tolerance_m and angular_error <= rotation_tolerance_rad)
    return {'success': success, 'encoder_rad': dict(zip(model.names, q.tolist())),
            'position_error_mm': float(position_error * 1000),
            'rotation_error_deg': float(np.degrees(angular_error)),
            'iterations': iteration, 'orientation_constrained': desired_r is not None,
            'motion_issued': False, 'collision_checked': False}


def rigid_fit(source, target):
    a, b = np.asarray(source), np.asarray(target)
    if np.linalg.matrix_rank(a - a.mean(0), tol=1e-8) < 2:
        raise ValueError('board points are collinear')
    u, _, vt = np.linalg.svd((a-a.mean(0)).T @ (b-b.mean(0)))
    d = np.eye(3)
    d[2, 2] = np.linalg.det(vt.T @ u.T)
    r = vt.T @ d @ u.T
    return r, b.mean(0) - r @ a.mean(0)


def estimate_board_and_tcp(model, anchors):
    """Diagnostic joint fit of board pose and wrist-local TCP from teaching.

    This is an estimate from historical anchors, never a validated runtime
    calibration. Different piece contact points can invalidate a shared TCP.
    """
    if len(anchors) < 4:
        raise ValueError('at least four explicit joint/board correspondences required')
    b = np.array([a['board_tcp_mm'] for a in anchors]) / 1000
    frames = np.array([model.fk(model.encoder_to_model([a['pose_rad'][n] for n in model.names]))[-1]
                       for a in anchors])
    rr, oo = frames[:, :3, :3], frames[:, :3, 3]
    best = None
    for seed in ([0, 0, 0], [0, 0, .2], [0, 0, -.2], [.2, 0, 0], [0, .2, 0]):
        p = np.array(seed, dtype=float)
        for _ in range(1500):
            r, t = rigid_fit(b, oo + np.einsum('nij,j->ni', rr, p))
            next_p = np.linalg.lstsq(rr.reshape(-1, 3), (b @ r.T + t - oo).ravel(), rcond=None)[0]
            if np.linalg.norm(next_p-p) < 1e-10:
                p = next_p
                break
            p = next_p
        r, t = rigid_fit(b, oo + np.einsum('nij,j->ni', rr, p))
        residual = oo + np.einsum('nij,j->ni', rr, p) - (b @ r.T + t)
        score = np.linalg.norm(residual)
        if best is None or score < best[0]:
            best = score, r, t, p, residual
    _, r, t, p, residual = best
    return {'status': 'historical-anchor-estimate-only', 'runtime_enabled': False,
            'board_to_cad_rotation': r.tolist(), 'board_to_cad_translation_m': t.tolist(),
            'tcp_in_wrist_frame_m': p.tolist(), 'tcp_distance_mm': float(np.linalg.norm(p)*1000),
            'fit_errors_mm': {a['id']: float(np.linalg.norm(e)*1000) for a, e in zip(anchors, residual)},
            'independent_check_points': 0,
            'warning': 'Shared contact point and historical pose applicability are not independently verified.'}


def audit(root):
    root = Path(root)
    model_path = root/'scripts/data/left_arm_general_dynamics.json'
    geometry_path = root/'scripts/data/chess_crown_geometry_v2_8.json'
    saved_path = root/'scripts/data/left_arm_v2_6_saved_moves.json'
    model = RigidBodyModel.load(model_path)
    geometry = json.loads(geometry_path.read_text(encoding='utf8'))
    saved = json.loads(saved_path.read_text(encoding='utf8'))['moves']
    anchors = []
    for section in ('grasp_anchors', 'placement_anchors'):
        for key, value in geometry.get(section, {}).items():
            if value.get('board_tcp_mm') is not None:
                anchors.append(dict(value, id=section+'/'+key))
    report = {'schema': 1, 'motion_issued': False,
              'source_sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (model_path, geometry_path, saved_path)},
              'board_size_mm': 440, 'square_size_mm': 55,
              'board_coordinates': 'outer a1 corner; X toward h1, Y toward a8, Z above playing surface',
              'saved_joint_poses': list(saved),
              'explicit_correspondences': [{k: a.get(k) for k in ('id', 'saved_move_name', 'board_tcp_mm')}
                                           for a in anchors],
              'missing_runtime_inputs': [],
              'runtime_controllers_changed': False}
    for missing, value in [('wrist-to-finger-contact TCP', model.data.get('tool_tcp_transform')),
                           ('physical joint limits', model.data.get('physical_joint_limits')),
                           ('board-to-CAD registration', geometry.get('calibration')),
                           ('piece grasp profiles', geometry.get('profiles'))]:
        if not value:
            report['missing_runtime_inputs'].append(missing)
    report['anchor_fit_candidate'] = estimate_board_and_tcp(model, anchors)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
