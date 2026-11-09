"""Explicit lower25mm endpoint counterfactual, offline and non-executable."""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import hashlib
import json
import math
from pathlib import Path
import sys
import time
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parents[1]
SCRIPTS = OUT/'source_snapshot/scripts'
sys.path.insert(0, str(SCRIPTS))
import numpy as np
import left_arm_board_grasp as b
from left_arm_board_kinematics import tcp_pose, solve_ik, rotation_error


def read(p):
    return json.loads(Path(p).read_text(encoding='utf8'))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    begin = time.monotonic()
    model_path = SCRIPTS/'data/left_arm_general_dynamics.json'
    cfg_path = SCRIPTS/'data/left_arm_board_workspace_candidate.json'
    hashes = {str(p.relative_to(ROOT)).replace('\\', '/'):sha(p) for p in (model_path, cfg_path)}
    m = b.RigidBodyModel.load(model_path)
    cfg = b.load_board_config(cfg_path, m)
    raw_cfg = read(cfg_path)
    saved = read(ROOT/'outputs/a8_saved_reference_20261108/saved_pose_audit.json')['saved_move']['pose']
    q0 = np.array([saved[n] for n in m.names])
    tcp = np.array(cfg['tcp_in_wrist_frame_m'])
    r, t = np.array(cfg['board_to_cad_rotation']), np.array(cfg['board_to_cad_translation_m'])
    reference = tcp_pose(m, q0, tcp)
    point = (reference[:3, 3]-t)@r*1000
    target_board = point-np.array([0., 0., 25.])
    target = r@(target_board/1000)+t
    desired = reference[:3, :3]
    limits = np.array(cfg['joint_limits_encoder_rad'])
    scene = b.collision_scene(m, cfg)
    trials, continuation = [], []
    q = q0.copy()
    reached = True
    max_edge_rotation_error = 0.
    for z in np.linspace(point[2], target_board[2], 14)[1:]:
        if time.monotonic()-begin > 27.:
            reached = False
            break
        waypoint = np.array([point[0], point[1], z])
        answer = solve_ik(m, r@(waypoint/1000)+t, q, tcp, limits, desired,
            max_iterations=250, position_tolerance_m=.0005,
            rotation_tolerance_rad=.005, clearance_function=scene)
        nq = np.array([answer['encoder_rad'][n] for n in m.names])
        rec = dict(target_board_mm=waypoint.tolist(), **answer)
        continuation.append(rec)
        if not answer['success']:
            reached = False
            q = nq
            break
        b.check_transit_edge(m, q, nq, cfg)
        count = max(1, int(np.ceil(np.max(np.abs(nq-q))/math.radians(.1))))
        for f in np.linspace(0., 1., count+1):
            orientation = tcp_pose(m, q+f*(nq-q), tcp)[:3, :3]
            max_edge_rotation_error = max(max_edge_rotation_error,
                float(np.degrees(np.linalg.norm(rotation_error(desired, orientation)))))
        q = nq
    seeds = [q, q0]
    height_report = read(OUT/'height_diagnosis.json')
    for trial in ('v33_current', 'v32_previous'):
        seed = height_report['trials'][trial]['motion_endpoint']['encoder_rad']
        seeds.append(np.array([seed[n] for n in m.names]))
    seeds.extend(np.array([seed[n] for n in m.names]) for seed in cfg['seeds'].values())
    for index, seed in enumerate(seeds):
        if time.monotonic()-begin > 29.:
            break
        answer = solve_ik(m, target, np.clip(seed, limits[:, 0], limits[:, 1]), tcp, limits, desired,
            max_iterations=300, position_tolerance_m=.0005,
            rotation_tolerance_rad=.005, clearance_function=scene)
        answer['seed_index'] = index
        trials.append(answer)
        if answer['success']:
            break
    best = min(trials, key=lambda a: (not a['success'], a['position_error_mm']+.15*math.radians(a['rotation_error_deg'])*1000))
    q = np.array([best['encoder_rad'][n] for n in m.names])
    pose = tcp_pose(m, q, tcp)
    board = (pose[:3, 3]-t)@r*1000
    clearance = scene(q)
    hard_endpoint_pass = False
    endpoint_failure = None
    try:
        b.check_board_intersection(m, q, cfg)
        hard_endpoint_pass = True
    except Exception as exc:
        endpoint_failure = dict(error=str(exc), diagnostics=getattr(exc, 'diagnostics', None))
    success = bool(best['success'] and hard_endpoint_pass and np.all(clearance >= 0.))
    result = dict(schema=1, purpose='User reports current gripping region at least25mm too high: explicitly labelled lower25mm Cartesian endpoint counterfactual.',
        motion_issued=False, hardware_accessed=False, network_accessed=False, executable=False,
        prepared_execution_stamp_present=False, runtime_or_model_modified=False,
        user_visual_gap=dict(approximate_lower_bound_mm=25., calibrated_measurement=False, exact_physical_TCP_height_known=False),
        reference_board_mm=point.tolist(), diagnostic_target_board_mm=target_board.tolist(),
        target_change_semantics='Explicit prospective task-height change, not a hidden per-square calibration or a runtime offset.',
        nominal_A8_XYZ_consistency_resolved=False, physical_grasp_success_claimed=False,
        orientation_policy='full wrist rotation identical to savedA8_take frame',
        desired_wrist_rotation_in_cad=desired.tolist(), shared_bounds_unchanged=True,
        bounds_encoder_rad=dict(zip(m.names, limits.tolist())),
        numerical_endpoint_found=success, best=best, predicted_endpoint_board_mm=board.tolist(),
        endpoint_body_and_board_check_passed=hard_endpoint_pass, endpoint_failure=endpoint_failure,
        minimum_common_scene_clearance_mm=float(np.min(clearance)*1000),
        active_lower=[m.names[i] for i in range(m.n) if q[i]-limits[i, 0] < 1e-6],
        active_upper=[m.names[i] for i in range(m.n) if limits[i, 1]-q[i] < 1e-6],
        direct_vertical_continuation=dict(start='saved reference endpoint, not current robot pose',
            reached_diagnostic_height=reached, solver_steps=continuation,
            accepted_edges_checked=True, maximum_sampled_full_orientation_error_deg=max_edge_rotation_error,
            complete_Home_Clearance_approach_planned=False),
        full_endpoint_multistart_trials=trials,
        source_sha256=hashes, runtime_geometry_conformance={key: raw_cfg[key] == read(ROOT/'scripts/data/left_arm_board_workspace_candidate.json')[key]
            for key in ('tcp_in_wrist_frame_m', 'board_to_cad_rotation', 'board_to_cad_translation_m', 'joint_limits_encoder_rad', 'gripper_vertex_envelope')},
        elapsed_s=time.monotonic()-begin,
        limitations=['Endpoint and optional local vertical branch only; no fresh actual robot start or complete executable approach path.',
            'Common model/CAD clearance and fixed orientation, not independently measured physical heights, contacts, pieces or cables.',
            'No physical motion authorization is inferred from this counterfactual.', 'Failure of finite seeds is not a global infeasibility proof.'])
    output = OUT/'lower25_endpoint_diagnostic.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps({k:result[k] for k in ('diagnostic_target_board_mm', 'numerical_endpoint_found', 'predicted_endpoint_board_mm',
        'minimum_common_scene_clearance_mm', 'active_lower', 'active_upper', 'elapsed_s')} | {'best':best, 'vertical_continuation_reached':reached}, ensure_ascii=True, indent=2))


if __name__ == '__main__':
    main()
