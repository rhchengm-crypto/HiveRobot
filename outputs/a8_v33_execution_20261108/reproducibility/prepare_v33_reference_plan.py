"""Local-only v33 reference-endpoint path reconstruction and verification.

No serial, network, motion entry point, old trajectory input, runtime model
write, prepared execution stamp, angle compensation, or gain modification.
The taught endpoint is a boundary condition; intermediate Cartesian IK and
the complete ordinary-Clearance transit are solved anew from frozen inputs.
"""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import collections
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import types
sys.dont_write_bytecode = True

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(Path(path).read_text(encoding='utf8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf8')


def main():
    began = time.monotonic()
    context_path = OUT/'preflight.json'
    context = read(context_path)
    context_hash = sha(context_path)
    snapshot = OUT/'source_snapshot'
    scripts = snapshot/'scripts'
    available, unavailable = {}, {}
    for name, expected in context['source_hashes'].items():
        source = ROOT/'scripts'/name
        if not source.is_file():
            unavailable[name] = expected
            continue
        assert sha(source) == expected, ('preflight source mismatch', name)
        available[name] = expected
    # Mathematical dependencies omitted from the prepared13 manifest remain
    # explicit additional frozen inputs; this does not claim a remote hash.
    extras = ('left_arm_rigid_body.py', 'left_arm_board_transit.py',
              'left_arm_board_home_handoff.py')
    frozen = dict(available)
    for name in extras:
        frozen.setdefault(name, sha(ROOT/'scripts'/name))
    for name, expected in frozen.items():
        source, target = ROOT/'scripts'/name, scripts/name
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            assert sha(target) == expected, ('frozen input already differs', name)
        else:
            target.write_bytes(source.read_bytes())
    snapshot.mkdir(parents=True, exist_ok=True)
    (snapshot/'preflight.json').write_bytes(context_path.read_bytes())
    approved_path = ROOT/'outputs/a8_saved_reference_20261108/saved_pose_audit.json'
    approved = read(approved_path)
    (snapshot/'saved_pose_audit.json').write_bytes(approved_path.read_bytes())
    sys.path.insert(0, str(scripts))
    import numpy as np
    import left_arm_board_grasp as b
    from left_arm_board_kinematics import tcp_pose, solve_ik, rotation_error
    from left_arm_trajectory_support import gravity_envelope
    assert b.PLANNER_BUILD == 'seven-axis-board-grasp-v33-quasistatic-support'
    prepared_args = types.SimpleNamespace(model=scripts/'data/left_arm_general_dynamics.json',
        board_model=scripts/'data/left_arm_board_workspace_candidate.json')
    prepared_hashes = b.prepared_source_hashes(prepared_args)
    assert len(prepared_hashes) == 13
    assert prepared_hashes == context['prepared_source_hashes'], 'prepared13 differs from remote'
    m = b.RigidBodyModel.load(prepared_args.model)
    cfg = b.load_board_config(prepared_args.board_model, m)
    saved = approved['saved_move']['pose']
    assert context['saved_A8_pose'] == saved
    goal = np.array([saved[n] for n in m.names])
    bounds = np.array(cfg['joint_limits_encoder_rad'])
    assert np.all(goal >= bounds[:, 0]) and np.all(goal <= bounds[:, 1])
    fixed = read(scripts/'data/left_arm_v2_table_clearance.json')['joints']
    assert fixed == context['ordinary_clearance']
    start = dict(fixed)
    start['shoulder_rotate'] = float(context['feedback']['shoulder_rotate']['pos'])
    q0 = np.array([start[n] for n in m.names])
    assert np.all(q0 >= bounds[:, 0]) and np.all(q0 <= bounds[:, 1])
    required = cfg['required_clearance_entry']['reference_encoder_rad']
    assert all(start[n] == value for n, value in required.items())
    r, t = np.array(cfg['board_to_cad_rotation']), np.array(cfg['board_to_cad_translation_m'])
    tcp = np.array(cfg['tcp_in_wrist_frame_m'])
    endpoint = tcp_pose(m, goal, tcp)
    desired = endpoint[:3, :3]
    contact = (endpoint[:3, 3]-t)@r*1000
    fixed_target = np.array([11.798050592563328, 405.6006774366655, 84.62267445748284])
    np.testing.assert_allclose(contact, fixed_target, atol=1e-10, rtol=0.)
    scene = b.collision_scene(m, cfg)
    b.check_board_intersection(m, goal, cfg)
    height = max(float(cfg['default_clearance_mm']), float(contact[2])+50.)
    q, current, step = goal.copy(), float(contact[2]), 8.
    reverse = [(contact.copy(), goal.copy())]
    ik_calls = []
    while current < height-1e-8:
        z = min(height, current+step)
        point = np.array([contact[0], contact[1], z])
        answer = solve_ik(m, r@(point/1000)+t, q, tcp, bounds, desired,
            max_iterations=150, position_tolerance_m=.0005,
            rotation_tolerance_rad=.005, clearance_function=scene)
        nq = np.array([answer['encoder_rad'][n] for n in m.names])
        continuous = np.max(np.abs(nq-q)) <= math.radians(12)
        ik_calls.append(dict(target_board_mm=point.tolist(), success=answer['success'],
            iterations=answer['iterations'], position_error_mm=answer['position_error_mm'],
            rotation_error_deg=answer['rotation_error_deg'], continuity_passed=bool(continuous)))
        if not answer['success'] or not continuous:
            if step > 1.:
                step /= 2
                continue
            raise b.BoardPlanningError('new reverse Cartesian IK failed', dict(answer=answer, target_board_mm=point.tolist()))
        b.check_transit_edge(m, q, nq, cfg)
        reverse.append((point, nq))
        q, current, step = nq, z, min(8., step*2)
    print(json.dumps(dict(phase='fresh-reverse-IK-complete', samples=len(reverse), height_mm=height)), flush=True)
    route, blocked = b.find_entry_transit(m, q0, [(float(np.linalg.norm(q-q0)), q)], cfg)
    if route is None:
        raise b.BoardPlanningError('new Clearance-to-hover transit failed', dict(blocked_paths=blocked))
    print(json.dumps(dict(phase='fresh-transit-complete', geometric_route_edges=len(route))), flush=True)
    rows, previous = [], q0.copy()
    for stage, entry in route:
        count = max(1, int(np.ceil(np.max(np.abs(entry-previous))/math.radians(5))))
        initial = previous.copy()
        for i in range(1, count+1):
            value = initial+(entry-initial)*i/count
            pose = tcp_pose(m, value, tcp)
            rows.append(dict(stage=stage, encoder_rad=dict(zip(m.names, value.tolist())),
                target_cad_m=pose[:3, 3].tolist(), target_board_mm=((pose[:3, 3]-t)@r*1000).tolist(),
                position_error_mm=0., rotation_error_deg=None))
        previous = entry
    for point, value in reversed(reverse[:-1]):
        pose = tcp_pose(m, value, tcp)
        target = r@(point/1000)+t
        rows.append(dict(stage='contact', encoder_rad=dict(zip(m.names, value.tolist())),
            target_cad_m=target.tolist(), target_board_mm=point.tolist(),
            position_error_mm=float(np.linalg.norm(pose[:3, 3]-target)*1000),
            rotation_error_deg=float(np.degrees(np.linalg.norm(rotation_error(desired, pose[:3, :3]))))))
    plan = dict(ok=True, build=b.PLANNER_BUILD, motion_issued=False,
        stage='approach', piece='white_knight', start_rad=start,
        target_board_mm=contact.tolist(), waypoints=rows,
        selected_wrist_rotation_in_board=(r.T@desired).tolist(),
        approved_task_pose_reference='saved:A8_take', tool_orientation_constraint='full-wrist',
        endpoint_settle_enabled=True, precomputed_before_motion=True,
        target_source='FK-of-explicit-user-approved-endpoint; nominal-A8-XYZ remains unresolved',
        reference_endpoint_task=dict(name='A8_take', nominal_square='A8',
            physical_grasp_endpoint_user_accepted=True, original_nominal_XYZ_request_solved=False,
            target_source='FK of user-approved endpoint, not coordinate compensation'),
        path_generation=dict(intermediate_source='new reverse Cartesian IK and new bounded geometric transit',
            old_trajectory_read=False, endpoint_is_boundary_condition=True,
            full_descent_required=True, fixed_cartesian_compensation_mm=[0., 0., 0.],
            Kp_or_Kd_modified=False))
    assert rows[-1]['encoder_rad'] == saved
    plan['orientation_acceptance'] = b.check_planned_tool_orientation(m, cfg, plan)
    plan['gripper_clearance_acceptance'] = b.check_planned_gripper_clearance(m, cfg, plan)
    plan = b.candidate_approach_trial(plan, cfg)
    plan['planned_gravity_envelope'] = gravity_envelope(m, plan)
    assert all(math.isfinite(x) for pair in plan['planned_gravity_envelope']['gravity_encoder_Nm'].values() for x in pair)
    groups = b.execution_waypoints(plan)
    grouped = dict(plan, waypoints=groups)
    group_clearance = b.check_planned_gripper_clearance(m, cfg, grouped)
    group_orientation = b.check_planned_tool_orientation(m, cfg, grouped)
    previous = q0.copy()
    edge_count, max_fk_poe = 0, 0.
    for row in rows:
        value = np.array([row['encoder_rad'][n] for n in m.names])
        stage_cfg = b.transit_clearance_config(cfg) if row['stage'] in ('entry_clearance', 'entry_search', 'source_above') else cfg
        b.check_transit_edge(m, previous, value, stage_cfg)
        model_q = m.encoder_to_model(value)
        max_fk_poe = max(max_fk_poe, float(np.max(np.abs(m.fk(model_q)-m.fk_poe(model_q)))))
        previous = value
        edge_count += 1
    dense, lateral, angular, heights = 0, [], [], []
    task = [row for row in rows if row['stage'] == 'source_above'][-1:]+[row for row in rows if row['stage'] == 'contact']
    for first, second in zip(task, task[1:]):
        first_q = np.array([first['encoder_rad'][n] for n in m.names])
        second_q = np.array([second['encoder_rad'][n] for n in m.names])
        count = max(1, int(np.ceil(np.max(np.abs(second_q-first_q))/math.radians(.1))))
        for fraction in np.linspace(0., 1., count+1):
            pose = tcp_pose(m, first_q+fraction*(second_q-first_q), tcp)
            point = (pose[:3, 3]-t)@r*1000
            dense += 1
            lateral.append(float(np.linalg.norm(point[:2]-contact[:2])))
            angular.append(float(np.degrees(np.linalg.norm(rotation_error(desired, pose[:3, :3])))))
            heights.append(float(point[2]))
    assert max(lateral) <= .5 and max(angular) <= math.degrees(.005)
    assert all(bv <= av+1e-7 for av, bv in zip(heights, heights[1:]))
    assert sha(context_path) == context_hash
    assert all(sha(ROOT/'scripts'/name) == expected for name, expected in frozen.items())
    assert b.prepared_source_hashes(prepared_args) == prepared_hashes
    (OUT/'board_workspace_candidate.json').write_bytes(prepared_args.board_model.read_bytes())
    write(OUT/'forecast_clearance.json', start)
    write(OUT/'local_plan.json', plan)
    report = dict(schema=1, motion_issued=False, hardware_accessed=False, network_accessed=False,
        runtime_sources_or_models_modified=False, nominal_A8_XYZ_consistency_resolved=False,
        original_nominal_XYZ_request_solved=False, full_descent_included=True,
        request=dict(x_mm=contact[0], y_mm=contact[1], piece='white_knight', stage='approach', grip_height_mm=contact[2]),
        forecast_clearance=start, forecast_rotate_source='fresh current Home feedback; ordinary fixed Clearance skips this joint',
        home_feedback=context['feedback'], preflight_sha256=context_hash,
        source_hashes_before=context['source_hashes'], prepared_source_hashes=prepared_hashes,
        prepared_source_count=len(prepared_hashes), prepared_sources_all_match_remote=True,
        matching_available_protected_source_count=len(available), protected_sources_absent_locally=unavailable,
        additional_local_only_frozen_dependencies={name:frozen[name] for name in extras if name not in available},
        snapshot_directory=str(snapshot.relative_to(ROOT)).replace('\\', '/'),
        frozen_input_hashes=frozen, all_frozen_sources_unchanged_at_finish=True,
        saved_endpoint_source_sha256=sha(approved_path), saved_endpoint_identical=True,
        endpoint_delta_deg=dict.fromkeys(m.names, 0.), default_hover_height_mm=height,
        intermediate_old_trajectory_replay=False, freshly_computed_ik_calls=ik_calls,
        fresh_transit_edges=len(route), waypoints=len(rows), execution_groups=len(groups),
        stages=dict(collections.Counter(row['stage'] for row in rows)),
        raw_edges_checked=edge_count, grouped_clearance=group_clearance, grouped_orientation=group_orientation,
        orientation=plan['orientation_acceptance'], gripper_clearance=plan['gripper_clearance_acceptance'],
        dense_task_audit=dict(samples=dense, max_lateral_error_mm=max(lateral), max_rotation_error_deg=max(angular),
            min_height_mm=min(heights), max_height_mm=max(heights), descent_monotonic=True),
        fk_vs_poe_max_matrix_difference=max_fk_poe,
        planned_gravity_envelope=plan['planned_gravity_envelope'],
        gravity_envelope_independent_recomputation_equal=gravity_envelope(m, plan)==plan['planned_gravity_envelope'],
        local_plan_sha256=sha(OUT/'local_plan.json'), board_model_sha256=sha(OUT/'board_workspace_candidate.json'),
        prepared_execution_stamp_present=False, execution_authorization_owned_by_root=True,
        remaining_runtime_checks=['fresh actual ordinary-Clearance entry and entry-load anchor',
            'MIT-range support_limits with actual entry load before approach', 'root remote byte/source verification and fresh prepared stamp'],
        physical_clearance_or_tracking_success_certified=False, elapsed_s=time.monotonic()-began)
    write(OUT/'preparation.json', report)
    print(json.dumps({k:report[k] for k in ('waypoints', 'execution_groups', 'stages', 'forecast_clearance',
        'prepared_source_count', 'prepared_sources_all_match_remote', 'orientation', 'gripper_clearance',
        'dense_task_audit', 'local_plan_sha256', 'elapsed_s')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
