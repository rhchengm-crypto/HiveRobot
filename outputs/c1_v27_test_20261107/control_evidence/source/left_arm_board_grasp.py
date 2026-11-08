"""Board XY -> oriented Cartesian path -> seven-axis model-controlled motion.

Default is a no-motion preview. Above/touch tests never command the claw.
The supplied board model is explicitly nominal, not metrology calibration.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import sys
import time
from collections import deque

import numpy as np

from left_arm_board_kinematics import (board_point_to_cad, rotation, rotation_error,
                                      solve_ik, tcp_pose)
from left_arm_rigid_body import RigidBodyModel, axis_rotation, vector
from left_arm_board_collision import collision_scene, board_prism_clearances
from left_arm_fault_support import FaultSupport, BoardWatchdog, MotionStop, segment_timing

HERE = Path(__file__).resolve().parent
PLANNER_BUILD = 'seven-axis-board-grasp-v27-bounded-pose-exact-box'
BOARD_EXECUTION_SUSPENDED = False
SUPPORTED_EXECUTION_STAGES = ('above', 'touch', 'approach', 'cartesian-check')
BOARD_EXECUTION_SUSPENSION_REASON = (
    'Board Point Grasp execution is suspended after an unintended descent during '
    'watchdog shutdown. Preview only; loaded fault-stop support is not validated.')


class BoardPlanningError(ValueError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


class PlanningBudgetExceeded(RuntimeError):
    def __init__(self):
        super().__init__('common planner time budget exhausted; physical unreachability is not established')
        self.diagnostics = {'code': 'planning_budget_exhausted', 'physical_unreachable_proven': False}


def check_planning_budget(cfg):
    if time.monotonic() > cfg.get('_planning_deadline', float('inf')):
        raise PlanningBudgetExceeded()


def load_board_config(path, model):
    cfg = json.loads(Path(path).read_text(encoding='utf8'))
    if cfg.get('schema') != 1 or cfg['joint_order'] != list(model.names):
        raise ValueError('board model schema/joint order mismatch')
    rotation(cfg['board_to_cad_rotation'])
    vector(cfg['board_to_cad_translation_m'], 3)
    vector(cfg['tcp_in_wrist_frame_m'], 3)
    limits = np.asarray(cfg['joint_limits_encoder_rad'], dtype=float)
    if limits.shape != (model.n, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
        raise ValueError('invalid operating bounds')
    collision_scene(model, cfg)  # Validate an explicitly enabled body scene.
    if cfg.get('continuous_grasp_search', {}).get('enabled'):
        from left_arm_board_task_pose import task_settings
        task_settings(cfg)
    return cfg


def solve_board_ik(model, cfg, *args, **kwargs):
    """One constraint source for initial, Cartesian and live-feedback IK."""
    check_planning_budget(cfg)
    scene = collision_scene(model, cfg)
    if scene is not None:
        kwargs['clearance_function'] = scene
        kwargs.setdefault('max_iterations', 100)
    if cfg.get('tool_orientation_constraint') == 'axis-only':
        kwargs['tool_axis_local'] = cfg['tool_axis_in_wrist_frame']
    return solve_ik(model, *args, **kwargs)


def validate_request(x_mm, y_mm, piece, stage, height_mm, config):
    if stage not in ('above', 'touch', 'approach', 'cartesian-check', 'grasp'):
        raise ValueError('stage must be above, touch, approach or grasp')
    profile = config['profiles'].get(piece)
    if profile is None:
        raise ValueError('piece needs a grasp-height/orientation profile; available: '+','.join(config['profiles']))
    xyz = [float(x_mm), float(y_mm), float(profile['grip_height_mm'] if height_mm is None else height_mm)]
    if not np.isfinite(xyz).all() or not 0 <= xyz[0] <= 440 or not 0 <= xyz[1] <= 440 or not 10 <= xyz[2] <= 150:
        raise ValueError('XY must be in [0,440]mm and grip height in [10,150]mm')
    return np.array(xyz), profile


def check_board_intersection(model, encoder, cfg):
    """Conservative link-axis capsules against the finite board, not other pieces."""
    scene = collision_scene(model, cfg)
    if scene is not None:
        gaps = scene(encoder)
        if np.any(gaps < 0):
            rows = scene.evaluate(encoder, details=True)
            worst = rows[int(np.argmin(gaps))]
            raise BoardPlanningError('arm capsule intersects configured world clearance envelope', {
                'code': worst.get('code', 'body_collision'), 'worst_pair': worst,
                'scene_registration_status': scene.spec.get('registration_status'),
                'physical_validation_complete': scene.spec.get('physically_validated', False)})
    if cfg.get('workspace_planning_policy') == 'shared-body-constrained-IK':
        gaps = board_prism_clearances(model, cfg, encoder)
        if np.any(gaps < 0):
            row = board_prism_clearances(model, cfg, encoder, details=True)[int(np.argmin(gaps))]
            raise BoardPlanningError('whole link/tool segment intersects finite board envelope', row)
        return
    frames = model.fk(model.encoder_to_model(encoder))
    points = frames[:, :3, 3]
    r = rotation(cfg['board_to_cad_rotation'])
    t = np.asarray(cfg['board_to_cad_translation_m'])
    fractions = np.linspace(0, 1, 13)
    samples = ((1-fractions[None, :, None])*points[:-1, None, :]
               +fractions[None, :, None]*points[1:, None, :]-t)@r
    blocked = ((samples[:, :, 0] >= -.03) & (samples[:, :, 0] <= .47)
               & (samples[:, :, 1] >= -.03) & (samples[:, :, 1] <= .47)
               & (samples[:, :, 2] < .03))
    if np.any(blocked):
        index, sample = np.argwhere(blocked)[0]
        p = samples[index, sample]
        raise BoardPlanningError('arm link envelope intersects board plane', {
            'code': 'link_board_intersection', 'link_from': model.names[index],
            'link_to': model.names[index+1], 'link_fraction': float(fractions[sample]),
            'board_point_mm': (p*1000).tolist(), 'minimum_height_mm': 30.})
    # End tool centreline; fingers are outside this simplified capsule model.
    end = tcp_pose(model, encoder, cfg['tcp_in_wrist_frame_m'])[:3, 3]
    fractions = np.linspace(0, 1, 15)
    samples = ((1-fractions[:, None])*points[-1]+fractions[:, None]*end-t)@r
    blocked = ((samples[:, 0] >= -.02) & (samples[:, 0] <= .46)
               & (samples[:, 1] >= -.02) & (samples[:, 1] <= .46) & (samples[:, 2] < .005))
    if np.any(blocked):
        index = np.flatnonzero(blocked)[0]
        raise BoardPlanningError('tool centreline intersects board plane', {
            'code': 'tool_board_intersection', 'tool_fraction': float(fractions[index]),
            'board_point_mm': (samples[index]*1000).tolist(), 'minimum_height_mm': 5.})


def check_transit_edge(model, start, goal, cfg, keepalive=None):
    """Check the same joint interpolation used by the continuous controller."""
    if cfg.get('workspace_planning_policy') == 'shared-body-constrained-IK':
        scene = collision_scene(model, cfg)
        # Triangle-inequality radius from each axis to the TCP bounds motion
        # of every checked link-axis endpoint as well. Encoder signs are +/-1.
        lengths = np.linalg.norm(np.diff(model.points, axis=0), axis=1)
        radii = np.r_[np.cumsum(lengths[::-1])[::-1], 0.]+np.linalg.norm(cfg['tcp_in_wrist_frame_m'])
        def clearance(q):
            if keepalive is not None:keepalive()
            check_planning_budget(cfg)
            values = board_prism_clearances(model, cfg, q)
            if scene is not None:
                values = np.r_[values, scene(q)]
            if np.min(values) < 0:
                check_board_intersection(model, q, cfg)
            if keepalive is not None:keepalive()
            return float(np.min(values))
        clearance(start)
        clearance(goal)
        intervals = [(np.asarray(start), np.asarray(goal), 0)]
        while intervals:
            a, b, depth = intervals.pop()
            midpoint = (a+b)/2
            gap = clearance(midpoint)
            movement_bound = float(np.abs(b-a)@radii)/2
            # Box-plane distance and its segment minimum are 1-Lipschitz.
            # This bound covers the COMPLETE angular interval, so a different
            # waypoint partition cannot hide a collision between grid samples.
            if gap >= movement_bound+1e-10:
                continue
            if movement_bound < 1e-10 and gap >= 0:
                continue
            if depth >= 16:
                raise BoardPlanningError('swept interval clearance not established', {
                    'code': 'swept_clearance_not_certified', 'depth': depth,
                    'clearance_lower_bound_mm': gap*1000,
                    'point_movement_upper_bound_mm': movement_bound*1000,
                    'physical_collision_proven': False})
            intervals.append((a, midpoint, depth+1))
            intervals.append((midpoint, b, depth+1))
        return
    spacing = .5 if collision_scene(model, cfg) is not None else 2.
    count = max(1, int(np.ceil(np.max(np.abs(goal-start))/np.radians(spacing))))
    for alpha in np.linspace(0, 1, count+1):
        if keepalive is not None:keepalive()
        check_planning_budget(cfg)
        try:
            check_board_intersection(model, start+(goal-start)*alpha, cfg)
        except BoardPlanningError as exc:
            raise BoardPlanningError(str(exc), {'transit_fraction': float(alpha), **exc.diagnostics}) from exc


def plan_direction_check(model, cfg, start, joint, step_deg):
    """Isolate one encoder axis to measure its physical board-frame response.

    This is a calibration experiment, never a Cartesian grasp solution. The
    other six targets are the fresh measured positions, not saved poses.
    """
    if joint not in ('shoulder_side', 'shoulder_rotate', 'wrist_side'):
        raise ValueError('direction check requires one of the three lateral axes')
    if not math.isfinite(step_deg) or not 0 < abs(step_deg) <= 3.:
        raise ValueError('direction check step must be nonzero and at most 3 degrees')
    initial = np.array([start[n] for n in model.names], dtype=float)
    if not np.isfinite(initial).all():
        raise ValueError('direction check requires finite measured positions')
    goal = initial.copy()
    goal[model.names.index(joint)] += math.radians(step_deg)
    limits = np.array(cfg['joint_limits_encoder_rad'])
    if np.any(goal < limits[:, 0]) or np.any(goal > limits[:, 1]):
        raise ValueError('direction check exceeds the current operating envelope')
    check_transit_edge(model, initial, goal, cfg)
    r = np.array(cfg['board_to_cad_rotation'])
    t = np.array(cfg['board_to_cad_translation_m'])
    points = [(tcp_pose(model, q, cfg['tcp_in_wrist_frame_m'])[:3, 3]-t) @ r*1000
              for q in (initial, goal)]
    if abs(step_deg) > 1. and min(p[2] for p in points) < 100.:
        raise ValueError('direction checks above 1 degree require at least 100 mm modeled height')
    return dict(ok=True, build=PLANNER_BUILD, motion_issued=False,
                stage='cartesian-check', target_source='isolated-axis-calibration',
                direction_check=dict(joint=joint, encoder_step_deg=step_deg,
                    predicted_tcp_delta_board_mm=(points[1]-points[0]).tolist(),
                    physical_response_verified=False),
                start_rad=dict(start), target_board_mm=points[1].tolist(),
                waypoints=[dict(stage='direction_check', target_board_mm=points[1].tolist(),
                    encoder_rad=dict(zip(model.names, goal.tolist())))])


def lower_elbow_goal(model, cfg, goal, target_position, desired_rotation):
    """Lower the elbow only in the nullspace of the configured pose task."""
    if not cfg.get('lower_elbow_nullspace', False):
        return goal
    bounds=np.asarray(cfg['joint_limits_encoder_rad'])
    up=np.asarray(cfg['board_to_cad_rotation'])[:,2]
    tcp=np.asarray(cfg['tcp_in_wrist_frame_m'])
    local_axis=np.asarray(cfg['tool_axis_in_wrist_frame'])
    elbow=model.names.index('elbow')
    current=np.asarray(goal).copy()
    deadline=min(time.monotonic()+2.,cfg.get('_planning_deadline',float('inf'))-5.)
    for _ in range(12):
        if time.monotonic()>deadline:break
        q=model.encoder_to_model(current)
        frames=model.fk(q)
        jv,jw=model.point_jacobian(q,model.n-1,tcp)
        jv*=model.signs;jw*=model.signs
        axis=frames[-1,:3,:3]@local_axis
        ja=np.cross(jw.T,axis).T
        angular_task=ja if cfg.get('tool_orientation_constraint')=='axis-only' else jw
        task=np.vstack([jv,.15*angular_task])
        elbow_jv,_=model.point_jacobian(q,elbow)
        gradient=(up@elbow_jv)*model.signs
        direction=-(np.eye(model.n)-np.linalg.pinv(task,rcond=1e-6)@task)@gradient
        scale=np.max(np.abs(direction))
        if scale<1e-7:break
        direction*=.05/scale
        old_height=float(up@frames[elbow,:3,3])
        accepted=False
        for fraction in (1.,.5,.25):
            if time.monotonic()>deadline:break
            seed=np.clip(current+fraction*direction,bounds[:,0],bounds[:,1])
            try:
                answer=solve_board_ik(model,cfg,target_position,seed,tcp,bounds,desired_rotation,
                    position_tolerance_m=.0005,rotation_tolerance_rad=.005,max_iterations=20)
            except PlanningBudgetExceeded:
                return current
            if not answer['success']:continue
            trial=np.array([answer['encoder_rad'][n] for n in model.names])
            height=float(up@model.fk(model.encoder_to_model(trial))[elbow,:3,3])
            if height<old_height-.0001:
                current=trial;accepted=True;break
        if not accepted:break
    return current


def find_entry_transit(model, start, candidates, cfg):
    """Try direct edges first, then checked raised reference waypoints.

    A reference is a configuration-space waypoint, not a saved grasp action.
    Both legs must pass all existing board checks and operating bounds.
    """
    blocked = []
    ordered = sorted(candidates, key=lambda pair: pair[0])
    if cfg.get('lower_elbow_nullspace'):
        up=np.asarray(cfg['board_to_cad_rotation'])[:,2]
        elbow=model.names.index('elbow')
        ordered=sorted(candidates,key=lambda pair:(float(up@model.fk(
            model.encoder_to_model(pair[1]))[elbow,:3,3]),pair[0]))
    if not ordered:
        return None, blocked
    for _, goal in ordered:
        try:
            check_transit_edge(model, start, goal, cfg)
            return [('source_above', goal)], blocked
        except BoardPlanningError as exc:
            blocked.append({'route': 'direct', **exc.diagnostics})
    bounds = np.asarray(cfg['joint_limits_encoder_rad'])
    for reference in cfg.get('transit_references', []):
        via = vector([reference['encoder_rad'][n] for n in model.names], model.n)
        for name in reference.get('preserve_start_joints', []):
            via[model.names.index(name)] = start[model.names.index(name)]
        if np.any(via < bounds[:, 0]) or np.any(via > bounds[:, 1]):
            blocked.append({'route': reference['name'], 'code': 'transit_reference_outside_operating_bounds'})
            continue
        try:
            check_transit_edge(model, start, via, cfg)
        except BoardPlanningError as exc:
            blocked.append({'route': reference['name'], 'leg': 'start_to_reference', **exc.diagnostics})
            continue
        for _, goal in ordered:
            try:
                check_transit_edge(model, via, goal, cfg)
                return [('entry_clearance', via), ('source_above', goal)], blocked
            except BoardPlanningError as exc:
                blocked.append({'route': reference['name'], 'leg': 'reference_to_goal', **exc.diagnostics})
    if cfg.get('workspace_planning_policy') == 'shared-body-constrained-IK':
        from left_arm_board_transit import connect_path
        def edge_is_clear(a, b):
            try:
                check_transit_edge(model, a, b, cfg)
                return True
            except BoardPlanningError:
                return False
        for _, goal in ordered[:3]:
            path = connect_path(start, goal, bounds, edge_is_clear)
            if path is not None:
                return [('entry_search', q) for q in path[1:-1]]+[('source_above', path[-1])], blocked
        blocked.append({'route': 'bounded-geometric-search', 'code': 'transit_search_budget_exhausted',
                        'physical_unreachable_proven': False})
    return None, blocked


def _plan_fixed_orientation(model, cfg, start, x_mm, y_mm, piece='white_bishop', stage='above', height_mm=None):
    contact, profile = validate_request(x_mm, y_mm, piece, stage, height_mm, cfg)
    q = vector([start[n] for n in model.names], model.n, 'start')
    limits = np.asarray(cfg['joint_limits_encoder_rad'])
    if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
        raise BoardPlanningError('current pose outside recorded operating envelope', {
            'code': 'start_outside_operating_bounds',
            'joints': {n: {'position_rad': float(q[i]), 'bounds_rad': limits[i].tolist()}
                       for i, n in enumerate(model.names) if not limits[i, 0] <= q[i] <= limits[i, 1]}})
    r = rotation(cfg['board_to_cad_rotation'])
    t = np.asarray(cfg['board_to_cad_translation_m'])
    tcp = np.asarray(cfg['tcp_in_wrist_frame_m'])
    clearance = max(float(cfg['default_test_height_mm'] if stage == 'above' else cfg['default_clearance_mm']),
                    float(contact[2])+50)
    desired_r = r @ rotation(profile['wrist_rotation_in_board'])
    # Free transit above the board does not require a fixed grasp orientation.
    # Constraining the clearance entry to a vertical Cartesian line can make
    # a reachable task fail at a singularity. Solve the above pose first, then
    # check the actual joint-space transit at <=2deg intervals.
    above = r @ (np.array([*contact[:2], clearance])/1000) + t
    seed_poses = [q, np.array([profile['seed_encoder_rad'][n] for n in model.names])]
    seed_poses += [np.array([pose[n] for n in model.names]) for pose in cfg['seeds'].values()]
    if cfg.get('workspace_planning_policy') == 'shared-body-constrained-IK':
        # Identical seed bank for every requested coordinate, never a square
        # lookup. Existing poses only initialize numerical optimization.
        seed_poses += [(limits[:, 0]+limits[:, 1])/2, model.reference.copy()]
    candidates = []
    ik_attempts = []
    for seed in seed_poses:
        if np.any(seed < limits[:, 0]) or np.any(seed > limits[:, 1]):
            continue
        answer = solve_board_ik(model, cfg, above, seed, tcp, limits, desired_r,
                          position_tolerance_m=.0005, rotation_tolerance_rad=.005)
        ik_attempts.append({key: answer[key] for key in
                           ('success', 'position_error_mm', 'rotation_error_deg', 'iterations')})
        if answer['success']:
            goal = np.array([answer['encoder_rad'][n] for n in model.names])
            if (cfg.get('workspace_planning_policy') != 'shared-body-constrained-IK'
                    or not any(np.max(np.abs(goal-other)) < .002 for _, other in candidates)):
                candidates.append((float(np.linalg.norm(goal-q)), goal))
    if cfg.get('lower_elbow_nullspace') and candidates:
        refined=[]
        for _,goal in sorted(candidates,key=lambda pair:pair[0])[:2]:
            improved=lower_elbow_goal(model,cfg,goal,above,desired_r)
            if np.max(np.abs(improved-goal))>1e-5:
                refined.append((float(np.linalg.norm(improved-q)),improved))
        candidates=refined+candidates
    route, blocked_paths = find_entry_transit(model, q, candidates, cfg)
    if (not candidates and stage not in ('touch', 'above')
            and cfg.get('workspace_planning_policy') == 'shared-body-constrained-IK'
            and not cfg.get('_entry_height_search_active')):
        import copy
        alternatives = []
        for height in (100., 180.):
            actual_height = max(height, float(contact[2])+50.)
            if abs(actual_height-clearance) < .1:
                continue
            alternate = copy.deepcopy(cfg)
            alternate['_entry_height_search_active'] = True
            alternate['default_clearance_mm'] = actual_height
            try:
                result = _plan_fixed_orientation(model, alternate, start, x_mm, y_mm, piece, stage, height_mm)
                result['entry_height_selection'] = dict(policy='common feasible above height search',
                                                        selected_height_mm=actual_height,
                                                        per_square_parameters=False)
                return result
            except ValueError as exc:
                alternatives.append(dict(height_mm=actual_height, message=str(exc),
                                         diagnostics=getattr(exc, 'diagnostics', {})))
        blocked_paths.append(dict(route='common-entry-height-search', alternatives=alternatives))
    if stage == 'touch':
        # Touch is a separate, vertical-only validation from the completed hover.
        # Never make an unrequested free joint-space transit before descending.
        current_tcp = tcp_pose(model, q, tcp)
        current_board = (current_tcp[:3, 3]-t)@r*1000
        xy_error = float(np.linalg.norm(current_board[:2]-contact[:2]))
        descent = float(current_board[2]-contact[2])
        if xy_error > 5. or not 1. <= descent <= 25.:
            raise BoardPlanningError('Touch requires matching hover XY and a 1..25mm descent', {
                'code': 'touch_start_mismatch', 'start_tcp_board_mm': current_board.tolist(),
                'requested_contact_board_mm': contact.tolist(), 'xy_error_mm': xy_error,
                'descent_mm': descent, 'max_descent_per_test_mm': 25.})
        # Keep the current wrist orientation while testing vertical approach.
        desired_r = current_tcp[:3, :3]
        route = []
    if route is None:
        code = 'above_transit_blocked' if candidates else 'above_ik_unsolved'
        raise BoardPlanningError('no reachable above pose with a board-plane-clear joint transit', {
            'code': code, 'above_target_cad_m': above.tolist(), 'ik_attempt_count': len(ik_attempts),
            'best_ik_attempts': sorted(ik_attempts, key=lambda a: a['position_error_mm'])[:3],
            'ik_solutions': len(candidates), 'blocked_path_count': len(blocked_paths),
            'blocked_paths': blocked_paths[:3]+blocked_paths[-3:]})
    records = []
    for entry_stage, entry in route:
        count = max(1, int(np.ceil(np.max(np.abs(entry-q))/np.radians(5))))
        entry_start = q.copy()
        for i in range(1, count+1):
            q = entry_start+(entry-entry_start)*i/count
            point = tcp_pose(model, q, tcp)[:3, 3]
            records.append({'stage': entry_stage, 'target_cad_m': point.tolist(),
                            'target_board_mm': ((point-t)@r*1000).tolist(),
                            'encoder_rad': dict(zip(model.names, q.tolist())),
                            'position_error_mm': 0., 'rotation_error_deg': None})
    stages = []
    if stage != 'above':
        stages.append(('contact', contact))
    if stage == 'grasp':
        stages.append(('lift', np.array([*contact[:2], clearance])))
    current = tcp_pose(model, q, tcp)[:3, 3]
    for label, board_end in stages:
        # The retract XY may be outside the board; only the target is a board request.
        end = r @ (board_end/1000) + t
        begin = current.copy()
        start_r = tcp_pose(model, q, tcp)[:3, :3]
        turn = rotation_error(desired_r, start_r)
        angle = np.linalg.norm(turn)
        count = max(1, int(np.ceil(np.linalg.norm(end-begin)/.008)), int(np.ceil(angle/.04)))
        for i in range(1, count+1):
            fraction = i/count
            xyz = begin+(end-begin)*fraction
            target_r = start_r if angle < 1e-10 else axis_rotation(turn/angle, angle*fraction) @ start_r
            answer = solve_board_ik(model, cfg, xyz, q, tcp, limits, target_r,
                              position_tolerance_m=.0005, rotation_tolerance_rad=.005)
            if not answer['success']:
                raise ValueError(f'IK failed at {label} sample {i}/{count}: {answer["position_error_mm"]:.2f}mm')
            next_q = np.array([answer['encoder_rad'][n] for n in model.names])
            if np.max(np.abs(next_q-q)) > np.radians(12):
                raise ValueError('IK discontinuity exceeds 12deg per sample')
            # Check the joint interpolation that the controller will actually run.
            if cfg.get('workspace_planning_policy') == 'shared-body-constrained-IK':
                check_transit_edge(model, q, next_q, cfg)
            else:
                for alpha in np.linspace(0, 1, 5):
                    check_board_intersection(model, q+(next_q-q)*alpha, cfg)
            records.append({'stage': label, 'target_cad_m': xyz.tolist(),
                            'target_board_mm': ((xyz-t)@r*1000).tolist(),
                            'encoder_rad': answer['encoder_rad'],
                            'position_error_mm': answer['position_error_mm'],
                            'rotation_error_deg': answer['rotation_error_deg']})
            q = next_q
        current = end
    return {'ok': True, 'build': PLANNER_BUILD, 'motion_issued': False, 'stage': stage, 'piece': piece,
            'start_rad': dict(start),
            'target_source': 'requested-board-xyz',
            'joint_target_source': 'geometric-IK; recorded poses are solver seeds or checked transit references only',
            'target_board_mm': contact.tolist(), 'waypoints': records,
            'endpoint_settle_enabled': stage != 'cartesian-check',
            'registration_status': cfg['registration_status'], 'fit_errors_mm': cfg['fit_errors_mm'],
            'tcp_datum': cfg['tcp_datum']['definition'], 'joint_bound_source': cfg['joint_bound_source'],
            'entry_route': [label for label, _ in route],
            'collision_scope': ('sampled link-axis envelopes vs finite board and configured body obstacles; '
                                'other pieces and remaining self collisions not modeled'
                                if collision_scene(model, cfg) is not None else
                                'sampled link-axis envelopes vs finite board; other pieces/self collision not modeled'),
            'body_collision_physically_validated': bool(cfg.get('body_collision', {}).get('physically_validated', False)),
            'tool_orientation_constraint': cfg.get('tool_orientation_constraint', 'full-wrist'),
            'workflow': 'checked-above-transit/contact/pressure-close/lift; above and touch omit claw commands'}


def posture_load_cost(model, cfg, start, rows):
    """Shared posture preference; nominal load estimates are not ratings."""
    weights = cfg.get('posture_cost_weights')
    if weights is None:
        return 0., {}
    up = np.asarray(cfg['board_to_cad_rotation'])[:,2]
    origin = np.asarray(cfg['board_to_cad_translation_m'])
    elbow = model.names.index('elbow')
    states = [np.array([start[n] for n in model.names])]
    states += [np.array([row['encoder_rad'][n] for n in model.names]) for row in rows]
    heights,loads = [], []
    for state in states:
        q = model.encoder_to_model(state)
        heights.append(float((model.fk(q)[elbow,:3,3]-origin)@up))
        tau = model.signs*model.gravity_torques(q)
        loads.append(float(tau@tau))
    lengths = np.array([np.linalg.norm(b-a) for a,b in zip(states,states[1:])])
    def mean(values):
        values = np.asarray(values)
        return float(lengths@((values[:-1]+values[1:])/2)/lengths.sum()) if lengths.sum()>1e-12 else float(values[-1])
    height_mean,load_mean = mean(heights),mean(loads)
    cost = float(weights['elbow_height_per_m'])*max(0.,height_mean)+float(weights['nominal_gravity_squared_per_Nm2'])*load_mean
    return cost, dict(elbow_height_mean_mm=height_mean*1000,elbow_height_peak_mm=max(heights)*1000,
        elbow_height_terminal_mm=heights[-1]*1000,nominal_gravity_squared_mean_Nm2=load_mean,
        added_cost=cost,physical_load_independently_verified=False)


def check_planned_tool_orientation(model, cfg, result):
    """Audit task endpoints; free transit need not already have grasp orientation."""
    if cfg.get('tool_orientation_constraint') != 'full-wrist':
        return None
    variable = float(cfg.get('continuous_grasp_search', {}).get('descent_axis_cone_deg', 0.))
    if variable:
        from left_arm_board_task_pose import audit_bounded_descent_orientation
        return audit_bounded_descent_orientation(model, cfg, result)
    desired=np.asarray(cfg['board_to_cad_rotation'])@np.asarray(result['selected_wrist_rotation_in_board'])
    above=[row for row in result['waypoints'] if row['stage']=='source_above']
    rows=above[-1:]+[row for row in result['waypoints'] if row['stage'] in ('contact','lift')]
    if not rows:
        return None
    errors=[]
    for row in rows:
        q=np.array([row['encoder_rad'][n] for n in model.names])
        actual=tcp_pose(model,q,cfg['tcp_in_wrist_frame_m'])[:3,:3]
        errors.append(float(np.linalg.norm(rotation_error(desired,actual))))
    audit=dict(constraint='full-gripper-frame',checked_task_samples=len(errors),
        max_error_deg=math.degrees(max(errors)),tolerance_deg=math.degrees(.005),
        free_transit_excluded=True,physical_pose_verified=False)
    if max(errors)>.005+1e-8:
        raise BoardPlanningError('planned complete gripper orientation violates task constraint',
                                 dict(code='full_tool_orientation_mismatch',**audit))
    return audit


def plan(model, cfg, start, x_mm, y_mm, piece='white_bishop', stage='above', height_mm=None):
    import copy
    validate_request(x_mm, y_mm, piece, stage, height_mm, cfg)
    entry = cfg.get('required_clearance_entry')
    if entry is not None and stage in ('above', 'approach', 'grasp'):
        errors = {n:math.degrees(float(start[n])-float(value))
                  for n,value in entry['reference_encoder_rad'].items()}
        if any(abs(v)>float(entry['tolerance_deg']) for v in errors.values()):
            raise BoardPlanningError('staged Clearance is mandatory before common board approach',
                {'code':'clearance_entry_required','errors_deg':errors,
                 'tolerance_deg':entry['tolerance_deg'],
                 'ordinary_clearance_skips':['shoulder_rotate'],
                 'direct_approach_not_permitted':True})
    if cfg.get('support_surface', {}).get('enabled'):
        # Home may hang below tabletop outside the finite chessboard. Such a
        # pose must first use the existing staged Clearance controller; a
        # direct common-planner shortcut from below the table is forbidden.
        try:
            check_board_intersection(model, np.array([start[n] for n in model.names]), cfg)
        except BoardPlanningError as exc:
            raise BoardPlanningError('common transit must start above the support surface; use staged Clearance first',
                {'code':'clearance_entry_required','cause':exc.diagnostics}) from exc
    if cfg.get('workspace_planning_policy') == 'shared-body-constrained-IK':
        cfg = copy.deepcopy(cfg)
        budget = float(cfg.get('planning_time_budget_s', 60.))
        if not math.isfinite(budget) or not 1 <= budget <= 600:
            raise ValueError('common planner budget must be in 1..600 seconds')
        cfg['_planning_deadline'] = time.monotonic()+budget
    if stage == 'cartesian-check':
        return plan_cartesian_check(model, cfg, start, x_mm, y_mm, piece, height_mm)
    if stage == 'approach' and cfg.get('continuous_grasp_search', {}).get('enabled'):
        from left_arm_board_task_pose import plan_continuous_approach
        result = plan_continuous_approach(model, cfg, start, x_mm, y_mm, piece,
                                          height_mm, sys.modules[__name__])
        result['orientation_acceptance'] = check_planned_tool_orientation(model, cfg, result)
        return result
    hints = sorted(cfg.get('orientation_hints', []),
                   key=lambda h: np.linalg.norm(np.array(h['board_xy_mm'])-[x_mm, y_mm]))
    choices = hints + [cfg['profiles'][piece]]
    errors = []
    feasible = []
    for hint in choices:
        if time.monotonic() > cfg.get('_planning_deadline', float('inf')) and feasible:
            break
        candidate = copy.deepcopy(cfg)
        candidate['profiles'][piece]['wrist_rotation_in_board'] = hint['wrist_rotation_in_board']
        candidate['profiles'][piece]['seed_encoder_rad'] = hint['seed_encoder_rad']
        try:
            result = _plan_fixed_orientation(model, candidate, start, x_mm, y_mm, piece, stage, height_mm)
            if stage == 'touch':
                pose = tcp_pose(model, np.array([start[n] for n in model.names]),
                                np.array(cfg['tcp_in_wrist_frame_m']))
                result['selected_wrist_rotation_in_board'] = (
                    np.array(cfg['board_to_cad_rotation']).T@pose[:3, :3]).tolist()
                result['orientation_policy'] = 'current wrist orientation; vertical-only staged descent'
            else:
                result['selected_wrist_rotation_in_board'] = hint['wrist_rotation_in_board']
                result['orientation_policy'] = (
                    'taught approach axis; free tool yaw; geometric IK at requested XY'
                    if cfg.get('tool_orientation_constraint') == 'axis-only' else
                    'nearest feasible taught tool orientation; geometric IK at requested XY')
            orientation_audit=check_planned_tool_orientation(model,cfg,result)
            if orientation_audit is not None:
                result['orientation_acceptance']=orientation_audit
            if cfg.get('workspace_planning_policy') != 'shared-body-constrained-IK':
                return result
            # Evaluate complete descent/lift branches rather than accepting the
            # first nearest taught orientation. Clearance is posture-dependent.
            scene = collision_scene(model, candidate)
            q0 = np.array([start[n] for n in model.names])
            travel, minimum_gap = 0., float('inf')
            for row in result['waypoints']:
                q1 = np.array([row['encoder_rad'][n] for n in model.names])
                travel += float(np.linalg.norm(q1-q0))
                if scene is not None:
                    minimum_gap = min(minimum_gap, float(scene(q1).min()))
                q0 = q1
            # Prefer separation up to a useful 30mm, then shorter joint travel.
            score = travel-20.*min(.03, minimum_gap) if scene is not None else travel
            posture_cost,posture_audit = posture_load_cost(model,cfg,start,result['waypoints'])
            score += posture_cost
            result['posture_selection'] = dict(policy='common complete-path clearance and joint travel',
                score=score, joint_travel_rad=travel,
                waypoint_body_clearance_lower_bound_mm=(minimum_gap*1000 if scene is not None else None),
                per_square_parameters=False,posture_load_preference=posture_audit)
            feasible.append((score, result))
        except PlanningBudgetExceeded:
            if feasible:
                break
            raise
        except ValueError as exc:
            errors.append({'message': str(exc), 'details': getattr(exc, 'diagnostics', {}),
                           'orientation_reference_xy_mm': hint.get('board_xy_mm')})
    if feasible:
        result = min(feasible, key=lambda pair: pair[0])[1]
        result['posture_selection']['feasible_orientation_count'] = len(feasible)
        return result
    raise BoardPlanningError('no feasible complete board path', {
        'code': 'no_complete_board_path', 'start_rad': dict(start),
        'target_board_mm': [float(x_mm), float(y_mm),
                            float(cfg['profiles'][piece]['grip_height_mm'] if height_mm is None else height_mm)],
        'orientation_attempts': errors})


def plan_cartesian_check(model, cfg, start, x_mm, y_mm, piece, height_mm):
    """Small XYZ probe to compare physical directions, never a saved joint pose."""
    target, _ = validate_request(x_mm, y_mm, piece, 'cartesian-check', height_mm, cfg)
    q = np.array([start[n] for n in model.names])
    limits = np.asarray(cfg['joint_limits_encoder_rad'])
    if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
        raise ValueError('Cartesian check starts outside recorded operating bounds')
    r = rotation(cfg['board_to_cad_rotation'])
    t = np.array(cfg['board_to_cad_translation_m'])
    pose = tcp_pose(model, q, cfg['tcp_in_wrist_frame_m'])
    begin = (pose[:3, 3]-t)@r*1000
    delta = target-begin
    if not .5 <= np.linalg.norm(delta) <= 5.:
        raise ValueError('Cartesian direction check requires 0.5..5mm displacement')
    rows = []
    count = max(1, int(np.ceil(np.linalg.norm(delta))))
    for i in range(1, count+1):
        point = begin+delta*i/count
        answer = solve_board_ik(model, cfg, r@(point/1000)+t, q, cfg['tcp_in_wrist_frame_m'],
                          limits, pose[:3, :3], position_tolerance_m=.0001,
                          rotation_tolerance_rad=.002)
        if not answer['success']:
            raise BoardPlanningError('Cartesian direction check IK failed', answer)
        next_q = np.array([answer['encoder_rad'][n] for n in model.names])
        if np.max(np.abs(next_q-q)) > np.radians(3):
            raise ValueError('Cartesian check IK exceeds 3deg per sample')
        check_transit_edge(model, q, next_q, cfg)
        rows.append(dict(stage='cartesian_check', target_board_mm=point.tolist(),
                         encoder_rad=answer['encoder_rad'],
                         position_error_mm=answer['position_error_mm']))
        q = next_q
    return dict(ok=True, build=PLANNER_BUILD, stage='cartesian-check', piece=piece,
                motion_issued=False, start_rad=dict(start), start_tcp_board_mm=begin.tolist(),
                target_board_mm=target.tolist(), requested_delta_board_mm=delta.tolist(),
                target_source='requested-board-xyz', entry_route=[], waypoints=rows,
                orientation_policy='measured current wrist orientation',
                physical_direction_verified=False)


def execution_waypoints(trajectory):
    """Merge only exactly collinear transit samples, keeping the checked path.

    Planner samples are collision-check samples, not separate stop/restart
    requests. Contact/lift IK edges are retained individually.
    """
    rows = trajectory['waypoints']
    if not trajectory.get('start_rad'):
        return [dict(row, original_index=i, first_original_index=i) for i, row in enumerate(rows)]
    names = list(trajectory['start_rad'])
    previous = np.array([trajectory['start_rad'][n] for n in names])
    result = []
    index = 0
    while index < len(rows):
        end = index
        label = rows[index]['stage']
        if label in ('entry_clearance', 'entry_search', 'source_above'):
            # Extend only until the next actual turn. Do not treat an entire
            # RRT polyline as one line, or restart at each sampling point.
            while end+1 < len(rows) and rows[end+1]['stage'] == label:
                candidate = end+1
                target = np.array([rows[candidate]['encoder_rad'][n] for n in names])
                direction = target-previous
                denominator = float(direction@direction)
                fractions = []
                valid = True
                for row in rows[index:candidate+1]:
                    point = np.array([row['encoder_rad'][n] for n in names])
                    alpha = float((point-previous)@direction/denominator) if denominator > 1e-16 else 1.
                    if not np.allclose(point,previous+alpha*direction,atol=1e-9,rtol=0.):
                        valid = False
                        break
                    fractions.append(alpha)
                if (not valid or min(fractions)<-1e-9 or max(fractions)>1.+1e-9
                        or any(b<a for a,b in zip(fractions,fractions[1:]))):
                    break
                end = candidate
        result.append(dict(rows[end], original_index=end, first_original_index=index))
        previous = np.array([rows[end]['encoder_rad'][n] for n in names])
        index = end+1
    return result


def motion_watchdog(arm, supervisor, max_velocity, max_error_deg, context):
    return BoardWatchdog(arm, supervisor, max_velocity, max_error_deg,
        context=context, record_only=True)


def confirmed_encoder_oscillation(samples, row):
    """Detect repeated fast reversals with encoder travel, not a lone spike."""
    samples.append(row)
    while samples and row['monotonic_s']-samples[0]['monotonic_s']>.25:
        samples.popleft()
    if len(samples)<4 or row['monotonic_s']-samples[0]['monotonic_s']<.04:return None
    for name in row['feedback']:
        feedback=[s['feedback'][name] for s in samples]
        if any(v.get('pos') is None or v.get('vel') is None for v in feedback):continue
        speeds=[v['vel'] for v in feedback if abs(v['vel'])>.5]
        reversals=sum(a*b<0 for a,b in zip(speeds,speeds[1:]))
        travel=math.degrees(max(v['pos'] for v in feedback)-min(v['pos'] for v in feedback))
        references=[s['commands'][name]['q'] for s in samples]
        reference_travel=math.degrees(max(references)-min(references))
        if reversals>=2 and travel>.5 and reference_travel<=2.:
            return dict(joint=name,fast_velocity_reversals=reversals,
                encoder_span_deg=travel,reference_span_deg=reference_travel,
                peak_velocity_rad_s=max(abs(v['vel']) for v in feedback),
                window_seconds=row['monotonic_s']-samples[0]['monotonic_s'],
                feedback_source='MIT-reply-cache',individual_feedback_freshness_verified=False)
    return None


class SettleWatchdog(BoardWatchdog):
    """Abort endpoint retries on confirmed excess motion, not one spike."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.previous_excess = {}

    def __call__(self, reference):
        before = self.checks
        super().__call__(reference)
        if self.checks == before:
            return
        limit = max(.3, self.max_velocity)
        current = {}
        for name,row in self.support.snapshot.items():
            if abs(row['vel']) <= limit:
                continue
            current[name] = row['pos']
            if name in self.previous_excess:
                drift = abs(math.degrees(row['pos']-self.previous_excess[name]))
                if drift > .1:
                    diagnostic = {**self.context,'joint':name,'velocity_rad_s':row['vel'],
                        'confirmed_drift_deg':drift,'velocity_limit_rad_s':limit,
                        'retry_aborted':True}
                    self._log('board endpoint correction abort= ', diagnostic)
                    fault = MotionStop('endpoint correction confirmed excess motion',diagnostic)
                    fault.trace = list(self.samples)
                    raise fault
        self.previous_excess = current


def bridge_segment_feedforward(previous, positions, velocities, gains, baseline, limits, reference_starts=None):
    """Match total motor torque at the new origin; keep the model endpoint."""
    result = {}; audit = {}
    for name,pos in positions.items():
        old = previous[name]; vel = float(velocities[name]); pos = float(pos)
        kp = float(gains[name]['kp']); kd = float(gains[name]['kd'])
        row = baseline[name]
        end = float(row.get('end_tau',row.get('tau',0.))) if isinstance(row,dict) else float(row)
        total = float(old['kp'])*(float(old['q'])-pos)+float(old['kd'])*(float(old['dq'])-vel)+float(old['tau_ff'])
        origin = pos if reference_starts is None else float(reference_starts[name])
        start = total-kp*(origin-pos)+kd*vel
        if (origin==float(old['q']) and kp==float(old['kp']) and kd==float(old['kd'])
                and float(old['dq'])==0.):
            start=float(old['tau_ff'])  # keep the literal MIT fields, not just their ideal sum
        if (not all(math.isfinite(v) for v in (pos,vel,origin,kp,kd,end,total,start))
                or abs(start)>float(limits[name]) or abs(end)>float(limits[name])):
            raise ValueError('segment continuation exceeds finite MIT torque range for '+name)
        result[name] = dict(start_tau=start,end_tau=end,ramp_fraction=1.,ramp='smoothstep')
        audit[name] = dict(position_rad=pos,velocity_rad_s=vel,previous_total_tau=total,
            reference_start_rad=origin,new_start_tau_ff=start,
            new_total_tau=kp*(origin-pos)+start-kd*vel,modeled_end_tau=end,
            target_unchanged=True,gravity_anchor_unchanged=True)
    return result,audit


def motion_timing(delta, max_velocity):
    # Relax feedback acceptance without increasing reference speed/acceleration.
    result = segment_timing(delta, min(.2,max_velocity))
    result['feedback_velocity_limit_rad_s'] = max_velocity
    return result


def board_motion_gains(legacy, geometry, positions):
    """Bound shoulder swivel compliance using the current modeled TCP lever.

    This is a position-controller stiffness policy, not an angle/XYZ offset.
    The existing motor-local damping and other joint gains are retained.
    """
    gains = {n: dict(legacy.HOME_GAINS[n]) for n in positions}
    if geometry is None or 'shoulder_rotate' not in gains:
        return gains, None
    model, cfg = geometry
    q = model.encoder_to_model([positions[n] for n in model.names])
    jv, _ = model.point_jacobian(q, model.n-1, cfg['tcp_in_wrist_frame_m'])
    lever_sq = float(jv[:, model.names.index('shoulder_rotate')] @
                     jv[:, model.names.index('shoulder_rotate')])
    base = float(gains['shoulder_rotate']['kp'])
    # 300 N/m Cartesian stiffness, bounded by the existing shoulder swivel
    # nudge envelope, with a validation cap of Kp=36 to limit the
    # stiffness change. This cap is not a measured hardware torque limit.
    cap = max(base, min(36., float(getattr(legacy, 'NUDGE_GAINS', {}).get(
        'shoulder_rotate', {}).get('kp', 36.))))
    requested = 300.*lever_sq
    gains['shoulder_rotate']['kp'] = max(base, min(cap, requested))
    return gains, dict(joint='shoulder_rotate', tcp_lever_m=math.sqrt(lever_sq),
                      requested_cartesian_stiffness_N_per_m=300.,
                      kp_base=base, kp_requested=requested, kp_cap=cap,
                      kp_applied=gains['shoulder_rotate']['kp'],
                      kd_applied=gains['shoulder_rotate']['kd'],
                      target_offset_rad=0., physical_validation_complete=False)


def soft_posture_seed_tolerance(model, cfg):
    """Explicit, inward-only initial-guess projection for soft posture bounds."""
    values = cfg.get('ik_soft_posture_seed_tolerance_deg', {})
    if not isinstance(values, dict) or set(values)-{'shoulder_side', 'shoulder_rotate'}:
        raise ValueError('soft posture seed tolerance only supports the two shoulder posture axes')
    tolerance = np.array([float(values.get(n, 0.)) for n in model.names])
    if not np.isfinite(tolerance).all() or np.any(tolerance < 0) or np.any(tolerance > .1):
        raise ValueError('soft posture seed tolerance must be within 0..0.1deg')
    return np.radians(tolerance)


def feedback_contact_path(model, cfg, measured, target_board_mm):
    """Acquire XY at measured height, then descend with measured orientation.

    The above transit may finish with a load-dependent joint tracking error.
    Its nominal IK solution must not become the start of the descent. Neither
    saved angles nor a board-coordinate offset enters this reconstruction.
    """
    q = np.array([measured[n] for n in model.names], dtype=float)
    r = rotation(cfg['board_to_cad_rotation'])
    t = np.array(cfg['board_to_cad_translation_m'])
    pose = tcp_pose(model, q, cfg['tcp_in_wrist_frame_m'])
    start = (pose[:3, 3]-t) @ r*1000
    target = vector(target_board_mm, 3)
    if target[2] >= start[2]:
        raise ValueError('measured approach entry must be above the contact target')
    desired_r = pose[:3, :3]
    limits = np.asarray(cfg['joint_limits_encoder_rad'])
    seed_tolerance = soft_posture_seed_tolerance(model, cfg)
    rows = []
    previous = start.copy()
    # Explicitly separate lateral acquisition from descent; no height chase.
    align = np.array([target[0], target[1], start[2]])
    for label, end in [('entry_xy_alignment', align), ('contact', target)]:
        distance = np.linalg.norm(end-previous)
        if distance < .1:
            previous = end
            continue
        begin = previous.copy()
        for i in range(1, max(1, int(np.ceil(distance/4.)))+1):
            count = max(1, int(np.ceil(distance/4.)))
            point = begin+(end-begin)*i/count
            answer = solve_board_ik(model, cfg, r@(point/1000)+t, q,
                cfg['tcp_in_wrist_frame_m'], limits, desired_r,
                position_tolerance_m=.0005, rotation_tolerance_rad=.005,
                seed_projection_tolerance_rad=seed_tolerance)
            if not answer['success']:
                raise BoardPlanningError('measured entry Cartesian IK failed', {
                    'stage': label, 'target_board_mm': point.tolist(), **answer})
            next_q = np.array([answer['encoder_rad'][n] for n in model.names])
            if np.max(np.abs(next_q-q)) > np.radians(12):
                raise ValueError('measured entry IK discontinuity exceeds 12deg')
            check_transit_edge(model, q, next_q, cfg)
            rows.append(dict(stage=label, target_board_mm=point.tolist(),
                             encoder_rad=answer['encoder_rad'],
                             seed_projection_delta_deg=answer['seed_projection_delta_deg'],
                             position_error_mm=answer['position_error_mm'],
                             rotation_error_deg=answer['rotation_error_deg']))
            q = next_q
        previous = end
    return rows, {'start_measured_rad': dict(measured),
                  'start_tcp_board_mm': start.tolist(),
                  'requested_target_board_mm': target.tolist(),
                  'orientation_policy': 'measured wrist orientation at descent entry',
                  'wrist_rotation_in_cad': desired_r.tolist(),
                  'xy_alignment_at_measured_height': True,
                  'board_offset_applied_mm': [0., 0., 0.]}


def candidate_above_trial(trajectory, cfg):
    """Use the common complete-path solution, execute ONLY its above transit."""
    if (trajectory['stage'] != 'approach'
            or cfg.get('workspace_planning_policy') != 'shared-body-constrained-IK'):
        raise ValueError('candidate above validation requires the common approach planner')
    rows = []
    for row in trajectory['waypoints']:
        if row['stage'] == 'contact':
            break
        if row['stage'] not in ('entry_clearance', 'entry_search', 'source_above'):
            raise ValueError('unexpected stage before candidate above validation')
        rows.append(row)
    if not rows or rows[-1]['stage'] != 'source_above' or rows[-1]['target_board_mm'][2] < 90.:
        raise ValueError('candidate validation requires a checked above pose at least 90mm high')
    return dict(trajectory, stage='above', waypoints=rows,
        requested_contact_board_mm=trajectory['target_board_mm'],
        target_board_mm=rows[-1]['target_board_mm'],
        validation_trial={'above_only': True, 'descent_executed': False, 'claw_commanded': False,
                          'body_physically_validated': False,
                          'registration_status': cfg['body_collision']['registration_status']})


def candidate_approach_trial(trajectory, cfg):
    """Explicit full geometric approach/contact test, without closing or lift."""
    if (trajectory['stage'] != 'approach'
            or cfg.get('workspace_planning_policy') != 'shared-body-constrained-IK'):
        raise ValueError('candidate approach validation requires the common approach planner')
    rows = trajectory['waypoints']
    labels = [row['stage'] for row in rows]
    if (not rows or labels[-1] != 'contact' or 'source_above' not in labels
            or any(label not in {'entry_clearance','entry_search','source_above','contact'} for label in labels)
            or any(label != 'contact' for label in labels[labels.index('contact'):])):
        raise ValueError('candidate approach requires above transit followed by contact only')
    acceptance = trajectory.get('orientation_acceptance', {})
    bounded = float(cfg.get('continuous_grasp_search', {}).get('descent_axis_cone_deg', 0.))
    if bounded:
        if (acceptance.get('constraint') != 'bounded-axis-with-fixed-terminal-frame'
                or not acceptance.get('passed')):
            raise ValueError('candidate approach requires bounded descent orientation acceptance')
    elif (acceptance.get('constraint') != 'full-gripper-frame'
            or acceptance.get('max_error_deg', float('inf')) > acceptance.get('tolerance_deg', -1)):
        raise ValueError('candidate approach requires full-frame orientation acceptance')
    return dict(trajectory, validation_trial={'approach_contact':True,'above_only':False,
        'touch_only':False,'claw_commanded':False,'body_physically_validated':False})


def candidate_touch_trial(model, trajectory, cfg):
    """Explicit bounded continuation trial; does not certify the body model."""
    if cfg.get('workspace_planning_policy')!='shared-body-constrained-IK' or trajectory['stage']!='touch':
        raise ValueError('candidate touch validation requires common staged Touch')
    rows=trajectory['waypoints']
    if not rows or any(row['stage']!='contact' for row in rows):
        raise ValueError('candidate Touch must contain contact-path samples only')
    q=np.array([trajectory['start_rad'][n] for n in model.names])
    xyz=(tcp_pose(model,q,cfg['tcp_in_wrist_frame_m'])[:3,3]-np.asarray(cfg['board_to_cad_translation_m']))@np.asarray(cfg['board_to_cad_rotation'])*1000
    end=np.asarray(trajectory['target_board_mm']);descent=float(xyz[2]-end[2])
    if not 1.<=descent<=25. or np.linalg.norm(xyz[:2]-end[:2])>5.:
        raise ValueError('candidate Touch requires matched XY and 1..25mm descent')
    return dict(trajectory,validation_trial={'above_only':False,'touch_only':True,
        'requested_descent_mm':descent,'max_descent_mm':25.,'claw_commanded':False,
        'body_physically_validated':False})


class TrajectorySupportBaseline:
    """One entry load anchor, never recycle later PD torque as new gravity."""
    def __init__(self, model, positions, measured_tau, status, gains, limits, hardware_limits=None):
        self.model = model
        self.names = list(positions)
        self.initial_gravity = self.gravity(positions)
        self.anchor = {n:float(measured_tau[n]) for n in self.names}
        self.model_limits = {n:tuple(map(float, limits[n])) for n in self.names}
        self.limits = {}
        for n in self.names:
            low, high = self.model_limits[n]
            maximum = float(hardware_limits[n]) if hardware_limits is not None else None
            velocity = float(status[n]['vel'])
            if (not all(math.isfinite(v) for v in (self.anchor[n],velocity,low,high))
                    or (maximum is not None and (not math.isfinite(maximum) or maximum <= 0
                                                 or abs(self.anchor[n]) > maximum))
                    or (maximum is None and not low <= self.anchor[n] <= high)):
                raise RuntimeError('trajectory entry support exceeds valid torque range for '+n)
            # As in the existing fixed-pose takeover, identified model limits
            # are not the preceding controller's total load support. Extend
            # only to this single entry observation, never later PD feedback.
            bridge = self.anchor[n]+gains[n]['kd']*velocity
            if maximum is not None:
                bridge = min(maximum,max(-maximum,bridge))
                self.limits[n] = (max(-maximum,min(low,self.anchor[n],bridge)),
                                  min(maximum,max(high,self.anchor[n],bridge)))
            else:
                self.limits[n] = (low,high)

    def gravity(self, positions):
        q = self.model.encoder_to_model([positions[n] for n in self.model.names])
        tau = self.model.signs*self.model.gravity_torques(q)
        return dict(zip(self.model.names, map(float,tau)))

    def command(self, positions, status, gains):
        gravity = self.gravity(positions)
        result, audit = {}, {}
        for n in self.names:
            low,high = self.limits[n]
            raw = self.anchor[n]+gravity[n]-self.initial_gravity[n]
            baseline = min(high,max(low,raw))
            start = min(high,max(low,baseline+gains[n]['kd']*float(status[n]['vel'])))
            result[n] = dict(start_tau=start,end_tau=baseline,ramp_fraction=.25,ramp='smoothstep')
            audit[n] = dict(anchor_tau=self.anchor[n],gravity_change_tau=gravity[n]-self.initial_gravity[n],
                            raw_tau=raw,applied_tau=baseline,clamped=baseline!=raw,
                            model_tau_limits=self.model_limits[n],trajectory_tau_limits=self.limits[n])
        return result,audit


def execute(arm, legacy, trajectory, max_velocity=.2, max_error_deg=5., geometry=None,
            allow_body_validation_trial=False):
    """On a trajectory error, main latches the same-process support controller."""
    if geometry is not None:
        body = geometry[1].get('body_collision', {})
        trial = allow_body_validation_trial and (
            (trajectory.get('validation_trial', {}).get('above_only') and trajectory.get('stage')=='above')
            or (trajectory.get('validation_trial', {}).get('touch_only') and trajectory.get('stage')=='touch')
            or (trajectory.get('validation_trial', {}).get('approach_contact') and trajectory.get('stage')=='approach'))
        if body.get('enabled') and not body.get('physically_validated', False) and not trial:
            raise RuntimeError('body registration is an offline candidate; hardware clearance is not verified')
    if BOARD_EXECUTION_SUSPENDED:
        raise RuntimeError(BOARD_EXECUTION_SUSPENSION_REASON)
    if trajectory['stage'] not in SUPPORTED_EXECUTION_STAGES:
        raise RuntimeError('Only Above Only and staged Touch are released for hardware validation')
    from left_arm_v3_1_model_control import MIT_TAU_MAX
    from left_arm_v2_8_move_library import (claw_home_while_holding_arm,
                                           close_claw_while_holding_arm)
    names = list(trajectory['waypoints'][0]['encoder_rad'])
    arm.board_gravity_feedforward = True
    gains = {n: dict(legacy.HOME_GAINS[n]) for n in names}
    supervisor = FaultSupport(arm, names, gains)
    arm._board_fault_support = supervisor
    print('board watchdog policy= '+json.dumps({
        'build': PLANNER_BUILD, 'warning_velocity_rad_s': min(.2,max_velocity),
        'mode': 'record-only', 'velocity_error_automatic_stop': False,
        'tracking_error_automatic_stop': False,
        'endpoint_correction_confirmed_motion_abort': True,
        'confirmed_encoder_oscillation_abort': True,
        'warning_position_error_deg': max_error_deg,
        'reference_peak_velocity_limit_rad_s': .35*min(.2,max_velocity)}),flush=True)
    from left_arm_board_home_handoff import install_home_handoff, home_command_observed
    install_home_handoff(supervisor, arm, legacy)
    original_control = arm.ctrl.controlMIT
    motor_names = {id(arm.motors[n]): n for n in names}
    trace = None; trace_context={'stage':'initializing'}; frame_seen=set(); completed={}; trace_previous=None
    oscillation_samples=deque(maxlen=24)
    if all(callable(getattr(arm.motors[n],method,None)) for n in names
           for method in ('getPosition','getVelocity','getTorque')):
        from left_arm_motion_trace import MotionTrace
        trace=MotionTrace(HERE/'data/board_motion_traces'/('trace_'+str(time.time_ns())+'.jsonl'))
        arm._board_trace_recorder=trace;arm._board_trace_context=trace_context
        trace.submit({'event':'trace_started','build':PLANNER_BUILD,
            'names':names,'requested_target_board_mm':trajectory['target_board_mm'],
            'source_hashes':trajectory.get('runtime_source_hashes',{}),
            'prepared_execution':trajectory.get('prepared_execution'),
            'model_feedback_is_external_measurement':False})
        print('board motor trace= '+json.dumps({'path':str(trace.path),
            'writer':'bounded-asynchronous','sampling':'each complete command frame',
            'additional_CAN_queries':False}),flush=True)
    def observed_control(motor, kp, kd, q, dq, tau):
        nonlocal trace_previous
        result = original_control(motor, kp, kd, q, dq, tau)
        name = motor_names.get(id(motor))
        if name is not None:
            supervisor.last_commands[name] = dict(kp=float(kp), kd=float(kd), q=float(q),
                                                  dq=float(dq), tau_ff=float(tau))
            arm._board_last_command_time = time.monotonic()
            home_command_observed(supervisor, name, kp, kd)
            if trace is not None:
                completed[name]=time.monotonic();frame_seen.add(name)
                if len(frame_seen)==len(names):
                    row=None
                    try:
                        from left_arm_motion_trace import capture_frame
                        context=dict(trace_context)
                        if getattr(supervisor,'home_active',False):context['stage']='explicit_home_handoff'
                        row=capture_frame({n:arm.motors[n] for n in names},
                            supervisor.last_commands,context,completed,trace_previous)
                        row['queue_dropped_frames_before_submit']=trace.dropped
                        row['writer_error']=trace.error
                        trace_previous=row['monotonic_s'];trace.submit(row)
                    except Exception as exc:
                        trace.capture_errors+=1
                        trace.submit({'event':'capture_error','error':str(exc),'wall_s':time.time()})
                    finally:frame_seen.clear()
                    if (row is not None and geometry is not None
                            and geometry[1].get('workspace_planning_policy')=='shared-body-constrained-IK'
                            and trace_context.get('stage') not in ('fault_support','model_hold')
                            and not getattr(supervisor,'home_active',False)):
                        diagnostic=confirmed_encoder_oscillation(oscillation_samples,row)
                        if diagnostic is not None:
                            trace.submit({'event':'confirmed_encoder_oscillation','diagnostics':diagnostic,
                                          'wall_s':row['wall_s'],'context':dict(trace_context)})
                            supervisor.observe(row['feedback'])
                            raise MotionStop('confirmed encoder oscillation during board trajectory',diagnostic)
        return result
    arm.ctrl.controlMIT = observed_control
    claw_target = None
    support_anchor = None

    def entry_feedforward(positions, tau, status, gains):
        nonlocal support_anchor
        if geometry is not None and geometry[1].get('motion_feedforward_policy') == 'single-entry-gravity-anchor':
            if support_anchor is None:
                support_anchor = TrajectorySupportBaseline(geometry[0],positions,tau,status,gains,
                                                          arm.board_gravity_limits,MIT_TAU_MAX)
            reference_pose = ( {n:float(supervisor.last_commands[n]['q']) for n in names}
                if all(n in supervisor.last_commands for n in names) else positions )
            command,audit = support_anchor.command(reference_pose,status,gains)
            print('board support baseline= '+json.dumps({'policy':'single-entry-gravity-anchor',
                'later_measured_pd_torque_reused':False,
                'gravity_origin':'continued-command-reference',
                'reference_pose_rad':reference_pose,'joints':audit}),flush=True)
            return command
        return {n:{'start_tau':tau[n]+gains[n]['kd']*float(status[n]['vel']),
                   'end_tau':tau[n],'ramp_fraction':.25,'ramp':'smoothstep'} for n in names}

    def refresh_terminal_commands():
        if not all(n in supervisor.last_commands for n in names):return
        last=getattr(arm,'_board_last_command_time',None)
        if last is None or time.monotonic()-last<.02:return
        # Same owner and serial connection. Repeat the existing command law;
        # do not change targets or reconstruct support from measured torque.
        commands={n:dict(supervisor.last_commands[n]) for n in names}
        for n,row in commands.items():
            arm.ctrl.controlMIT(arm.motors[n],row['kp'],row['kd'],row['q'],row['dq'],row['tau_ff'])

    def trace_boundary(phase):
        trace_context['phase']=phase
        if trace is not None:
            trace.submit({'event':'segment_boundary','phase':phase,
                'monotonic_s':time.monotonic(),'wall_s':time.time(),'context':dict(trace_context)})

    def support():
        trace_boundary('status_read_start')
        refresh_terminal_commands()
        status = arm.read_status(names)
        refresh_terminal_commands()
        supervisor.observe(status)
        trace_boundary('status_read_complete')
        if any(not all(math.isfinite(float(s[k])) for k in ('pos', 'vel', 'tau'))
               or abs(float(s['tau'])) > MIT_TAU_MAX[n] for n, s in status.items()):
            raise RuntimeError('invalid board motion entry feedback')
        if any(abs(float(s['vel'])) > .1 for s in status.values()):
            print('board entry velocity audit= '+json.dumps({'mode':'record-only',
                  'protection_triggered':False,
                  'velocity_rad_s':{n:float(s['vel']) for n,s in status.items()}}),flush=True)
        return ({n: float(status[n]['pos']) for n in names},
                {n: float(status[n]['tau']) for n in names}, status)

    def continuation_kwargs(baseline):
        if (geometry is None or geometry[1].get('motion_feedforward_policy') != 'single-entry-gravity-anchor'
                or not all(n in supervisor.last_commands for n in names)):
            return {}
        previous = {n:dict(supervisor.last_commands[n]) for n in names}
        references={n:float(previous[n]['q']) for n in names}
        new_gains = {n:dict(gains[n]) for n in names}
        def initialize(starts):
            # positions() inside the primitive has just received replies. Read
            # velocity from that same cache, without another CAN query/sleep.
            velocity = {n:float(arm.motors[n].getVelocity()) for n in names}
            bridged,audit = bridge_segment_feedforward(previous,starts,velocity,new_gains,baseline,MIT_TAU_MAX,references)
            trace_boundary('interpolation_origin_ready')
            print('board segment torque continuation= '+json.dumps({
                'joints':audit,'feedback_source':'interpolation-start-MIT-cache',
                'additional_serial_reads':False,'gravity_anchor_replaced':False}),flush=True)
            return bridged
        return {'move_tau_ff_initializer':initialize,'move_reference_starts':references,
                'move_boundary_keepalive':refresh_terminal_commands}

    # Arm the support branch BEFORE issuing enable frames. No re-enable or
    # disable commands are issued by the support branch itself.
    supervisor.activate(arm.read_status(names))
    arm.enable(names)
    if trajectory['stage'] == 'grasp':
        positions, tau, _ = support()
        opened = claw_home_while_holding_arm(arm, legacy, 2., .4,
                    carry_hold={'hold_targets': positions, 'hold_gains': gains, 'hold_tau': tau},
                    workflow_label='Board grasp')
        claw_target = float(opened['target_rad'])
    closed = False
    rows = execution_waypoints(trajectory)
    print('board execution groups=', json.dumps({
        'planner_samples': len(trajectory['waypoints']), 'execution_groups': len(rows),
        'groups': [{'stage': row['stage'], 'first_original_index': row['first_original_index'],
                    'last_original_index': row['original_index']} for row in rows]}), flush=True)
    feedback_entry = None
    for group_index, row in enumerate(rows):
        index = row['original_index']
        trace_context.update(stage=row['stage'],waypoint_index=index,
                             target_board_mm=row['target_board_mm'],attempt=None)
        positions, tau, status = support()
        if (geometry is not None and not trajectory.get('precomputed_before_motion') and trajectory['stage'] == 'approach'
                and row['stage'] == 'contact' and feedback_entry is None):
            model, cfg = geometry
            replacement, feedback_entry = feedback_contact_path(
                model, cfg, positions, trajectory['target_board_mm'])
            replacement = [dict(v, original_index=index+i, first_original_index=index+i)
                           for i, v in enumerate(replacement)]
            rows[group_index:] = replacement
            row = rows[group_index]
            index = row['original_index']
            print('board feedback descent entry= '+json.dumps(feedback_entry), flush=True)
        # Each Cartesian edge is solved from fresh measured joints, rather than
        # continuing a nominal joint branch computed before the above transit.
        if feedback_entry is not None:
            model, cfg = geometry
            answer = solve_board_ik(model, cfg, board_point_to_cad(row['target_board_mm'],
                cfg['board_to_cad_rotation'], cfg['board_to_cad_translation_m']),
                np.array([positions[n] for n in model.names]), cfg['tcp_in_wrist_frame_m'],
                cfg['joint_limits_encoder_rad'], feedback_entry['wrist_rotation_in_cad'],
                position_tolerance_m=.0005, rotation_tolerance_rad=.005,
                seed_projection_tolerance_rad=soft_posture_seed_tolerance(model, cfg))
            if not answer['success']:
                raise BoardPlanningError('feedback-seeded Cartesian IK failed', answer)
            row['encoder_rad'] = answer['encoder_rad']
            if any(abs(v) > 0 for v in answer['seed_projection_delta_deg'].values()):
                print('board feedback seed projection= '+json.dumps({
                    'stage': row['stage'], 'waypoint_index': index,
                    'seed_projection_delta_deg': answer['seed_projection_delta_deg'],
                    'target_joint_bounds_unchanged': True}), flush=True)
        gains, stiffness_audit = board_motion_gains(legacy, geometry, positions)
        # A latched fault must retain the same gain used by the live motion.
        supervisor.gains = {n: {k: gains[n][k] for k in ('kp', 'kd')} for n in names}
        if stiffness_audit is not None:
            print('board shoulder swivel stiffness= '+json.dumps(stiffness_audit), flush=True)
        target = row['encoder_rad']
        if trajectory['stage'] == 'grasp' and row['stage'] == 'lift' and not closed:
            claw_target = close_claw_while_holding_arm(arm, positions, legacy,
                                                       arm_gains=gains, arm_tau=tau, require_contact=True)
            closed = True
            positions, tau, status = support()
        delta = max(abs(target[n]-positions[n]) for n in names)
        if row['stage'] not in ('entry_clearance', 'entry_search', 'source_above') and delta > math.radians(15):
            raise RuntimeError('actual waypoint delta exceeds 15deg')
        if geometry is not None:
            model, cfg = geometry
            trace_boundary('collision_check_start')
            check_transit_edge(model, np.array([positions[n] for n in model.names]),
                               np.array([target[n] for n in model.names]), cfg,
                               keepalive=refresh_terminal_commands)
            trace_boundary('collision_check_complete')
        timing = motion_timing(delta, max_velocity)
        trace_context.update(stage=row['stage'],waypoint_index=index,
                             target_board_mm=row['target_board_mm'],attempt=None)
        if row['stage'] == 'direction_check':
            timing.update(seconds=max(6., timing['seconds']))
            timing['reference_peak_velocity_rad_s'] = 1.875*delta/timing['seconds']
            timing['reference_peak_acceleration_rad_s2'] = (10./math.sqrt(3))*delta/timing['seconds']**2
        duration = timing['seconds']
        entry_tau = entry_feedforward(positions, tau, status, gains)
        holds = {} if claw_target is None else {'claw': claw_target}
        hold_gains = {} if claw_target is None else {'claw': {'kp': legacy.CLAW_KP_HOLD, 'kd': legacy.CLAW_KD_HOLD}}
        print('board waypoint=', json.dumps({'index': index, 'stage': row['stage'],
                                             'group_index': group_index,
                                             'first_original_index': row['first_original_index'],
                                             'target_board_mm': row['target_board_mm'],
                                             'timing': timing, 'entry_feedback': status}), flush=True)
        trace_boundary('trajectory_primitive_start')
        arm.move_targets_with_holds(target, seconds_per_step=duration, move_gains=gains,
            hold_targets=holds, hold_gains=hold_gains, hold_tau={'claw': 0.},
            move_tau_ff=entry_tau, fallback_kp=2., fallback_kd=.4, preload_seconds=.05,
            control_dt=.01, trajectory='smootherstep',
            watchdog=motion_watchdog(arm, supervisor, max_velocity, max_error_deg,
                                  {'waypoint_index': index, 'stage': row['stage'], 'timing': timing}),
            **continuation_kwargs(entry_tau))
    # Restore the former measured-load endpoint re-anchor on explicit operator
    # request. This tracks the last model-generated goal, not a D/XYZ offset.
    # Keep isolated-axis probes isolated; they are diagnostic motions only.
    target = rows[-1]['encoder_rad']
    settle_enabled = (trajectory['stage'] != 'cartesian-check'
                      and trajectory.get('endpoint_settle_enabled', True))
    settle_attempts = 0
    if settle_enabled:
        for attempt in range(3):
            trace_context.update(stage='endpoint_settle',attempt=attempt+1)
            positions, tau, status = support()
            error = max(abs(target[n]-positions[n]) for n in names)
            if error <= math.radians(.35):
                break
            excess = {n:float(s['vel']) for n,s in status.items() if abs(float(s['vel']))>max(.3,max_velocity)}
            if excess:
                raise MotionStop('endpoint retry refused while joints are moving',
                    {'stage':'endpoint_settle','attempt':attempt+1,'velocity_rad_s':excess,
                     'retry_aborted':True})
            if geometry is not None:
                model, cfg = geometry
                trace_boundary('collision_check_start')
                check_transit_edge(model, np.array([positions[n] for n in model.names]),
                                   np.array([target[n] for n in model.names]), cfg,
                                   keepalive=refresh_terminal_commands)
                trace_boundary('collision_check_complete')
            timing = motion_timing(error, max_velocity)
            entry_tau = entry_feedforward(positions, tau, status, gains)
            print('board endpoint settle= '+json.dumps({
                'attempt': attempt+1, 'max_error_deg': math.degrees(error),
                'entry_feedback': status, 'target_rad': target,
                'target_source': 'last geometric IK waypoint',
                'fixed_board_offset_mm': [0.,0.,0.],
                'timing': timing}), flush=True)
            trace_boundary('trajectory_primitive_start')
            arm.move_targets_with_holds(target, seconds_per_step=timing['seconds'], move_gains=gains,
                hold_targets=holds, hold_gains=hold_gains, hold_tau={'claw': 0.},
                move_tau_ff=entry_tau, fallback_kp=2., fallback_kd=.4,
                preload_seconds=.05, control_dt=.01, trajectory='smootherstep',
                watchdog=SettleWatchdog(arm, supervisor, max_velocity, max_error_deg,
                    context={'stage':'endpoint_settle','attempt':attempt+1},record_only=True),
                **continuation_kwargs(entry_tau))
            settle_attempts += 1
    final = arm.read_status(names)
    supervisor.observe(final)
    target = rows[-1]['encoder_rad']
    errors = {n: math.degrees(final[n]['pos']-target[n]) for n in names}
    endpoint = {'automatic_endpoint_correction': settle_enabled,
                'endpoint_settle_attempts': settle_attempts,
                'fixed_board_offset_mm': [0.,0.,0.],
                'model_hold_motion_gains': {n:{'kp':float(supervisor.last_commands.get(n,gains[n])['kp']),
                                               'kd':float(supervisor.last_commands.get(n,gains[n])['kd'])} for n in names},
                'nominal_target_rad': target,
                'measured_pose_rad': {n: final[n]['pos'] for n in names},
                'nominal_error_deg': errors,
                'max_nominal_error_deg': max(abs(v) for v in errors.values()),
                'nominal_joint_target_reached': all(abs(v) <= .35 for v in errors.values()),
                'physical_board_target_verified': False,
                'completion_scope': 'trajectory-and-endpoint-settle-complete; measured-pose-handoff'
                                    if settle_enabled else 'planned-trajectory-complete; measured-pose-handoff'}
    # Carry the actual final MIT command, not a reconstructed feedback torque.
    # Omit the extension in test/legacy callers without complete command capture.
    if all(n in supervisor.last_commands for n in names):
        for n in names:
            command = supervisor.last_commands[n]
            endpoint['model_hold_motion_gains'][n].update(
                {k: float(command[k]) for k in ('q', 'dq', 'tau_ff')})
    print('board endpoint report= '+json.dumps(endpoint), flush=True)
    return {'ok': True, 'motion_issued': True, 'claw_contact_required': trajectory['stage']=='grasp',
            'grasp_contact_confirmed': closed, 'endpoint': endpoint, 'final': final}



def reanchor_terminal_commands(commands, status, hardware_limits):
    """Change the PD origin without changing its torque law or stiffness."""
    rebased={};audit={}
    for name,row in commands.items():
        q=float(status[name]['pos']);v=float(status[name]['vel'])
        old={k:float(row[k]) for k in ('kp','kd','q','dq','tau_ff')}
        if not all(math.isfinite(value) for value in [q,v,*old.values()]):
            raise ValueError('non-finite terminal reanchor for '+name)
        tau=old['tau_ff']+old['kp']*(old['q']-q)
        if abs(tau)>float(hardware_limits[name]):
            raise ValueError('terminal reanchor exceeds MIT torque range for '+name)
        rebased[name]=dict(old,q=q,tau_ff=tau)
        audit[name]=dict(old_target_rad=old['q'],measured_target_rad=q,
            nominal_error_deg=math.degrees(q-old['q']),old_tau_ff=old['tau_ff'],new_tau_ff=tau,
            stiffness_unchanged=True,torque_law_unchanged=True)
    return rebased,audit


def same_session_model_hold(arm, legacy, result, hold_model, rigid_model, hold_model_path,
                            reanchor_terminal=False):
    """Continue support without process startup or serial close/open in between."""
    from left_arm_v3_1_model_control import build_parser, model_hold
    from left_arm_general_move import hold_command
    context=getattr(arm,'_board_trace_context',None)
    if isinstance(context,dict):context.update(stage='model_hold',attempt=None)
    inherited = result['endpoint']['model_hold_motion_gains']
    if not inherited or not all('q' in row for row in inherited.values()):
        raise RuntimeError('same-session hold requires complete terminal motor commands')
    # Refresh before model coverage or logging work. No enable frame, no new
    # target, and no dynamics transform is applied to these already final commands.
    last = getattr(arm,'_board_last_command_time',None)
    first = time.monotonic()
    for name,row in inherited.items():
        arm.ctrl.controlMIT(arm.motors[name],row['kp'],row['kd'],row['q'],row['dq'],row['tau_ff'])
    if reanchor_terminal:
        from left_arm_v3_1_model_control import MIT_TAU_MAX
        status=arm.read_status(list(inherited))
        inherited,audit=reanchor_terminal_commands(inherited,status,MIT_TAU_MAX)
        for name,row in inherited.items():
            arm.ctrl.controlMIT(arm.motors[name],row['kp'],row['kd'],row['q'],row['dq'],row['tau_ff'])
        print('board terminal torque-equivalent reanchor= '+json.dumps({
            'joints':audit,'nominal_goal_errors_retained_in_endpoint_report':True,
            'learning_model_modified':False}),flush=True)
    args = build_parser().parse_args(hold_command(hold_model_path)[3:]+[
        '--motion-gain-floor-json',json.dumps(inherited)])
    report = {'same_process':True,'same_serial_session':True,
              'terminal_to_first_refresh_ms':None if last is None else (first-last)*1000,
              'enable_frames_sent':False}
    print('board same-session hold begin= '+json.dumps(report),flush=True)
    held = model_hold(args,arm_context=(legacy,arm),prepared_model=hold_model,fallback_model=rigid_model)
    print('board same-session hold complete= '+json.dumps(held),flush=True)
    return held


def prepared_request(args):
    return {'x_mm':args.x_mm,'y_mm':args.y_mm,'piece':args.piece,
            'stage':args.stage,'grip_height_mm':args.grip_height_mm}


def prepared_source_hashes(args):
    files={'rigid_model':Path(args.model),'board_model':Path(args.board_model),
           'planner':Path(__file__),'controller':HERE/'left_arm_v2_6.py',
           'motion_adapter':HERE/'left_arm_v3_1_motion_dynamics.py',
           'fault_support':HERE/'left_arm_fault_support.py',
           'kinematics':HERE/'left_arm_board_kinematics.py',
           'collision':HERE/'left_arm_board_collision.py','trace':HERE/'left_arm_motion_trace.py',
           'task_pose':HERE/'left_arm_board_task_pose.py'}
    return {n:hashlib.sha256(p.read_bytes()).hexdigest() for n,p in files.items()}


def prepared_digest(trajectory):
    return hashlib.sha256(json.dumps(trajectory,sort_keys=True,separators=(',',':'),
                                    allow_nan=False).encode()).hexdigest()


def stamp_prepared_approach(trajectory,args):
    return dict(trajectory,prepared_execution={'version':1,'created_wall_s':time.time(),
        'request':prepared_request(args),'source_hashes':prepared_source_hashes(args),
        'plan_sha256':prepared_digest(trajectory),
        'origin':'new geometric solution before fixed Clearance motion; not taught-angle replay'})


def load_prepared_approach(path,args,actual_start):
    trajectory=json.loads(Path(path).read_text(encoding='utf-8'))
    metadata=trajectory.pop('prepared_execution',None)
    if (not metadata or metadata.get('version')!=1
            or metadata.get('plan_sha256')!=prepared_digest(trajectory)
            or metadata.get('request')!=prepared_request(args)
            or metadata.get('source_hashes')!=prepared_source_hashes(args)):
        raise ValueError('prepared approach request, sources or plan digest differ')
    age=time.time()-float(metadata['created_wall_s'])
    if not 0<=age<=3600:
        raise ValueError('prepared approach expired')
    errors={n:math.degrees(float(actual_start[n])-float(q)) for n,q in trajectory['start_rad'].items()}
    if any(not math.isfinite(e) or abs(e)>5. for e in errors.values()):
        raise BoardPlanningError('actual Clearance differs from prepared entry',
            {'code':'prepared_entry_mismatch','errors_deg':errors,'tolerance_deg':5.})
    return dict(trajectory,prepared_execution=metadata,precomputed_before_motion=True,
                prepared_entry_errors_deg=errors)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=str(HERE/'data/left_arm_general_dynamics.json'))
    parser.add_argument('--board-model', default=str(HERE/'data/left_arm_board_grasp.json'))
    parser.add_argument('--hold-model', default=str(HERE/'data/left_arm_v3_1_dynamics.json'))
    parser.add_argument('--mode', choices=['apply', 'shadow'], default='apply')
    parser.add_argument('--x-mm', type=float)
    parser.add_argument('--y-mm', type=float)
    parser.add_argument('--piece', default='white_bishop')
    parser.add_argument('--stage', choices=['above', 'touch', 'approach', 'cartesian-check', 'grasp'], default='above')
    parser.add_argument('--direction-check-joint', choices=['shoulder_side', 'shoulder_rotate', 'wrist_side'])
    parser.add_argument('--direction-check-step-deg', type=float, default=1.)
    parser.add_argument('--grip-height-mm', type=float)
    parser.add_argument('--start-json', help='offline preview; cannot execute')
    parser.add_argument('--plan-output', type=Path)
    parser.add_argument('--prepare-execution',action='store_true',
                        help='bind a fresh complete geometric preview to request and source hashes')
    parser.add_argument('--prepared-plan',type=Path,
                        help='follow this fresh complete plan without IK during motion')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--validate-candidate-above', action='store_true',
                        help='explicit on-robot validation: common approach plan, transit only; no descent or claw')
    parser.add_argument('--validate-candidate-touch', action='store_true',
                        help='explicit common-model staged 1..25mm descent trial, no claw')
    parser.add_argument('--validate-candidate-approach', action='store_true',
                        help='explicit common-model complete approach to requested contact height, no claw')
    parser.add_argument('--no-model-hold', action='store_true')
    parser.add_argument('--inline-model-hold', action='store_true',
                        help='Web automatic hold on the existing serial session')
    parser.add_argument('--test-protective-stop', action='store_true',
                        help='no trajectory or enable frames; latch support at an already-enabled pose')
    args = parser.parse_args(argv)
    if (args.prepare_execution or args.prepared_plan) and not args.validate_candidate_approach:
        parser.error('prepared execution requires the explicit complete candidate approach')
    if args.prepare_execution and (args.execute or args.prepared_plan or not args.start_json or not args.plan_output):
        parser.error('prepare requires offline start and output, without execution or prepared input')
    if args.prepared_plan and args.start_json:
        parser.error('prepared execution must verify the live entry pose')
    if args.validate_candidate_approach and (args.validate_candidate_above or args.validate_candidate_touch
            or args.stage != 'approach' or args.direction_check_joint or args.test_protective_stop):
        parser.error('candidate complete approach requires approach without other validation/fault flags')
    if args.start_json and args.execute:
        parser.error('--start-json cannot execute')
    if args.validate_candidate_above and (args.stage != 'approach' or args.direction_check_joint or args.test_protective_stop):
        parser.error('candidate above validation requires approach without direction/fault injection')
    if args.validate_candidate_touch and (args.validate_candidate_above or args.stage!='touch'
                                         or args.direction_check_joint or args.test_protective_stop):
        parser.error('candidate touch validation requires Touch without other validation/fault flags')
    # Release only the above-board stage during staged hardware validation.
    if args.execute and BOARD_EXECUTION_SUSPENDED:
        raise RuntimeError(BOARD_EXECUTION_SUSPENSION_REASON)
    if args.execute and args.stage not in SUPPORTED_EXECUTION_STAGES:
        raise RuntimeError('Only Above Only and staged Touch are released for hardware validation')
    if args.test_protective_stop and not args.execute:
        print(json.dumps({'ok': True, 'motion_issued': False, 'test': 'protective-stop',
                          'requires_already_enabled_motors': True, 'persistent_until_explicit_stop': True}))
        return
    if not args.test_protective_stop and (args.x_mm is None or args.y_mm is None):
        parser.error('--x-mm and --y-mm are required for board paths')
    model = RigidBodyModel.load(args.model)
    cfg = load_board_config(args.board_model, model)
    if args.execute and cfg.get('planning_only', False):
        raise RuntimeError('this mathematical search configuration is offline-only; hardware limits are not established')
    if (args.execute and cfg.get('body_collision', {}).get('enabled')
            and not cfg['body_collision'].get('physically_validated', False)
            and not (args.validate_candidate_above or args.validate_candidate_touch or args.validate_candidate_approach)):
        raise RuntimeError('body registration is an offline candidate; preview without --execute')
    if hashlib.sha256(Path(args.model).read_bytes()).hexdigest() != cfg['rigid_model_sha256']:
        raise ValueError('rigid model differs from the board registration source')
    if not args.test_protective_stop:
        validate_request(args.x_mm, args.y_mm, args.piece, args.stage, args.grip_height_mm, cfg)
    arm = None
    result = {}
    if args.start_json:
        start = json.loads(args.start_json)
    else:
        os.environ['HIVE_ROBOT_RIGID_BODY_MODEL'] = str(Path(args.model).resolve())
        os.environ['HIVE_ROBOT_RIGID_BODY_MODE'] = args.mode
        os.environ.setdefault('HIVE_ROBOT_MOTION_ACTION', 'board-grasp')
        os.environ.setdefault('HIVE_ROBOT_MOTION_RUN_ID', secrets.token_hex(8))
        from left_arm_general_motion import load_configuration
        load_configuration(args.model, args.mode)
        import left_arm_v2_6 as legacy
        arm = legacy.LeftArmV2()
    try:
        if arm is not None:
            start = arm.positions(model.names)
        if args.test_protective_stop:
            supervisor = FaultSupport(arm, model.names, {n: {'kp': 2., 'kd': .4} for n in model.names})
            arm._board_fault_support = supervisor
            status = arm.read_status(model.names)
            supervisor.observe(status)
            if any(abs(row['vel']) > .05 for row in supervisor.snapshot.values()):
                raise ValueError('protective-stop injection requires an already stationary arm')
            supervisor.activate(status)
            raise MotionStop('injected software velocity-watchdog stop; no trajectory was run',
                             {'test_injected': True, 'motor_enable_mode': 'reused-existing-enable'})
        print('board planning input=', json.dumps({
            'build': PLANNER_BUILD,
            'start_rad': start, 'x_mm': args.x_mm, 'y_mm': args.y_mm,
            'piece': args.piece, 'stage': args.stage,
            'board_model_sha256': hashlib.sha256(Path(args.board_model).read_bytes()).hexdigest(),
            'orientation_hint_count': len(cfg.get('orientation_hints', [])),
        }), flush=True)
        if args.prepared_plan:
            trajectory=load_prepared_approach(args.prepared_plan,args,start)
        elif args.direction_check_joint:
            if args.stage != 'cartesian-check' or args.test_protective_stop:
                raise ValueError('direction check requires cartesian-check stage without fault injection')
            trajectory = plan_direction_check(model, cfg, start, args.direction_check_joint,
                                              args.direction_check_step_deg)
        else:
            trajectory = plan(model, cfg, start, args.x_mm, args.y_mm, args.piece, args.stage, args.grip_height_mm)
        if args.validate_candidate_above:
            trajectory = candidate_above_trial(trajectory, cfg)
        if args.validate_candidate_touch:
            trajectory = candidate_touch_trial(model,trajectory,cfg)
        if args.validate_candidate_approach:
            trajectory = candidate_approach_trial(trajectory,cfg)
        if args.prepare_execution:
            trajectory=stamp_prepared_approach(trajectory,args)
        if args.plan_output:
            args.plan_output.parent.mkdir(parents=True, exist_ok=True)
            args.plan_output.write_text(json.dumps(trajectory, indent=2), encoding='utf8')
        print('board plan=', json.dumps({k: v for k, v in trajectory.items() if k != 'waypoints'}), flush=True)
        print('board plan samples=', len(trajectory['waypoints']), flush=True)
        if args.execute:
            # Validate hold data before enabling any motor.
            from left_arm_v3_1_model_control import load_model
            hold_model = load_model(Path(args.hold_model))
            arm.board_gravity_limits = {n: hold_model['joints'][n]['tau_limits'] for n in model.names}
            arm._board_motion_enabled = True
            arm.allow_home_handoff = True
            trajectory['runtime_source_hashes']=prepared_source_hashes(args)
            result = execute(arm, legacy, trajectory, geometry=(model, cfg),
                             allow_body_validation_trial=args.validate_candidate_above or args.validate_candidate_touch or args.validate_candidate_approach)
            if args.inline_model_hold or not args.no_model_hold:
                result['model_hold'] = same_session_model_hold(
                    arm,legacy,result,hold_model,model,args.hold_model,
                    reanchor_terminal=cfg.get('motion_feedforward_policy')=='single-entry-gravity-anchor')
                result['post_move_model_hold'] = {'enabled':True,'state':'complete',
                                                 'same_serial_session':True,'seconds':5}
            print(json.dumps(result), flush=True)
        else:
            print(json.dumps({'ok': True, 'motion_issued': False,
                              'last_waypoint': trajectory['waypoints'][-1]}), flush=True)
    except (KeyboardInterrupt, SystemExit):
        # Explicit operator interruption remains effective. In particular,
        # do not turn Ctrl+C into an uninterruptible protective-hold loop.
        raise
    except Exception as exc:
        supervisor = getattr(arm, '_board_fault_support', None) if arm is not None else None
        if supervisor is not None and supervisor.active:
            context=getattr(arm,'_board_trace_context',None)
            if isinstance(context,dict):context.update(stage='fault_support',fault=str(exc))
            recovered = supervisor.run(exc)  # Only explicit Home may finish support.
            if recovered is None:
                raise
            result = recovered
            print(json.dumps(recovered), flush=True)
        else:
            raise
    finally:
        if arm is not None:
            supervisor = getattr(arm, '_board_fault_support', None)
            if supervisor is not None:
                from left_arm_board_home_handoff import close_home_handoff
                close_home_handoff(supervisor)
            arm.close()
            trace=getattr(arm,'_board_trace_recorder',None)
            if trace is not None and isinstance(getattr(trace,'path',None),Path):
                trace.close()
                print('board motor trace complete= '+json.dumps({'path':str(trace.path),
                    'frames_written':trace.written,'dropped_frames':trace.dropped,
                    'capture_errors':trace.capture_errors,'error':trace.error}),flush=True)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError) as exc:
        print(json.dumps({'ok': False, 'error': str(exc),
                          'diagnostics': getattr(exc, 'diagnostics', {})}), flush=True)
        print('error: '+str(exc), file=sys.stderr, flush=True)
        sys.exit(1)
