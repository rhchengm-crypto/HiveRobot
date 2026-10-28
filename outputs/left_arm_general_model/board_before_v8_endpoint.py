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

import numpy as np

from left_arm_board_kinematics import (board_point_to_cad, rotation, rotation_error,
                                      solve_ik, tcp_pose)
from left_arm_rigid_body import RigidBodyModel, axis_rotation, vector
from left_arm_fault_support import FaultSupport, BoardWatchdog, MotionStop, segment_timing

HERE = Path(__file__).resolve().parent
PLANNER_BUILD = 'seven-axis-board-grasp-v7-record-only'
BOARD_EXECUTION_SUSPENDED = False
SUPPORTED_EXECUTION_STAGES = ('above', 'touch')
BOARD_EXECUTION_SUSPENSION_REASON = (
    'Board Point Grasp execution is suspended after an unintended descent during '
    'watchdog shutdown. Preview only; loaded fault-stop support is not validated.')


class BoardPlanningError(ValueError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


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
    return cfg


def validate_request(x_mm, y_mm, piece, stage, height_mm, config):
    if stage not in ('above', 'touch', 'grasp'):
        raise ValueError('stage must be above, touch or grasp')
    profile = config['profiles'].get(piece)
    if profile is None:
        raise ValueError('piece needs a grasp-height/orientation profile; available: '+','.join(config['profiles']))
    xyz = [float(x_mm), float(y_mm), float(profile['grip_height_mm'] if height_mm is None else height_mm)]
    if not np.isfinite(xyz).all() or not 0 <= xyz[0] <= 440 or not 0 <= xyz[1] <= 440 or not 10 <= xyz[2] <= 150:
        raise ValueError('XY must be in [0,440]mm and grip height in [10,150]mm')
    return np.array(xyz), profile


def check_board_intersection(model, encoder, cfg):
    """Conservative link-axis capsules against the finite board, not other pieces."""
    frames = model.fk(model.encoder_to_model(encoder))
    points = frames[:, :3, 3]
    r = rotation(cfg['board_to_cad_rotation'])
    t = np.asarray(cfg['board_to_cad_translation_m'])
    for index in range(len(points)-1):
        for s in np.linspace(0, 1, 13):
            p = ((1-s)*points[index]+s*points[index+1]-t) @ r
            if -.03 <= p[0] <= .47 and -.03 <= p[1] <= .47 and p[2] < .03:
                raise BoardPlanningError('arm link envelope intersects board plane', {
                    'code': 'link_board_intersection', 'link_from': model.names[index],
                    'link_to': model.names[index+1], 'link_fraction': float(s),
                    'board_point_mm': (p*1000).tolist(), 'minimum_height_mm': 30.})
    # End tool centreline; fingers are outside this simplified capsule model.
    end = tcp_pose(model, encoder, cfg['tcp_in_wrist_frame_m'])[:3, 3]
    for s in np.linspace(0, 1, 15):
        p = ((1-s)*points[-1]+s*end-t) @ r
        if -.02 <= p[0] <= .46 and -.02 <= p[1] <= .46 and p[2] < .005:
            raise BoardPlanningError('tool centreline intersects board plane', {
                'code': 'tool_board_intersection', 'tool_fraction': float(s),
                'board_point_mm': (p*1000).tolist(), 'minimum_height_mm': 5.})


def check_transit_edge(model, start, goal, cfg):
    """Check the same joint interpolation used by the continuous controller."""
    count = max(1, int(np.ceil(np.max(np.abs(goal-start))/np.radians(2))))
    for alpha in np.linspace(0, 1, count+1):
        try:
            check_board_intersection(model, start+(goal-start)*alpha, cfg)
        except BoardPlanningError as exc:
            raise BoardPlanningError(str(exc), {'transit_fraction': float(alpha), **exc.diagnostics}) from exc


def find_entry_transit(model, start, candidates, cfg):
    """Try direct edges first, then checked raised reference waypoints.

    A reference is a configuration-space waypoint, not a saved grasp action.
    Both legs must pass all existing board checks and operating bounds.
    """
    blocked = []
    ordered = sorted(candidates, key=lambda pair: pair[0])
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
    candidates = []
    ik_attempts = []
    for seed in seed_poses:
        if np.any(seed < limits[:, 0]) or np.any(seed > limits[:, 1]):
            continue
        answer = solve_ik(model, above, seed, tcp, limits, desired_r,
                          position_tolerance_m=.0005, rotation_tolerance_rad=.005)
        ik_attempts.append({key: answer[key] for key in
                           ('success', 'position_error_mm', 'rotation_error_deg', 'iterations')})
        if answer['success']:
            goal = np.array([answer['encoder_rad'][n] for n in model.names])
            candidates.append((float(np.linalg.norm(goal-q)), goal))
    route, blocked_paths = find_entry_transit(model, q, candidates, cfg)
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
            'blocked_paths': blocked_paths[:3]})
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
            answer = solve_ik(model, xyz, q, tcp, limits, target_r,
                              position_tolerance_m=.0005, rotation_tolerance_rad=.005)
            if not answer['success']:
                raise ValueError(f'IK failed at {label} sample {i}/{count}: {answer["position_error_mm"]:.2f}mm')
            next_q = np.array([answer['encoder_rad'][n] for n in model.names])
            if np.max(np.abs(next_q-q)) > np.radians(12):
                raise ValueError('IK discontinuity exceeds 12deg per sample')
            # Check the joint interpolation that the controller will actually run.
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
            'target_board_mm': contact.tolist(), 'waypoints': records,
            'registration_status': cfg['registration_status'], 'fit_errors_mm': cfg['fit_errors_mm'],
            'tcp_datum': cfg['tcp_datum']['definition'], 'joint_bound_source': cfg['joint_bound_source'],
            'entry_route': [label for label, _ in route],
            'collision_scope': 'sampled link-axis envelopes vs finite board; other pieces/self collision not modeled',
            'workflow': 'checked-above-transit/contact/pressure-close/lift; above and touch omit claw commands'}


def plan(model, cfg, start, x_mm, y_mm, piece='white_bishop', stage='above', height_mm=None):
    import copy
    validate_request(x_mm, y_mm, piece, stage, height_mm, cfg)
    hints = sorted(cfg.get('orientation_hints', []),
                   key=lambda h: np.linalg.norm(np.array(h['board_xy_mm'])-[x_mm, y_mm]))
    choices = hints + [cfg['profiles'][piece]]
    errors = []
    for hint in choices:
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
                result['orientation_policy'] = 'nearest feasible taught tool orientation; geometric IK at requested XY'
            return result
        except ValueError as exc:
            errors.append({'message': str(exc), 'details': getattr(exc, 'diagnostics', {}),
                           'orientation_reference_xy_mm': hint.get('board_xy_mm')})
    raise BoardPlanningError('no feasible complete board path', {
        'code': 'no_complete_board_path', 'start_rad': dict(start),
        'target_board_mm': [float(x_mm), float(y_mm),
                            float(cfg['profiles'][piece]['grip_height_mm'] if height_mm is None else height_mm)],
        'orientation_attempts': errors})


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
        if label in ('entry_clearance', 'source_above'):
            while end+1 < len(rows) and rows[end+1]['stage'] == label:
                end += 1
            target = np.array([rows[end]['encoder_rad'][n] for n in names])
            direction = target-previous
            denominator = float(direction@direction)
            fractions = []
            for row in rows[index:end+1]:
                point = np.array([row['encoder_rad'][n] for n in names])
                alpha = float((point-previous)@direction/denominator) if denominator > 1e-16 else 1.
                if not np.allclose(point, previous+alpha*direction, atol=1e-9, rtol=0.):
                    end = index
                    break
                fractions.append(alpha)
            if fractions and (min(fractions) < -1e-9 or max(fractions) > 1.+1e-9 or
                              any(b < a for a, b in zip(fractions, fractions[1:]))):
                end = index
        result.append(dict(rows[end], original_index=end, first_original_index=index))
        previous = np.array([rows[end]['encoder_rad'][n] for n in names])
        index = end+1
    return result


def motion_watchdog(arm, supervisor, max_velocity, max_error_deg, context):
    return BoardWatchdog(arm, supervisor, max_velocity, max_error_deg,
        context=context, record_only=True)


def motion_timing(delta, max_velocity):
    # Relax feedback acceptance without increasing reference speed/acceleration.
    result = segment_timing(delta, min(.2,max_velocity))
    result['feedback_velocity_limit_rad_s'] = max_velocity
    return result


def execute(arm, legacy, trajectory, max_velocity=.2, max_error_deg=5., geometry=None):
    """On a trajectory error, main latches the same-process support controller."""
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
        'warning_position_error_deg': max_error_deg,
        'reference_peak_velocity_limit_rad_s': .35*min(.2,max_velocity)}),flush=True)
    from left_arm_board_home_handoff import install_home_handoff, home_command_observed
    install_home_handoff(supervisor, arm, legacy)
    original_control = arm.ctrl.controlMIT
    motor_names = {id(arm.motors[n]): n for n in names}
    def observed_control(motor, kp, kd, q, dq, tau):
        result = original_control(motor, kp, kd, q, dq, tau)
        name = motor_names.get(id(motor))
        if name is not None:
            supervisor.last_commands[name] = dict(kp=float(kp), kd=float(kd), q=float(q),
                                                  dq=float(dq), tau_ff=float(tau))
            home_command_observed(supervisor, name, kp, kd)
        return result
    arm.ctrl.controlMIT = observed_control
    claw_target = None

    def support():
        status = arm.read_status(names)
        supervisor.observe(status)
        if any(not all(math.isfinite(float(s[k])) for k in ('pos', 'vel', 'tau'))
               or abs(float(s['tau'])) > MIT_TAU_MAX[n] for n, s in status.items()):
            raise RuntimeError('invalid board motion entry feedback')
        if any(abs(float(s['vel'])) > .1 for s in status.values()):
            print('board entry velocity audit= '+json.dumps({'mode':'record-only',
                  'protection_triggered':False,
                  'velocity_rad_s':{n:float(s['vel']) for n,s in status.items()}}),flush=True)
        return ({n: float(status[n]['pos']) for n in names},
                {n: float(status[n]['tau']) for n in names}, status)

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
    for group_index, row in enumerate(rows):
        index = row['original_index']
        positions, tau, status = support()
        target = row['encoder_rad']
        if trajectory['stage'] == 'grasp' and row['stage'] == 'lift' and not closed:
            claw_target = close_claw_while_holding_arm(arm, positions, legacy,
                                                       arm_gains=gains, arm_tau=tau, require_contact=True)
            closed = True
            positions, tau, status = support()
        delta = max(abs(target[n]-positions[n]) for n in names)
        if row['stage'] not in ('entry_clearance', 'source_above') and delta > math.radians(15):
            raise RuntimeError('actual waypoint delta exceeds 15deg')
        if geometry is not None:
            model, cfg = geometry
            check_transit_edge(model, np.array([positions[n] for n in model.names]),
                               np.array([target[n] for n in model.names]), cfg)
        timing = motion_timing(delta, max_velocity)
        duration = timing['seconds']
        entry_tau = {n: {'start_tau': tau[n]+gains[n]['kd']*float(status[n]['vel']),
                         'end_tau': tau[n], 'ramp_fraction': .25, 'ramp': 'smoothstep'} for n in names}
        holds = {} if claw_target is None else {'claw': claw_target}
        hold_gains = {} if claw_target is None else {'claw': {'kp': legacy.CLAW_KP_HOLD, 'kd': legacy.CLAW_KD_HOLD}}
        print('board waypoint=', json.dumps({'index': index, 'stage': row['stage'],
                                             'group_index': group_index,
                                             'first_original_index': row['first_original_index'],
                                             'target_board_mm': row['target_board_mm'],
                                             'timing': timing, 'entry_feedback': status}), flush=True)
        arm.move_targets_with_holds(target, seconds_per_step=duration, move_gains=gains,
            hold_targets=holds, hold_gains=hold_gains, hold_tau={'claw': 0.},
            move_tau_ff=entry_tau, fallback_kp=2., fallback_kd=.4, preload_seconds=.05,
            control_dt=.01, trajectory='smootherstep',
            watchdog=motion_watchdog(arm, supervisor, max_velocity, max_error_deg,
                                  {'waypoint_index': index, 'stage': row['stage'], 'timing': timing}))
    # A long continuous transit can finish with a static PD load offset.
    # Re-anchor only at its endpoint, never at every geometry sample. Use the
    # unchanged model adapter and measured-load bridge; keep the same watchdog.
    target = rows[-1]['encoder_rad']
    for attempt in range(3):
        positions, tau, status = support()
        error = max(abs(target[n]-positions[n]) for n in names)
        if error <= math.radians(.35):
            break
        if geometry is not None:
            check_transit_edge(model, np.array([positions[n] for n in model.names]),
                               np.array([target[n] for n in model.names]), cfg)
        timing = motion_timing(error, max_velocity)
        entry_tau = {n: {'start_tau': tau[n]+gains[n]['kd']*float(status[n]['vel']),
                         'end_tau': tau[n], 'ramp_fraction': .25, 'ramp': 'smoothstep'} for n in names}
        print('board endpoint settle=', json.dumps({'attempt': attempt+1,
            'max_error_deg': math.degrees(error), 'entry_feedback': status}), flush=True)
        arm.move_targets_with_holds(target, seconds_per_step=timing['seconds'], move_gains=gains,
            hold_targets=holds, hold_gains=hold_gains, hold_tau={'claw': 0.}, move_tau_ff=entry_tau,
            fallback_kp=2., fallback_kd=.4, preload_seconds=.05, control_dt=.01, trajectory='smootherstep',
            watchdog=motion_watchdog(arm, supervisor, max_velocity, max_error_deg,
                                  {'stage': 'endpoint_settle', 'attempt': attempt+1}))
    return {'ok': True, 'motion_issued': True, 'claw_contact_required': trajectory['stage']=='grasp',
            'grasp_contact_confirmed': closed, 'final': arm.read_status(names)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=str(HERE/'data/left_arm_general_dynamics.json'))
    parser.add_argument('--board-model', default=str(HERE/'data/left_arm_board_grasp.json'))
    parser.add_argument('--hold-model', default=str(HERE/'data/left_arm_v3_1_dynamics.json'))
    parser.add_argument('--mode', choices=['apply', 'shadow'], default='apply')
    parser.add_argument('--x-mm', type=float)
    parser.add_argument('--y-mm', type=float)
    parser.add_argument('--piece', default='white_bishop')
    parser.add_argument('--stage', choices=['above', 'touch', 'grasp'], default='above')
    parser.add_argument('--grip-height-mm', type=float)
    parser.add_argument('--start-json', help='offline preview; cannot execute')
    parser.add_argument('--plan-output', type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--no-model-hold', action='store_true')
    parser.add_argument('--test-protective-stop', action='store_true',
                        help='no trajectory or enable frames; latch support at an already-enabled pose')
    args = parser.parse_args(argv)
    if args.start_json and args.execute:
        parser.error('--start-json cannot execute')
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
    if hashlib.sha256(Path(args.model).read_bytes()).hexdigest() != cfg['rigid_model_sha256']:
        raise ValueError('rigid model differs from the board registration source')
    if not args.test_protective_stop:
        validate_request(args.x_mm, args.y_mm, args.piece, args.stage, args.grip_height_mm, cfg)
    arm = None
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
        trajectory = plan(model, cfg, start, args.x_mm, args.y_mm, args.piece, args.stage, args.grip_height_mm)
        if args.plan_output:
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
            result = execute(arm, legacy, trajectory, geometry=(model, cfg))
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
            recovered = supervisor.run(exc)  # Only explicit Home may finish support.
            if recovered is None:
                raise
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
    if args.execute and not args.no_model_hold:
        from left_arm_general_move import hold_command
        removed = {'HIVE_ROBOT_MOTION_MODEL', 'HIVE_ROBOT_RIGID_BODY_MODEL',
                   'HIVE_ROBOT_RIGID_BODY_MODE', 'HIVE_ROBOT_MOTION_ACTION', 'HIVE_ROBOT_MOTION_RUN_ID'}
        env = {k: v for k, v in os.environ.items() if k not in removed}
        subprocess.run(hold_command(args.hold_model)+['--general-fallback-model', args.model], env=env, check=True)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError) as exc:
        print(json.dumps({'ok': False, 'error': str(exc),
                          'diagnostics': getattr(exc, 'diagnostics', {})}), flush=True)
        print('error: '+str(exc), file=sys.stderr, flush=True)
        sys.exit(1)

