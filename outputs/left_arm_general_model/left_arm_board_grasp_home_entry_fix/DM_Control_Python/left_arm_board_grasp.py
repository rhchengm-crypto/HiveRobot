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

HERE = Path(__file__).resolve().parent
PLANNER_BUILD = 'seven-axis-board-grasp-v2-staged-entry'


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


def execute(arm, legacy, trajectory, max_velocity=.2, max_error_deg=5.):
    from left_arm_general_move import TrackingWatchdog
    from left_arm_v3_1_model_control import MIT_TAU_MAX
    from left_arm_v2_8_move_library import (claw_home_while_holding_arm,
                                           close_claw_while_holding_arm)
    names = list(trajectory['waypoints'][0]['encoder_rad'])
    gains = {n: dict(legacy.HOME_GAINS[n]) for n in names}
    claw_target = None

    def support():
        status = arm.read_status(names)
        if any(not all(math.isfinite(float(s[k])) for k in ('pos', 'vel', 'tau'))
               or abs(float(s['tau'])) > MIT_TAU_MAX[n] for n, s in status.items()):
            raise RuntimeError('invalid board motion entry feedback')
        if any(abs(float(s['vel'])) > .1 for s in status.values()):
            raise RuntimeError('board motion requires a stationary segment entry')
        return ({n: float(status[n]['pos']) for n in names},
                {n: float(status[n]['tau']) for n in names})

    arm.enable(names)
    if trajectory['stage'] == 'grasp':
        positions, tau = support()
        opened = claw_home_while_holding_arm(arm, legacy, 2., .4,
                    carry_hold={'hold_targets': positions, 'hold_gains': gains, 'hold_tau': tau},
                    workflow_label='Board grasp')
        claw_target = float(opened['target_rad'])
    closed = False
    for index, row in enumerate(trajectory['waypoints']):
        positions, tau = support()
        target = row['encoder_rad']
        if trajectory['stage'] == 'grasp' and row['stage'] == 'lift' and not closed:
            claw_target = close_claw_while_holding_arm(arm, positions, legacy,
                                                       arm_gains=gains, arm_tau=tau, require_contact=True)
            closed = True
            positions, tau = support()
        delta = max(abs(target[n]-positions[n]) for n in names)
        if delta > math.radians(15):
            raise RuntimeError('actual waypoint delta exceeds 15deg')
        duration = max(.4, 1.875*delta/(.7*max_velocity))
        holds = {} if claw_target is None else {'claw': claw_target}
        hold_gains = {} if claw_target is None else {'claw': {'kp': legacy.CLAW_KP_HOLD, 'kd': legacy.CLAW_KD_HOLD}}
        print('board waypoint=', json.dumps({'index': index, 'stage': row['stage'],
                                             'target_board_mm': row['target_board_mm']}), flush=True)
        arm.move_targets_with_holds(target, seconds_per_step=duration, move_gains=gains,
            hold_targets=holds, hold_gains=hold_gains, hold_tau={'claw': 0.},
            move_tau_ff=tau, fallback_kp=2., fallback_kd=.4, preload_seconds=.05,
            control_dt=.01, trajectory='smootherstep',
            watchdog=TrackingWatchdog(arm, max_velocity, max_error_deg))
    return {'ok': True, 'motion_issued': True, 'claw_contact_required': trajectory['stage']=='grasp',
            'grasp_contact_confirmed': closed, 'final': arm.read_status(names)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=str(HERE/'data/left_arm_general_dynamics.json'))
    parser.add_argument('--board-model', default=str(HERE/'data/left_arm_board_grasp.json'))
    parser.add_argument('--hold-model', default=str(HERE/'data/left_arm_v3_1_dynamics.json'))
    parser.add_argument('--mode', choices=['apply', 'shadow'], default='apply')
    parser.add_argument('--x-mm', type=float, required=True)
    parser.add_argument('--y-mm', type=float, required=True)
    parser.add_argument('--piece', default='white_bishop')
    parser.add_argument('--stage', choices=['above', 'touch', 'grasp'], default='above')
    parser.add_argument('--grip-height-mm', type=float)
    parser.add_argument('--start-json', help='offline preview; cannot execute')
    parser.add_argument('--plan-output', type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--no-model-hold', action='store_true')
    args = parser.parse_args(argv)
    if args.start_json and args.execute:
        parser.error('--start-json cannot execute')
    model = RigidBodyModel.load(args.model)
    cfg = load_board_config(args.board_model, model)
    if hashlib.sha256(Path(args.model).read_bytes()).hexdigest() != cfg['rigid_model_sha256']:
        raise ValueError('rigid model differs from the board registration source')
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
            load_model(Path(args.hold_model))
            arm._board_motion_enabled = True
            result = execute(arm, legacy, trajectory)
            print(json.dumps(result), flush=True)
        else:
            print(json.dumps({'ok': True, 'motion_issued': False,
                              'last_waypoint': trajectory['waypoints'][-1]}), flush=True)
    except BaseException:
        if arm is not None and getattr(arm, '_board_motion_enabled', False):
            arm.disable([*model.names, 'claw'] if args.stage == 'grasp' else model.names)
        raise
    finally:
        if arm is not None:
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
