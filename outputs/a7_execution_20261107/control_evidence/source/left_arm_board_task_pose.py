"""Offline task-pose search followed by a checked, fixed-orientation descent.

Historical wrist frames are preferences, not the only permitted orientations.
This module never opens a serial port or changes hardware operating limits.
"""
from __future__ import annotations

import math
import numpy as np

from left_arm_board_kinematics import solve_ik, tcp_pose, rotation, rotation_error
from left_arm_board_collision import collision_scene


def radical_inverse(index, base):
    value, scale = 0., 1.
    while index:
        index, digit = divmod(index, base)
        scale /= base
        value += digit*scale
    return value


def batch_tool_frames(model, encoder, tcp):
    """Vectorized FK for numerical initial guesses, checked against scalar FK."""
    encoder = np.asarray(encoder, dtype=float)
    if encoder.ndim != 2 or encoder.shape[1] != model.n or not np.isfinite(encoder).all():
        raise ValueError('finite batch of encoder vectors required')
    frames = np.broadcast_to(np.eye(4), (len(encoder), 4, 4)).copy()
    angles = (encoder-model.reference)*model.signs
    for index, parent in enumerate(model.parents):
        turn = np.broadcast_to(np.eye(4), frames.shape).copy()
        cosine, sine = np.cos(angles[:, index]), np.sin(angles[:, index])
        turn[:, 0, 0], turn[:, 0, 1] = cosine, -sine
        turn[:, 1, 0], turn[:, 1, 1] = sine, cosine
        frames = frames@parent@turn
    frames[:, :3, 3] += np.einsum('nij,j->ni', frames[:, :3, :3], tcp)
    return frames


def global_task_seeds(model, cfg, target, preferred, axis, cone_rad, check_budget):
    """Bounded differential evolution proposes seeds; it never accepts a path.

    Proposals need not be collision-free. Each is subsequently polished with
    the full body constraints and independently checked before acceptance.
    """
    generations = int(cfg['continuous_grasp_search'].get('global_seed_generations', 180))
    if not 0 <= generations <= 600:
        raise ValueError('global initial-guess generations must be in 0..600')
    if not generations:
        return [], dict(generations=0, evaluations=0, collision_acceptance=False)
    bounds = np.asarray(cfg['joint_limits_encoder_rad'])
    rng = np.random.default_rng(1729)
    size = max(32, 6*model.n)
    population = bounds[:, 0]+rng.random((size, model.n))*(bounds[:, 1]-bounds[:, 0])
    tcp = np.asarray(cfg['tcp_in_wrist_frame_m'])
    desired_axis = preferred@axis

    def scores(values):
        frames = batch_tool_frames(model, values, tcp)
        position = np.linalg.norm(frames[:, :3, 3]-target, axis=1)
        actual_axis = np.einsum('nij,j->ni', frames[:, :3, :3], axis)
        angle = np.arccos(np.clip(actual_axis@desired_axis, -1., 1.))
        return position+.15*np.maximum(angle-cone_rad, 0.)

    values = scores(population)
    for generation in range(generations):
        check_budget(cfg)
        trial = np.empty_like(population)
        for index in range(size):
            choices = rng.choice(np.delete(np.arange(size), index), 3, replace=False)
            mutant = population[choices[0]]+rng.uniform(.5, 1.)*(population[choices[1]]-population[choices[2]])
            mutant = np.clip(mutant, bounds[:, 0], bounds[:, 1])
            mask = rng.random(model.n) < .8
            mask[rng.integers(model.n)] = True
            trial[index] = np.where(mask, mutant, population[index])
        proposed = scores(trial)
        improved = proposed < values
        population[improved], values[improved] = trial[improved], proposed[improved]
    return [population[index].copy() for index in np.argsort(values)[:6]], dict(
        generations=generations, evaluations=size*(generations+1),
        best_weighted_residual_m=float(values.min()), collision_acceptance=False,
        scope='initial guesses only; full IK, axis, collision and whole-branch checks required')


def endpoint_seeds(model, cfg, start, count):
    """The same deterministic seed bank for every XYZ; no square lookup."""
    bounds = np.asarray(cfg['joint_limits_encoder_rad'], dtype=float)
    seeds = [np.asarray(start), model.reference, bounds.mean(axis=1)]
    # Historical path poses are numerical initial guesses only. They never
    # become commanded waypoints without solving and checking the new task.
    seeds += [np.array([row[n] for n in model.names])
              for row in cfg.get('_reference_path_seeds', [])]
    seeds += [np.array([row[n] for n in model.names]) for row in cfg.get('seeds', {}).values()]
    primes = [2, 3, 5, 7, 11, 13, 17]
    if model.n > len(primes):
        raise ValueError('task-pose seed bank supports up to seven axes')
    for index in range(1, count+1):
        fractions = np.array([radical_inverse(index, p) for p in primes[:model.n]])
        seeds.append(bounds[:, 0]+fractions*(bounds[:, 1]-bounds[:, 0]))
    unique = []
    for seed in seeds:
        seed = np.asarray(seed, dtype=float)
        if (np.isfinite(seed).all() and np.all(seed >= bounds[:, 0])
                and np.all(seed <= bounds[:, 1])
                and not any(np.max(np.abs(seed-old)) < 1e-8 for old in unique)):
            unique.append(seed.copy())
    return unique


def task_settings(cfg):
    settings = cfg['continuous_grasp_search']
    cone = float(settings.get('axis_cone_deg', 10.))
    tolerance = float(settings.get('position_tolerance_mm', .5))
    count = int(settings.get('endpoint_seed_count', 24))
    branches = int(settings.get('max_endpoint_branches', 6))
    complete = int(settings.get('max_complete_branches', 3))
    if not (0 <= cone < 90 and 0 < tolerance <= 2 and 0 <= count <= 64
            and 1 <= branches <= 16 and 1 <= complete <= 16):
        raise ValueError('invalid common task-pose search settings')
    if cfg.get('tool_orientation_constraint') != 'full-wrist':
        raise ValueError('continuous pose selection requires full-wrist descent auditing')
    for profile in cfg['profiles'].values():
        rotation(profile['wrist_rotation_in_board'])
    return cone, tolerance, count, branches


def plan_continuous_approach(model, cfg, start, x_mm, y_mm, piece, height_mm, planner):
    """Choose an endpoint pose, then prove its whole vertical branch backwards.

    The tool axis may vary inside a declared cone during *offline selection*.
    Once selected, the complete wrist rotation is frozen for the task path.
    Joint limits and every swept-edge collision check remain those of cfg.
    """
    contact, profile = planner.validate_request(x_mm, y_mm, piece, 'approach', height_mm, cfg)
    cone_deg, tolerance_mm, seed_count, max_branches = task_settings(cfg)
    rotation = np.asarray(cfg['board_to_cad_rotation'])
    translation = np.asarray(cfg['board_to_cad_translation_m'])
    tcp = np.asarray(cfg['tcp_in_wrist_frame_m'])
    bounds = np.asarray(cfg['joint_limits_encoder_rad'])
    q0 = np.array([start[n] for n in model.names])
    if np.any(q0 < bounds[:, 0]) or np.any(q0 > bounds[:, 1]):
        raise planner.BoardPlanningError('start outside declared operating bounds',
                                         {'code': 'start_outside_operating_bounds'})
    preferred = rotation@np.asarray(profile['wrist_rotation_in_board'])
    axis = np.asarray(cfg['tool_axis_in_wrist_frame'], dtype=float)
    if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-10:
        raise ValueError('finite nonzero tool axis required')
    axis /= np.linalg.norm(axis)
    descent_cone = float(cfg.get('continuous_grasp_search', {}).get('descent_axis_cone_deg', 0.))
    if not math.isfinite(descent_cone) or not 0. <= descent_cone <= 30.:
        raise ValueError('descent axis cone must be in 0..30 degrees')
    terminal_lock_mm = 10.
    terminal_reference_tolerance = float(cfg.get('continuous_grasp_search', {}).get('terminal_reference_tolerance_deg', 180.))
    if not math.isfinite(terminal_reference_tolerance) or not 0. < terminal_reference_tolerance <= 180.:
        raise ValueError('terminal reference tolerance must be in 0..180 degrees')
    scene = collision_scene(model, cfg)
    target = rotation@(contact/1000)+translation
    attempts, endpoints, failures = [], [], []
    if descent_cone:
        # Try the operator-approved full frame at the requested XYZ first;
        # saved joint angles are only an IK seed, never the commanded target.
        exact_seed = np.array([profile['seed_encoder_rad'][n] for n in model.names])
        exact = solve_ik(model, target, exact_seed, tcp, bounds, preferred,
            max_iterations=150, position_tolerance_m=tolerance_mm/1000,
            rotation_tolerance_rad=.005, clearance_function=scene)
        if exact['success']:
            exact_q = np.array([exact['encoder_rad'][n] for n in model.names])
            try:
                planner.check_board_intersection(model, exact_q, cfg)
                endpoints.append(exact_q)
            except planner.BoardPlanningError as exc:
                failures.append(dict(stage='preferred_endpoint_collision', details=exc.diagnostics))
    global_seeds, global_audit = global_task_seeds(model, cfg, target, preferred,
        axis, math.radians(cone_deg), planner.check_planning_budget)
    for seed in global_seeds+endpoint_seeds(model, cfg, q0, seed_count):
        planner.check_planning_budget(cfg)
        answer = solve_ik(model, target, seed, tcp, bounds, preferred,
            max_iterations=150, position_tolerance_m=tolerance_mm/1000,
            rotation_tolerance_rad=1e-5, clearance_function=scene,
            tool_axis_local=axis, tool_axis_cone_rad=math.radians(cone_deg))
        q = np.array([answer['encoder_rad'][n] for n in model.names])
        pose = tcp_pose(model, q, tcp)
        deviation = math.degrees(math.acos(float(np.clip(
            (pose[:3, :3]@axis)@(preferred@axis), -1., 1.))))
        jv, jw = model.point_jacobian(model.encoder_to_model(q), model.n-1, tcp)
        singular_values = np.linalg.svd(np.vstack([jv, .15*jw]), compute_uv=False)
        attempts.append(dict(position_error_mm=answer['position_error_mm'],
            axis_deviation_deg=deviation, success=answer['success'],
            encoder_rad=answer['encoder_rad'],
            active_operating_bounds=[n for j, n in enumerate(model.names)
                if min(q[j]-bounds[j, 0], bounds[j, 1]-q[j]) < 1e-5],
            task_jacobian_smallest_singular_value=float(singular_values[-1])))
        full_reference_error = math.degrees(float(np.linalg.norm(rotation_error(preferred, pose[:3, :3]))))
        if (not answer['success'] or deviation > cone_deg+math.degrees(1e-5)
                or full_reference_error > terminal_reference_tolerance):
            continue
        try:
            planner.check_board_intersection(model, q, cfg)
        except planner.BoardPlanningError as exc:
            failures.append(dict(stage='endpoint_collision', details=exc.diagnostics))
            continue
        if not any(np.max(np.abs(q-old)) < .01 for old in endpoints):
            endpoints.append(q)
    endpoints.sort(key=lambda q: (
        float(np.linalg.norm(rotation_error(preferred, tcp_pose(model, q, tcp)[:3, :3])))
        if descent_cone else 0., float(np.linalg.norm(q-q0))))
    heights = list(dict.fromkeys([max(float(cfg['default_clearance_mm']), contact[2]+50),
                                  max(100., contact[2]+50), max(180., contact[2]+50)]))
    feasible = []
    def select_complete():
        result = min(feasible, key=lambda row: row['posture_selection']['score'])
        result['task_pose_search']['complete_branch_scores'] = [row['posture_selection']['score'] for row in feasible]
        return result
    for endpoint in endpoints[:max_branches]:
        desired = tcp_pose(model, endpoint, tcp)[:3, :3]
        for above_height in heights:
            try:
                planner.check_planning_budget(cfg)
                # Endpoint first: an easy hover is not evidence of a reachable grasp.
                reverse = [(contact.copy(), endpoint.copy())]
                current, q, step = float(contact[2]), endpoint.copy(), 8.
                while current < above_height-1e-8:
                    planner.check_planning_budget(cfg)
                    next_height = min(above_height, current+step)
                    point = np.array([contact[0], contact[1], next_height])
                    xyz = rotation@(point/1000)+translation
                    moving_axis = descent_cone and next_height > float(contact[2])+terminal_lock_mm
                    orientation_kwargs = dict(tool_axis_local=axis,
                        tool_axis_cone_rad=math.radians(descent_cone)) if moving_axis else {}
                    answer = solve_ik(model, xyz, q, tcp, bounds, preferred if moving_axis else desired,
                        max_iterations=150, position_tolerance_m=tolerance_mm/1000,
                        rotation_tolerance_rad=1e-5 if moving_axis else .005,
                        clearance_function=scene, **orientation_kwargs)
                    next_q = np.array([answer['encoder_rad'][n] for n in model.names])
                    if descent_cone:
                        new_axis = tcp_pose(model, next_q, tcp)[:3, :3]@axis
                        axis_ok = math.acos(float(np.clip(new_axis@(preferred@axis), -1., 1.))) <= math.radians(descent_cone)+1e-5
                    else:
                        axis_ok = True
                    continuous = axis_ok and np.max(np.abs(next_q-q)) <= math.radians(12)
                    if descent_cone:
                        old_rotation = tcp_pose(model, q, tcp)[:3, :3]
                        new_rotation = tcp_pose(model, next_q, tcp)[:3, :3]
                        continuous = continuous and np.linalg.norm(rotation_error(new_rotation, old_rotation)) <= math.radians(6.)
                    if not answer['success'] or not continuous:
                        if step > 1.:
                            step /= 2
                            continue
                        raise planner.BoardPlanningError('reverse Cartesian branch failed',
                            dict(code='reverse_branch_ik_unsolved', target_board_mm=point.tolist(),
                                 position_error_mm=answer['position_error_mm'],
                                 rotation_error_deg=answer['rotation_error_deg'],
                                 joint_continuity_passed=bool(continuous)))
                    planner.check_transit_edge(model, q, next_q, cfg)
                    reverse.append((point, next_q))
                    current, q, step = next_height, next_q, min(8., step*2)
                route, blocked = planner.find_entry_transit(model, q0,
                    [(float(np.linalg.norm(q-q0)), q)], cfg)
                if route is None:
                    raise planner.BoardPlanningError('no checked Clearance-to-hover transit',
                        dict(code='above_transit_blocked', blocked_paths=blocked))
                rows, previous = [], q0.copy()
                for stage, entry in route:
                    count = max(1, int(np.ceil(np.max(np.abs(entry-previous))/math.radians(5))))
                    initial = previous.copy()
                    for index in range(1, count+1):
                        value = initial+(entry-initial)*index/count
                        pose = tcp_pose(model, value, tcp)
                        rows.append(dict(stage=stage, target_cad_m=pose[:3, 3].tolist(),
                            target_board_mm=((pose[:3, 3]-translation)@rotation*1000).tolist(),
                            encoder_rad=dict(zip(model.names, value.tolist())),
                            position_error_mm=0., rotation_error_deg=None))
                    previous = entry
                for point, value in reversed(reverse[:-1]):
                    pose = tcp_pose(model, value, tcp)
                    xyz = rotation@(point/1000)+translation
                    rows.append(dict(stage='contact', target_cad_m=xyz.tolist(),
                        target_board_mm=point.tolist(), encoder_rad=dict(zip(model.names, value.tolist())),
                        position_error_mm=float(np.linalg.norm(pose[:3, 3]-xyz)*1000),
                        rotation_error_deg=math.degrees(float(np.linalg.norm(rotation_error(desired, pose[:3, :3]))))))
                minimum = min(float(scene(np.array([row['encoder_rad'][n] for n in model.names])).min())
                              for row in rows) if scene is not None else None
                cost, load = planner.posture_load_cost(model, cfg, start, rows)
                states = np.array([q0]+[np.array([row['encoder_rad'][n] for n in model.names]) for row in rows])
                travel = float(np.linalg.norm(np.diff(states, axis=0), axis=1).sum())
                score = travel+cost-(0. if minimum is None else 20.*min(.03, minimum))
                result = dict(ok=True, build=planner.PLANNER_BUILD, motion_issued=False,
                    stage='approach', piece=piece, start_rad=dict(start), target_board_mm=contact.tolist(),
                    target_source='requested-board-xyz', joint_target_source='geometric-IK; common numerical seed bank',
                    waypoints=rows, endpoint_settle_enabled=True,
                    selected_wrist_rotation_in_board=(rotation.T@desired).tolist(),
                    orientation_policy=('bounded axis during upper descent; fixed full wrist frame final10mm'
                        if descent_cone else 'continuous task-axis cone; full wrist frame frozen over descent'),
                    tool_orientation_constraint='full-wrist',
                    task_pose_search=dict(axis_cone_deg=cone_deg, position_tolerance_mm=tolerance_mm,
                        descent_axis_cone_deg=descent_cone, terminal_full_frame_lock_mm=terminal_lock_mm,
                        preferred_wrist_rotation_in_board=(rotation.T@preferred).tolist(),
                        selected_axis_deviation_deg=math.degrees(math.acos(float(np.clip(
                            (desired@axis)@(preferred@axis), -1., 1.)))),
                        axis_numerical_tolerance_deg=math.degrees(1e-5),
                        endpoint_attempts=len(attempts), feasible_endpoints=len(endpoints),
                        global_initial_search=global_audit,
                        selected_above_height_mm=above_height, per_square_parameters=False,
                        preferred_axis_source='piece reference frame; preference cone, not fixed full orientation',
                        physical_orientation_verified=False),
                    posture_selection=dict(policy='endpoint-first complete reverse branch and checked transit',
                        score=score, feasible_orientation_count=len(endpoints),
                        waypoint_body_clearance_lower_bound_mm=None if minimum is None else minimum*1000,
                        posture_load_preference=load, per_square_parameters=False),
                    registration_status=cfg['registration_status'], fit_errors_mm=cfg['fit_errors_mm'],
                    tcp_datum=cfg['tcp_datum']['definition'], joint_bound_source=cfg['joint_bound_source'],
                    entry_route=[stage for stage, _ in route],
                    collision_scope='configured body/board envelopes and swept joint intervals; other pieces and incomplete self collisions excluded',
                    body_collision_physically_validated=bool(cfg.get('body_collision', {}).get('physically_validated', False)),
                    workflow='precomputed fixed Clearance entry / complete approach and descent; no claw command')
                feasible.append(result)
                if len(feasible) >= int(cfg['continuous_grasp_search'].get('max_complete_branches', 3)):
                    return select_complete()
            except planner.PlanningBudgetExceeded:
                if feasible:
                    return select_complete()
                raise
            except planner.BoardPlanningError as exc:
                failures.append(dict(stage='complete_branch', above_height_mm=above_height, details=exc.diagnostics))
    if feasible:
        return select_complete()
    raise planner.BoardPlanningError('continuous task-pose search found no complete approach',
        dict(code='no_complete_task_pose_branch', endpoint_attempts=attempts,
             global_initial_search=global_audit,
             feasible_endpoints=len(endpoints), branch_failures=failures,
             physical_unreachable_proven=False))


def audit_bounded_descent_orientation(model, cfg, result):
    """Recompute task geometry; never trust claimed per-waypoint errors."""
    from left_arm_board_grasp import BoardPlanningError
    settings = cfg.get('continuous_grasp_search', {})
    cone = float(settings.get('descent_axis_cone_deg', 0.))
    if not settings.get('enabled') or not 0. < cone <= 30.:
        raise ValueError('bounded descent requires explicit common task-pose configuration')
    rotation = np.asarray(cfg['board_to_cad_rotation'])
    translation = np.asarray(cfg['board_to_cad_translation_m'])
    profile = approved_profile(cfg, result['piece'], result.get('approved_task_pose_reference'))
    preferred = rotation@np.asarray(profile['wrist_rotation_in_board'])
    desired = rotation@np.asarray(result['selected_wrist_rotation_in_board'])
    axis = np.asarray(cfg['tool_axis_in_wrist_frame'], dtype=float)
    axis = axis/np.linalg.norm(axis)
    rows = [r for r in result['waypoints'] if r['stage']=='source_above'][-1:]
    rows += [r for r in result['waypoints'] if r['stage']=='contact']
    if not rows or rows[-1]['stage']!='contact':
        raise ValueError('bounded descent requires hover and contact samples')
    target = np.asarray(result['target_board_mm'])
    errors, steps, terminal, position = [], [], [], []
    previous = None
    for row in rows:
        q = np.array([row['encoder_rad'][n] for n in model.names])
        pose = tcp_pose(model, q, cfg['tcp_in_wrist_frame_m'])
        actual = pose[:3, :3]
        errors.append(math.degrees(math.acos(float(np.clip(
            (actual@axis)@(preferred@axis), -1., 1.)))))
        if previous is not None:
            steps.append(math.degrees(np.linalg.norm(rotation_error(actual, previous))))
        previous = actual
        xyz = (pose[:3, 3]-translation)@rotation*1000
        point = np.asarray(row['target_board_mm'])
        position.append(float(np.linalg.norm(xyz-point)))
        if row['stage']=='contact' and point[2] <= target[2]+10.+1e-8:
            terminal.append(math.degrees(np.linalg.norm(rotation_error(desired, actual))))
    tolerance = float(settings.get('position_tolerance_mm', .5))
    reference_error = math.degrees(float(np.linalg.norm(rotation_error(preferred, desired))))
    reference_tolerance = float(settings.get('terminal_reference_tolerance_deg', 180.))
    passed = (reference_error <= reference_tolerance+1e-6
        and max(errors) <= cone+math.degrees(1e-5)+1e-6
        and max(steps, default=0.) <= 6.+1e-6
        and bool(terminal) and max(terminal) <= math.degrees(.005)+1e-6
        and max(position) <= tolerance+1e-6)
    audit = dict(constraint='bounded-axis-with-fixed-terminal-frame', passed=bool(passed),
        max_axis_deviation_deg=max(errors), axis_cone_deg=cone,
        terminal_reference_error_deg=reference_error, terminal_reference_tolerance_deg=reference_tolerance,
        max_adjacent_rotation_deg=max(steps, default=0.), adjacent_rotation_limit_deg=6.,
        terminal_full_frame_max_error_deg=max(terminal, default=float('inf')),
        terminal_full_frame_tolerance_deg=math.degrees(.005), terminal_lock_mm=10.,
        max_position_error_mm=max(position), position_tolerance_mm=tolerance,
        checked_task_samples=len(rows), physical_pose_verified=False)
    if not passed:
        raise BoardPlanningError('bounded descent orientation audit failed',
                                 dict(code='bounded_descent_orientation_mismatch', **audit))
    return audit


def approved_profile(cfg, piece, name=None):
    """Resolve only registered references; no arbitrary frame in a plan claim."""
    if name is None:
        return cfg['profiles'][piece]
    rows = [r for r in cfg.get('approved_task_pose_references', []) if r.get('name')==name]
    if len(rows)!=1:
        raise ValueError('unknown or duplicate approved task pose reference')
    return rows[0]


def plan_from_approved_references(model, cfg, start, x_mm, y_mm, piece, height_mm, planner):
    """The same pose bank is available at every XY; angles remain IK seeds.

    The primary common model runs first. Under the validated-path policy,
    accepted paths are tried next in increasing requested board-XY distance.
    Proximity selects numerical references, never accepts or replays a path.
    """
    import copy
    references = cfg.get('approved_task_pose_references', [])
    if not references:
        result = plan_continuous_approach(model, cfg, start, x_mm, y_mm, piece, height_mm, planner)
        result['orientation_acceptance'] = planner.check_planned_tool_orientation(model, cfg, result)
        return result
    names = [r.get('name') for r in references]
    if any(not isinstance(n, str) or not n for n in names) or len(set(names))!=len(names):
        raise ValueError('unique approved task reference names required')
    target, _ = planner.validate_request(x_mm, y_mm, piece, 'approach', height_mm, cfg)
    R, t = np.asarray(cfg['board_to_cad_rotation']), np.asarray(cfg['board_to_cad_translation_m'])
    scores = []
    for ref in references:
        q = np.array([ref['seed_encoder_rad'][n] for n in model.names])
        pose = tcp_pose(model, q, cfg['tcp_in_wrist_frame_m'])
        point = (pose[:3, 3]-t)@R*1000
        distance = float(np.linalg.norm(point-target))
        if cfg.get('validated_path_reference_policy') == 'primary-then-nearest-validated-path':
            if ref.get('reference_kind') == 'validated_path':
                xy = np.asarray(ref['reference_board_xy_mm'], dtype=float)
                if xy.shape != (2,) or not np.isfinite(xy).all():
                    raise ValueError('finite validated path board XY required')
                scores.append((1, float(np.linalg.norm(xy-target[:2])), ref))
            else:
                scores.append((0, distance, ref))
        else:
            scores.append((0, distance, ref))
    failures = []
    for phase, distance, ref in sorted(scores, key=lambda pair:pair[:2]):
        planner.check_planning_budget(cfg)
        candidate = copy.deepcopy(cfg)
        if cfg.get('validated_path_reference_policy') == 'primary-then-nearest-validated-path':
            import time
            cap = float(cfg.get('reference_attempt_budget_s', 60.))
            if not math.isfinite(cap) or not 1 <= cap <= 600:
                raise ValueError('reference attempt budget must be in 1..600 seconds')
            candidate['_planning_deadline'] = min(cfg.get('_planning_deadline', float('inf')),
                                                  time.monotonic()+cap)
            candidate['_reference_path_seeds'] = copy.deepcopy(ref.get('path_seed_encoder_rad', []))
        for field in ('seed_encoder_rad','wrist_rotation_in_board'):
            candidate['profiles'][piece][field] = copy.deepcopy(ref[field])
        try:
            result = plan_continuous_approach(model, candidate, start, x_mm, y_mm, piece, height_mm, planner)
            result['approved_task_pose_reference'] = ref['name']
            result['reference_selection'] = dict(policy=cfg.get('validated_path_reference_policy',
                'common approved pose bank; FK distance search order'),
                search_phase='nearest-validated-path' if phase else 'primary-common-model',
                reference_distance_mm=distance, distance_basis='requested-board-XY' if phase else 'seed-FK-XYZ',
                reference_count=len(references),
                failed_references=failures, per_square_parameters=False, saved_angles_replayed=False)
            result['orientation_acceptance'] = planner.check_planned_tool_orientation(model, cfg, result)
            return result
        except planner.PlanningBudgetExceeded:
            # An individual reference may time out without starving every
            # later reference. The overall planning deadline still applies.
            planner.check_planning_budget(cfg)
            if 'reference_attempt_budget_s' not in cfg:
                raise
            failures.append(dict(reference=ref['name'], details={'code':'reference_attempt_timeout'}))
        except planner.BoardPlanningError as exc:
            failures.append(dict(reference=ref['name'], details=exc.diagnostics))
    raise planner.BoardPlanningError('no complete path from approved common pose bank',
        dict(code='approved_pose_bank_unsolved', references=failures, physical_unreachable_proven=False))
