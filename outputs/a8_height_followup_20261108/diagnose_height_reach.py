"""Offline height reach ablation; no runtime edits or hardware interfaces.

Uses pre-existing local SciPy only (no installation/network). Compare bounded
least squares and vectorized differential evolution, then select the nearest
full wrist frame using exact position constraints and the common CAD scene.
"""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time
sys.dont_write_bytecode = True
OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
FROZEN = ROOT/'outputs/a8_v33_execution_20261108/source_snapshot/scripts'
sys.path.insert(0, str(ROOT/'tmp/gripper_geometry_deps'))
sys.path.insert(0, str(FROZEN))
import numpy as np
from scipy.optimize import least_squares, minimize
import scipy
import left_arm_board_grasp as b
from left_arm_board_kinematics import tcp_pose, rotation_error


def read(p):
    return json.loads(Path(p).read_text(encoding='utf8'))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    start = time.monotonic()
    mp, cp = FROZEN/'data/left_arm_general_dynamics.json', FROZEN/'data/left_arm_board_workspace_candidate.json'
    model, cfg = b.RigidBodyModel.load(mp), read(cp)
    scene = b.collision_scene(model, cfg)
    names, parents = model.names, np.array(model.parents)
    tcp, r, t = np.array(cfg['tcp_in_wrist_frame_m']), np.array(cfg['board_to_cad_rotation']), np.array(cfg['board_to_cad_translation_m'])
    current = np.array(cfg['joint_limits_encoder_rad'])
    saved = read(ROOT/'outputs/a8_saved_reference_20261108/saved_pose_audit.json')['saved_move']['pose']
    qref = np.array([saved[n] for n in names])
    previous = read(ROOT/'outputs/a8_v33_execution_20261108/lower25_endpoint_diagnostic.json')['best']['encoder_rad']
    qprior = np.array([previous[n] for n in names])
    target_board = np.array([11.798050592563328, 405.6006774366655, 59.622674457482844])
    target = r@(target_board/1000)+t
    reference = tcp_pose(model, qref, tcp)[:3, :3]
    front, roll = names.index('shoulder_front'), names.index('arm_roll')
    evaluations = 0

    def frames(qs, jac=False):
        nonlocal evaluations
        qs = np.atleast_2d(qs)
        n = len(qs)
        evaluations += n
        rotations = np.broadcast_to(np.eye(3), (n, 3, 3)).copy()
        origins = np.zeros((n, 3))
        axes, points = [], []
        for k, angles in enumerate((model.signs*(qs-model.reference)).T):
            origins += rotations@parents[k, :3, 3]
            rotations = rotations@parents[k, :3, :3]
            if jac:
                axes.append(rotations[:, :, 2].copy())
                points.append(origins.copy())
            ss, cc = np.sin(angles), np.cos(angles)
            motion = np.zeros((n, 3, 3))
            motion[:, 0, 0], motion[:, 0, 1] = cc, -ss
            motion[:, 1, 0], motion[:, 1, 1], motion[:, 2, 2] = ss, cc, 1.
            rotations = rotations@motion
        positions = origins+rotations@tcp
        if not jac:
            return positions, rotations
        axes, points = np.array(axes)[:, 0, :], np.array(points)[:, 0, :]
        jp = np.cross(axes, positions[0]-points).T*model.signs[None, :]
        derivatives = []
        for axis, sign in zip(axes, model.signs):
            x, y, z = axis
            skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
            derivatives.append((sign*skew@rotations[0]).ravel())
        return positions[0], rotations[0], jp, np.array(derivatives).T

    def residual(q, fixed):
        p, rr = frames(q)
        return np.r_[p[0]-target, .15/np.sqrt(2)*(rr[0]-reference).ravel()] if fixed else p[0]-target

    def jacobian(q, fixed):
        p, rr, jp, jr = frames(q, True)
        return np.vstack((jp, .15/np.sqrt(2)*jr)) if fixed else jp

    def assessment(q, fixed):
        p = tcp_pose(model, q, tcp)
        pe = float(np.linalg.norm(p[:3, 3]-target)*1000)
        angle = float(np.degrees(np.linalg.norm(rotation_error(reference, p[:3, :3]))))
        gaps = scene(q)
        return dict(encoder_rad=dict(zip(names, q.tolist())), predicted_board_mm=((p[:3, 3]-t)@r*1000).tolist(),
            position_error_mm=pe, orientation_change_from_original_deg=angle,
            pose_passed=bool(pe <= .5 and (not fixed or angle <= math.degrees(.005))),
            common_scene_passed=bool(np.all(gaps >= 0.)), minimum_scene_clearance_mm=float(np.min(gaps)*1000),
            original_bound_violations_deg={name:dict(below=max(0., float(np.degrees(current[i, 0]-q[i]))), above=max(0., float(np.degrees(q[i]-current[i, 1]))))
                for i, name in enumerate(names) if q[i] < current[i, 0]-1e-7 or q[i] > current[i, 1]+1e-7})

    def de(bounds, fixed, seed, generations=500):
        rng = np.random.default_rng(seed)
        n = 98
        low, high = bounds[:, 0], bounds[:, 1]
        pop = rng.uniform(low, high, (n, 7))
        pop[0], pop[1] = np.clip(qref, low, high), np.clip(qprior, low, high)
        def score(qs):
            p, rr = frames(qs)
            value = np.sum((p-target)**2, axis=1)
            if fixed:
                value += .15**2/2*np.sum((rr-reference)**2, axis=(1, 2))
            return value
        values = score(pop)
        own = np.arange(n)
        for generation in range(generations):
            idx = rng.integers(0, n, (n, 3))
            while True:
                invalid = np.any(idx == own[:, None], axis=1) | (idx[:, 0] == idx[:, 1]) | (idx[:, 0] == idx[:, 2]) | (idx[:, 1] == idx[:, 2])
                if not np.any(invalid):
                    break
                idx[invalid] = rng.integers(0, n, (int(invalid.sum()), 3))
            mutant = pop[idx[:, 0]]+rng.uniform(.5, .95, (n, 1))*(pop[idx[:, 1]]-pop[idx[:, 2]])
            invalid = (mutant < low) | (mutant > high)
            mutant = np.where(invalid, rng.uniform(low, high, mutant.shape), mutant)
            mask = rng.random((n, 7)) < .85
            mask[own, rng.integers(0, 7, n)] = True
            trial = np.where(mask, mutant, pop)
            score_trial = score(trial)
            accepted = score_trial < values
            pop[accepted], values[accepted] = trial[accepted], score_trial[accepted]
        return pop[np.argsort(values)[:4]], dict(seed=seed, generations=generations, population=n, evaluations=n*(generations+1))

    domains = []
    for label, degrees in (('current', 0.), ('front_minus10', 10.), ('front_roll10', 10.), ('front_roll30', 30.)):
        bounds = current.copy()
        if degrees:
            bounds[front, 0] -= math.radians(degrees)
        if label.startswith('front_roll'):
            bounds[roll, 1] += math.radians(degrees)
        domains.append((label, bounds))
    domains.append(('all_full_FK_periods', np.column_stack((qref-np.pi, qref+np.pi))))
    results, solutions = [], {}
    warm = [qref, qprior]
    for domain_index, (label, bounds) in enumerate(domains):
        for fixed in (True, False):
            seeds, global_info = de(bounds, fixed, 20261108+domain_index*10+int(fixed))
            trials = []
            for seed in warm+list(seeds):
                solution = least_squares(lambda q:residual(q, fixed), np.clip(seed, bounds[:, 0]+1e-10, bounds[:, 1]-1e-10),
                    jac=lambda q:jacobian(q, fixed), bounds=(bounds[:, 0], bounds[:, 1]), max_nfev=500,
                    ftol=1e-13, xtol=1e-13, gtol=1e-13)
                trials.append(dict(assessment=assessment(solution.x, fixed), nfev=solution.nfev,
                    optimality=float(solution.optimality), squared_cost=float(2*solution.cost), message=solution.message))
            best = min(trials, key=lambda x:x['squared_cost'])
            q = np.array([best['assessment']['encoder_rad'][n] for n in names])
            solutions[(label, fixed)] = q
            warm = [qref, qprior, q]
            results.append(dict(domain=label, fixed_original_full_orientation=fixed,
                bounds_encoder_rad=dict(zip(names, bounds.tolist())), differential_evolution=global_info, best=best, trials=trials))
            print(json.dumps(dict(phase='ablation', domain=label, fixed_orientation=fixed, best=best['assessment'], elapsed_s=time.monotonic()-start)), flush=True)

    # Least requested full-frame change, under ORIGINAL joint bounds and scene.
    # This chooses a desired frame explicitly; its later IK/descent acceptance
    # must remain strict relative to that selected frame.
    def orientation_objective(q):
        p, rr, jp, jr = frames(q, True)
        dr = (rr-reference).ravel()
        return float(.5*(dr@dr)), jr.T@dr
    def position_constraint(q):
        return frames(q)[0][0]-target
    def position_constraint_jac(q):
        return frames(q, True)[2]
    selection = []
    seedbank = [solutions[('current', False)], solutions[('current', True)], qref]
    for index, seed in enumerate(seedbank):
        solution = minimize(lambda q:orientation_objective(q)[0], np.clip(seed, current[:, 0], current[:, 1]),
            jac=lambda q:orientation_objective(q)[1], method='SLSQP', bounds=list(map(tuple, current)),
            constraints=[dict(type='eq', fun=position_constraint, jac=position_constraint_jac),
                         dict(type='ineq', fun=scene, jac=scene.jacobian)],
            options=dict(maxiter=600, ftol=1e-12))
        report = assessment(solution.x, False)
        report.update(optimizer_success=bool(solution.success), message=str(solution.message),
            nfev=solution.nfev, selected_wrist_rotation_in_cad=frames(solution.x)[1][0].tolist(),
            requested_full_frame_within10deg=report['orientation_change_from_original_deg'] <= 10.,
            requested_full_frame_within20deg=report['orientation_change_from_original_deg'] <= 20.)
        selection.append(report)
        print(json.dumps(dict(phase='minimum_orientation_selection', result=report)), flush=True)

    # Local paired-bound extension objective, constrained to exact target pose.
    # SLSQP uses six independent position/SO3 equations, never nine redundant R equations.
    paired_bounds = current.copy()
    paired_bounds[front, 0] -= math.radians(30)
    paired_bounds[roll, 1] += math.radians(30)
    def pose_constraint(q):
        pp, rr = frames(q)
        return np.r_[pp[0]-target, rotation_error(reference, rr[0])]
    def excess(q):
        return np.array([max(0., current[front, 0]-q[front]), max(0., q[roll]-current[roll, 1])])
    def objective(q):
        return float(excess(q)@excess(q))
    paired = minimize(objective, solutions[('front_roll30', True)], method='SLSQP',
        bounds=list(map(tuple, paired_bounds)), constraints=[dict(type='eq', fun=pose_constraint),
        dict(type='ineq', fun=scene, jac=scene.jacobian)], options=dict(maxiter=600, ftol=1e-12))
    paired_report = assessment(paired.x, True)
    paired_report.update(optimizer_success=bool(paired.success), message=str(paired.message),
        front_extra_lower_deg=float(np.degrees(excess(paired.x)[0])), roll_extra_upper_deg=float(np.degrees(excess(paired.x)[1])),
        scope='local minimum sum of squared paired extensions; no verified mechanical limits or deployment permission')
    # Isolate the smallest demonstrated ONE-axis software change when a new
    # complete frame may be selected within an explicit10/20degree budget.
    front_bounds = current.copy()
    front_bounds[front, 0] -= math.radians(30.)
    front_orientation = []
    for seed in (solutions[('front_minus10', False)], solutions[('front_roll30', False)]):
        selected = minimize(lambda q:orientation_objective(q)[0], np.clip(seed, front_bounds[:, 0], front_bounds[:, 1]),
            jac=lambda q:orientation_objective(q)[1], method='SLSQP', bounds=list(map(tuple, front_bounds)),
            constraints=[dict(type='eq', fun=position_constraint, jac=position_constraint_jac),
                         dict(type='ineq', fun=scene, jac=scene.jacobian)],
            options=dict(maxiter=600, ftol=1e-12))
        report = assessment(selected.x, False)
        report.update(optimizer_success=bool(selected.success), message=str(selected.message),
            selected_wrist_rotation_in_cad=frames(selected.x)[1][0].tolist(),
            front_extra_lower_deg=float(np.degrees(max(0., current[front, 0]-selected.x[front]))))
        front_orientation.append(report)
    front_cones = []
    for cone in (10., 20.):
        cone_value = 2*(1-math.cos(math.radians(cone)))
        candidates = []
        for source in front_orientation:
            seed = np.array([source['encoder_rad'][n] for n in names])
            selected = minimize(lambda q:-q[front], seed, method='SLSQP', bounds=list(map(tuple, front_bounds)),
                jac=lambda q:-np.eye(7)[front], constraints=[
                    dict(type='eq', fun=position_constraint, jac=position_constraint_jac),
                    dict(type='ineq', fun=scene, jac=scene.jacobian),
                    dict(type='ineq', fun=lambda q:cone_value-orientation_objective(q)[0],
                         jac=lambda q:-orientation_objective(q)[1])],
                options=dict(maxiter=600, ftol=1e-12))
            report = assessment(selected.x, False)
            report.update(optimizer_success=bool(selected.success), message=str(selected.message),
                requested_full_frame_max_change_deg=cone,
                within_orientation_selection_budget=report['orientation_change_from_original_deg'] <= cone+1e-6,
                front_extra_lower_deg=float(np.degrees(max(0., current[front, 0]-selected.x[front]))),
                selected_wrist_rotation_in_cad=frames(selected.x)[1][0].tolist(),
                no_other_six_bound_changes=True, globally_minimum_extension_proven=False)
            candidates.append(report)
        front_cones.append(dict(requested_full_frame_max_change_deg=cone, candidates=candidates))
    # A genuine fixed-orientation comparison, rather than a weighted rotation
    # compromise: enforce SO3 equality while minimizing Cartesian distance.
    full_bounds = domains[-1][1]
    def position_objective(q):
        p, rr, jp, jr = frames(q, True)
        error = p-target
        return float(.5*(error@error)), jp.T@error
    constrained_fixed = minimize(lambda q:position_objective(q)[0], solutions[('all_full_FK_periods', True)],
        jac=lambda q:position_objective(q)[1], method='SLSQP', bounds=list(map(tuple, full_bounds)),
        constraints=[dict(type='eq', fun=lambda q:rotation_error(reference, frames(q)[1][0])),
                     dict(type='ineq', fun=scene, jac=scene.jacobian)],
        options=dict(maxiter=1000, ftol=1e-14))
    constrained_fixed_report = assessment(constrained_fixed.x, True)
    constrained_fixed_report.update(optimizer_success=bool(constrained_fixed.success),
        message=str(constrained_fixed.message), exact_original_orientation_enforced=True)
    # Verify optimized FK/Jacobian independently against repository routines.
    pp, rr, jp, jr = frames(qref+.1, True)
    repository = tcp_pose(model, qref+.1, tcp)
    epsilon = 1e-6
    numeric = np.column_stack([(residual(qref+.1+np.eye(7)[k]*epsilon, True)-residual(qref+.1-np.eye(7)[k]*epsilon, True))/(2*epsilon) for k in range(7)])
    validation = dict(FK_max_difference=float(max(np.max(np.abs(pp-repository[:3, 3])), np.max(np.abs(rr-repository[:3, :3])))),
        jacobian_finite_difference_max_difference=float(np.max(np.abs(jacobian(qref+.1, True)-numeric))))
    output = dict(schema=1, motion_issued=False, network_accessed=False, hardware_accessed=False,
        runtime_sources_or_models_modified=False, executable=False, target_board_mm=target_board.tolist(),
        physical_gap_is_approximate_visual_lower_bound=True, measured_physical_contact_Z_mm=None,
        original_nominal_A8_geometry_consistency_resolved=False, software_bounds_are_not_mechanical_limits=True,
        scipy_version=scipy.__version__, dependency_source='pre-existing tmp/gripper_geometry_deps; no installation',
        method='Vectorized DE/rand/1/bin plus analytic-Jacobian bounded least squares; independent SLSQP orientation and paired-bound selection',
        ablations=results, nearest_orientation_under_original_bounds_and_scene=selection,
        local_minimum_paired_front_roll_extension=paired_report, validation=validation,
        minimum_orientation_with_only_front_relaxed30deg=front_orientation,
        smallest_demonstrated_front_extension_with_bounded_new_frame=front_cones,
        exact_fixed_original_orientation_all_full_periods=constrained_fixed_report,
        source_sha256={str(p.relative_to(ROOT)).replace('\\', '/'):sha(p) for p in (mp, cp, FROZEN/'left_arm_rigid_body.py', FROZEN/'left_arm_board_kinematics.py', FROZEN/'left_arm_board_collision.py')},
        evaluations=evaluations, elapsed_s=time.monotonic()-start,
        limitations=['Finite search and local constrained optimization do not prove global infeasibility or globally minimal changes.',
            'Relaxed bounds are offline mathematical domains only, not allowable hardware travel.',
            'Selected new orientation must be frozen over a separately recomputed strict descent; no full executable route is produced.',
            'CAD-scene feasibility is not physical reachability, gripping success, contact sensing, or calibrated height verification.'])
    (OUT/'height_reach_ablation.json').write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf8')
    print('RESULT '+json.dumps(dict(paired_extension=paired_report, nearest_orientation=selection,
        front_only_new_orientation=front_orientation, front_only_bounded_frame=front_cones,
        exact_original_orientation_all_periods=constrained_fixed_report,
        elapsed_s=output['elapsed_s'], validation=validation), ensure_ascii=True))


if __name__ == '__main__':
    main()
