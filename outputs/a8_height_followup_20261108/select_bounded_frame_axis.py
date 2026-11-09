"""Reproducible, offline-only bounded full-frame/tool-axis selection.

Only the front software lower bound is extended in memory. No model/config
write, network, motor interface, or executable trajectory is involved.
"""
import os
for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[k] = '1'
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
from scipy.optimize import minimize
import scipy
import left_arm_board_grasp as b
from left_arm_board_kinematics import tcp_pose, rotation_error


def read(p):
    return json.loads(Path(p).read_text(encoding='utf8'))


def main():
    started = time.monotonic()
    mp = FROZEN/'data/left_arm_general_dynamics.json'
    cp = FROZEN/'data/left_arm_board_workspace_candidate.json'
    m, cfg = b.RigidBodyModel.load(mp), read(cp)
    names, n = m.names, m.n
    scene = b.collision_scene(m, cfg)
    current = np.array(cfg['joint_limits_encoder_rad'])
    bounds = current.copy()
    front = names.index('shoulder_front')
    bounds[front, 0] -= math.radians(30.)
    tcp = np.array(cfg['tcp_in_wrist_frame_m'])
    rb, tb = np.array(cfg['board_to_cad_rotation']), np.array(cfg['board_to_cad_translation_m'])
    target_board = np.array([11.798050592563328, 405.6006774366655, 59.622674457482844])
    target = rb@(target_board/1000)+tb
    saved = read(ROOT/'outputs/a8_saved_reference_20261108/saved_pose_audit.json')['saved_move']['pose']
    qref = np.array([saved[k] for k in names])
    rref = tcp_pose(m, qref, tcp)[:3, :3]
    local_axis = np.array(cfg['tool_axis_in_wrist_frame'])
    local_axis /= np.linalg.norm(local_axis)
    aref = rref@local_axis
    evaluations = 0

    def fk(q):
        nonlocal evaluations
        evaluations += 1
        frames = m.fk(m.encoder_to_model(q))
        rr = frames[-1, :3, :3]
        pos = frames[-1, :3, 3]+rr@tcp
        axes = frames[:, :3, 2]
        jp = np.cross(axes, pos-frames[:, :3, 3]).T*m.signs[None, :]
        derivative = []
        for axis, sign in zip(axes, m.signs):
            x, y, z = axis
            skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
            derivative.append((sign*skew@rr).ravel())
        jr = np.array(derivative).T
        actual_axis = rr@local_axis
        jaxis = np.cross(axes, actual_axis).T*m.signs[None, :]
        return pos, rr, jp, jr, actual_axis, jaxis

    def full(q):
        p, rr, jp, jr, aa, ja = fk(q)
        dr = (rr-rref).ravel()
        return .5*float(dr@dr), jr.T@dr

    def axis(q):
        p, rr, jp, jr, aa, ja = fk(q)
        return float(aa@aref), ja.T@aref

    def constraints(cone, axis_cone=None):
        result = [dict(type='eq', fun=lambda q:fk(q)[0]-target, jac=lambda q:fk(q)[2]),
            dict(type='ineq', fun=scene, jac=scene.jacobian),
            dict(type='ineq', fun=lambda q:2*(1-math.cos(math.radians(cone)))-full(q)[0],
                 jac=lambda q:-full(q)[1])]
        if axis_cone is not None:
            result.append(dict(type='ineq', fun=lambda q:axis(q)[0]-math.cos(math.radians(axis_cone)),
                               jac=lambda q:axis(q)[1]))
        return result

    def assess(solution, cone, axis_cone, objective):
        q = solution.x
        pp, rr, jp, jr, aa, ja = fk(q)
        poserr = float(np.linalg.norm(pp-target)*1000)
        full_angle = float(np.degrees(np.linalg.norm(rotation_error(rref, rr))))
        axis_angle = float(np.degrees(math.acos(float(np.clip(aa@aref, -1, 1)))))
        gaps = scene(q)
        within_other = bool(np.all(q[1:] >= current[1:, 0]-1e-8) and np.all(q[1:] <= current[1:, 1]+1e-8))
        try:
            b.check_board_intersection(m, q, cfg)
            full_scene = True
            details = None
        except b.BoardPlanningError as exc:
            full_scene, details = False, exc.diagnostics
        passed = poserr <= .5 and full_angle <= cone+1e-6 and (axis_cone is None or axis_angle <= axis_cone+1e-6) and np.min(gaps) >= -1e-8 and full_scene and within_other
        return dict(objective=objective, optimizer_success=bool(solution.success), optimizer_message=str(solution.message),
            nfev=solution.nfev, encoder_rad=dict(zip(names, q.tolist())),
            predicted_board_mm=((pp-tb)@rb*1000).tolist(), position_error_mm=poserr,
            original_full_frame_change_deg=full_angle, original_tool_axis_change_deg=axis_angle,
            maximum_requested_full_frame_change_deg=cone, maximum_requested_tool_axis_change_deg=axis_cone,
            front_additional_lower_travel_deg=float(np.degrees(max(0, current[front, 0]-q[front]))),
            no_other_six_bound_changes=within_other, scene_minimum_clearance_mm=float(np.min(gaps)*1000),
            common_scene_and_gripper_passed=full_scene, common_scene_failure=details,
            selected_wrist_rotation_in_cad=rr.tolist(), selected_tool_axis_in_cad=aa.tolist(),
            mathematically_passed=bool(passed), executable=False, globally_optimal_proven=False)

    prior = read(OUT/'height_reach_ablation.json')
    seeds = [np.array([row['encoder_rad'][n] for n in names]) for row in prior['minimum_orientation_with_only_front_relaxed30deg']]
    for row in prior['ablations']:
        if row['domain'] == 'front_minus10' and not row['fixed_original_full_orientation']:
            seeds.append(np.array([row['best']['assessment']['encoder_rad'][n] for n in names]))
    results = []
    # The requested case, then its independent nearest-axis obstruction check.
    for cone in (15., 20.):
        for index, seed in enumerate(seeds):
            solution = minimize(lambda q:-q[front], seed, jac=lambda q:-np.eye(n)[front], method='SLSQP',
                bounds=list(map(tuple, bounds)), constraints=constraints(cone, 10.),
                options=dict(maxiter=300, ftol=1e-12))
            result = assess(solution, cone, 10., 'minimum front extension with full frame and tool-axis budgets')
            result['seed_index'] = index
            results.append(result)
            print(json.dumps(dict(phase='bounded_front', result=result)), flush=True)
        for index, seed in enumerate(seeds[:2]):
            solution = minimize(lambda q:1-axis(q)[0], seed, jac=lambda q:-axis(q)[1], method='SLSQP',
                bounds=list(map(tuple, bounds)), constraints=constraints(cone),
                options=dict(maxiter=300, ftol=1e-12))
            result = assess(solution, cone, None, 'minimum tool-axis change with bounded full frame')
            result['seed_index'] = index
            results.append(result)
            print(json.dumps(dict(phase='axis_obstruction', result=result)), flush=True)
    # One bounded coupled-axis counterfactual, requested after front-only fails.
    # Keep the other five current limits and all scene constraints unchanged.
    coupled_bounds = bounds.copy()
    roll = names.index('arm_roll')
    coupled_bounds[roll, 1] += math.radians(30.)
    seed_report = min((r for r in results if r['maximum_requested_full_frame_change_deg'] == 15.
        and r['objective'].startswith('minimum tool-axis')), key=lambda r:r['original_tool_axis_change_deg'])
    coupled_seed = np.array([seed_report['encoder_rad'][k] for k in names])
    def extension_cost(q):
        ef, er = max(0., current[front, 0]-q[front]), max(0., q[roll]-current[roll, 1])
        grad = np.zeros(n)
        grad[front], grad[roll] = -2*ef, 2*er
        return ef*ef+er*er, grad
    coupled_solution = minimize(lambda q:extension_cost(q)[0], coupled_seed,
        jac=lambda q:extension_cost(q)[1], method='SLSQP', bounds=list(map(tuple, coupled_bounds)),
        constraints=constraints(15., 10.), options=dict(maxiter=400, ftol=1e-12))
    coupled = assess(coupled_solution, 15., 10., 'minimum squared front-lower and roll-upper extensions; one bounded local solve')
    coupled['roll_additional_upper_travel_deg'] = float(np.degrees(max(0., coupled_solution.x[roll]-current[roll, 1])))
    remaining = [i for i in range(n) if i not in (front, roll)]
    coupled['no_other_five_bound_changes'] = bool(np.all(coupled_solution.x[remaining] >= current[remaining, 0]-1e-8) and np.all(coupled_solution.x[remaining] <= current[remaining, 1]+1e-8))
    coupled['mathematically_passed_in_coupled_domain'] = bool(coupled['position_error_mm'] <= .5
        and coupled['original_full_frame_change_deg'] <= 15.+1e-6
        and coupled['original_tool_axis_change_deg'] <= 10.+1e-6
        and coupled['common_scene_and_gripper_passed'] and coupled['no_other_five_bound_changes'])
    print(json.dumps(dict(phase='one_coupled_front_roll_counterfactual', result=coupled)), flush=True)
    eps = 1e-6
    qcheck = seeds[0]
    numeric_axis = np.array([(axis(qcheck+eps*np.eye(n)[i])[0]-axis(qcheck-eps*np.eye(n)[i])[0])/(2*eps) for i in range(n)])
    numeric_full = np.array([(full(qcheck+eps*np.eye(n)[i])[0]-full(qcheck-eps*np.eye(n)[i])[0])/(2*eps) for i in range(n)])
    passed = [r for r in results if r['mathematically_passed'] and r['maximum_requested_tool_axis_change_deg'] == 10.]
    output = dict(schema=1, motion_issued=False, hardware_accessed=False, network_accessed=False,
        runtime_sources_or_models_modified=False, executable=False, target_board_mm=target_board.tolist(),
        selected_candidates=passed, selection_results=results,
        one_coupled_front_roll_counterfactual=coupled,
        tool_axis_in_wrist_frame=local_axis.tolist(),
        original_tool_axis_in_cad=aref.tolist(), original_reference_rotation_in_cad=rref.tolist(),
        method='Analytic-Jacobian SLSQP, three pre-existing exact-position branches; exact target position, common scene inequalities, explicit full-frame and tool-axis budgets; front-only lower software bound extended30deg in memory',
        scipy_version=scipy.__version__, evaluations=evaluations, elapsed_s=time.monotonic()-started,
        validation=dict(tool_axis_dot_jacobian_FD_max_error=float(np.max(np.abs(numeric_axis-axis(qcheck)[1]))),
                        frame_chordal_jacobian_FD_max_error=float(np.max(np.abs(numeric_full-full(qcheck)[1])))),
        source_sha256={str(p.relative_to(ROOT)).replace('\\', '/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in (mp, cp, FROZEN/'left_arm_rigid_body.py', FROZEN/'left_arm_board_kinematics.py', FROZEN/'left_arm_board_collision.py', Path(__file__))},
        limitations=['Finite local constrained search, not a global proof of infeasibility or minimum extension.',
            'Software envelope extension is an offline counterfactual, not verified mechanical travel or hardware eligibility.',
            'No physical contact datum, absolute TCP height calibration, or gripping success is established.',
            'No runtime activation or executable plan; any endpoint requires strict frozen-frame descent and full checked transit before an execution-readiness claim.'])
    (OUT/'bounded_frame_axis_selection.json').write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(dict(output=str(OUT/'bounded_frame_axis_selection.json'), passed_candidates=len(passed), elapsed_s=output['elapsed_s'])), flush=True)


if __name__ == '__main__':
    main()
