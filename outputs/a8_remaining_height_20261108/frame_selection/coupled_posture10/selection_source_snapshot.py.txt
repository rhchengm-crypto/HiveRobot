"""Offline-only new-frame selection for a further 10 mm from the held pose.

Existing scoped trial bounds and CAD scene only. No SDK, network, persistent
configuration updates, execution stamp, or motor operations are used.
"""
import os
for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
import hashlib
import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
EVIDENCE = ROOT/'outputs/a8_lower25_execution_20261108/control_evidence'
sys.path.insert(0, str(ROOT/'tmp/gripper_geometry_deps'))
sys.path.insert(0, str(EVIDENCE/'source'))
import numpy as np
from scipy.optimize import minimize
import scipy
import left_arm_board_grasp as board
from left_arm_board_kinematics import tcp_pose, rotation_error, solve_ik
from left_arm_rigid_body import axis_rotation


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(name, value):
    (OUT/name).write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--additional-front-deg',type=float,default=0.)
    parser.add_argument('--coupled-posture-10deg',action='store_true')
    parser.add_argument('--label',default='')
    args=parser.parse_args()
    if args.additional_front_deg not in (0.,5.,8.):
        raise ValueError('Only original12 and requested counterfactual totals17/20 are allowed')
    if args.label:
        if args.label not in ('front_total17','front_total20','coupled_posture10'):
            raise ValueError('Unexpected output label')
        OUT=OUT/args.label;OUT.mkdir(parents=True,exist_ok=True)
    began = time.monotonic()
    model_path = EVIDENCE/'models/left_arm_general_dynamics.json'
    config_path = EVIDENCE/'models/left_arm_board_workspace_candidate.json'
    audit_path = ROOT/'outputs/a8_lower25_execution_20261108/execution_audit.json'
    audit = read(audit_path)
    model, cfg = board.RigidBodyModel.load(model_path), read(config_path)
    held = audit['inline_hold']['final']
    q0 = np.array([held[name]['pos'] for name in model.names])
    # An explicit offline counterfactual only; never write this configuration.
    original_trial_bounds=np.asarray(cfg['joint_limits_encoder_rad']).copy()
    cfg['joint_limits_encoder_rad'][model.names.index('shoulder_front')][0] -= math.radians(args.additional_front_deg)
    if args.coupled_posture_10deg:
        if args.additional_front_deg:
            raise ValueError('Coupled posture proposal must keep front at original trial12deg')
        cfg['joint_limits_encoder_rad'][model.names.index('shoulder_rotate')][0] -= math.radians(10.)
        cfg['joint_limits_encoder_rad'][model.names.index('arm_roll')][1] += math.radians(10.)
    bounds = np.asarray(cfg['joint_limits_encoder_rad'])
    tcp = np.asarray(cfg['tcp_in_wrist_frame_m'])
    rb, tb = np.asarray(cfg['board_to_cad_rotation']), np.asarray(cfg['board_to_cad_translation_m'])
    held_pose = tcp_pose(model, q0, tcp)
    initial, reference = held_pose[:3, 3], held_pose[:3, :3]
    initial_board = (initial-tb)@rb*1000.
    target_board = initial_board.copy(); target_board[2] -= 10.
    target = rb@(target_board/1000.)+tb
    local_axis = np.asarray(cfg['tool_axis_in_wrist_frame']); local_axis /= np.linalg.norm(local_axis)
    reference_axis = reference@local_axis
    scene = board.collision_scene(model, cfg)
    goal = np.array([audit['endpoint']['nominal_target_rad'][name] for name in model.names])
    inputs = {
        'motion_issued':False, 'network_accessed':False, 'active_configuration_changed':False,
        'scope':'Existing lower25 scoped trial bounds, not certified mechanical travel.',
        'source_files':{str(p.relative_to(ROOT)).replace('\\','/'):sha(p)
                        for p in (model_path, config_path, audit_path, Path(__file__))},
        'scipy_version':scipy.__version__,
        'held_encoder_rad':dict(zip(model.names, q0.tolist())),
        'held_model_board_mm':initial_board.tolist(), 'requested_board_mm':target_board.tolist(),
        'held_wrist_rotation_in_cad':reference.tolist(),
        'bounds_encoder_rad':dict(zip(model.names, bounds.tolist())),
        'front_total_software_extension_from_original_common_deg':12.+args.additional_front_deg,
        'front_additional_software_extension_from_completed_trial_deg':args.additional_front_deg,
        'other_six_bounds_unchanged':not args.coupled_posture_10deg,
        'unchanged_axes':[name for name in model.names if name not in (
            {'shoulder_rotate','arm_roll'} if args.coupled_posture_10deg else {'shoulder_front'})],
        'coupled_posture_counterfactual':args.coupled_posture_10deg,
        'counterfactual_rotation_lower_extension_deg':10. if args.coupled_posture_10deg else 0.,
        'counterfactual_roll_upper_extension_deg':10. if args.coupled_posture_10deg else 0.,
        'mechanical_qualification':False,
        'strict_position_tolerance_mm':.5, 'strict_new_frame_tolerance_rad':.005,
        'model_predictions_not_physical_height_or_XY_measurements':True,
    }
    # Keep the exact analysis implementation with each numerical result.
    (OUT/'selection_source_snapshot.py.txt').write_bytes(Path(__file__).read_bytes())
    write('inputs.json', inputs)

    def fk(q):
        frames = model.fk(model.encoder_to_model(q))
        rr = frames[-1, :3, :3]
        position = frames[-1, :3, 3]+rr@tcp
        axes = frames[:, :3, 2]
        jp = np.cross(axes, position-frames[:, :3, 3]).T*model.signs[None, :]
        derivative = []
        for axis, sign in zip(axes, model.signs):
            x,y,z = axis
            skew = np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
            derivative.append((sign*skew@rr).ravel())
        return position, rr, jp, np.asarray(derivative).T

    def full(q):
        _, rr, _, jr = fk(q)
        delta = (rr-reference).ravel()
        return .5*float(delta@delta), jr.T@delta

    def assessment(solution, budget=None):
        q = solution.x
        position, rr, _, _ = fk(q)
        angle = math.degrees(float(np.linalg.norm(rotation_error(reference, rr))))
        axis_angle = math.degrees(math.acos(float(np.clip((rr@local_axis)@reference_axis,-1,1))))
        gaps = scene(q)
        try:
            board.check_board_intersection(model,q,cfg)
            common_scene = True; collision = None
        except board.BoardPlanningError as exc:
            common_scene = False; collision = exc.diagnostics
        error = float(np.linalg.norm(position-target)*1000.)
        within = bool(np.all(q>=bounds[:,0]-1e-9) and np.all(q<=bounds[:,1]+1e-9))
        return {
            'optimizer_success':bool(solution.success),'message':str(solution.message),
            'iterations':solution.nit,'encoder_rad':dict(zip(model.names,q.tolist())),
            'model_board_mm':((position-tb)@rb*1000.).tolist(),'position_error_mm':error,
            'full_frame_change_from_held_deg':angle,'axis_change_from_held_deg':axis_angle,
            'maximum_new_frame_change_deg':budget,'scene_minimum_clearance_mm':float(np.min(gaps)*1000),
            'scene_passed':common_scene and bool(np.min(gaps)>=-1e-9),'scene_failure':collision,
            'bounds_passed':within,'active_bounds':[name for index,name in enumerate(model.names)
                if min(q[index]-bounds[index,0],bounds[index,1]-q[index])<1e-5],
            'actual_bound_extensions_from_completed_trial_deg':{
                name:{'below':float(np.degrees(max(0.,original_trial_bounds[index,0]-q[index]))),
                      'above':float(np.degrees(max(0.,q[index]-original_trial_bounds[index,1])))}
                for index,name in enumerate(model.names)
                if q[index]<original_trial_bounds[index,0] or q[index]>original_trial_bounds[index,1]},
            'selected_new_wrist_rotation_in_cad':rr.tolist(),
            'selected_new_wrist_rotation_in_board':(rb.T@rr).tolist(),
            'endpoint_only_passed':bool(error<=.5 and common_scene and np.min(gaps)>=-1e-9
                and within and (budget is None or angle<=budget+1e-6)),
            'executable':False,'globally_optimal_proven':False,
        }

    def constraints(budget=None):
        result = [
            {'type':'eq','fun':lambda q:(fk(q)[0]-target)*1000.,'jac':lambda q:fk(q)[2]*1000.},
            {'type':'ineq','fun':lambda q:scene(q)*1000.,'jac':lambda q:scene.jacobian(q)*1000.},
        ]
        if budget is not None:
            result.append({'type':'ineq','fun':lambda q:2*(1-math.cos(math.radians(budget)))-full(q)[0],
                           'jac':lambda q:-full(q)[1]})
        return result

    seeds = [q0,goal]
    crosscheck=Path(__file__).resolve().parent/'free_orientation_position_crosscheck.json'
    if crosscheck.exists():
        warm=read(crosscheck)['best']['encoder_rad']
        seeds.append(np.array([warm[name] for name in model.names]))
    if args.coupled_posture_10deg:
        # Seed selection using the exact held frame is a numerical proposal,
        # not a change to the final requested position or tracking tolerance.
        for seed in (q0,goal):
            exact=solve_ik(model,target,seed,tcp,bounds,reference,max_iterations=300,
                position_tolerance_m=.0001,rotation_tolerance_rad=.001,clearance_function=scene)
            seeds.append(np.array([exact['encoder_rad'][name] for name in model.names]))
    minimum = []
    for index, seed in enumerate(seeds):
        solution = minimize(lambda q:full(q)[0], seed, jac=lambda q:full(q)[1],
            method='SLSQP',bounds=list(map(tuple,bounds)),constraints=constraints(),
            options={'maxiter':300,'ftol':1e-12})
        record = assessment(solution);record['seed_index']=index;minimum.append(record)
        print(json.dumps({'phase':'minimum-new-frame','result':record}),flush=True)
    best = min((row for row in minimum if row['endpoint_only_passed']),
               key=lambda row:row['full_frame_change_from_held_deg'], default=None)
    if best is not None:
        seeds.append(np.array([best['encoder_rad'][name] for name in model.names]))
    bounded = []
    for budget in (1.,2.,3.,5.):
        trials = []
        for index,seed in enumerate(seeds):
            solution = minimize(lambda q:float((q-q0)@(q-q0)),seed,jac=lambda q:2*(q-q0),
                method='SLSQP',bounds=list(map(tuple,bounds)),constraints=constraints(budget),
                options={'maxiter':200,'ftol':1e-12})
            record=assessment(solution,budget);record['seed_index']=index;trials.append(record)
        bounded.append({'budget_deg':budget,'trials':trials,
            'best':min(trials,key=lambda row:(not row['endpoint_only_passed'],row['position_error_mm']))})
        print(json.dumps({'phase':'bounded-new-frame','budget_deg':budget,'best':bounded[-1]['best']}),flush=True)
        write('selection_progress.json',{'minimum_frame_trials':minimum,'bounded_trials':bounded})

    # Prefer the nearest explicit full frame; this is endpoint selection only.
    selected = best
    transition = []
    branch = []
    failure = None
    if selected is not None:
        desired = np.array(selected['selected_new_wrist_rotation_in_cad'])
        rotvec = rotation_error(desired, reference)
        radians = float(np.linalg.norm(rotvec))
        steps=max(1,math.ceil(math.degrees(radians)))
        previous=q0.copy()
        for index in range(1,steps+1):
            requested = axis_rotation(rotvec/radians, radians*index/steps)@reference if radians>1e-12 else reference
            answer=solve_ik(model,initial,previous,tcp,bounds,requested,max_iterations=200,
                position_tolerance_m=.0002,rotation_tolerance_rad=.002,clearance_function=scene)
            q=np.array([answer['encoder_rad'][name] for name in model.names])
            row={'stage':'orientation_at_held_XYZ','fraction':index/steps,
                'target_board_mm':initial_board.tolist(),'requested_wrist_rotation_in_cad':requested.tolist(),
                'position_error_mm':answer['position_error_mm'],'rotation_error_deg':answer['rotation_error_deg'],
                'success':answer['success'],'encoder_rad':answer['encoder_rad'],
                'maximum_joint_step_deg':float(np.degrees(np.max(np.abs(q-previous))))}
            try:
                if not answer['success']:raise ValueError('strict in-place orientation IK failed')
                board.check_transit_edge(model,previous,q,cfg)
                row['complete_edge_checked']=True
            except (board.BoardPlanningError,ValueError) as exc:
                failure={'stage':'orientation-transition','error':str(exc),'details':getattr(exc,'diagnostics',None)}
                row['complete_edge_checked']=False
            transition.append(row)
            if failure:break
            previous=q
        if failure is None:
            for index in range(1,11):
                point=initial_board.copy();point[2]-=index
                answer=solve_ik(model,rb@(point/1000.)+tb,previous,tcp,bounds,desired,
                    max_iterations=200,position_tolerance_m=.0005,rotation_tolerance_rad=.005,
                    clearance_function=scene)
                q=np.array([answer['encoder_rad'][name] for name in model.names])
                row={'stage':'new-frame-vertical-descent','target_board_mm':point.tolist(),
                    'position_error_mm':answer['position_error_mm'],'rotation_error_deg':answer['rotation_error_deg'],
                    'success':answer['success'],'encoder_rad':answer['encoder_rad'],
                    'maximum_joint_step_deg':float(np.degrees(np.max(np.abs(q-previous))))}
                try:
                    if not answer['success']:raise ValueError('strict new-frame vertical IK failed')
                    if np.max(np.abs(q-previous))>math.radians(12):raise ValueError('joint continuity step exceeds 12deg')
                    board.check_transit_edge(model,previous,q,cfg)
                    row['complete_edge_checked']=True
                except (board.BoardPlanningError,ValueError) as exc:
                    failure={'stage':'vertical-descent','error':str(exc),'details':getattr(exc,'diagnostics',None)}
                    row['complete_edge_checked']=False
                branch.append(row)
                if failure:break
                previous=q
    report={'inputs':inputs,'minimum_frame_trials':minimum,'bounded_trials':bounded,
        'selected_new_frame_endpoint':selected,'orientation_transition':transition,
        'strict_new_frame_vertical_branch':branch,'failure':failure,
        'complete_orientation_transition_plus_10mm_descent':bool(selected and not failure and len(branch)==10),
        'motion_issued':False,'network_accessed':False,'runtime_or_models_modified':False,
        'physical_validation_complete':False,'executable':False,
        'elapsed_s':time.monotonic()-began,
        'limitations':['Local numerical candidates, not proof of global reachability or certified mechanical travel.',
            'No claim of measured physical descent or lateral accuracy from encoder FK.',
            'A selected frame is a new explicit task, not a relaxed tracking tolerance.',
            'Current-pose orientation transition and every descent edge must pass before any full-path claim.']}
    write('frame_selection_report.json',report)
    print(json.dumps({'output':str(OUT/'frame_selection_report.json'),
        'minimum_frame_deg':None if best is None else best['full_frame_change_from_held_deg'],
        'complete':report['complete_orientation_transition_plus_10mm_descent'],
        'failure':failure,'elapsed_s':report['elapsed_s']}),flush=True)


if __name__=='__main__':
    main()
