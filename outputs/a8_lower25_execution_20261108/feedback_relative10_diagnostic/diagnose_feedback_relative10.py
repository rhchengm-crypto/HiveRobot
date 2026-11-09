"""Offline actual-held-FK-relative10mm test; no hardware/activation/stamp.

Unlike the separately preserved request-relative diagnostic, use the final
same-serial hold encoder feedback, its model XY, and its actual FK wrist R.
The generated Cartesian target is a model target, not physical board metrology.
"""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[k]='1'
import hashlib
import json
import math
from pathlib import Path
import sys
import time
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent
TRIAL=OUT.parent
ROOT=TRIAL.parents[1]
SCRIPTS=TRIAL/'source_snapshot/scripts'
sys.path.insert(0,str(SCRIPTS))
import numpy as np
import left_arm_board_grasp as b
from left_arm_board_kinematics import tcp_pose,solve_ik,rotation_error


def read(p):
    return json.loads(Path(p).read_text(encoding='utf8'))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write(p,x):
    Path(p).write_text(json.dumps(x,indent=2,ensure_ascii=False),encoding='utf8')


def main():
    began=time.monotonic()
    ap=TRIAL/'execution_audit.json';audit=read(ap)
    assert audit['inline_hold']['ok'] and audit['inline_hold']['reason']=='duration_complete'
    cp,mp=TRIAL/'common_trial_config.json',SCRIPTS/'data/left_arm_general_dynamics.json'
    assert sha(cp)==read(TRIAL/'preparation.json')['board_model_sha256']
    m=b.RigidBodyModel.load(mp);cfg=b.load_board_config(cp,m)
    bounds=np.array(cfg['joint_limits_encoder_rad'])
    rb,tb,tcp=np.array(cfg['board_to_cad_rotation']),np.array(cfg['board_to_cad_translation_m']),np.array(cfg['tcp_in_wrist_frame_m'])
    q0=np.array([audit['inline_hold']['final'][n]['pos'] for n in m.names])
    pose0=tcp_pose(m,q0,tcp);desired=pose0[:3,:3];board0=(pose0[:3,3]-tb)@rb*1000
    np.testing.assert_allclose(board0,[9.639697000830376,406.65055148807653,62.17518852976162],atol=1e-8,rtol=0)
    target_board=board0.copy();target_board[2]-=10.
    target=rb@(target_board/1000)+tb;scene=b.collision_scene(m,cfg)
    scene(q0);b.check_board_intersection(m,q0,cfg)
    jv,jw=m.point_jacobian(m.encoder_to_model(q0),m.n-1,tcp)
    delta=np.linalg.pinv(np.vstack((jv*m.signs,.15*jw*m.signs)))@np.r_[target-pose0[:3,3],np.zeros(3)]
    rng=np.random.default_rng(20261110)
    seeds=[q0,q0+delta]
    seeds.extend(q0+rng.normal(0,scale,7) for scale in (.03,.03,.06,.06,.10,.10))
    plan=read(TRIAL/'local_plan.json')
    seeds.extend((np.array([plan['waypoints'][-1]['encoder_rad'][n] for n in m.names]),np.mean(bounds,axis=1),
        bounds[:,0]+.25*(bounds[:,1]-bounds[:,0]),bounds[:,0]+.75*(bounds[:,1]-bounds[:,0])))
    attempts=[];solutions=[]
    def assessment(q):
        pose=tcp_pose(m,q,tcp);pe=float(np.linalg.norm(pose[:3,3]-target)*1000)
        re=float(np.linalg.norm(rotation_error(desired,pose[:3,:3])))
        try:
            b.check_board_intersection(m,q,cfg);scene_pass=True;failure=None
        except b.BoardPlanningError as exc:
            scene_pass=False;failure=exc.diagnostics
        within=bool(np.all(q>=bounds[:,0]-1e-9) and np.all(q<=bounds[:,1]+1e-9))
        predicted=(pose[:3,3]-tb)@rb*1000
        return dict(passed=bool(pe<=.5 and re<=.005 and scene_pass and within),position_error_mm=pe,
            strict_frame_error_rad=re,strict_frame_error_deg=math.degrees(re),predicted_board_mm=predicted.tolist(),
            residual_board_mm=(predicted-target_board).tolist(),encoder_rad=dict(zip(m.names,q.tolist())),bounds_passed=within,
            common_scene_and_gripper_passed=scene_pass,scene_failure=failure,minimum_scene_margin_mm=float(np.min(scene(q))*1000),
            nearest_limit_margin_deg={n:dict(to_lower=math.degrees(float(q[i]-bounds[i,0])),to_upper=math.degrees(float(bounds[i,1]-q[i]))) for i,n in enumerate(m.names)})
    for index,seed in enumerate(seeds):
        answer=solve_ik(m,target,np.clip(seed,bounds[:,0],bounds[:,1]),tcp,bounds,desired,
            max_iterations=180,position_tolerance_m=.0005,rotation_tolerance_rad=.005,clearance_function=scene)
        q=np.array([answer['encoder_rad'][n] for n in m.names]);row=assessment(q)
        row.update(seed_index=index,solver_success=answer['success'],iterations=answer['iterations'])
        attempts.append(row);solutions.append(q)
        print(json.dumps(dict(phase='held-relative-endpoint',seed=index,position_error_mm=row['position_error_mm'],frame_error_deg=row['strict_frame_error_deg'],passed=row['passed'])),flush=True)
        if row['passed']:
            break
    bi=min(range(len(attempts)),key=lambda i:attempts[i]['position_error_mm']**2+(150*attempts[i]['strict_frame_error_rad'])**2)
    best=attempts[bi]
    report=dict(schema=1,motion_issued=False,hardware_accessed=False,network_accessed=False,executable=False,
        active_config_or_source_modified=False,execution_stamp_produced=False,physical_acceptance=None,
        target_definition='FK(actual final inline-hold feedback).boardXYZ + [0,0,-10]mm; complete frame frozen from the SAME actual feedback',
        actual_held_encoder_rad=dict(zip(m.names,q0.tolist())),actual_held_model_board_mm=board0.tolist(),
        target_board_mm=target_board.tolist(),desired_wrist_rotation_in_cad=desired.tolist(),
        orientation_source='actual inline_hold.final feedback FK, not previous requested frame',
        XY_held_model_coordinates_preserved=True,requested_model_vertical_change_mm=-10.,
        calibrated_physical_contact_datum_mm=None,model_target_is_not_measured_board_datum=True,
        operator_gap_is_approximate_visual_observation=True,joint_limits_encoder_rad=dict(zip(m.names,bounds.tolist())),
        bounds_orientation_geometry_or_gains_relaxed=False,position_tolerance_mm=.5,full_frame_tolerance_rad=.005,
        attempts=attempts,best=best,endpoint_passed=best['passed'],short_vertical_path_attempted=False,
        previous_request_relative_diagnostic_unchanged=True,
        previous_request_relative_target_board_mm=[11.798050592563328,405.6006774366655,49.622674457482844],
        source_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in (ap,cp,mp,
            SCRIPTS/'left_arm_board_kinematics.py',SCRIPTS/'left_arm_board_collision.py',SCRIPTS/'left_arm_board_grasp.py',Path(__file__))},
        limitations=['Finite12-start local IK diagnostic is not a global infeasibility proof or physical reachability result.',
            'Encoder FK height and feedback-relative target are model predictions, not measured board/contact datum.',
            'No new movement is authorized or prepared by this diagnostic. No runtime activation, stamp or hardware interface is used.'])
    if not best['passed']:
        report['status']='held-relative-endpoint-failed-no-path'
        report['failure_reason']='No bounded start met BOTH0.5mm position and0.005rad actual-held full-frame tolerance with the unchanged trial scene.'
    else:
        report['short_vertical_path_attempted']=True
        q=q0.copy();rows=[];failed=None
        for down in (2.,4.,6.,8.,10.):
            point=board0-np.array([0.,0.,down])
            answer=solve_ik(m,rb@(point/1000)+tb,q,tcp,bounds,desired,max_iterations=180,
                position_tolerance_m=.0005,rotation_tolerance_rad=.005,clearance_function=scene)
            nq=np.array([answer['encoder_rad'][n] for n in m.names]);step=math.degrees(float(np.max(np.abs(nq-q))))
            if not answer['success'] or step>15.:
                failed=dict(code='first-short-IK-failure',requested_down_mm=down,maximum_joint_step_deg=step,answer=answer);break
            try:
                b.check_transit_edge(m,q,nq,cfg)
            except b.BoardPlanningError as exc:
                failed=dict(code='first-short-edge-failure',requested_down_mm=down,details=exc.diagnostics);break
            rows.append(dict(target_board_mm=point.tolist(),encoder_rad=dict(zip(m.names,nq.tolist())),maximum_joint_step_deg=step))
            q=nq
        write(OUT/'short_vertical_diagnostic.json',dict(executable=False,start_encoder_rad=dict(zip(m.names,q0.tolist())),waypoints=rows,failure=failed))
        report['short_vertical_path_passed']=failed is None
        report['status']='held-relative-short-vertical-passed-in-model-only' if failed is None else 'first-short-vertical-failure'
        report['failure_reason']=failed
    report['elapsed_s']=time.monotonic()-began
    write(OUT/'diagnostic.json',report)
    print(json.dumps(dict(status=report['status'],endpoint_passed=best['passed'],attempts=len(attempts),best_position_error_mm=best['position_error_mm'],frame_error_deg=best['strict_frame_error_deg'],elapsed_s=report['elapsed_s'])),flush=True)


if __name__=='__main__':
    main()
