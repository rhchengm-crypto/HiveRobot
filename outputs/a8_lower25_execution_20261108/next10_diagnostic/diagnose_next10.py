"""Offline next-10mm endpoint diagnostic, unchanged authorized-trial domain.

No active config/source changes, hardware/network, motor interface, execution
stamp, or movement. Fixed new trial frame; exact same trial software limits.
At most12 endpoint IK starts; stop on endpoint failure without path planning.
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
    starttime=time.monotonic()
    cp,mp=TRIAL/'common_trial_config.json',SCRIPTS/'data/left_arm_general_dynamics.json'
    prep=read(TRIAL/'preparation.json')
    assert sha(cp)==prep['board_model_sha256']
    plan=read(TRIAL/'local_plan.json')
    assert sha(TRIAL/'local_plan.json')==prep['local_plan_sha256']
    m=b.RigidBodyModel.load(mp);cfg=b.load_board_config(cp,m)
    bounds=np.array(cfg['joint_limits_encoder_rad']);tcp=np.array(cfg['tcp_in_wrist_frame_m'])
    rb,tb=np.array(cfg['board_to_cad_rotation']),np.array(cfg['board_to_cad_translation_m'])
    desired=rb@np.array(plan['selected_wrist_rotation_in_board'])
    target_board=np.array([11.798050592563328,405.6006774366655,49.622674457482844])
    target=rb@(target_board/1000)+tb
    scene=b.collision_scene(m,cfg)
    qlast=np.array([plan['waypoints'][-1]['encoder_rad'][n] for n in m.names])
    qmodel=m.encoder_to_model(qlast)
    jv,jw=m.point_jacobian(qmodel,m.n-1,tcp)
    task=np.vstack((jv*m.signs,.15*jw*m.signs))
    delta=np.linalg.pinv(task)@np.r_[target-tcp_pose(m,qlast,tcp)[:3,3],np.zeros(3)]
    predicted=qlast+delta
    rng=np.random.default_rng(20261109)
    seeds=[qlast,predicted]
    seeds.extend(qlast+rng.normal(0,scale,7) for scale in (.03,.03,.06,.06,.10,.10))
    seeds.extend((np.array([cfg['profiles']['white_knight']['seed_encoder_rad'][n] for n in m.names]),
                  np.mean(bounds,axis=1),bounds[:,0]+.25*(bounds[:,1]-bounds[:,0]),bounds[:,0]+.75*(bounds[:,1]-bounds[:,0])))
    attempts=[];solutions=[]
    for index,seed in enumerate(seeds):
        answer=solve_ik(m,target,np.clip(seed,bounds[:,0],bounds[:,1]),tcp,bounds,desired,
            max_iterations=180,position_tolerance_m=.0005,rotation_tolerance_rad=.005,clearance_function=scene)
        q=np.array([answer['encoder_rad'][n] for n in m.names]);pose=tcp_pose(m,q,tcp)
        gaps=scene(q)
        try:
            b.check_board_intersection(m,q,cfg);scene_pass=True;scene_failure=None
        except b.BoardPlanningError as exc:
            scene_pass=False;scene_failure=exc.diagnostics
        pe=float(np.linalg.norm(pose[:3,3]-target)*1000)
        re=float(np.linalg.norm(rotation_error(desired,pose[:3,:3])))
        within=bool(np.all(q>=bounds[:,0]-1e-9) and np.all(q<=bounds[:,1]+1e-9))
        passed=bool(pe<=.5 and re<=.005 and scene_pass and within)
        row=dict(seed_index=index,solver_success=answer['success'],passed=passed,iterations=answer['iterations'],
            position_error_mm=pe,strict_frame_error_rad=re,strict_frame_error_deg=math.degrees(re),
            predicted_board_mm=((pose[:3,3]-tb)@rb*1000).tolist(),
            encoder_rad=dict(zip(m.names,q.tolist())),bounds_passed=within,
            active_limits={n:('lower' if abs(q[i]-bounds[i,0])<1e-5 else 'upper') for i,n in enumerate(m.names)
                if min(abs(q[i]-bounds[i,0]),abs(q[i]-bounds[i,1]))<1e-5},
            common_scene_and_gripper_passed=scene_pass,scene_failure=scene_failure,
            minimum_scene_margin_mm=float(np.min(gaps)*1000))
        attempts.append(row);solutions.append(q)
        print(json.dumps(dict(phase='endpoint',**row)),flush=True)
        if passed:
            break
    best_index=min(range(len(attempts)),key=lambda i:attempts[i]['position_error_mm']**2+(.15*1000*attempts[i]['strict_frame_error_rad'])**2)
    best=attempts[best_index];q=solutions[best_index]
    report=dict(schema=1,motion_issued=False,network_accessed=False,hardware_accessed=False,
        active_config_or_sources_modified=False,executable=False,prepared_stamp_produced=False,
        target_board_mm=target_board.tolist(),previous_test_target_board_mm=plan['target_board_mm'],
        next_requested_height_change_mm=-10.,XY_unchanged=True,
        measured_physical_gap_is_approximate=True,calibrated_contact_datum_mm=None,
        new_trial_frame_frozen=True,desired_wrist_rotation_in_cad=desired.tolist(),
        position_tolerance_mm=.5,strict_full_frame_tolerance_rad=.005,
        joint_limits_encoder_rad=dict(zip(m.names,bounds.tolist())),bounds_or_orientation_relaxed=False,
        attempts=attempts,best=best,endpoint_passed=best['passed'],
        endpoint_method='At most12 deterministic bounded damped IK starts; same strict full-frame/scene; finite diagnostic not a global reach proof.',
        local_linear_prediction_unclipped_encoder_rad=dict(zip(m.names,predicted.tolist())),
        source_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in (cp,mp,TRIAL/'local_plan.json',
            SCRIPTS/'left_arm_board_grasp.py',SCRIPTS/'left_arm_board_kinematics.py',SCRIPTS/'left_arm_board_collision.py',Path(__file__))},
        full_reverse_branch_attempted=False,Clearance_transit_attempted=False,
        limitations=['Finite multi-start failure is not proof of global infeasibility or physical unreachability.',
            'Visual remaining gap is not a calibrated TCP/contact height measurement.',
            'No further bounds, orientation, geometry, XYZ offset or gain change was allowed.',
            'No movement or next-test authorization is implied. A future test requires Home and fresh ordinary Clearance, not direct descent from present endpoint.'])
    if not best['passed']:
        report['status']='endpoint-failed-no-path-prepared'
        report['failure_reason']='No start met BOTH0.5mm Cartesian and0.005rad fixed-frame limits under unchanged trial bounds and common scene.'
    else:
        # Only a passing endpoint earns one deterministic complete reverse line.
        reverse=[dict(target_board_mm=target_board.tolist(),encoder_rad=dict(zip(m.names,q.tolist())))]
        height=max(float(cfg['default_clearance_mm']),target_board[2]+50.)
        current=float(target_board[2]);report['full_reverse_branch_attempted']=True
        report['status']='reverse-branch-in-progress'
        while current<height-1e-8:
            z=min(height,current+4.);point=np.array([target_board[0],target_board[1],z])
            answer=solve_ik(m,rb@(point/1000)+tb,q,tcp,bounds,desired,max_iterations=180,
                position_tolerance_m=.0005,rotation_tolerance_rad=.005,clearance_function=scene)
            nq=np.array([answer['encoder_rad'][n] for n in m.names])
            if not answer['success'] or np.max(np.abs(nq-q))>math.radians(15):
                report['status']='first-reverse-IK-failed';report['failure_reason']=dict(from_height_mm=current,target_board_mm=point.tolist(),answer=answer);break
            try:
                b.check_transit_edge(m,q,nq,cfg)
            except b.BoardPlanningError as exc:
                report['status']='first-reverse-edge-failed';report['failure_reason']=exc.diagnostics;break
            reverse.append(dict(target_board_mm=point.tolist(),encoder_rad=dict(zip(m.names,nq.tolist()))));q,current=nq,z
        write(OUT/'reverse_branch.json',reverse)
        if current>=height-1e-8:
            report['status']='reverse-branch-passed-not-executable'
            report['reverse_branch_samples']=len(reverse)
            report['Clearance_transit_attempted']=True
            forecast=read(TRIAL/'forecast_clearance.json')
            q0=np.array([forecast[n] for n in m.names])
            route,blocked=b.find_entry_transit(m,q0,[(float(np.linalg.norm(q-q0)),q)],cfg)
            report['historical_Home_Clearance_forecast_used_only']=True
            report['future_fresh_Home_and_Clearance_required']=True
            report['forecast_source']='previous completed Home/fresh preflight for last trial; not present endpoint or new live state'
            report['Clearance_transit_passed']=route is not None
            report['Clearance_blocked_paths']=blocked
            if route is not None:
                write(OUT/'historical_forecast_transit.json',[dict(stage=stage,encoder_rad=dict(zip(m.names,v.tolist()))) for stage,v in route])
            else:
                report['status']='Clearance-transit-failed';report['failure_reason']=blocked
    report['elapsed_s']=time.monotonic()-starttime
    write(OUT/'diagnostic.json',report)
    print(json.dumps(dict(status=report['status'],endpoint_passed=best['passed'],best_position_error_mm=best['position_error_mm'],
        best_frame_error_deg=best['strict_frame_error_deg'],attempts=len(attempts),elapsed_s=report['elapsed_s'])),flush=True)


if __name__=='__main__':
    main()
