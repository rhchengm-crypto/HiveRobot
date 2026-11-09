"""Reverse geometry from an operator-approved endpoint, not a nominal-XYZ pass.

The saved pose is a boundary condition. All intermediate poses are new IK
solutions; the old board-XYZ request remains separately labelled unresolved.
This diagnostic neither releases a prepared plan nor opens hardware.
"""
import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import sys,json,time,copy,math,hashlib,collections
from pathlib import Path
import numpy as np
sys.path.insert(0,'scripts')
import left_arm_board_grasp as b
from left_arm_board_kinematics import tcp_pose,solve_ik,rotation_error
from left_arm_trajectory_support import gravity_envelope

out=Path('outputs/a8_saved_reference_20261108')
audit=json.loads((out/'saved_pose_audit.json').read_text(encoding='utf-8'))
preflight=json.loads(Path('outputs/a8_test_20261108/preflight.json').read_text(encoding='utf-8'))
m=b.RigidBodyModel.load('scripts/data/left_arm_general_dynamics.json')
active=b.load_board_config('scripts/data/left_arm_board_workspace_candidate.json',m)
cfg=copy.deepcopy(active)
goal=np.array([audit['saved_move']['pose'][n] for n in m.names])
bounds=np.array(cfg['joint_limits_encoder_rad'])
# Admit exactly the newly demonstrated endpoint into a diagnostic common
# observed envelope; do not invent a mechanical travel limit or extra margin.
bounds[:,0]=np.minimum(bounds[:,0],goal)
bounds[:,1]=np.maximum(bounds[:,1],goal)
cfg['joint_limits_encoder_rad']=bounds.tolist()
cfg['joint_bound_source']='offline union of existing operating envelope and operator-verified A8_take endpoint; no physical limit claim'
R=np.array(cfg['board_to_cad_rotation']);t=np.array(cfg['board_to_cad_translation_m'])
tcp=np.array(cfg['tcp_in_wrist_frame_m'])
end=tcp_pose(m,goal,tcp);desired=end[:3,:3]
contact=(end[:3,3]-t)@R*1000
start=copy.deepcopy(preflight['forecast_clearance'])
# Ordinary Clearance preserves the live shoulder_rotate. Captured A8_take
# is the most recent evidence here, but the future real start must be read anew.
start['shoulder_rotate']=audit['saved_move']['pose']['shoulder_rotate']
q0=np.array([start[n] for n in m.names])
scene=b.collision_scene(m,cfg)
report=dict(schema=1,request_kind='operator-approved-endpoint inverse-path diagnostic',
    nominal_request_board_mm=audit['target_board_mm'],
    nominal_request_solved=False, model_fk_of_reference_board_mm=contact.tolist(),
    original_request_was_not_relabelled_as_solved=True,
    endpoint_source='Jetson Saved Moves A8_take; user confirms white knight near A8 centre',
    intermediate_waypoints_source='new Cartesian reverse IK plus checked geometric transit',
    saved_angles_replayed_as_trajectory=False,motion_issued=False,serial_opened=False,
    runtime_model_modified=False,execution_eligible=False,fit_ground_truth=False,
    common_envelope_candidate_changes=audit['operating_bound_violations_deg'],
    source_hashes=preflight['source_hashes'],branches=[])
beg=time.monotonic()
valid=[]
for height in dict.fromkeys([max(float(cfg['default_clearance_mm']),contact[2]+50),
                             max(180.,contact[2]+50),max(220.,contact[2]+50)]):
    branch=dict(above_height_mm=height)
    q=goal.copy();current=contact[2];reverse=[(contact.copy(),goal.copy())];step=8.
    try:
        b.check_board_intersection(m,goal,cfg)
        while current < height-1e-8:
            z=min(height,current+step);point=np.array([contact[0],contact[1],z])
            target=R@(point/1000)+t
            answer=solve_ik(m,target,q,tcp,bounds,desired,max_iterations=150,
                position_tolerance_m=.0005,rotation_tolerance_rad=.005,clearance_function=scene)
            next_q=np.array([answer['encoder_rad'][n] for n in m.names])
            continuous=np.max(np.abs(next_q-q))<=math.radians(12)
            if not answer['success'] or not continuous:
                if step>1.:step/=2;continue
                raise b.BoardPlanningError('reverse fixed-frame IK branch failed',
                    dict(target_board_mm=point.tolist(),answer=answer,joint_continuity_passed=bool(continuous)))
            b.check_transit_edge(m,q,next_q,cfg)
            reverse.append((point,next_q));q=next_q;current=z;step=min(8.,step*2)
        branch['reverse_samples']=len(reverse)
        route,blocked=b.find_entry_transit(m,q0,[(float(np.linalg.norm(q-q0)),q)],cfg)
        if route is None:raise b.BoardPlanningError('no checked Clearance-to-hover transit',dict(blocked_paths=blocked))
        rows=[];previous=q0.copy()
        for stage,entry in route:
            count=max(1,int(np.ceil(np.max(np.abs(entry-previous))/math.radians(5))))
            initial=previous.copy()
            for i in range(1,count+1):
                value=initial+(entry-initial)*i/count;pose=tcp_pose(m,value,tcp)
                rows.append(dict(stage=stage,encoder_rad=dict(zip(m.names,value.tolist())),
                    target_cad_m=pose[:3,3].tolist(),target_board_mm=((pose[:3,3]-t)@R*1000).tolist(),
                    position_error_mm=0.,rotation_error_deg=None))
            previous=entry
        for point,value in reversed(reverse[:-1]):
            pose=tcp_pose(m,value,tcp);target=R@(point/1000)+t
            rows.append(dict(stage='contact',encoder_rad=dict(zip(m.names,value.tolist())),
                target_cad_m=target.tolist(),target_board_mm=point.tolist(),
                position_error_mm=float(np.linalg.norm(pose[:3,3]-target)*1000),
                rotation_error_deg=float(np.degrees(np.linalg.norm(rotation_error(desired,pose[:3,:3]))))))
        plan=dict(ok=True,build=b.PLANNER_BUILD,motion_issued=False,planning_only=True,
            execution_eligible=False,stage='approach',piece='white_knight',start_rad=start,
            target_board_mm=contact.tolist(),target_source='FK-of-explicit-user-approved-endpoint; nominal-A8-XYZ remains unresolved',
            waypoints=rows,selected_wrist_rotation_in_board=(R.T@desired).tolist(),
            tool_orientation_constraint='full-wrist',endpoint_settle_enabled=True)
        profile=cfg['profiles']['white_knight']
        profile['seed_encoder_rad']=audit['saved_move']['pose']
        profile['wrist_rotation_in_board']=(R.T@desired).tolist()
        branch['gripper_clearance']=b.check_planned_gripper_clearance(m,cfg,plan)
        branch['orientation_audit']=b.check_planned_tool_orientation(m,cfg,plan)
        plan['planned_gravity_envelope']=gravity_envelope(m,plan)
        branch.update(passed=True,waypoints=len(rows),stages=dict(collections.Counter(r['stage'] for r in rows)),
            endpoint_delta_from_saved_deg={n:float(np.degrees(rows[-1]['encoder_rad'][n]-goal[i])) for i,n in enumerate(m.names)},
            planned_gravity_envelope=plan['planned_gravity_envelope'])
        valid.append(plan)
        (out/f'reference_inverse_plan_hover{height:.1f}.json').write_text(json.dumps(plan,indent=2),encoding='utf-8')
    except b.BoardPlanningError as exc:
        branch.update(passed=False,error=str(exc),diagnostics=exc.diagnostics,
            reverse_samples=len(reverse),highest_solved_board_height_mm=float(current))
    report['branches'].append(branch)
    print(json.dumps({k:v for k,v in branch.items() if k not in ('diagnostics','planned_gravity_envelope')},ensure_ascii=False),flush=True)
    if valid:break
report.update(geometric_inverse_path_found=bool(valid),elapsed_s=time.monotonic()-beg)
report['source_hashes_still_match']=all(hashlib.sha256((Path('scripts')/n).read_bytes()).hexdigest()==h
    for n,h in preflight['source_hashes'].items())
assert report['source_hashes_still_match']
(out/'reference_inverse_path_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
(out/'reference_envelope_candidate.json').write_text(json.dumps(cfg,indent=2),encoding='utf-8')
print('RESULT '+json.dumps(dict(geometric_inverse_path_found=bool(valid),elapsed_s=report['elapsed_s'],
    nominal_request_solved=False,motion_issued=False)))
