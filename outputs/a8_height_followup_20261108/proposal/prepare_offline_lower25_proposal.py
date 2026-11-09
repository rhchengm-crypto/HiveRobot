"""Inactive offline proposal: selected NEW frame, unchanged lower25 XYZ.

No hardware, network, motor execution API, prepared stamp, or runtime writes.
The proposal domain extends only front lower software bound12deg and permits
new full-frame/tool-axis selection15deg; strict path frame tolerance=.005rad.
"""
import os
for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[k] = '1'
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time
sys.dont_write_bytecode = True
OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
FROZEN = ROOT/'outputs/a8_v33_execution_20261108/source_snapshot/scripts'
sys.path.insert(0, str(FROZEN))
import numpy as np
import left_arm_board_grasp as b
from left_arm_board_kinematics import tcp_pose, solve_ik, rotation_error
from left_arm_trajectory_support import gravity_envelope


def read(p):
    return json.loads(Path(p).read_text(encoding='utf8'))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write(p, obj):
    Path(p).write_text(json.dumps(obj, indent=2, ensure_ascii=False), encoding='utf8')


def main():
    began = time.monotonic()
    active = ROOT/'scripts/data/left_arm_board_workspace_candidate.json'
    active_hash = sha(active)
    mp, cp = FROZEN/'data/left_arm_general_dynamics.json', FROZEN/'data/left_arm_board_workspace_candidate.json'
    m, original = b.RigidBodyModel.load(mp), read(cp)
    cfg = copy.deepcopy(original)
    front = m.names.index('shoulder_front')
    cfg['joint_limits_encoder_rad'][front][0] -= math.radians(12.)
    cfg['continuous_grasp_search']['axis_cone_deg'] = 15.
    cfg['continuous_grasp_search']['descent_axis_cone_deg'] = 0.
    cfg['planning_only'] = True
    cfg['_offline_proposal_only'] = True
    cfg['_proposal_new_full_frame_selection_max_deg'] = 15.
    cfg['_proposal_new_tool_axis_selection_max_deg'] = 15.
    # Do not create an approved task-pose reference or deployment/stamp metadata.
    write(OUT/'candidate_board_config_UNAPPROVED.json', cfg)
    bounds = np.array(cfg['joint_limits_encoder_rad'])
    report = dict(schema=1, status='in-progress-offline-only', motion_issued=False,
        hardware_accessed=False, network_accessed=False, executable=False,
        approved_reference=False, active_configuration_modified=False,
        physical_acceptance=None, mechanical_qualification_pending=True,
        full_model_pose_proposal_only=True, nominal_A8_geometry_consistency_resolved=False,
        measured_physical_contact_datum_mm=None,
        source_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in (mp,cp,
            FROZEN/'left_arm_board_grasp.py', FROZEN/'left_arm_board_kinematics.py',
            FROZEN/'left_arm_board_collision.py', FROZEN/'left_arm_trajectory_support.py', Path(__file__))},
        candidate_policy=dict(front_lower_software_extension_deg=12., other_six_bounds_unchanged=True,
            requested_new_full_frame_selection_max_deg=15., requested_new_tool_axis_selection_max_deg=15.,
            strict_descent_full_frame_tolerance_rad=.005, position_tolerance_mm=.5,
            body_board_gripper_geometry_unchanged=True, planning_only_runtime_guard=True),
        source_active_config_sha256=active_hash)
    def finish(status, failure=None):
        report['status'] = status
        report['failure'] = failure
        report['elapsed_s'] = time.monotonic()-began
        report['active_configuration_unchanged_at_finish'] = sha(active) == active_hash
        report['limitations'] = ['Counterfactual software domain is not verified mechanical travel or execution permission.',
            'No calibrated contact height, physical clearance, orientation suitability, or gripping success is established.',
            'No prepared execution stamp; standard ordinary Clearance has only been forecast, not executed.',
            'Fresh actual entry, current sources, load anchoring, support bounds, and separate movement authorization remain required.']
        write(OUT/'proposal_report.json', report)
        print(json.dumps(dict(status=status, failure=failure, elapsed_s=report['elapsed_s'])), flush=True)
    prior = read(OUT.parent/'height_reach_ablation.json')
    source = next(row['best']['assessment'] for row in prior['ablations']
        if row['domain'] == 'front_minus10' and not row['fixed_original_full_orientation'])
    goal = np.array([source['encoder_rad'][n] for n in m.names])
    target_board = np.array([11.798050592563328,405.6006774366655,59.622674457482844])
    rb, tb, tcp = np.array(cfg['board_to_cad_rotation']), np.array(cfg['board_to_cad_translation_m']), np.array(cfg['tcp_in_wrist_frame_m'])
    target = rb@(target_board/1000)+tb
    desired = tcp_pose(m,goal,tcp)[:3,:3]
    saved = read(ROOT/'outputs/a8_saved_reference_20261108/saved_pose_audit.json')['saved_move']['pose']
    rref = tcp_pose(m,np.array([saved[n] for n in m.names]),tcp)[:3,:3]
    axis = np.array(cfg['tool_axis_in_wrist_frame']); axis /= np.linalg.norm(axis)
    full_change = float(np.degrees(np.linalg.norm(rotation_error(rref,desired))))
    axis_change = float(np.degrees(math.acos(float(np.clip((rref@axis)@(desired@axis),-1,1)))))
    report['target_board_mm'] = target_board.tolist()
    report['selected_endpoint_encoder_rad'] = dict(zip(m.names,goal.tolist()))
    report['selected_new_wrist_rotation_in_cad'] = desired.tolist()
    report['selected_new_wrist_rotation_in_board'] = (rb.T@desired).tolist()
    report['new_full_frame_deviation_from_old_deg'] = full_change
    report['new_tool_axis_deviation_from_old_deg'] = axis_change
    report['requested_frame'] = dict(source='explicit NEW offline proposal frame; not an approved reference',
        wrist_rotation_in_cad=desired.tolist(), wrist_rotation_in_board=(rb.T@desired).tolist(),
        original_frame_deviation_deg=full_change, original_tool_axis_deviation_deg=axis_change,
        historical_global_selection_policy_passed=False,
        historical_axis_cone_deg=original['continuous_grasp_search']['axis_cone_deg'],
        historical_terminal_reference_tolerance_deg=original['continuous_grasp_search']['terminal_reference_tolerance_deg'],
        historical_tolerance_unchanged_in_candidate_config=True,
        strict_descent_tracking_tolerance_rad=.005)
    report['front_additional_lower_travel_deg'] = float(np.degrees(max(0,np.array(original['joint_limits_encoder_rad'])[front,0]-goal[front])))
    if not (full_change <=15 and axis_change <=15 and np.all(goal>=bounds[:,0]) and np.all(goal<=bounds[:,1])):
        finish('failed-endpoint-policy',dict(full_change=full_change,axis_change=axis_change));return
    home_log = ROOT/'outputs/a8_v33_home_20261108/control_evidence/data/left_arm_v2_8_full_runs/arm_run_1794188707909_37371.txt'
    home_status = ROOT/'outputs/a8_v33_home_20261108/control_evidence/data/board_workspace_trials/a8_v33_home_20261108/home_hold_status.json'
    text = home_log.read_text(encoding='utf8')
    home = json.loads(text[text.index('\n{')+1:])
    assert home['ok'] and read(home_status)['run']['returncode'] == 0
    ordinary = read(FROZEN/'data/left_arm_v2_table_clearance.json')['joints']
    start = dict(ordinary)
    start['shoulder_rotate'] = float(home['final']['shoulder_rotate']['pos'])
    q0 = np.array([start[n] for n in m.names])
    assert np.all(q0>=bounds[:,0]) and np.all(q0<=bounds[:,1])
    report['forecast_clearance'] = start
    report['ordinary_clearance_executed_for_proposal'] = False
    report['forecast_rotate_source'] = dict(file=str(home_log.relative_to(ROOT)).replace('\\','/'),
        sha256=sha(home_log), field='final.shoulder_rotate.pos', actual_final_encoder_rad=start['shoulder_rotate'],
        hold_finished_at=read(home_status)['run']['finished_at'])
    write(OUT/'forecast_clearance.json',start)
    scene = b.collision_scene(m,cfg)
    b.check_board_intersection(m,goal,cfg)
    height = max(float(cfg['default_clearance_mm']),target_board[2]+50.)
    q, current = goal.copy(),float(target_board[2])
    reverse = [(target_board.copy(),goal.copy())]
    ikcalls=[]
    while current < height-1e-8:
        z = min(height,current+4.)
        point=np.array([target_board[0],target_board[1],z])
        answer=solve_ik(m,rb@(point/1000)+tb,q,tcp,bounds,desired,
            max_iterations=180,position_tolerance_m=.0005,rotation_tolerance_rad=.005,clearance_function=scene)
        nq=np.array([answer['encoder_rad'][n] for n in m.names])
        continuous=bool(np.max(np.abs(nq-q))<=math.radians(15.))
        call=dict(target_board_mm=point.tolist(),success=answer['success'],iterations=answer['iterations'],
            position_error_mm=answer['position_error_mm'],rotation_error_deg=answer['rotation_error_deg'],
            maximum_joint_step_deg=float(np.degrees(np.max(np.abs(nq-q)))),continuity_passed=continuous)
        ikcalls.append(call)
        report['reverse_vertical_ik_calls']=ikcalls
        if not answer['success'] or not continuous:
            write(OUT/'partial_reverse_branch.json',[dict(target_board_mm=p.tolist(),encoder_rad=dict(zip(m.names,v.tolist()))) for p,v in reverse])
            finish('failed-first-reverse-IK',dict(from_board_z_mm=current,to_board_z_mm=z,answer=answer,continuity_passed=continuous));return
        try:
            b.check_transit_edge(m,q,nq,cfg)
        except b.BoardPlanningError as exc:
            finish('failed-first-reverse-edge',dict(from_board_z_mm=current,to_board_z_mm=z,diagnostics=exc.diagnostics));return
        reverse.append((point,nq));q,current=nq,z
    report['reverse_vertical_branch_completed']=True
    report['hover_height_mm']=height
    print(json.dumps(dict(phase='reverse-vertical-complete',samples=len(reverse))),flush=True)
    route,blocked=b.find_entry_transit(m,q0,[(float(np.linalg.norm(q-q0)),q)],cfg)
    report['transit_blocked_attempts']=blocked
    if route is None:
        finish('failed-Clearance-transit',dict(blocked=blocked));return
    rows=[];previous=q0.copy()
    for stage,entry in route:
        count=max(1,int(np.ceil(np.max(np.abs(entry-previous))/math.radians(5))))
        initial=previous.copy()
        for i in range(1,count+1):
            value=initial+(entry-initial)*i/count
            pose=tcp_pose(m,value,tcp)
            rows.append(dict(stage=stage,encoder_rad=dict(zip(m.names,value.tolist())),
                target_cad_m=pose[:3,3].tolist(),target_board_mm=((pose[:3,3]-tb)@rb*1000).tolist(),position_error_mm=0.,rotation_error_deg=None))
        previous=entry
    for point,value in reversed(reverse[:-1]):
        pose=tcp_pose(m,value,tcp)
        rows.append(dict(stage='contact',encoder_rad=dict(zip(m.names,value.tolist())),
            target_cad_m=(rb@(point/1000)+tb).tolist(),target_board_mm=point.tolist(),
            position_error_mm=float(np.linalg.norm(pose[:3,3]-(rb@(point/1000)+tb))*1000),
            rotation_error_deg=float(np.degrees(np.linalg.norm(rotation_error(desired,pose[:3,:3]))))))
    plan=dict(ok=True,executable=False,planning_only=True,offline_proposal_only=True,approved_reference=False,
        mechanical_qualification_pending=True,physical_acceptance=None,motion_issued=False,
        build=b.PLANNER_BUILD,stage='approach',piece='white_knight',start_rad=start,
        target_board_mm=target_board.tolist(),waypoints=rows,selected_wrist_rotation_in_board=(rb.T@desired).tolist(),
        requested_frame=report['requested_frame'],
        tool_orientation_constraint='full-wrist',target_source='explicit-lower25-model-target-with-new-requested-frame',
        path_generation=dict(new_Cartesian_reverse_IK=True,old_trajectory_replay=False,
            fixed_cartesian_compensation_mm=[0.,0.,0.],Kp_or_Kd_modified=False,
            complete_full_descent=True,strict_frame_tolerance_rad=.005))
    try:
        plan['orientation_acceptance']=b.check_planned_tool_orientation(m,cfg,plan)
        plan['gripper_clearance_acceptance']=b.check_planned_gripper_clearance(m,cfg,plan)
        groups=b.execution_waypoints(plan)
        grouped=dict(plan,waypoints=groups)
        report['grouped_orientation']=b.check_planned_tool_orientation(m,cfg,grouped)
        report['grouped_gripper_clearance']=b.check_planned_gripper_clearance(m,cfg,grouped)
        previous=q0.copy();raw_joint_steps=[]
        for index,row in enumerate(rows):
            value=np.array([row['encoder_rad'][n] for n in m.names])
            stagecfg=b.transit_clearance_config(cfg) if row['stage'] in ('entry_clearance','entry_search','source_above') else cfg
            assert np.all(value>=bounds[:,0]-1e-9) and np.all(value<=bounds[:,1]+1e-9)
            raw_joint_steps.append(float(np.degrees(np.max(np.abs(value-previous)))))
            b.check_transit_edge(m,previous,value,stagecfg)
            previous=value
    except b.BoardPlanningError as exc:
        finish('failed-full-or-grouped-audit',dict(diagnostics=exc.diagnostics));return
    task=[r for r in rows if r['stage']=='source_above'][-1:]+[r for r in rows if r['stage']=='contact']
    lateral=[];angular=[];heights=[];jointsteps=[];poeerror=0.
    for first,second in zip(task,task[1:]):
        a=np.array([first['encoder_rad'][n] for n in m.names]);bb=np.array([second['encoder_rad'][n] for n in m.names])
        jointsteps.append(float(np.degrees(np.max(np.abs(bb-a)))))
        count=max(1,int(np.ceil(np.max(np.abs(bb-a))/math.radians(.1))))
        for fraction in np.linspace(0.,1.,count+1):
            value=a+fraction*(bb-a);pose=tcp_pose(m,value,tcp);p=(pose[:3,3]-tb)@rb*1000
            lateral.append(float(np.linalg.norm(p[:2]-target_board[:2])))
            angular.append(float(np.degrees(np.linalg.norm(rotation_error(desired,pose[:3,:3])))))
            heights.append(float(p[2]));mq=m.encoder_to_model(value)
            poeerror=max(poeerror,float(np.max(np.abs(m.fk(mq)-m.fk_poe(mq)))))
    report['dense_task_audit']=dict(samples=len(lateral),maximum_lateral_error_mm=max(lateral),
        maximum_strict_new_frame_error_deg=max(angular),strict_tolerance_deg=math.degrees(.005),
        minimum_height_mm=min(heights),maximum_height_mm=max(heights),
        descent_monotonic=bool(all(nxt<=old+1e-7 for old,nxt in zip(heights,heights[1:]))),
        maximum_joint_edge_step_deg=max(jointsteps),fk_poe_max_matrix_error=poeerror)
    if max(lateral)>.5 or max(angular)>math.degrees(.005) or not report['dense_task_audit']['descent_monotonic']:
        finish('failed-dense-task-audit',report['dense_task_audit']);return
    plan['planned_gravity_envelope']=gravity_envelope(m,plan)
    report['posture_cost'],report['posture_metrics']=b.posture_load_cost(m,cfg,start,rows)
    report['gravity_envelope']=plan['planned_gravity_envelope']
    report['waypoints']=len(rows);report['execution_groups']=len(groups)
    report['transit_geometric_edges']=len(route)
    report['all_raw_and_grouped_edges_checked']=True
    report['maximum_raw_waypoint_joint_step_deg']=max(raw_joint_steps)
    report['orientation']=plan['orientation_acceptance']
    report['gripper_clearance']=plan['gripper_clearance_acceptance']
    report['complete_geometry_computed']=True
    write(OUT/'local_plan_UNAPPROVED.json',plan)
    report['proposal_plan_sha256']=sha(OUT/'local_plan_UNAPPROVED.json')
    report['proposal_config_sha256']=sha(OUT/'candidate_board_config_UNAPPROVED.json')
    finish('complete-inactive-geometric-proposal')


if __name__=='__main__':
    main()
