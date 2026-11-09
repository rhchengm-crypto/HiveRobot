"""Offline preparation for the explicitly authorized, scoped lower25 trial.

Never opens motor/network interfaces, edits active config, stamps a plan, or
deploys. Fresh root preflight is required before final forecast/path output.
"""
import os
for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[k] = '1'
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import types
sys.dont_write_bytecode = True
OUT = Path(__file__).resolve().parents[1]
ROOT = OUT.parents[1]
LIVE = ROOT/'scripts'
OLD = ROOT/'outputs/a8_v33_execution_20261108/source_snapshot/scripts'
PROPOSAL = ROOT/'outputs/a8_height_followup_20261108/proposal'


def read(p):
    return json.loads(Path(p).read_text(encoding='utf8'))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write(p, obj):
    Path(p).parent.mkdir(parents=True, exist_ok=True)
    Path(p).write_text(json.dumps(obj, indent=2, ensure_ascii=False), encoding='utf8')


def ast_function(path, name):
    tree = ast.parse(Path(path).read_text(encoding='utf-8-sig'))
    return ast.dump(next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name), include_attributes=False)


def main():
    began = time.monotonic()
    OUT.mkdir(parents=True,exist_ok=True)
    active_path = LIVE/'data/left_arm_board_workspace_candidate.json'
    active_hash = sha(active_path)
    proposal = read(PROPOSAL/'local_plan_UNAPPROVED.json')
    proposal_report = read(PROPOSAL/'proposal_report.json')
    assert proposal_report['status'] == 'complete-inactive-geometric-proposal'
    assert sha(PROPOSAL/'local_plan_UNAPPROVED.json') == proposal_report['proposal_plan_sha256']
    assert sha(PROPOSAL/'candidate_board_config_UNAPPROVED.json') == proposal_report['proposal_config_sha256']
    candidate = read(PROPOSAL/'candidate_board_config_UNAPPROVED.json')
    assert candidate['planning_only'] is True
    original = read(active_path)
    assert sha(active_path) == sha(OLD/'data/left_arm_board_workspace_candidate.json')
    assert not original['independently_validated'] and not original['body_collision']['physically_validated']
    # This is the one explicitly authorized trial release of the inactive copy.
    scoped = copy.deepcopy(candidate)
    scoped['planning_only'] = False
    scoped.pop('_offline_proposal_only',None)
    scoped['profiles']['white_knight']['wrist_rotation_in_board'] = copy.deepcopy(proposal['selected_wrist_rotation_in_board'])
    scoped['authorized_trial_scope'] = dict(name='a8_lower25_execution_20261108',
        physical_validation_complete=False, deployment_as_general_model=False,
        fixed_target_board_mm=proposal['target_board_mm'],
        requested_frame=proposal['requested_frame'],
        front_lower_extension_deg=12., selected_front_additional_travel_deg=proposal_report['front_additional_lower_travel_deg'],
        full_frame_selection_budget_deg=15., tool_axis_selection_budget_deg=15.,
        strict_path_rotation_tolerance_rad=.005,position_tolerance_mm=.5,
        original_profile_wrist_rotation_in_board=original['profiles']['white_knight']['wrist_rotation_in_board'],
        original_approved_reference_bank_preserved_for_history_only=True,
        no_approved_reference_added=True,no_fitting_eligibility_granted=True,
        full_descent_required=True,claw_commanded=False,
        authorized_by='root-reported explicit user full-test authorization; operator onsite, E-stop, empty A8/channel, unchanged board/base/height')
    assert not scoped['independently_validated'] and not scoped['body_collision']['physically_validated']
    config_path = OUT/'common_trial_config.json'
    write(config_path,scoped)
    # Source identity establishes identical geometry/auditors independently of
    # a changed endpoint-search implementation, which this frozen path bypasses.
    geometry_files=('left_arm_rigid_body.py','left_arm_board_kinematics.py','left_arm_board_collision.py',
                    'left_arm_board_grasp.py','left_arm_board_transit.py','left_arm_trajectory_support.py',
                    'data/left_arm_general_dynamics.json','data/left_arm_board_workspace_candidate.json')
    geometry = {name:dict(frozen_sha256=sha(OLD/name),current_sha256=sha(LIVE/name),same_bytes=sha(OLD/name)==sha(LIVE/name)) for name in geometry_files}
    assert all(row['same_bytes'] for row in geometry.values()),'geometry/auditor source differs from proposal'
    batch_same = ast_function(OLD/'left_arm_board_task_pose.py','batch_tool_frames') == ast_function(LIVE/'left_arm_board_task_pose.py','batch_tool_frames')
    assert batch_same
    common = dict(schema=1,motion_issued=False,hardware_accessed=False,network_accessed=False,
        active_configuration_modified=False,source_active_board_sha256=active_hash,
        cross_version_geometry_and_auditors=geometry,task_pose_batch_FK_AST_identical=batch_same,
        old_task_pose_sha256=sha(OLD/'left_arm_board_task_pose.py'),current_task_pose_sha256=sha(LIVE/'left_arm_board_task_pose.py'),
        task_pose_search_not_rerun=True,chosen_new_frame_explicit=True,strict_tracking_rotation_tolerance_rad=.005,
        trial_config_path=str(config_path.relative_to(ROOT)).replace('\\','/'),trial_config_sha256=sha(config_path),
        main_configuration_not_replaced=True,planning_only_disabled_only_in_scoped_trial_copy=True,
        physical_validation_complete=False,physical_acceptance=None,approved_reference=False,
        source_proposal_plan_sha256=sha(PROPOSAL/'local_plan_UNAPPROVED.json'),
        source_proposal_config_sha256=sha(PROPOSAL/'candidate_board_config_UNAPPROVED.json'))
    context_path=OUT/'preflight.json'
    if not context_path.is_file():
        common.update(status='waiting-for-root-fresh-preflight',final_plan_produced=False,
            no_preflight_fallback_used=True,elapsed_s=time.monotonic()-began)
        write(OUT/'preflight_pending_preparation.json',common)
        print(json.dumps(dict(status=common['status'],scoped_config_sha256=sha(config_path),task_pose_sha256=common['current_task_pose_sha256'])),flush=True)
        return
    context=read(context_path)
    assert context.get('motion_issued') is False
    assert context['last_action']['running'] is False and context['last_action']['returncode']==0
    assert set(context['feedback']) >= set(proposal['start_rad'])
    # Capture all locally available protected input bytes without mutating them.
    protected,absent={},{}
    for name,expected in context['source_hashes'].items():
        p=LIVE/name
        if p.is_file():
            assert sha(p)==expected,('fresh preflight/local mismatch',name,sha(p),expected)
            protected[name]=expected
        else:
            absent[name]=expected
    snapshot=OUT/'source_snapshot/scripts'
    frozen_names=set(protected)|set(geometry_files)|{'left_arm_board_task_pose.py','left_arm_board_home_handoff.py'}
    for name in sorted(frozen_names):
        source,target=LIVE/name,snapshot/name
        target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():
            assert sha(target)==sha(source),('existing trial frozen input changed',name)
        else:
            target.write_bytes(source.read_bytes())
    # Import the CURRENT preflight-checked snapshot; do not run main/execute.
    sys.path.insert(0,str(snapshot))
    import numpy as np
    import left_arm_board_grasp as b
    from left_arm_board_kinematics import tcp_pose,rotation_error
    from left_arm_trajectory_support import gravity_envelope,path_digest
    m=b.RigidBodyModel.load(snapshot/'data/left_arm_general_dynamics.json')
    cfg=b.load_board_config(config_path,m)
    old_args=types.SimpleNamespace(model=snapshot/'data/left_arm_general_dynamics.json',board_model=snapshot/'data/left_arm_board_workspace_candidate.json')
    base_prepared=b.prepared_source_hashes(old_args)
    assert len(base_prepared)==13 and base_prepared==context['prepared_source_hashes'],('fresh base prepared13 mismatch',base_prepared,context['prepared_source_hashes'])
    scoped_args=types.SimpleNamespace(model=old_args.model,board_model=config_path)
    scoped_prepared=b.prepared_source_hashes(scoped_args)
    assert all(scoped_prepared[k]==base_prepared[k] for k in base_prepared if k!='board_model')
    assert scoped_prepared['board_model']==sha(config_path)
    plan=copy.deepcopy(proposal)
    for key in ('executable','planning_only','offline_proposal_only'):
        plan.pop(key,None)
    plan.update(executable=True,planning_only=False,authorized_candidate_trial=True,precomputed_before_motion=True,
        released_for_this_trial_only=True,physical_acceptance=None,approved_reference=False,
        fitting_eligible=False,physical_validation_complete=False,
        mechanical_qualification_pending=True,prepared_execution_stamp_present=False)
    plan['requested_frame']=copy.deepcopy(proposal['requested_frame'])
    plan['requested_frame']['source']='explicit NEW requested frame authorized for this scoped candidate trial; not an approved reference'
    ordinary=read(snapshot/'data/left_arm_v2_table_clearance.json')['joints']
    assert ordinary==context['ordinary_clearance']
    start=dict(ordinary)
    start['shoulder_rotate']=float(context['feedback']['shoulder_rotate']['pos'])
    plan['start_rad']=start
    assert plan['waypoints']==proposal['waypoints'],'raw path changed'
    bounds=np.array(cfg['joint_limits_encoder_rad']);q0=np.array([start[n] for n in m.names])
    assert np.all(q0>=bounds[:,0]) and np.all(q0<=bounds[:,1])
    rb,tb,tcp=np.array(cfg['board_to_cad_rotation']),np.array(cfg['board_to_cad_translation_m']),np.array(cfg['tcp_in_wrist_frame_m'])
    desired=rb@np.array(plan['selected_wrist_rotation_in_board'])
    np.testing.assert_allclose(desired,plan['requested_frame']['wrist_rotation_in_cad'],atol=1e-14,rtol=0)
    saved=read(ROOT/'outputs/a8_saved_reference_20261108/saved_pose_audit.json')['saved_move']['pose']
    oldr=tcp_pose(m,np.array([saved[n] for n in m.names]),tcp)[:3,:3]
    axis=np.array(cfg['tool_axis_in_wrist_frame']);axis/=np.linalg.norm(axis)
    full_change=math.degrees(float(np.linalg.norm(rotation_error(oldr,desired))))
    axis_change=math.degrees(math.acos(float(np.clip((oldr@axis)@(desired@axis),-1,1))))
    assert full_change<=15. and axis_change<=15.
    original_bounds=np.array(original['joint_limits_encoder_rad'])
    np.testing.assert_array_equal(bounds[1:],original_bounds[1:])
    np.testing.assert_array_equal(bounds[0,1],original_bounds[0,1])
    assert abs(bounds[0,0]-(original_bounds[0,0]-math.radians(12.)))<1e-14
    plan['orientation_acceptance']=b.check_planned_tool_orientation(m,cfg,plan)
    plan['gripper_clearance_acceptance']=b.check_planned_gripper_clearance(m,cfg,plan)
    # This is the actual runtime candidate-mode shape/gating helper.
    plan=b.candidate_approach_trial(plan,cfg)
    plan['planned_gravity_envelope']=gravity_envelope(m,plan)
    groups=b.execution_waypoints(plan)
    grouped=dict(plan,waypoints=groups)
    group_orientation=b.check_planned_tool_orientation(m,cfg,grouped)
    group_clearance=b.check_planned_gripper_clearance(m,cfg,grouped)
    previous=q0.copy();raw_step=[];fkerror=0.
    for index,row in enumerate(plan['waypoints']):
        value=np.array([row['encoder_rad'][n] for n in m.names])
        assert np.all(value>=bounds[:,0]-1e-9) and np.all(value<=bounds[:,1]+1e-9)
        raw_step.append(math.degrees(float(np.max(np.abs(value-previous)))))
        assert raw_step[-1]<=15.
        stagecfg=b.transit_clearance_config(cfg) if row['stage'] in ('entry_clearance','entry_search','source_above') else cfg
        b.check_transit_edge(m,previous,value,stagecfg)
        mq=m.encoder_to_model(value);fkerror=max(fkerror,float(np.max(np.abs(m.fk(mq)-m.fk_poe(mq)))))
        previous=value
    assert plan['waypoints']==proposal['waypoints']
    assert sha(active_path)==active_hash
    assert all(sha(LIVE/n)==expected for n,expected in protected.items())
    write(OUT/'forecast_clearance.json',start)
    write(OUT/'local_plan.json',plan)
    report=dict(common,status='scoped-trial-preparation-complete-no-stamp',final_plan_produced=True,
        request=dict(x_mm=plan['target_board_mm'][0],y_mm=plan['target_board_mm'][1],piece=plan['piece'],stage=plan['stage'],grip_height_mm=plan['target_board_mm'][2]),
        target_board_mm=plan['target_board_mm'],forecast_clearance=start,
        fresh_preflight_sha256=sha(context_path),fresh_preflight_captured_wall_s=context.get('captured_wall_s'),
        forecast_rotate_source='fresh preflight feedback; ordinary Clearance preserves shoulder_rotate',
        forecast_differs_from_inactive_proposal_deg={n:math.degrees(start[n]-proposal['start_rad'][n]) for n in m.names},
        raw_waypoint_geometry_identical_to_proposal=True,raw_waypoints=len(plan['waypoints']),execution_groups=len(groups),
        new_requested_full_frame_deviation_deg=full_change,new_requested_tool_axis_deviation_deg=axis_change,
        historical_global_selection_policy_passed=False,explicit_trial_frame_policy_passed=True,
        strict_full_frame_orientation=plan['orientation_acceptance'],gripper_clearance=plan['gripper_clearance_acceptance'],
        grouped_orientation=group_orientation,grouped_gripper_clearance=group_clearance,
        maximum_raw_joint_step_deg=max(raw_step),all_raw_and_grouped_swept_edges_checked=True,
        independent_FK_PoE_matrix_max_error=fkerror,planned_gravity_envelope=plan['planned_gravity_envelope'],
        planned_gravity_independent_recomputation_equal=gravity_envelope(m,plan)==plan['planned_gravity_envelope'],
        old_and_new_path_digest_equal=path_digest(proposal)==path_digest(plan),
        available_protected_sources_match_fresh_preflight=len(protected),absent_local_protected_sources=absent,
        prepared_source_hashes=scoped_prepared,prepared_source_count=len(scoped_prepared),
        base_prepared_sources_all_match_fresh_preflight=True,
        scoped_board_is_only_prepared_hash_difference_from_active_preflight=True,
        runtime_prepared_source_verification_after_upload_required=True,
        additional_local_only_frozen_dependencies={name:sha(snapshot/name) for name in ('left_arm_rigid_body.py','left_arm_board_transit.py','left_arm_board_home_handoff.py') if name not in protected},
        local_plan_sha256=sha(OUT/'local_plan.json'),board_model_sha256=sha(config_path),
        prepared_execution_stamp_present=False,hardware_or_network_accessed=False,
        active_configuration_unchanged_at_finish=sha(active_path)==active_hash,
        candidate_approach_trial_passed=True,claw_commanded=False,full_descent_included=True,
        remaining_runtime_checks=['root uploads scoped config/path and verifies bytes/current prepared13',
            'root creates fresh stamp without changing chosen frame or raw path',
            'actual ordinary Clearance, fresh entry load/stationary clocks and MIT-bounded support qualification',
            'root authorized execution; record physical result separately from mathematical checks'],
        elapsed_s=time.monotonic()-began)
    write(OUT/'preparation.json',report)
    print(json.dumps(dict(status=report['status'],waypoints=report['raw_waypoints'],groups=report['execution_groups'],
        forecast_rotate=start['shoulder_rotate'],local_plan_sha256=report['local_plan_sha256'],scoped_config_sha256=report['board_model_sha256'])),flush=True)


if __name__=='__main__':
    main()
