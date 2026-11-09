"""Activate the expressly authorised common-model trial and bind its precomputed path.
No serial port, enable command, trajectory, server restart, or learning write.
"""
import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse,copy,hashlib,json,math,stat,sys,time,urllib.request
import numpy as np
ROOT=Path('/home/nvidia/hive_robot/DM_Control_Python')
OUT=ROOT/'data/board_workspace_trials/a8_lower25_execution_20261108'
read=lambda p:json.loads(Path(p).read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
preflight=read(OUT/'preflight.json');prep=read(OUT/'preparation.json');plan=read(OUT/'local_plan.json')
assert not (OUT/'activation_receipt.json').exists(),'Trial already activated; inspect instead of resubmitting'
assert sha(OUT/'local_plan.json')==prep['local_plan_sha256']
assert sha(OUT/'common_trial_config.json')==prep['trial_config_sha256']
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
state=json.load(client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5));run=state.get('run') or {}
assert not run.get('running') and not run.get('support_running')
assert run.get('action')=='dynamics:model-hold-after:home' and run.get('returncode')==0
assert state['execution_enabled'] and state['auto_model_hold_enabled'] and state['rigid_body_mode']=='apply'
for name,digest in preflight['source_hashes'].items():assert sha(ROOT/name)==digest,name
for name,digest in prep['additional_local_only_frozen_dependencies'].items():
    assert sha(ROOT/name)==digest,name
    dest=OUT/'runtime_snapshot'/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes((ROOT/name).read_bytes())
target=ROOT/'data/left_arm_board_workspace_candidate.json'
original=read(target);trial=read(OUT/'common_trial_config.json')
expected=copy.deepcopy(original)
front=original['joint_order'].index('shoulder_front')
expected['joint_limits_encoder_rad'][front][0]-=math.radians(12.)
expected['continuous_grasp_search']['axis_cone_deg']=15.
expected['profiles']['white_knight']['wrist_rotation_in_board']=plan['selected_wrist_rotation_in_board']
for key in ('_proposal_new_full_frame_selection_max_deg','_proposal_new_tool_axis_selection_max_deg','authorized_trial_scope'):
    expected[key]=trial[key]
assert expected==trial,'Trial contains changes beyond authorised frame/front domain and explicit scope'
assert not trial.get('planning_only') and trial['authorized_trial_scope']['full_descent_required']
assert not trial['authorized_trial_scope']['claw_commanded']
assert trial['continuous_grasp_search']['descent_axis_cone_deg']==0.
assert trial['continuous_grasp_search']['terminal_reference_tolerance_deg']==original['continuous_grasp_search']['terminal_reference_tolerance_deg']
assert plan['start_rad']==prep['forecast_clearance']
sys.path.insert(0,str(ROOT));import left_arm_board_grasp as board
args=argparse.Namespace(model=str(ROOT/'data/left_arm_general_dynamics.json'),board_model=str(OUT/'common_trial_config.json'),**prep['request'])
model=board.RigidBodyModel.load(args.model);cfg=board.load_board_config(args.board_model,model)
board.validate_request(args.x_mm,args.y_mm,args.piece,args.stage,args.grip_height_mm,cfg)
np.testing.assert_allclose(plan['target_board_mm'],[args.x_mm,args.y_mm,args.grip_height_mm],atol=1e-9)
beg=time.monotonic();q=np.array([plan['start_rad'][n] for n in model.names])
for row in board.execution_waypoints(plan):
    goal=np.array([row['encoder_rad'][n] for n in model.names])
    edge_cfg=board.transit_clearance_config(cfg) if row['stage'] in ('entry_clearance','entry_search','source_above') else cfg
    board.check_transit_edge(model,q,goal,edge_cfg);q=goal
orientation=board.check_planned_tool_orientation(model,cfg,plan)
clearance=board.check_planned_gripper_clearance(model,cfg,plan)
plan=board.candidate_approach_trial(plan,cfg)
(OUT/'original_board_workspace_candidate.json').write_bytes(target.read_bytes())
(OUT/'board_workspace_trial.json').write_bytes((OUT/'common_trial_config.json').read_bytes())
pending=target.with_name(target.name+'.lower25-'+str(time.time_ns())+'.pending')
with pending.open('xb') as stream:stream.write((OUT/'common_trial_config.json').read_bytes());stream.flush();os.fsync(stream.fileno())
pending.chmod(stat.S_IMODE(target.stat().st_mode));os.replace(str(pending),str(target))
assert sha(target)==prep['trial_config_sha256']
activation=dict(motion_issued=False,active_path=str(target),original_sha256=preflight['source_hashes']['data/left_arm_board_workspace_candidate.json'],
    trial_sha256=sha(target),restore_required_when_idle=True,activated_wall_s=time.time(),
    scope='authorised common envelope and explicit new requested frame; no XYZ offsets, no saved-angle replay')
(OUT/'activation_receipt.json').write_text(json.dumps(activation,indent=2))
args.board_model=str(target)
expected_sources=dict(preflight['prepared_source_hashes'],board_model=sha(target))
assert board.prepared_source_hashes(args)==expected_sources
plan=board.stamp_prepared_approach(plan,args)
path=ROOT/'data/board_workspace_trials/approach_a8_lower25_20261108.json'
assert not path.exists(),'Prepared path exists; no accidental reuse'
path.write_text(json.dumps(plan,indent=2))
board.load_prepared_approach(path,args,prep['forecast_clearance'])
(OUT/'prepared_plan.json').write_bytes(path.read_bytes())
report=dict(motion_issued=False,prepared_plan_path=str(path),prepared_plan=path.name,
    plan_sha256=sha(path),request=prep['request'],source_hashes=expected_sources,
    common_config_sha256=sha(target),original_config_sha256=activation['original_sha256'],
    waypoints=len(plan['waypoints']),execution_groups=len(board.execution_waypoints(plan)),
    orientation=orientation,gripper_clearance=clearance,remote_verification_s=time.monotonic()-beg,
    geometry_modified=False,full_path_precomputed_before_Clearance=True,nominal_A8_XYZ_consistency_resolved=False)
(OUT/'verified_plan.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
