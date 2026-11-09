import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import json,copy,hashlib,sys,math,collections
import numpy as np
sys.path.insert(0,'scripts')
import left_arm_board_grasp as b
from left_arm_trajectory_support import gravity_envelope
root=Path.cwd();out=root/'outputs/a8_reference_execution_20261108';out.mkdir(exist_ok=True)
previous=root/'outputs/a8_saved_reference_20261108'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
context=read(out/'preflight.json');active=root/'scripts/data/left_arm_board_workspace_candidate.json'
assert hashlib.sha256(active.read_bytes()).hexdigest()==context['source_hashes']['data/left_arm_board_workspace_candidate.json']
candidate=read(previous/'shared_reference_candidate.json')
provenance=candidate.pop('pending_reference_provenance')
provenance.update(runtime_enabled=True,full_inverse_path_physically_validated=False,
    execution_authorization='2026-11-08 user: 现在实测; empty load, no claw; operator present and path cleared',
    nominal_A8_XYZ_consistency_resolved=False)
candidate['operator_approved_endpoint_reference_provenance']=provenance
candidate.setdefault('operating_bound_adjustments',{})['shoulder_front_observed_extension_20261108']=dict(
    old_lower_rad=read(active)['joint_limits_encoder_rad'][0][0],
    lower_rad=candidate['joint_limits_encoder_rad'][0][0],
    reason='include user-demonstrated A8_take encoder endpoint in shared observed envelope',
    extra_margin_rad=0.,mechanical_limit_independently_measured=False,applies_to_all_xy=True)
for key in ('tcp_in_wrist_frame_m','board_to_cad_rotation','board_to_cad_translation_m','profiles',
            'body_collision','motion_feedforward_policy','continuous_grasp_search','required_clearance_entry'):
    assert candidate[key]==read(active)[key],key
(out/'before_board_workspace_candidate.json').write_bytes(active.read_bytes())
(out/'board_workspace_candidate.json').write_text(json.dumps(candidate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
cfg=b.load_board_config(out/'board_workspace_candidate.json',b.RigidBodyModel.load('scripts/data/left_arm_general_dynamics.json'))
m=b.RigidBodyModel.load('scripts/data/left_arm_general_dynamics.json')
plan=read(previous/'reference_inverse_plan_hover140.0.json')
home=read(root/'scripts/data/left_arm_v2_home.json')['joints']
forecast=read(root/'scripts/data/left_arm_v2_table_clearance.json')['joints']
forecast['shoulder_rotate']=home['shoulder_rotate']
plan['start_rad']=forecast
plan['approved_task_pose_reference']='saved:A8_take'
plan['reference_endpoint_task']=dict(name='A8_take',nominal_square='A8',
    physical_grasp_endpoint_user_accepted=True,original_nominal_XYZ_request_solved=False,
    target_source='FK of user-approved physical endpoint; not an arbitrary XY solution or coordinate compensation')
plan['orientation_acceptance']=b.check_planned_tool_orientation(m,cfg,plan)
plan['gripper_clearance_acceptance']=b.check_planned_gripper_clearance(m,cfg,plan)
q=np.array([forecast[n] for n in m.names])
for row in plan['waypoints']:
    goal=np.array([row['encoder_rad'][n] for n in m.names]);b.check_transit_edge(m,q,goal,cfg);q=goal
pose=b.tcp_pose(m,q,cfg['tcp_in_wrist_frame_m'])
point=(pose[:3,3]-np.array(cfg['board_to_cad_translation_m']))@np.array(cfg['board_to_cad_rotation'])*1000
np.testing.assert_allclose(point,plan['target_board_mm'],atol=1e-8)
saved=read(previous/'saved_pose_audit.json')['saved_move']['pose']
assert plan['waypoints'][-1]['encoder_rad']==saved
plan=b.candidate_approach_trial(plan,cfg)
plan['planned_gravity_envelope']=gravity_envelope(m,plan)
plan.pop('planning_only',None);plan.pop('execution_eligible',None)
groups=b.execution_waypoints(plan)
q=np.array([forecast[n] for n in m.names])
for row in groups:
    goal=np.array([row['encoder_rad'][n] for n in m.names]);b.check_transit_edge(m,q,goal,cfg);q=goal
(out/'local_plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2),encoding='utf-8')
request=dict(x_mm=point[0],y_mm=point[1],piece='white_knight',stage='approach',grip_height_mm=point[2])
report=dict(motion_issued=False,request=request,reference_name='A8_take',nominal_square='A8',
    original_nominal_XYZ_request_solved=False,forecast_clearance=forecast,
    source_hashes_before=context['source_hashes'],
    deployed_candidate_sha256=hashlib.sha256((out/'board_workspace_candidate.json').read_bytes()).hexdigest(),
    local_plan_sha256=hashlib.sha256((out/'local_plan.json').read_bytes()).hexdigest(),
    waypoints=len(plan['waypoints']),execution_groups=len(groups),
    stages=dict(collections.Counter(r['stage'] for r in plan['waypoints'])),
    orientation=plan['orientation_acceptance'],gripper_clearance=plan['gripper_clearance_acceptance'],
    operator_preflight='已清空、未夹棋子，我在旁可急停',
    all_geometry_precomputed_before_Home=True,
    shared_model_changes='one common approved endpoint reference/seed plus observed shoulder-front envelope; geometry, profiles, gains and controller unchanged')
(out/'preparation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False))
