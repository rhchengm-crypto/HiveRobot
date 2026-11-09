import os
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import json,hashlib,sys,argparse,urllib.request,time,numpy as np
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
out=root/'data/board_workspace_trials/a8_reference_execution_20261108'
read=lambda p:json.loads(Path(p).read_text())
prep=read(out/'preparation.json')
c=urllib.request.build_opener(urllib.request.ProxyHandler({}))
s=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5));run=s.get('run') or {}
assert not run.get('running') and not run.get('support_running')
for name,digest in prep['source_hashes_before'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
assert hashlib.sha256((out/'board_workspace_candidate.json').read_bytes()).hexdigest()==prep['deployed_candidate_sha256']
assert hashlib.sha256((out/'local_plan.json').read_bytes()).hexdigest()==prep['local_plan_sha256']
sys.path.insert(0,str(root));import left_arm_board_grasp as b
m=b.RigidBodyModel.load(root/'data/left_arm_general_dynamics.json')
cfg=b.load_board_config(out/'board_workspace_candidate.json',m)
plan=read(out/'local_plan.json');request=prep['request']
b.validate_request(request['x_mm'],request['y_mm'],request['piece'],request['stage'],request['grip_height_mm'],cfg)
assert plan['start_rad']==prep['forecast_clearance']
assert np.allclose(plan['target_board_mm'],[request['x_mm'],request['y_mm'],request['grip_height_mm']],atol=1e-10)
assert plan['waypoints'][-1]['encoder_rad']==read(root/'data/left_arm_v2_6_saved_moves.json')['moves']['A8_take']['pose']
beg=time.monotonic();q=np.array([plan['start_rad'][n] for n in m.names])
for row in plan['waypoints']:
    goal=np.array([row['encoder_rad'][n] for n in m.names]);b.check_transit_edge(m,q,goal,cfg);q=goal
clearance=b.check_planned_gripper_clearance(m,cfg,plan)
plan['orientation_acceptance']=b.check_planned_tool_orientation(m,cfg,plan)
plan=b.candidate_approach_trial(plan,cfg)
active=root/'data/left_arm_board_workspace_candidate.json'
frozen=out/'runtime_snapshot';frozen.mkdir(exist_ok=True)
for name in prep['source_hashes_before']:
    dest=frozen/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes((root/name).read_bytes())
(out/'before_board_workspace_candidate.json').write_bytes(active.read_bytes())
temporary=active.with_suffix('.a8_reference_test.tmp')
temporary.write_bytes((out/'board_workspace_candidate.json').read_bytes());os.replace(temporary,active)
assert hashlib.sha256(active.read_bytes()).hexdigest()==prep['deployed_candidate_sha256']
args=argparse.Namespace(model=str(root/'data/left_arm_general_dynamics.json'),board_model=str(active),**request)
plan=b.stamp_prepared_approach(plan,args)
dest=root/'data/board_workspace_trials/approach_a8_reference_20261108.json'
assert not dest.exists(),'prepared artifact already exists; inspect before any repeat'
dest.write_text(json.dumps(plan,indent=2));b.load_prepared_approach(dest,args,prep['forecast_clearance'])
(out/'prepared_plan.json').write_bytes(dest.read_bytes())
report=dict(motion_issued=False,prepared_plan=dest.name,prepared_plan_path=str(dest),
    plan_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),request=request,
    common_config_sha256=prep['deployed_candidate_sha256'],previous_config_sha256=prep['source_hashes_before']['data/left_arm_board_workspace_candidate.json'],
    source_hashes=plan['prepared_execution']['source_hashes'],waypoints=len(plan['waypoints']),
    reference_endpoint='A8_take',original_nominal_A8_XYZ_solved=False,
    orientation=plan['orientation_acceptance'],gripper_clearance=clearance,
    gravity_envelope=plan['planned_gravity_envelope'],remote_check_s=time.monotonic()-beg,
    runtime_code_changed=False,shared_reference_configuration_activated=True,
    full_path_precomputed_before_Home=True,operator_preflight=prep['operator_preflight'])
(out/'verified_plan.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
