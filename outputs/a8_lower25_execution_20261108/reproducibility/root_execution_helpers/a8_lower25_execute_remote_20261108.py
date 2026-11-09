from pathlib import Path
import json,urllib.request,argparse,hashlib,sys
import numpy as np
root=Path('/home/nvidia/hive_robot/DM_Control_Python');out=root/'data/board_workspace_trials/a8_lower25_execution_20261108'
c=urllib.request.build_opener(urllib.request.ProxyHandler({}));base='http://127.0.0.1:8100'
s=json.load(c.open(base+'/api/v3/dynamics/status',timeout=5));r=s.get('run') or {}
assert r.get('action')=='dynamics:model-hold-after:table-clearance'
assert not r.get('running') and not r.get('support_running') and r.get('returncode')==0 and r.get('ok')
assert r.get('post_move_model_hold',{}).get('state')=='complete'
assert s['execution_enabled'] and s['auto_model_hold_enabled'] and s['rigid_body_mode']=='apply'
assert not (out/'execution_start.json').exists(),'execution already submitted; do not resend'
(out/'clearance_complete_status.json').write_text(json.dumps(s,indent=2))
verified=json.loads((out/'verified_plan.json').read_text());path=Path(verified['prepared_plan_path'])
assert hashlib.sha256(path.read_bytes()).hexdigest()==verified['plan_sha256']
request=verified['request']
decoder=json.JSONDecoder();final=None;text=Path(r['full_log_path']).read_text()
for i,ch in enumerate(text):
    if ch!='{':continue
    try:item,_=decoder.raw_decode(text[i:])
    except ValueError:continue
    if isinstance(item,dict) and isinstance(item.get('final'),dict) and len(item['final'])==7 and all(isinstance(v,dict) and 'pos' in v and 'tau' in v for v in item['final'].values()):
        final=item['final'];break
assert final is not None,'no complete final Clearance hold feedback'
positions={n:v['pos'] for n,v in final.items()}
sys.path.insert(0,str(root));import left_arm_board_grasp as b
args=argparse.Namespace(model=str(root/'data/left_arm_general_dynamics.json'),board_model=str(root/'data/left_arm_board_workspace_candidate.json'),**request)
plan=b.load_prepared_approach(path,args,positions)
m=b.RigidBodyModel.load(args.model);cfg=b.load_board_config(args.board_model,m)
group=b.execution_waypoints(plan)[0]
b.check_transit_edge(m,np.array([positions[n] for n in m.names]),
    np.array([group['encoder_rad'][n] for n in m.names]),b.transit_clearance_config(cfg))
from left_arm_v3_1_model_control import load_model,MIT_TAU_MAX
hold=load_model(Path(root/'data/left_arm_v3_1_dynamics.json'))
limits={n:hold['joints'][n]['tau_limits'] for n in m.names}
gains,_=b.board_motion_gains(__import__('left_arm_v2_6'),(m,cfg),positions)
baseline=b.TrajectorySupportBaseline(m,positions,{n:v['tau'] for n,v in final.items()},final,gains,
    limits,MIT_TAU_MAX,plan['planned_gravity_envelope'],plan)
precheck=dict(entry_errors_deg=plan['prepared_entry_errors_deg'],first_executed_edge_checked=True,
    planned_support_limits=baseline.limits,entry_final_feedback=final,
    request=request,reference_endpoint='fresh-geometric-lower25-new-frame',original_nominal_A8_XYZ_solved=False)
(out/'execution_entry_check.json').write_text(json.dumps(precheck,indent=2))
payload={'action':'board-grasp','payload':{**request,'validate_candidate_approach':True,'prepared_plan':path.name,'execute':True}}
req=urllib.request.Request(base+'/api/v3/dynamics/run',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'},method='POST')
response=json.load(c.open(req,timeout=10))
(out/'execution_start.json').write_text(json.dumps(response,indent=2))
print(json.dumps(dict(run={k:response.get(k) for k in ('action','ok','running','error','command')},entry_precheck=precheck)))
