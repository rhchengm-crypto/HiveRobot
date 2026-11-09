from pathlib import Path
import urllib.request,json,hashlib,time
root=Path('/home/nvidia/hive_robot/DM_Control_Python');out=root/'data/board_workspace_trials/a8_lower25_execution_20261108'
c=urllib.request.build_opener(urllib.request.ProxyHandler({}));base='http://127.0.0.1:8100'
s=json.load(c.open(base+'/api/v3/dynamics/status',timeout=5));r=s.get('run') or {}
assert not r.get('running') and not r.get('support_running') and r.get('returncode')==0
assert r.get('action')=='dynamics:model-hold-after:home' and r.get('post_move_model_hold',{}).get('state')=='complete'
assert s['execution_enabled'] and s['auto_model_hold_enabled'] and s['rigid_body_mode']=='apply'
assert not (out/'clearance_start.json').exists(),'Clearance already submitted'
v=json.loads((out/'verified_plan.json').read_text());p=Path(v['prepared_plan_path'])
assert hashlib.sha256(p.read_bytes()).hexdigest()==v['plan_sha256']
assert hashlib.sha256((root/'data/left_arm_board_workspace_candidate.json').read_bytes()).hexdigest()==v['common_config_sha256']
assert time.time()-json.loads(p.read_text())['prepared_execution']['created_wall_s']<3300
(out/'home_complete_status.json').write_text(json.dumps(s,indent=2))
req=urllib.request.Request(base+'/api/action/table-clearance',data=b'{}',headers={'Content-Type':'application/json'},method='POST')
result=json.load(c.open(req,timeout=10));(out/'clearance_start.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:result.get(k) for k in ('action','ok','running','error','command','full_log_path')}))
