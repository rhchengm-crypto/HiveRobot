import json,sys,shlex,subprocess
from pathlib import Path
phase=sys.argv[1]
assert phase in ('status','home','preview','approach')
expected=json.loads(Path(__file__).with_name('board_v21_deploy_result.json').read_text())['hashes']
code=r"""
import json,urllib.request,hashlib
from pathlib import Path
c=urllib.request.build_opener(urllib.request.ProxyHandler({}));base='http://127.0.0.1:8100'
s=json.load(c.open(base+'/api/v3/dynamics/status',timeout=5));run=s.get('run') or {}
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
for n,h in EXPECTED.items():assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,'code changed: '+n
motors=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:a=[v.decode() for v in (p/'cmdline').read_bytes().split(bytes([0])) if v]
 except (OSError,UnicodeError):continue
 if any(Path(v).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','left_arm_v3_0_move_library.py'} for v in a[:3]):motors.append({'pid':p.name,'argv':a})
if PHASE=='status':
 print(json.dumps({'settings':{k:s.get(k) for k in ('execution_enabled','auto_model_hold_enabled','motion_dynamics_enabled','rigid_body_mode')},'run':run,'motor_processes':motors}))
else:
 assert not run.get('running') and run.get('control_state')!='protective-hold',str(run)[:300]
 assert not motors,motors
 assert s.get('execution_enabled') and s.get('auto_model_hold_enabled') and s.get('rigid_body_mode')=='apply',s.keys()
 if PHASE=='home':endpoint='/api/action/home';body={}
 else:
  endpoint='/api/v3/dynamics/run'
  body={'action':'board-grasp','payload':{'x_mm':113.8,'y_mm':190.3,'piece':'white_knight','stage':'approach','grip_height_mm':50.2,'execute':PHASE=='approach'}}
 req=urllib.request.Request(base+endpoint,data=json.dumps(body).encode(),headers={'Content-Type':'application/json'},method='POST')
 print(json.dumps(json.load(c.open(req,timeout=10))))
""".replace('EXPECTED',repr(expected)).replace('PHASE',repr(phase))
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(code)],capture_output=True,timeout=25)
if r.returncode:raise RuntimeError(r.stderr.decode('utf8',errors='replace'))
data=json.loads(r.stdout)
Path(__file__).with_name('v21_restored_full_test_'+phase+'.json').write_bytes(r.stdout)
if phase=='status':
 run=data['run'];audits=[]
 for line in run.get('stdout','').splitlines():
  if line.startswith('board tracking audit= '):
   try:audits.append(json.loads(line.split('= ',1)[1]))
   except ValueError:pass
 progress=None
 if audits:
  a=audits[-1];progress={'stage':a['stage'],'waypoint_index':a['waypoint_index'],'axes':{n:{'q_ref':a['reference_rad'][n],'q_actual':a['feedback'][n]['pos'],'error_deg':a['error_deg'][n],'kp':a['last_sent_commands'][n]['kp'],'kd':a['last_sent_commands'][n]['kd']} for n in ('shoulder_side','shoulder_rotate','wrist_side')}}
 print(json.dumps({'settings':data['settings'],**{k:run.get(k) for k in ('action','ok','running','returncode','control_state','full_log_path','post_move_model_hold')},'progress':progress,'stderr':run.get('stderr','')[-800:]}))
else:print(json.dumps({k:data.get(k) for k in ('ok','running','action','command','error','full_log_path')}))
