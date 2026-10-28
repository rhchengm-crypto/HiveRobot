import json,subprocess,shlex
from pathlib import Path
out=Path(__file__).parent;capture=json.loads((out/'rgbd_axis_audit_capture.json').read_text());expected=json.loads((out/'board_v19_deploy_result.json').read_text())['hashes']
code=r'''
import json,urllib.request,time,hashlib
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python');dest=Path(OUTPUT)/'axis_commands.jsonl';client=urllib.request.build_opener(urllib.request.ProxyHandler({}));base='http://127.0.0.1:8100'
def status():return json.load(client.open(base+'/api/v3/dynamics/status',timeout=5))
def idle():
 s=status();r=s.get('run') or {};assert not r.get('running') and r.get('control_state')!='protective-hold',str(r)[:400]
 assert s.get('execution_enabled') and s.get('auto_model_hold_enabled') and s.get('rigid_body_mode')=='apply'
 for n,h in EXPECTED.items():assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,n
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:a=[v.decode() for v in (p/'cmdline').read_bytes().split(bytes([0])) if v]
  except (OSError,UnicodeError):continue
  assert not any(Path(v).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py'} for v in a[:3]),(p.name,a[:3])
def wait_run(command):
 deadline=time.monotonic()+45
 while time.monotonic()<deadline:
  r=status().get('run') or {}
  assert [x for x in r.get('command',[]) if x!='--inline-model-hold']==[x for x in command if x!='--inline-model-hold'],'Command changed externally; do not continue'
  if not r.get('running'):
   assert r.get('ok') and r.get('returncode')==0,str(r)[-1500:]
   return r
  time.sleep(1)
 raise RuntimeError('Timed out waiting; do not resubmit')
for joint in ['shoulder_rotate','shoulder_side','wrist_side']:
 for step in [3.,-3.]:
  for execute in [False,True]:
   idle();payload={'action':'board-grasp','payload':{'x_mm':113.8,'y_mm':190.3,'piece':'white_knight','stage':'cartesian-check','direction_check_joint':joint,'direction_check_step_deg':step,'execute':execute}}
   req=urllib.request.Request(base+'/api/v3/dynamics/run',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'},method='POST')
   result=json.load(client.open(req,timeout=10));assert result.get('ok'),result
   finished=wait_run(result['command']);row={'joint':joint,'step_deg':step,'execute':execute,'run':finished}
   with dest.open('a') as f:f.write(json.dumps(row)+'\n')
   print(json.dumps({'joint':joint,'step_deg':step,'execute':execute,'ok':True,'full_log_path':finished['full_log_path']}),flush=True)
print(json.dumps({'complete':True,'log':str(dest)}),flush=True)
'''.replace('OUTPUT',repr(capture['output'])).replace('EXPECTED',repr(expected))
p=subprocess.Popen(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -u -c '+shlex.quote(code)],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
with (out/'v19_axis_sequence_progress.jsonl').open('w') as f:
 for raw in p.stdout:
  text=raw.decode('utf8');print(text,end='',flush=True);f.write(text);f.flush()
error=p.stderr.read().decode('utf8',errors='replace');rc=p.wait()
if rc:raise RuntimeError(error)
