import json,shlex,subprocess,sys
from pathlib import Path
execute='--execute' in sys.argv
remote=r'''
import json,time,urllib.request
from pathlib import Path
c=urllib.request.build_opener(urllib.request.ProxyHandler({}))
base='http://127.0.0.1:8100'
def get():return json.load(c.open(base+'/api/v3/dynamics/status',timeout=5))
s=get();r=s.get('run') or {}
assert not r.get('running') and r.get('control_state')!='protective-hold',r
assert s.get('auto_model_hold_enabled'), 'automatic Model Hold is not enabled'
payload={'action':'board-grasp','payload':{'x_mm':104.6,'y_mm':183.7,'piece':'white_knight','stage':'touch','grip_height_mm':64.6,'execute':EXECUTE}}
request=urllib.request.Request(base+'/api/v3/dynamics/run',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'},method='POST')
accepted=json.load(c.open(request,timeout=10))
assert accepted.get('ok'),accepted
print(json.dumps({'submitted':True,'execute':EXECUTE,'command':accepted.get('command')}),flush=True)
deadline=time.monotonic()+45
while time.monotonic()<deadline:
 time.sleep(.5);r=get().get('run') or {}
 if r.get('control_state')=='protective-hold':break
 if not r.get('running') and r.get('finished_at'):break
else:raise RuntimeError('Task still running after observation timeout; do not resubmit')
print(json.dumps(r),flush=True)
'''.replace('EXECUTE',repr(execute))
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(remote)],capture_output=True,timeout=65)
name='auto_touch_v12_execute_result.txt' if execute else 'auto_touch_v12_preview_result.txt'
Path(__file__).with_name(name).write_bytes(r.stdout)
if r.returncode:raise RuntimeError(r.stderr.decode('utf8',errors='replace'))
lines=r.stdout.decode('utf8').splitlines();data=json.loads(lines[-1]);print(json.dumps({k:data.get(k) for k in ('ok','running','action','returncode','full_log_path','stderr','control_state')}))
print(data.get('stdout','')[-1200:])
