"""Read and retain the explicitly requested closing Home; never issues motion."""
import base64,hashlib,json,shlex,subprocess,zlib
from pathlib import Path
OUT=Path(__file__).resolve().parent
code=r'''
import base64,hashlib,json,urllib.request,zlib
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
status=json.load(client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5));run=status.get('run') or {}
assert not run.get('running') and run.get('action')=='dynamics:model-hold-after:home'
names=['data/left_arm_v2_8_full_runs/arm_run_1793688050475_27058.txt','data/left_arm_v2_8_full_runs/arm_run_1793688109693_27238.txt']
names+=['data/'+n for n in ['left_arm_v2_6_adaptive_active_tau.json','left_arm_v2_6_adaptive_hold_tau.json','left_arm_v2_6_adaptive_hold_target_bias.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_8_clearance_bias.json','left_arm_v2_8_local_target_bias.json','left_arm_v2_8_placement1_clearance_bias.json','left_arm_v2_8_placement_c4_clearance_bias.json']]
files=[]
for name in names:
 p=root/name;raw=p.read_bytes();files.append({'path':name,'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest(),'base64':base64.b64encode(raw).decode()})
payload={'status':status,'files':files,'motion_issued_by_collector':False}
print(base64.b64encode(zlib.compress(json.dumps(payload).encode(),9)).decode())
'''
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(code.encode(),9)),capture_output=True,timeout=35)
assert r.returncode==0,r.stderr.decode(errors='replace')
payload=json.loads(zlib.decompress(base64.b64decode(r.stdout)))
(OUT/'web_status_complete.json').write_text(json.dumps(payload['status'],indent=2));manifest=[]
for row in payload['files']:
 raw=base64.b64decode(row.pop('base64'));assert hashlib.sha256(raw).hexdigest()==row['sha256']
 p=OUT/'jetson_raw'/row['path'];p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(raw);manifest.append(row)
(OUT/'manifest.json').write_text(json.dumps({'source_root':'/home/nvidia/hive_robot/DM_Control_Python','files':manifest,'motion_issued_by_collector':False},indent=2))
run=payload['status']['run'];text=(OUT/'jetson_raw/data/left_arm_v2_8_full_runs/arm_run_1793688109693_27238.txt').read_text();decoder=json.JSONDecoder();hold=None
for i,c in enumerate(text):
 if c!='{':continue
 try:v,_=decoder.raw_decode(text[i:])
 except ValueError:continue
 if isinstance(v,dict) and v.get('reason')=='duration_complete':hold=v
assert run.get('ok') and run.get('returncode')==0 and hold and hold.get('ok')
motion=run.get('preceding_motion') or {};assert motion.get('action')=='home' and motion.get('ok') and motion.get('returncode')==0
summary={'user_instruction':'After writing handoff, return the arm Home','home_log':'arm_run_1793688050475_27058.txt','model_hold_log':'arm_run_1793688109693_27238.txt','action':run['action'],'ok':run['ok'],'running':run['running'],'returncode':run['returncode'],'hold_reason':hold['reason'],'hold_duration_s':hold.get('duration_s'),'final':hold.get('final'),'max_abs_error_deg':hold.get('max_abs_error_deg'),'max_velocity_rad_s':hold.get('max_velocity_rad_s'),'physical_oscillation':'not independently reported for this closing Home; do not infer from logs alone','latest_position':'Home; supersedes earlier V24 end-pose archival snapshot'}
summary.update({'home_ok':motion['ok'],'home_returncode':motion['returncode'],'home_started_at_local':motion['started_at'],'home_finished_at_local':motion['finished_at'],'hold_finished_at_local':run['finished_at'],'home_command':motion['command']})
(OUT/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
