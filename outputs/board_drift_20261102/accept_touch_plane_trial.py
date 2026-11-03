"""Record operator acceptance and verify/snapshot Jetson; never sends motor commands."""
import json,hashlib,base64,zlib,subprocess,shlex
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
p=OUT/'touch_plane_trial_record.json';record=json.loads(p.read_text())
record['physical_validation']={
 'result':'operator-accepted for tested empty-load target and unchanged gripper opening',
 'fingertip_clearance':'clearly separated from board; visible gap',
 'B_drift_mm':2,'B_drift_operator_assessment':'acceptable working range',
 'oscillation':False,
 'extra_post_move_displacement':'not separately specified in operator reply',
 'scope':'same target113.8/190.3/50.2mm, fixed camera/board, empty-load approach; does not certify whole-board accuracy or loaded grasp',
}
record['next_action']='Retain accepted model and gains. No additional parameter change, descent, claw closure, or Home was requested after acceptance.'
local_model=ROOT/'scripts/data/left_arm_board_grasp.json';expected=hashlib.sha256(local_model.read_bytes()).hexdigest();assert expected==record['model_sha256']
canonical_log_sha=hashlib.sha256(Path(record['source_log']).read_text(encoding='utf8').encode()).hexdigest()
record['canonical_LF_log_sha256']=canonical_log_sha
remote=r'''
import json,hashlib,zlib,base64,urllib.request
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python');data=root/'data'
assert hashlib.sha256((data/'left_arm_board_grasp.json').read_bytes()).hexdigest()==EXPECTED
log=data/'left_arm_v2_8_full_runs/arm_run_1793681494917_16715.txt'
assert hashlib.sha256(log.read_text().encode()).hexdigest()==LOG_SHA,'Canonical log text changed'
c=urllib.request.build_opener(urllib.request.ProxyHandler({}));run=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5)).get('run')or{}
assert not run.get('running'),'A new task started; do not mark current state idle'
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:args=[a.decode() for a in (proc/'cmdline').read_bytes().split(bytes([0])) if a]
 except (OSError,UnicodeError):continue
 assert not any(Path(a).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','left_arm_v3_0_move_library.py'} for a in args[:3]),args
folder=data/'board_validation';folder.mkdir(exist_ok=True)
evidence=folder/'v24_1793681494917_operator_acceptance.json';evidence.write_text(json.dumps(RECORD,indent=2))
names=['left_arm_general_dynamics.json','left_arm_board_grasp.json','left_arm_v3_1_dynamics.json','left_arm_v2_6_adaptive_hold_tau.json','left_arm_v2_6_adaptive_active_tau.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_6_adaptive_hold_target_bias.json']
files={n:(data/n).read_text() for n in names if (data/n).exists()}
result={'files':files,'remote_acceptance_record':str(evidence),'model_sha256':EXPECTED,'source_log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'canonical_LF_log_sha256':LOG_SHA,'running':False,'motion_issued':False,'action':run.get('action')}
print(base64.b64encode(zlib.compress(json.dumps(result).encode(),9)).decode())
'''.replace('EXPECTED',repr(expected)).replace('LOG_SHA',repr(canonical_log_sha)).replace('RECORD',repr(record))
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(remote.encode(),9)),capture_output=True,timeout=40)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
result=json.loads(zlib.decompress(base64.b64decode(r.stdout)));dest=OUT/'jetson_snapshot_v24_accepted';dest.mkdir(exist_ok=True)
files=result.pop('files');hashes={}
for name,text in files.items():
 assert Path(name).name==name
 (dest/name).write_text(text,encoding='utf8');hashes[name]=hashlib.sha256((dest/name).read_bytes()).hexdigest()
record['remote_acceptance_record']=result['remote_acceptance_record'];p.write_text(json.dumps(record,indent=2))
result['snapshot_files']=hashes;(OUT/'accepted_v24_sync_manifest.json').write_text(json.dumps(result,indent=2))
(OUT/'model_identification_status.json').write_text(json.dumps({'active_build':record['build'],'active_model_sha256':expected,'local_test_result':record['physical_validation'],'global_accuracy_validated':False,'stopped_after_trial':True,'further_motion_issued':False,'do_not_retune_accepted_2mm_drift':True,'unchanged':['model hold serial takeover','adaptive learning','ordinary Clearance skips shoulder_rotate','shoulder_rotate cap36/kd4.5'],'accepted_log':'arm_run_1793681494917_16715.txt','remote_acceptance_record':result['remote_acceptance_record']},indent=2))
print(json.dumps({'accepted':True,'local_remote_model_match':True,'snapshot_file_count':len(files),'remote_acceptance_record':result['remote_acceptance_record'],'running':False,'motion_issued':False}))
