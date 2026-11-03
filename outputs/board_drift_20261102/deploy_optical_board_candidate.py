"""Back up and deploy only a validated optical board registration candidate."""
import json,hashlib,subprocess,shlex,base64,zlib
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
verification=json.loads((OUT/'optical_candidate_plan_verification.json').read_text());assert verification['passed']
name='data/left_arm_board_grasp.json';destination=ROOT/'scripts'/name;old=destination.read_bytes();data=(OUT/'left_arm_board_grasp_optical_candidate.json').read_bytes()
code=r'''
import json,hashlib,base64,time,os,urllib.request
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python');p=root/'data/left_arm_board_grasp.json'
assert hashlib.sha256(p.read_bytes()).hexdigest()==EXPECTED,'Remote board model changed'
c=urllib.request.build_opener(urllib.request.ProxyHandler({}));run=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5)).get('run')or{}
assert not run.get('running'),'Existing motion task'
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:args=[a.decode() for a in (proc/'cmdline').read_bytes().split(bytes([0])) if a]
 except (OSError,UnicodeError):continue
 assert not any(Path(a).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','left_arm_v3_0_move_library.py'} for a in args[:3]),args
data=base64.b64decode(PAYLOAD);candidate=json.loads(data)
from left_arm_board_grasp import load_board_config
from left_arm_rigid_body import RigidBodyModel
model=RigidBodyModel.load(root/'data/left_arm_general_dynamics.json')
backup=root/'data/code_backups'/('v23_optical_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()));backup.mkdir(parents=True)
(backup/p.name).write_bytes(p.read_bytes());temp=p.with_name(p.name+'.v23.tmp');temp.write_bytes(data)
load_board_config(temp,model);os.replace(temp,p)
print(json.dumps({'motion_issued':False,'backup':str(backup/p.name),'old_sha256':EXPECTED,'new_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'scope':'board registration only; no motor-controller or learning-file change'}))
'''.replace('EXPECTED',repr(hashlib.sha256(old).hexdigest())).replace('PAYLOAD',repr(base64.b64encode(data).decode()))
bootstrap="import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))"
shell='cd /home/nvidia/hive_robot/DM_Control_Python && exec python3 -c '+shlex.quote(bootstrap)
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199',shell],input=base64.b64encode(zlib.compress(code.encode(),9)),capture_output=True,timeout=30)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
result=json.loads(r.stdout);assert result['new_sha256']==hashlib.sha256(data).hexdigest()
(OUT/'board_model_before_optical.json').write_bytes(old);destination.write_bytes(data)
(OUT/'optical_registration_deployment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
