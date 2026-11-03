"""Back up and atomically deploy two checked Python files; never starts motion."""
import base64,hashlib,json,shlex,subprocess,zlib
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
files={n:base64.b64encode((ROOT/'scripts'/n).read_bytes()).decode() for n in ['left_arm_board_grasp.py','left_arm_board_kinematics.py']}
remote=r'''
import base64,hashlib,json,urllib.request,time,os
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
c=urllib.request.build_opener(urllib.request.ProxyHandler({}))
run=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5)).get('run') or {}
assert not run.get('running'), 'Existing motion task'
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:a=[v.decode() for v in (p/'cmdline').read_bytes().split(bytes([0])) if v]
 except (OSError,UnicodeError):continue
 assert not any(Path(v).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','left_arm_v3_0_move_library.py'} for v in a[:3]),a
backup=root/'data'/'code_backups'/('v22_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()));backup.mkdir(parents=True)
report={}
for name,payload in FILES.items():
 p=root/name;data=base64.b64decode(payload);compile(data,str(p),'exec')
 old=p.read_bytes();(backup/name).write_bytes(old)
 tmp=p.with_name(p.name+'.v22.tmp');tmp.write_bytes(data);os.replace(tmp,p)
 report[name]={'old_sha256':hashlib.sha256(old).hexdigest(),'new_sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
print(json.dumps({'motion_issued':False,'backup':str(backup),'files':report}))
'''.replace('FILES',repr(files))
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(remote.encode(),9)),capture_output=True,timeout=25)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
data=json.loads(r.stdout)
for n,v in data['files'].items():assert v['new_sha256']==hashlib.sha256((ROOT/'scripts'/n).read_bytes()).hexdigest()
(OUT/'v22_deployment.json').write_text(json.dumps(data,indent=2));print(json.dumps(data))
