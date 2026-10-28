"""Code-only upload and idle Web restart; never commands robot hardware."""
import base64,json,shlex,subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[2]
files={n:base64.b64encode((root/'scripts'/n).read_bytes()).decode()
       for n in ('left_arm_board_grasp.py','left_arm_board_kinematics.py','left_arm_v3_1_model_control.py','chessboard_vision_v3_0_web_control.py','left_arm_fault_support.py','record_left_arm_rgbd_audit.py','LEFT_ARM_BOARD_GRASP_DEPLOY.md')}
previous=json.loads(Path(__file__).with_name('board_v20_deploy_result.json').read_text())['hashes']
remote=r'''
import json,base64,hashlib,os,signal,subprocess,time,datetime,shlex,urllib.request
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def status():return json.load(client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5))
def processes():
 result=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try: a=[v.decode() for v in (p/'cmdline').read_bytes().split(b'\0') if v]
  except (OSError,UnicodeError):continue
  result.append((int(p.name),a))
 return result
motor_files={'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','left_arm_v3_0_move_library.py'}
def active():return [(p,a[:3]) for p,a in processes() if any(Path(v).name in motor_files for v in a[:3])]
run=status().get('run') or {}
assert not run.get('running') and not run.get('supporting_load') and run.get('control_state')!='protective-hold',run
assert not active(),active()
servers=[(p,a) for p,a in processes() if any(Path(v).name=='chessboard_vision_v3_0_web_control.py' for v in a[:3])]
assert len(servers)==1,servers
pid,argv=servers[0]
def protected():return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'data').glob('*.json')}
before=protected();stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
backup=root/'data/code_backups'/('board_v21_'+stamp);backup.mkdir(parents=True)
payload=json.loads(__import__('sys').stdin.read())
expected=payload.pop('__expected__')
for name,digest in expected.items():
 assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,'Jetson code changed: '+name
for name,encoded in payload.items():
 data=base64.b64decode(encoded);compile(data,name,'exec') if name.endswith('.py') else None
 (backup/name).write_bytes((root/name).read_bytes()) if (root/name).exists() else None
 temp=root/('.'+name+'.v21');temp.write_bytes(data);os.replace(str(temp),str(root/name))
assert before==protected()
assert not active(),active()
run=status().get('run') or {};assert not run.get('running'),run
os.kill(pid,signal.SIGTERM)
deadline=time.monotonic()+5
while Path('/proc/'+str(pid)).exists() and time.monotonic()<deadline:time.sleep(.2)
assert not Path('/proc/'+str(pid)).exists()
log=root/'data'/('web_control_v21_'+stamp+'.log')
shell='source /opt/ros/noetic/setup.bash && source /home/nvidia/ascam_ws/devel/setup.bash && exec '+shlex.join(argv)
with log.open('wb') as handle:
 proc=subprocess.Popen(['/bin/bash','-c',shell],cwd=str(root),stdin=subprocess.DEVNULL,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
deadline=time.monotonic()+15
while time.monotonic()<deadline:
 if proc.poll() is not None:raise RuntimeError(log.read_text()[-2000:])
 try:status();break
 except Exception:time.sleep(.5)
else:raise RuntimeError('Web restart timeout')
assert not active(),active()
assert before==protected()
print(json.dumps({'ok':True,'web_pid':proc.pid,'backup':str(backup),'log':str(log),
 'protected_json_unchanged':True,'motion_issued':False,'hashes':{n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in payload}}))
'''
cmd=['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(remote)]
r=subprocess.run(cmd,input=json.dumps({**files,'__expected__':previous}).encode(),capture_output=True,timeout=55)
if r.returncode:raise RuntimeError(r.stderr.decode('utf8',errors='replace'))
print(r.stdout.decode('utf8'));Path(__file__).with_name('board_v21_deploy_result.json').write_bytes(r.stdout)
