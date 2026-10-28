import json,shlex,subprocess
from pathlib import Path
code=r"""
import sys,json,time,urllib.request
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python');sys.path.insert(0,str(root))
c=urllib.request.build_opener(urllib.request.ProxyHandler({}))
s=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5));run=s.get('run') or {}
assert not run.get('running') and run.get('control_state')!='protective-hold',run
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:a=[v.decode() for v in (p/'cmdline').read_bytes().split(b'\0') if v]
 except (OSError,UnicodeError):continue
 assert not any(Path(v).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','left_arm_v3_0_move_library.py'} for v in a[:3]),a[:3]
import left_arm_v2_6 as legacy
arm=legacy.LeftArmV2();frames={};original=arm.ctrl._MotorControl__process_packet
names={id(v):n for n,v in arm.motors.items()}
def observe(data,canid,cmd):
 motor=arm.ctrl.motors_map.get(canid) if canid else arm.ctrl.motors_map.get(data[0]&15)
 if cmd==0x11 and motor is not None:
  n=names.get(id(motor));frames.setdefault(n,[]).append(dict(canid=canid,raw_data=list(data),byte0_high_nibble=int(data[0]>>4),received_monotonic=time.monotonic()))
 return original(data,canid,cmd)
arm.ctrl._MotorControl__process_packet=observe
try:
 samples=[]
 for i in range(5):
  samples.append(arm.read_status(legacy.DEFAULT_JOINTS));time.sleep(.1)
 print(json.dumps({'motion_issued':False,'enable_disable_sent':False,'mit_setpoints_sent':False,'samples':samples,'raw_frames':frames}))
finally:arm.close()
"""
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(code)],capture_output=True,timeout=30)
if r.returncode:raise RuntimeError(r.stderr.decode('utf8',errors='replace'))
lines=r.stdout.decode('utf8').splitlines();data=json.loads(lines[-1]);Path(__file__).with_name('motor_raw_state_audit.json').write_text(json.dumps(data,indent=2))
print(json.dumps({'motion_issued':data['motion_issued'],'shoulder_rotate':data['samples'][-1]['shoulder_rotate'],'raw':data['raw_frames'].get('shoulder_rotate',[])[-1:], 'counts':{n:len(v) for n,v in data['raw_frames'].items()}}))
