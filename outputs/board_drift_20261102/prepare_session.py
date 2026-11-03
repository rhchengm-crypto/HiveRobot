"""Prepare the existing Web/camera session; do not submit any motion request."""
import json, subprocess, shlex
from pathlib import Path
OUT=Path(__file__).resolve().parent
code=r'''
import json,socket,subprocess,time,urllib.request,hashlib,os
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
def port(p):
    with socket.socket() as s:
        s.settimeout(1);return s.connect_ex(('127.0.0.1',p))==0
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
result={'motion_issued':False,'ros_master_available':port(11311),'web_available':port(8100)}
active=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:a=[v.decode() for v in (p/'cmdline').read_bytes().split(b'\0') if v]
    except (OSError,UnicodeError):continue
    if any(Path(v).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','left_arm_v3_0_move_library.py'} for v in a[:3]):
        active.append(int(p.name))
result['motor_processes']=active
if active:raise RuntimeError('Existing motor process; preparation stopped')
if result['ros_master_available'] and not result['web_available']:
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'data').glob('*.json')}
    stamp=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
    log=root/'data'/('web_resume_drift_'+stamp+'.log')
    shell='source /opt/ros/noetic/setup.bash && source /home/nvidia/ascam_ws/devel/setup.bash && exec python3 -u chessboard_vision_v3_0_web_control.py --host 0.0.0.0 --port 8100 --python-bin python3 --arm-script left_arm_v2_8.py --move-script left_arm_v3_0_move_library.py --enable-execute --dynamics-model data/left_arm_v3_1_dynamics.json --motion-dynamics --rigid-body-model data/left_arm_general_dynamics.json --rigid-body-mode apply'
    with log.open('w') as f:
        child=subprocess.Popen(['/bin/bash','-c',shell],cwd=str(root),stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    result.update(started_web_pid=child.pid,web_log=str(log))
    for _ in range(24):
        if child.poll() is not None:raise RuntimeError(log.read_text()[-1500:])
        if port(8100):break
        time.sleep(.5)
    after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'data').glob('*.json')}
    result['protected_json_unchanged']=before==after
if port(8100):
    with client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5) as f:s=json.load(f)
    result['settings']={k:s.get(k) for k in ('execution_enabled','auto_model_hold_enabled','motion_dynamics_enabled','rigid_body_mode')}
    result['run']={k:(s.get('run')or{}).get(k) for k in ('action','running','control_state')}
if result['ros_master_available']:
    import xmlrpc.client
    state=xmlrpc.client.ServerProxy('http://127.0.0.1:11311').getPublishedTopics('/hive_readonly_probe','')[2]
    result['camera_topics']=[p for p in state if 'ascamera' in p[0]]
print(json.dumps(result))
'''
p=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(code)],capture_output=True,timeout=30)
if p.returncode:raise RuntimeError(p.stderr.decode(errors='replace'))
result=json.loads(p.stdout);(OUT/'session_preparation.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps(result))
