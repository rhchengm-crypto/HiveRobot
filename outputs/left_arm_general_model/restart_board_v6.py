"""Run only after the operator confirms full mechanical support."""
import json
from pathlib import Path
import shlex
import subprocess

REMOTE = '''
import datetime,json,os,signal,subprocess,time,urllib.request
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def programs():
    found=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:args=[v.decode() for v in (p/'cmdline').read_bytes().split(b'\\0') if v]
        except (OSError,UnicodeError):continue
        found.append((int(p.name),args))
    return found
servers=[(pid,args) for pid,args in programs() if any(Path(v).name=='chessboard_vision_v3_0_web_control.py' for v in args[:3])]
assert len(servers)==1,servers
old_pid,argv=servers[0]
assert old_pid==50851,'Web PID changed; inspect before stopping'
request=urllib.request.Request('http://127.0.0.1:8100/api/action/emergency-stop',
                             data=b'{}',headers={'Content-Type':'application/json'},method='POST')
stop=json.load(client.open(request,timeout=15))
assert stop.get('ok') is True,stop
motor_files={'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','left_arm_v3_0_move_library.py'}
def active():
    return [(pid,args[:3]) for pid,args in programs() if any(Path(v).name in motor_files for v in args[:3])]
deadline=time.monotonic()+5
while active() and time.monotonic()<deadline:time.sleep(.2)
assert not active(),active()
os.kill(old_pid,signal.SIGTERM)
deadline=time.monotonic()+5
while Path('/proc/'+str(old_pid)).exists() and time.monotonic()<deadline:time.sleep(.2)
assert not Path('/proc/'+str(old_pid)).exists(),'old Web did not exit'
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
log=root/'data'/('web_control_v6_ros_'+stamp+'.log')
import shlex
shell='source /opt/ros/noetic/setup.bash && source /home/nvidia/ascam_ws/devel/setup.bash && exec '+shlex.join(argv)
with log.open('wb') as handle:
    proc=subprocess.Popen(['/bin/bash','-c',shell],cwd=str(root),stdin=subprocess.DEVNULL,
                          stdout=handle,stderr=subprocess.STDOUT,start_new_session=True,close_fds=True)
deadline=time.monotonic()+15
error=None
while time.monotonic()<deadline:
    if proc.poll() is not None:raise RuntimeError(log.read_text()[-3000:])
    try:
        html=client.open('http://127.0.0.1:8100/dynamics',timeout=2).read().decode()
        status=json.load(client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=2))
        assert 'v6: continuous transit includes pose-relative gravity support.' in html
        assert 'Home Move takes over in the same serial session' in html
        assert not active(),active()
        print(json.dumps(dict(ok=True,old_support_stopped=True,emergency_stop_ok=stop.get('ok'),
                              web_pid=proc.pid,log=str(log),new_v6_page_verified=True,
                              motion_issued=False,active_motor_programs=active(),
                              status_keys=list(status))))
        break
    except Exception as exc:
        error=str(exc);time.sleep(.5)
else:raise RuntimeError(error)
'''
cmd=['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),
     '-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5',
     'jetson','python3 -c '+shlex.quote(REMOTE)]
r=subprocess.run(cmd,capture_output=True,text=True,timeout=50)
if r.returncode:raise RuntimeError(r.stdout+'\n'+r.stderr)
result=json.loads(r.stdout)
(Path(__file__).parent/'board_v6_restart_result.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps(result,indent=2))
