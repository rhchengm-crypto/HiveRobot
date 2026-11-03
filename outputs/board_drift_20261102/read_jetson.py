"""Read Web state and file hashes only; no CAN or robot-control imports."""
import json, subprocess, shlex
from pathlib import Path
root=Path(__file__).resolve().parent
remote=r'''
import json, hashlib, urllib.request
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
result={'motion_issued':False}
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
try:
    with client.open('http://127.0.0.1:8100/api/state',timeout=4) as f:
        state=json.load(f)
    result['web_state']=state
except Exception as exc: result['web_error']=str(exc)
files=['left_arm_board_grasp.py','left_arm_board_kinematics.py','left_arm_rigid_body.py',
       'left_arm_v3_1_model_control.py','data/left_arm_general_dynamics.json',
       'data/left_arm_board_grasp.json','data/left_arm_v3_1_dynamics.json']
result['files']={}
for name in files:
    p=root/name
    if p.is_file(): result['files'][name]={'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'mtime':p.stat().st_mtime}
logs=sorted((root/'data/left_arm_v2_8_full_runs').glob('*.txt'),key=lambda p:p.stat().st_mtime,reverse=True)
result['latest_logs']=[{'path':str(p),'size':p.stat().st_size,'mtime':p.stat().st_mtime} for p in logs[:12]]
result['robot_processes']=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        args=(p/'cmdline').read_bytes().split(b'\0')
        scripts=[a.decode(errors='replace') for a in args if a.endswith(b'.py') and b'DM_Control_Python' in a]
        if scripts:result['robot_processes'].append({'pid':int(p.name),'scripts':scripts})
    except (OSError,PermissionError):pass
print(json.dumps(result))
'''
args=['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=1','nvidia@192.168.201.199','python3 -c '+shlex.quote(remote)]
try:
    p=subprocess.run(args,capture_output=True,timeout=25)
    if p.returncode:raise RuntimeError(p.stderr.decode(errors='replace'))
    result=json.loads(p.stdout)
    (root/'jetson_readonly_state.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'connected':True,'motion_issued':False,'web_state_keys':list(result.get('web_state',{})),
                      'web_error':result.get('web_error'),'latest_logs':result['latest_logs'][:4],
                      'robot_processes':result['robot_processes']},ensure_ascii=True))
except Exception as exc:
    (root/'jetson_readonly_failure.json').write_text(json.dumps({'connected':False,'error':str(exc),'motion_issued':False},indent=2))
    print(json.dumps({'connected':False,'error':str(exc),'motion_issued':False}))
