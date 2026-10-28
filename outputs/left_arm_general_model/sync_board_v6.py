"""Atomic code-only Jetson upload. Does not stop processes or command motors."""
import base64
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import zlib

ROOT = Path(__file__).resolve().parents[2]
FILES = ['left_arm_board_grasp.py', 'left_arm_board_home_handoff.py',
         'left_arm_fault_support.py', 'left_arm_general_motion.py',
         'left_arm_dynamics_worker.py', 'left_arm_v3_1_motion_dynamics.py',
         'chessboard_vision_v3_0_web_control.py', 'LEFT_ARM_BOARD_GRASP_DEPLOY.md']
payload = {}
for name in FILES:
    content = (ROOT/'scripts'/name).read_bytes()
    if name.endswith('.py'):
        compile(content, name, 'exec')
    payload[name] = dict(content=base64.b64encode(content).decode(),
                         sha256=hashlib.sha256(content).hexdigest())
REMOTE = '''
import base64,datetime,hashlib,json,os,sys,tempfile,zlib
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
payload=json.loads(zlib.decompress(sys.stdin.buffer.read()))
def protected():
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'data').glob('*.json')}
before=protected()
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
backup=root/'data'/'code_backups'/('board_v6_'+stamp)
backup.mkdir(parents=True)
decoded={}
for name,row in payload.items():
    if Path(name).name!=name:raise ValueError('invalid code path')
    content=base64.b64decode(row['content'])
    assert hashlib.sha256(content).hexdigest()==row['sha256']
    if name.endswith('.py'):compile(content,name,'exec')
    decoded[name]=content
for name,content in decoded.items():
    target=root/name
    if target.exists():(backup/name).write_bytes(target.read_bytes())
    fd,tmp=tempfile.mkstemp(prefix='.'+name+'.',dir=str(root))
    with os.fdopen(fd,'wb') as handle:
        handle.write(content);handle.flush();os.fsync(handle.fileno())
    os.replace(tmp,str(target))
after=protected()
assert before==after,'protected model/learning JSON changed'
hashes={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in decoded}
assert hashes=={n:r['sha256'] for n,r in payload.items()}
print(json.dumps(dict(ok=True,backup=str(backup),files=hashes,
                     protected_json_unchanged=True,protected_json_count=len(before),
                     processes_restarted=False,motion_issued=False)))
'''
command=['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),
         '-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5',
         'jetson','python3 -c '+shlex.quote(REMOTE)]
result=subprocess.run(command,input=zlib.compress(json.dumps(payload).encode()),
                      capture_output=True,timeout=50)
if result.returncode:
    raise RuntimeError(result.stderr.decode(errors='replace'))
report=json.loads(result.stdout)
(Path(__file__).parent/'board_v6_sync_result.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
