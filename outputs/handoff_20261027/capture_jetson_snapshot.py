"""Read-only archive of paused robot state; never imports motor control."""
from pathlib import Path
import base64, io, json, shlex, subprocess, zipfile

out = Path(__file__).resolve().parent
code = r'''
from pathlib import Path
import base64,datetime,hashlib,io,json,urllib.request,zipfile
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
c=urllib.request.build_opener(urllib.request.ProxyHandler({}))
s=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5))
run=s.get('run') or {}
assert not run.get('running') and not run.get('supporting_load'), 'Robot is not idle'
processes=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:a=[v.decode() for v in (p/'cmdline').read_bytes().split(bytes([0])) if v]
 except (OSError,UnicodeError):continue
 if any(Path(v).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','record_left_arm_rgbd_audit.py','chessboard_vision_v3_0_web_control.py'} for v in a[:3]):
  processes.append({'pid':int(p.name),'argv':a})
assert not [p for p in processes if not any(Path(a).name=='chessboard_vision_v3_0_web_control.py' for a in p['argv'][:3])],processes
files=sorted((root/'data').glob('*.json'))
files += [root/'data/left_arm_v2_8_full_runs'/n for n in ['arm_run_1793165507803_95280.txt','arm_run_1793165568902_95441.txt']]
data={str(p.relative_to(root)):p.read_bytes() for p in files}
manifest={n:{'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()} for n,b in data.items()}
source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.glob('*.py')}
runtime_logs={p.name:{'bytes':p.stat().st_size,'mtime_ns':p.stat().st_mtime_ns} for p in (root/'data').glob('*.jsonl')}
metadata={'captured_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'motion_issued':False,'paused':True,'processes':processes,'run':run,'source_sha256':source_hashes,'files':manifest,'runtime_jsonl_inventory':runtime_logs}
b=io.BytesIO()
with zipfile.ZipFile(b,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for n,v in data.items():z.writestr(n,v)
 z.writestr('snapshot_manifest.json',json.dumps(metadata,indent=2))
print(base64.b64encode(b.getvalue()).decode())
'''
r = subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),
    '-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5',
    '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=1','jetson',
    'python3 -c '+shlex.quote(code)],capture_output=True,timeout=55)
assert r.returncode == 0, r.stderr.decode('utf8',errors='replace')
blob=base64.b64decode(r.stdout)
(out/'jetson_paused_snapshot.zip').write_bytes(blob)
with zipfile.ZipFile(io.BytesIO(blob)) as z:
    for n in z.namelist():
        parts=Path(n).parts
        assert not Path(n).is_absolute() and '..' not in parts
        p=out/'jetson_snapshot'/n
        p.parent.mkdir(parents=True,exist_ok=True)
        p.write_bytes(z.read(n))
m=json.loads((out/'jetson_snapshot/snapshot_manifest.json').read_text())
print(json.dumps({'snapshot_bytes':len(blob),'files':len(m['files']),
    'paused':m['paused'],'run_ok':m['run']['ok'],'running':m['run']['running'],
    'processes':m['processes']}))
