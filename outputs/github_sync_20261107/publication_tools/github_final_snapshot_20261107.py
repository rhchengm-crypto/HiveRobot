from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED
import hashlib,json,time,urllib.request
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def status():
 with client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=8) as r:s=json.load(r)
 run=s.get('run') or {};assert not run.get('running') and not run.get('support_running')
 return s
before=status();paths=set()
for pattern in ('left_arm*.py','chessboard_vision_v3_0_web_control.py','DM_CAN.py'):
 paths.update(p for p in root.glob(pattern) if p.is_file())
for pattern in ('left_arm*.json','left_arm*.jsonl','geometry_calibration*.json'):
 paths.update(p for p in (root/'data').glob(pattern) if p.is_file())
paths.update(p for p in (root/'data/left_arm_v2_8_full_runs').glob('arm_run_*.txt') if int(p.name.split('_')[2])>=1794113244000)
directory=root/'data/experiment_archives/github_20261107_backfill';directory.mkdir(exist_ok=True)
archive=directory/'final_runtime_snapshot_20261107.zip'
manifest={}
with ZipFile(archive,'w',ZIP_DEFLATED,compresslevel=6) as z:
 for p in sorted(paths):
  stat=p.stat();raw=p.read_bytes();after=p.stat()
  assert stat.st_size==after.st_size==len(raw) and stat.st_mtime_ns==after.st_mtime_ns,'Changing file: '+str(p)
  rel=p.relative_to(root).as_posix();manifest[rel]=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
  z.writestr(rel,raw)
 z.writestr('web_status.json',json.dumps(before,indent=2)+'\n')
 z.writestr('archive_manifest.json',json.dumps(dict(motion_issued=False,files=manifest),indent=2)+'\n')
after=status()
assert (before.get('run') or {}).get('full_log_path')==(after.get('run') or {}).get('full_log_path')
print(json.dumps(dict(motion_issued=False,path=str(archive),bytes=archive.stat().st_size,
 sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),files=len(manifest),
 source_bytes=sum(x['bytes'] for x in manifest.values()),run_unchanged=True)))
