"""Run on Jetson to snapshot today's files without importing motor control."""
from datetime import datetime, timezone
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import hashlib
import json
import urllib.request

ROOT = Path('/home/nvidia/hive_robot/DM_Control_Python')
START_MS = int(datetime(2026, 11, 8, 8, tzinfo=timezone.utc).timestamp() * 1000)
client = urllib.request.build_opener(urllib.request.ProxyHandler({}))

def status():
    with client.open('http://127.0.0.1:8100/api/v3/dynamics/status', timeout=8) as response:
        value = json.load(response)
    run = value.get('run') or {}
    assert not run.get('running') and not run.get('support_running')
    return value

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()

before = status()
paths = set()
for pattern in ('left_arm*.py', 'build_left_arm*.py', 'audit_left_arm*.py',
                'test_left_arm*.py', 'chessboard_vision_v3_0_web_control.py', 'DM_CAN.py'):
    paths.update(p for p in ROOT.glob(pattern) if p.is_file())
for pattern in ('left_arm*.json', 'left_arm*.jsonl', 'geometry_calibration*.json'):
    paths.update(p for p in (ROOT/'data').glob(pattern) if p.is_file())
paths.update((ROOT/'data/board_workspace_trials').glob('*.json'))
for p in (ROOT/'data/left_arm_v2_8_full_runs').glob('arm_run_*.txt'):
    if int(p.name.split('_')[2]) >= START_MS:
        paths.add(p)
for p in (ROOT/'data/board_motion_traces').glob('trace_*.jsonl'):
    if int(p.name.split('_')[1].split('.')[0]) >= START_MS * 1000000:
        paths.add(p)
destination = ROOT/'data/experiment_archives/github_20261108'
destination.mkdir(parents=True, exist_ok=True)
archive = destination/'final_runtime_snapshot_20261108.zip'
manifest = {}
with ZipFile(archive, 'w', ZIP_DEFLATED, compresslevel=6) as z:
    for p in sorted(paths):
        prior = p.stat()
        raw = p.read_bytes()
        after = p.stat()
        assert prior.st_size == after.st_size == len(raw) and prior.st_mtime_ns == after.st_mtime_ns, str(p)
        relative = p.relative_to(ROOT).as_posix()
        manifest[relative] = dict(bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())
        z.writestr(relative, raw)
    z.writestr('web_status.json', json.dumps(before, ensure_ascii=False, indent=2)+'\n')
    z.writestr('archive_manifest.json', json.dumps(dict(motion_issued=False, files=manifest), indent=2)+'\n')
with ZipFile(archive) as z:
    assert z.testzip() is None
after = status()
assert (before.get('run') or {}).get('full_log_path') == (after.get('run') or {}).get('full_log_path')
print(json.dumps(dict(motion_issued=False, path=str(archive), bytes=archive.stat().st_size,
    sha256=sha(archive), files=len(manifest), source_bytes=sum(v['bytes'] for v in manifest.values()),
    latest_run={k:(after.get('run') or {}).get(k) for k in ('action','running','support_running','returncode','full_log_path','finished_at')},
    run_unchanged=True)))
