"""Copy and verify the frozen final runtime archive; no motor access."""
from pathlib import Path
import hashlib
import json
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
info = json.loads((OUT/'final_snapshot_remote.json').read_text(encoding='utf-8-sig'))
target = OUT/'final_runtime_snapshot_20261108.zip'
options = ['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199',
    '-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5']
result = subprocess.run(['scp']+options+['nvidia@192.168.201.199:'+info['path'],str(target)],capture_output=True,timeout=180)
assert result.returncode == 0, result.stderr.decode(errors='replace')
assert target.stat().st_size == info['bytes']
assert hashlib.sha256(target.read_bytes()).hexdigest() == info['sha256']
with zipfile.ZipFile(target) as z:
    assert z.testzip() is None
    manifest = json.loads(z.read('archive_manifest.json'))
    for name, row in manifest['files'].items():
        raw = z.read(name)
        assert len(raw) == row['bytes'] and hashlib.sha256(raw).hexdigest() == row['sha256'], name
    web = json.loads(z.read('web_status.json'))
    last = web['run']
    assert last['running'] is False
    logs = [last['full_log_path']]
    if last.get('preceding_motion',{}).get('full_log_path'):
        logs.append(last['preceding_motion']['full_log_path'])
    latest = OUT/'latest_home_evidence'
    latest.mkdir(exist_ok=True)
    for remote in logs:
        relative = remote.split('/DM_Control_Python/',1)[1]
        (latest/Path(relative).name).write_bytes(z.read(relative))
    (latest/'web_status.json').write_bytes(z.read('web_status.json'))
    (OUT/'final_snapshot_manifest.json').write_bytes(z.read('archive_manifest.json'))
    differences = []
    for relative, row in manifest['files'].items():
        local = ROOT/'scripts'/relative
        if local.is_file() and hashlib.sha256(local.read_bytes()).hexdigest() != row['sha256']:
            differences.append(dict(path=relative,local_sha256=hashlib.sha256(local.read_bytes()).hexdigest(),
                remote_sha256=row['sha256'],note='preserved in final runtime archive; not silently overwritten'))
receipt = dict(ok=True,motion_issued=False,local_path=target.relative_to(ROOT).as_posix(),bytes=info['bytes'],
    sha256=info['sha256'],all_members_verified=True,crc_verified=True,files=info['files'],source_bytes=info['source_bytes'],
    latest_run=info['latest_run'],local_remote_differences=differences)
(OUT/'final_snapshot_verification.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in receipt.items() if k != 'local_remote_differences'},ensure_ascii=False))
print('LOCAL_REMOTE_DIFFERENCES '+str(len(differences)))
