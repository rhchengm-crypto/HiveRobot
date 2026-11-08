import hashlib,json
from pathlib import Path
from zipfile import ZipFile
root=Path(__file__).resolve().parents[1];out=root/'outputs/github_sync_20261107'
remote=json.loads((out/'final_snapshot_remote.json').read_text())
p=out/'final_runtime_snapshot_20261107.zip';assert hashlib.sha256(p.read_bytes()).hexdigest()==remote['sha256']
differences=[];manifest=None
with ZipFile(p) as z:
 assert z.testzip() is None
 manifest=json.loads(z.read('archive_manifest.json'))
 for n,row in manifest['files'].items():
  raw=z.read(n);assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256']
  local=root/'scripts'/n
  if local.is_file() and hashlib.sha256(local.read_bytes()).hexdigest()!=row['sha256']:
   differences.append(dict(path=n,local_sha256=hashlib.sha256(local.read_bytes()).hexdigest(),jetson_sha256=row['sha256'],copied_over_runtime=False))
(out/'final_snapshot_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
report=dict(verified=True,archive_sha256=remote['sha256'],files=len(manifest['files']),
 source_bytes=remote['source_bytes'],local_vs_live_differences=differences,motion_issued=False,
 note='Authoritative final runtime preserved in release archive; local historical bytes not silently overwritten.')
(out/'final_snapshot_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
