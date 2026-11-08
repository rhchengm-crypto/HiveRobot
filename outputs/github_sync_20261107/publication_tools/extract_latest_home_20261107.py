import base64,hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
out=root/'outputs/github_sync_20261107'
r=json.loads((out/'latest_home_download.json').read_text())
dest=out/'latest_home_evidence';dest.mkdir(exist_ok=True)
(dest/'web_status.json').write_text(json.dumps(r['status'],indent=2)+'\n')
manifest={}
for row in r['logs']:
    raw=base64.b64decode(row['base64']);assert hashlib.sha256(raw).hexdigest()==row['sha256']
    name=Path(row['path']).name;(dest/name).write_bytes(raw)
    manifest[name]={k:v for k,v in row.items() if k!='base64'}
run=r['status'].get('run') or {}
result=dict(motion_issued=False,collected_read_only=True,run={k:run.get(k) for k in ('action','ok','running','support_running','returncode','finished_at','full_log_path')},logs=manifest)
(dest/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
