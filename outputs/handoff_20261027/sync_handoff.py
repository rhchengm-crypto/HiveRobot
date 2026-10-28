"""Sync handoff documentation only, without changing runtime configuration."""
from pathlib import Path
import base64,hashlib,json,shlex,subprocess,zlib

out=Path(__file__).resolve().parent
root=out.parents[1]
payload={n:base64.b64encode(p.read_bytes()).decode() for n,p in [
    ('HANDOFF_20261027.md',root/'HANDOFF_20261027.md'),
    ('records_index.json',out/'records_index.json'),
    ('snapshot_manifest.json',out/'jetson_snapshot/snapshot_manifest.json')]}
remote=r'''
from pathlib import Path
import base64,hashlib,json,sys,os,zlib
root=Path('/home/nvidia/hive_robot/DM_Control_Python');folder=root/'data/handoffs/20261027'
before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'data').glob('*.json')}
folder.mkdir(parents=True,exist_ok=True)
payload=json.loads(zlib.decompress(sys.stdin.buffer.read()));result={}
for n,v in payload.items():
 assert Path(n).name==n
 b=base64.b64decode(v);p=folder/n
 if p.exists() and p.read_bytes()!=b:
  (folder/(n+'.previous.'+hashlib.sha256(p.read_bytes()).hexdigest()[:12])).write_bytes(p.read_bytes())
 temp=folder/('.'+n+'.tmp');temp.write_bytes(b);os.replace(str(temp),str(p))
 result[n]=hashlib.sha256(p.read_bytes()).hexdigest()
assert before=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'data').glob('*.json')}
print(json.dumps({'ok':True,'destination':str(folder),'motion_issued':False,'runtime_json_unchanged':True,'sha256':result}))
'''
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),
    '-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5',
    '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=1','jetson',
    'python3 -c '+shlex.quote(remote)],input=zlib.compress(json.dumps(payload).encode(),9),
    capture_output=True,timeout=35)
assert r.returncode==0,r.stderr.decode('utf8',errors='replace')
m=json.loads(r.stdout)
assert all(m['sha256'][n]==hashlib.sha256(base64.b64decode(v)).hexdigest() for n,v in payload.items())
(out/'handoff_jetson_sync.json').write_bytes(r.stdout)
print(r.stdout.decode('utf8'))
