"""Save current full run and compact constraint/hold audit without motion."""
import base64,zlib,json,subprocess,shlex
from pathlib import Path
OUT=Path(__file__).resolve().parent
code=r'''
import json,zlib,base64,urllib.request
from pathlib import Path
c=urllib.request.build_opener(urllib.request.ProxyHandler({}))
s=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5));run=s.get('run')or{}
p=Path(run['full_log_path']);assert p.parent==Path('/home/nvidia/hive_robot/DM_Control_Python/data/left_arm_v2_8_full_runs')
print(base64.b64encode(zlib.compress(json.dumps({'run':run,'name':p.name,'text':p.read_text()}).encode(),6)).decode())
'''
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(code)],capture_output=True,timeout=40)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(zlib.decompress(base64.b64decode(r.stdout)));(OUT/'jetson_snapshot'/d['name']).write_text(d['text'],encoding='utf8')
(OUT/'v22_run_latest.json').write_text(json.dumps(d['run'],ensure_ascii=True))
special=[l for l in d['text'].splitlines() if any(l.startswith(p) for p in ['board descent constraint=', 'board endpoint report=', 'board feedback descent entry=','v3.1 inline model hold'])]
print(json.dumps({'name':d['name'],'run':{k:d['run'].get(k) for k in ['ok','running','returncode','action']},'audit_lines':special,'stderr':d['run'].get('stderr','')[-1000:]}))
