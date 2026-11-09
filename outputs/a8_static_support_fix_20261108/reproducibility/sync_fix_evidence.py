"""Copy fix records only, verifying deployed code and never opening a motor."""
from pathlib import Path
import base64
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT/'outputs/a8_static_support_fix_20261108'
report = json.loads((OUT/'deployment_report.json').read_text(encoding='utf8'))
assert report['ok'] and not report['motion_issued']
registry = 'data/board_workspace_trials/fix_20261108_a8_quasistatic_support.json'
files = {registry:(ROOT/'scripts'/registry).read_bytes()}
for name in ('README.md','recorded_window_audit.json','regression_tests.log',
             'deployment_report.json','release_manifest.json'):
    files['data/board_workspace_trials/a8_static_support_fix_20261108/'+name] = (OUT/name).read_bytes()
payload = {name:dict(sha256=hashlib.sha256(data).hexdigest(),
                    base64=base64.b64encode(data).decode()) for name,data in files.items()}
remote = '''from pathlib import Path
import base64,hashlib,json,os,urllib.request
root=Path('/home/nvidia/hive_robot/DM_Control_Python').resolve()
expected=__EXPECTED__
payload=__PAYLOAD__
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
with client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5) as response:state=json.load(response)
run=state.get('run') or {}
assert not run.get('running') and not run.get('support_running')
for name,digest in expected.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
checked={}
for name,record in payload.items():
    target=(root/name).resolve()
    assert root/'data/board_workspace_trials' in target.parents
    data=base64.b64decode(record['base64']);digest=hashlib.sha256(data).hexdigest()
    assert digest==record['sha256']
    if target.exists():assert hashlib.sha256(target.read_bytes()).hexdigest()==digest,'existing evidence differs: '+name
    target.parent.mkdir(parents=True,exist_ok=True)
    if not target.exists():
        pending=target.with_name(target.name+'.pending')
        with pending.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
        os.replace(str(pending),str(target))
    assert hashlib.sha256(target.read_bytes()).hexdigest()==digest
    checked[name]=digest
print(json.dumps(dict(ok=True,motion_issued=False,serial_opened=False,files=checked)))
'''.replace('__EXPECTED__',repr(report['deployed_sha256'])).replace('__PAYLOAD__',repr(payload))
options=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),
         '-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256',
         '-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5']
result=subprocess.run(['ssh']+options+['nvidia@192.168.201.199','python3 -'],
                      input=remote.encode(),capture_output=True,timeout=35)
assert result.returncode==0,result.stderr.decode(errors='replace')
receipt=json.loads(result.stdout)
assert receipt['ok'] and not receipt['motion_issued']
(OUT/'evidence_sync_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(ok=True,files=len(receipt['files']),motion_issued=False)))
