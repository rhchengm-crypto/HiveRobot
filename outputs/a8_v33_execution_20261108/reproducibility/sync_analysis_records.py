"""Archive post-test findings on Jetson without changing any motion code."""
from pathlib import Path
import base64,hashlib,json,subprocess,zipfile

ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).resolve().parents[1]
summary=json.loads((OUT/'trial_summary.json').read_text(encoding='utf8'))
preflight=json.loads((OUT/'preflight.json').read_text(encoding='utf8'))
learning=set('data/'+n for n in summary['next_sync_expected_learning_sha256'])
expected={n:h for n,h in preflight['source_hashes'].items() if n not in learning}
expected.update({'data/'+n:h for n,h in summary['next_sync_expected_learning_sha256'].items()})
names=['README.md','trial_summary.json','operator_feedback.json','execution_audit.json',
       'height_diagnosis.json','lower25_endpoint_diagnostic.json','height_followup_feasibility.json',
       'download_verification.json','source_and_learning_verification.json',
       'learning_sync/old_new_hash_audit.json','collection_report.json']
names += [p.relative_to(OUT).as_posix() for p in (OUT/'reproducibility').iterdir() if p.suffix in ('.py','.json')]
bundle=OUT/'analysis_evidence.zip';manifest={}
with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
    for name in sorted(set(names)):
        data=(OUT/name).read_bytes();archive.writestr(name,data)
        manifest[name]=dict(bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
    archive.writestr('manifest.json',json.dumps(manifest,indent=2))
digest=hashlib.sha256(bundle.read_bytes()).hexdigest()
registry=(ROOT/'scripts/data/board_workspace_trials/validation_20261108_a8_v33.json').read_bytes()
payload=dict(zip_base64=base64.b64encode(bundle.read_bytes()).decode(),zip_sha256=digest,
             registry_base64=base64.b64encode(registry).decode(),registry_sha256=hashlib.sha256(registry).hexdigest())
remote='''from pathlib import Path,PurePosixPath
import base64,hashlib,io,json,os,urllib.request,zipfile
root=Path('/home/nvidia/hive_robot/DM_Control_Python');out=root/'data/board_workspace_trials/a8_v33_execution_20261108'
expected=__EXPECTED__;payload=__PAYLOAD__
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
state=json.load(client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5));run=state.get('run') or {}
assert not run.get('running') and not run.get('support_running')
for name,digest in expected.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
data=base64.b64decode(payload['zip_base64']);assert hashlib.sha256(data).hexdigest()==payload['zip_sha256']
dest=out/'analysis_evidence.zip'
assert not dest.exists() or hashlib.sha256(dest.read_bytes()).hexdigest()==payload['zip_sha256']
if not dest.exists():dest.write_bytes(data)
with zipfile.ZipFile(io.BytesIO(data)) as archive:
    assert archive.testzip() is None
    manifest=json.loads(archive.read('manifest.json'));assert set(archive.namelist())==set(manifest)|{'manifest.json'}
    for name,row in manifest.items():
        path=PurePosixPath(name);assert not path.is_absolute() and '..' not in path.parts
        content=archive.read(name);assert len(content)==row['bytes'] and hashlib.sha256(content).hexdigest()==row['sha256']
        target=out/'analysis_evidence'/name;target.parent.mkdir(parents=True,exist_ok=True)
        assert not target.exists() or hashlib.sha256(target.read_bytes()).hexdigest()==row['sha256']
        if not target.exists():target.write_bytes(content)
registry=base64.b64decode(payload['registry_base64']);assert hashlib.sha256(registry).hexdigest()==payload['registry_sha256']
target=root/'data/board_workspace_trials/validation_20261108_a8_v33.json'
assert not target.exists() or hashlib.sha256(target.read_bytes()).hexdigest()==payload['registry_sha256']
if not target.exists():target.write_bytes(registry)
print(json.dumps(dict(ok=True,motion_issued=False,serial_opened=False,sha256=payload['zip_sha256'],analysis_files=len(manifest),registry_sha256=payload['registry_sha256'],current_action=run.get('action'))))
'''.replace('__EXPECTED__',repr(expected)).replace('__PAYLOAD__',repr(payload))
options=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199',
         '-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5']
result=subprocess.run(['ssh']+options+['nvidia@192.168.201.199','python3 -'],input=remote.encode(),capture_output=True,timeout=45)
assert result.returncode==0,result.stderr.decode(errors='replace')
receipt=json.loads(result.stdout);assert receipt['ok'] and not receipt['motion_issued']
(OUT/'analysis_sync_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8')
print(json.dumps(receipt))
