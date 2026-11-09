"""Prepare a read-only source snapshot archive on Jetson, then verify local copy."""
from pathlib import Path
import base64,hashlib,json,subprocess,zipfile
ROOT=Path(__file__).resolve().parents[2]; OUT=ROOT/'outputs/github_sync_20261108'
gap=json.loads((OUT/'remote_gap_audit.json').read_text(encoding='utf-8'))
expected=gap['unmatched_nonraw_files']
remote_code='''from pathlib import Path
import hashlib,json,zipfile,time
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
expected=EXPECTED
out=root/'data/experiment_archives/github_20261108/remote_gap_evidence_20261108.zip'
out.parent.mkdir(parents=True,exist_ok=True)
entries=[]
started=time.time()
with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
    for record in expected:
        p=Path(record['remote_path']); before=p.stat()
        with p.open('rb') as f: b=f.read(before.st_size)
        after=p.stat()
        h=hashlib.sha256(b).hexdigest()
        stable=(before.st_size==after.st_size==len(b) and before.st_mtime_ns==after.st_mtime_ns)
        islog=p.suffix=='.log'
        assert len(b)==before.st_size, 'Short source read: '+str(p)
        if not islog:
            assert stable, 'Source changed during read: '+str(p)
            assert h==record['sha256'] and len(b)==record['bytes'], 'Source changed since inventory: '+str(p)
        arc=p.relative_to(root).as_posix(); z.writestr(arc,b)
        entries.append({'member':arc,'source':str(p),'bytes':len(b),'sha256':h,'source_before_mtime_ns':before.st_mtime_ns,'source_after_mtime_ns':after.st_mtime_ns,'source_size_before':before.st_size,'source_size_after':after.st_size,'source_stable_during_read':stable,'sealed_log':False if islog else None,'finite_prefix_snapshot':islog,'inventory_bytes':record['bytes'],'inventory_sha256':record['sha256'],'matches_earlier_inventory':h==record['sha256'] and len(b)==record['bytes']})
    z.writestr('remote_gap_manifest.json',json.dumps({'motion_issued':False,'source_read_interval_start_epoch':started,'source_read_interval_end_epoch':time.time(),'files':entries},ensure_ascii=False,indent=2)+'\\n')
with zipfile.ZipFile(out) as z: assert z.testzip() is None
b=out.read_bytes()
print(json.dumps({'motion_issued':False,'motors_opened':False,'remote_path':str(out),'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest(),'payload_files':len(entries),'all_sources_stable_during_read':all(x['source_stable_during_read'] for x in entries),'all_sources_match_inventory':all(x['matches_earlier_inventory'] for x in entries),'all_nonlog_sources_match_inventory':all(x['matches_earlier_inventory'] for x in entries if x['sealed_log'] is None),'files':entries},ensure_ascii=False))
'''.replace('EXPECTED',repr(expected))
(OUT/'capture_remote_gap_jetson.py').write_text(remote_code,encoding='utf-8')
ssh=['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2','nvidia@192.168.201.199']
encoded=base64.b64encode(remote_code.encode()).decode()
r=subprocess.run(ssh+['python3 -c "import base64;exec(base64.b64decode(\''+encoded+'\'))"'],capture_output=True,timeout=100)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
remote=json.loads(r.stdout)
local=OUT/'remote_gap_evidence_20261108.zip'
with local.open('wb') as target:
    r=subprocess.run(ssh+['cat '+remote['remote_path']],stdout=target,stderr=subprocess.PIPE,timeout=100)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
b=local.read_bytes(); assert len(b)==remote['bytes'] and hashlib.sha256(b).hexdigest()==remote['sha256']
with zipfile.ZipFile(local) as z:
    assert z.testzip() is None
    for f in remote['files']:
        x=z.read(f['member']); assert len(x)==f['bytes'] and hashlib.sha256(x).hexdigest()==f['sha256']
        if f['finite_prefix_snapshot']:
            f['earlier_inventory_prefix_preserved']=len(x)>=f['inventory_bytes'] and hashlib.sha256(x[:f['inventory_bytes']]).hexdigest()==f['inventory_sha256']
            assert f['earlier_inventory_prefix_preserved'], 'Earlier inventoried log was not preserved as prefix'
receipt=dict(remote,local_path=local.relative_to(ROOT).as_posix(),local_sha256_verified=True,all_member_sha256_verified=True,zip_crc_verified=True,no_original_source_modified=True)
(OUT/'remote_gap_capture_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:receipt[k] for k in ('motion_issued','bytes','sha256','payload_files','all_sources_stable_during_read','local_sha256_verified','all_member_sha256_verified','zip_crc_verified')},indent=2))
