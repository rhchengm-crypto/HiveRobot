"""Archive remote historical RGB-D only, copy locally, verify every member."""
import base64, concurrent.futures, hashlib, json, subprocess, time, zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/github_sync_20261107/remote_rgbd'
OUT.mkdir(parents=True,exist_ok=True)
GAPS=json.loads((OUT.parent/'remote_gap_audit.json').read_text(encoding='utf-8'))['nov6_unpublished_missing_raw_rgbd_sessions']
SSH=['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3','nvidia@192.168.201.199']
SCP=['scp','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3']
REMOTE='/home/nvidia/hive_robot/DM_Control_Python/data/experiment_archives/github_20261107_backfill'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def remote(code,timeout=45):
    p=subprocess.run(SSH+['python3 -'],input=code.encode(),capture_output=True,timeout=timeout)
    if p.returncode:raise RuntimeError(p.stderr.decode(errors='replace'))
    return p.stdout.decode('utf-8')
remote_code=r'''
import concurrent.futures,datetime,hashlib,json,os,shutil,time,traceback,zipfile
from pathlib import Path
DEST=Path(REMOTE)
DEST.mkdir(parents=True,exist_ok=True)
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def archive(rec):
 src=Path(rec['path']).resolve()
 allowed=Path('/home/nvidia/hive_robot/DM_Control_Python/data/rgbd_audits').resolve()
 assert src.parent==allowed and src.is_dir()
 name=src.name; path=DEST/(name+'.zip'); receipt=DEST/(name+'.manifest.json')
 if path.exists() and receipt.exists():
  old=json.loads(receipt.read_text())
  if old.get('verified') and old.get('sha256')==digest(path):return old
 files=sorted(p for p in src.rglob('*') if p.is_file())
 before={p.relative_to(src).as_posix():(p.stat().st_size,p.stat().st_mtime_ns) for p in files}
 members=[]; tmp=DEST/(name+'.zip.partial')
 print('START '+name+' '+str(sum(x[0] for x in before.values())),flush=True)
 with zipfile.ZipFile(tmp,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6,allowZip64=True) as z:
  for p in files:
   rel=p.relative_to(src).as_posix(); h=hashlib.sha256(); count=0
   with p.open('rb') as inp,z.open(rel,'w',force_zip64=True) as dst:
    for block in iter(lambda:inp.read(1024*1024),b''):
     dst.write(block);h.update(block);count+=len(block)
   assert count==before[rel][0]
   members.append({'path':rel,'bytes':count,'sha256':h.hexdigest()})
  after={p.relative_to(src).as_posix():(p.stat().st_size,p.stat().st_mtime_ns) for p in sorted(src.rglob('*')) if p.is_file()}
  assert before==after,'source changed during archive'
  manifest={'schema':1,'source_directory':str(src),'motion_issued':False,'source_deleted':False,'source_unchanged_during_archive':True,'files':members,'file_count':len(members),'source_bytes':sum(x['bytes'] for x in members),'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
  z.writestr('archive_manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
 tmp.replace(path)
 with zipfile.ZipFile(path) as z:assert z.testzip() is None
 result={'session':name,'source_directory':str(src),'path':str(path),'bytes':path.stat().st_size,'sha256':digest(path),'file_count':len(members),'source_bytes':sum(x['bytes'] for x in members),'verified':True,'source_unchanged_during_archive':True,'motion_issued':False}
 receipt.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 print('COMPLETE '+json.dumps(result),flush=True)
 return result
print('FREE '+str(shutil.disk_usage(DEST).free),flush=True)
results=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 jobs={pool.submit(archive,r):r for r in SESSIONS}
 for future in concurrent.futures.as_completed(jobs):
  try:results.append(future.result())
  except Exception as e:results.append({'session':jobs[future]['name'],'error':repr(e)});traceback.print_exc()
  (DEST/'job_status.json').write_text(json.dumps({'completed':results,'expected':len(SESSIONS),'all_finished':len(results)==len(SESSIONS),'motion_issued':False},indent=2)+'\n')
print('FINISHED',flush=True)
'''.replace('REMOTE',repr(REMOTE)).replace('SESSIONS',repr(GAPS))
bootstrap="""
import base64,json,subprocess
from pathlib import Path
d=Path(REMOTE);d.mkdir(parents=True,exist_ok=True)
p=d/'archive_sessions.py';p.write_bytes(base64.b64decode(SOURCE))
log=(d/'archive_sessions.log').open('ab')
proc=subprocess.Popen(['python3','-u',str(p)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
print(json.dumps({'pid':proc.pid,'script':str(p),'log':str(d/'archive_sessions.log'),'motion_issued':False}))
""".replace('REMOTE',repr(REMOTE)).replace('SOURCE',repr(base64.b64encode(remote_code.encode()).decode()))
if not (OUT/'job_started.json').exists():
    started=json.loads(remote(bootstrap));(OUT/'job_started.json').write_text(json.dumps(started,indent=2)+'\n')
    print('Started archive-only worker: '+str(started),flush=True)
expected={r['name'] for r in GAPS}; verified={}
if (OUT/'download_receipt.json').exists():
    verified={r['session']:r for r in json.loads((OUT/'download_receipt.json').read_text())['verified_downloads']}
def download(item):
    path=OUT/(item['session']+'.zip')
    if not path.exists() or path.stat().st_size!=item['bytes'] or sha(path)!=item['sha256']:
        partial=path.with_suffix('.zip.partial')
        p=subprocess.run(SCP+['nvidia@192.168.201.199:'+item['path'],str(partial)],capture_output=True,timeout=900)
        if p.returncode:raise RuntimeError(p.stderr.decode(errors='replace'))
        assert partial.stat().st_size==item['bytes'] and sha(partial)==item['sha256']
        partial.replace(path)
    with zipfile.ZipFile(path) as z:
        man=json.loads(z.read('archive_manifest.json'))
        expected_members={m['path'] for m in man['files']}|{'archive_manifest.json'}
        assert {n for n in z.namelist() if not n.endswith('/')}==expected_members
        for m in man['files']:
            h=hashlib.sha256(); n=0
            with z.open(m['path']) as f:
                for b in iter(lambda:f.read(1024*1024),b''):h.update(b);n+=len(b)
            assert n==m['bytes'] and h.hexdigest()==m['sha256'],m['path']
        # Full member reads verify the stored CRC in addition to SHA-256.
        rgb=sum(m['path'].lower().endswith('.jpg') for m in man['files'])
        depth=sum(m['path'].lower().endswith('.npy') for m in man['files'])
    result=dict(item,local_path=path.relative_to(ROOT).as_posix(),local_sha256=sha(path),zip_crc_verified=True,every_member_sha256_verified=True,rgb_frames=rgb,depth_frames=depth,verified=True)
    (OUT/(item['session']+'.manifest.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Downloaded and verified '+item['session']+' '+str(path.stat().st_size),flush=True)
    return result
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    inflight={}
    while set(verified)!=expected:
        status=json.loads(remote("from pathlib import Path; p=Path("+repr(REMOTE+'/job_status.json')+"); print(p.read_text() if p.exists() else '{\"completed\":[]}')"))
        (OUT/'remote_status.json').write_text(json.dumps(status,indent=2)+'\n')
        for item in status.get('completed',[]):
            if item.get('error'):raise RuntimeError(item)
            name=item['session']
            if name not in verified and name not in inflight:inflight[name]=pool.submit(download,item)
        for name,f in list(inflight.items()):
            if f.done():verified[name]=f.result();del inflight[name]
        receipt={'motion_issued':False,'expected_sessions':len(expected),'downloaded_sessions':len(verified),'complete':set(verified)==expected,'verified_downloads':list(verified.values()),'total_archive_bytes':sum(r['bytes'] for r in verified.values()),'total_source_bytes':sum(r['source_bytes'] for r in verified.values()),'remote_archive_directory':REMOTE}
        (OUT/'download_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print('Progress '+str(len(verified))+'/'+str(len(expected))+'; available '+str(len(status.get('completed',[])))+'; transferring '+str(len(inflight)),flush=True)
        if set(verified)!=expected:time.sleep(20)
print('All historical RGB-D sessions copied and verified.',flush=True)
