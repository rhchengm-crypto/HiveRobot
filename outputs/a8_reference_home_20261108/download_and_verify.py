"""Retrieve completed Home evidence and byte-audit its learning synchronization."""
from pathlib import Path,PurePosixPath
from concurrent.futures import ThreadPoolExecutor
import base64,hashlib,json,os,stat,subprocess,zipfile

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
report=json.loads((OUT/'collection_report.json').read_text(encoding='utf-8-sig'))
OPTS=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3']
LEARNING=['left_arm_v2_8_clearance_bias.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_6_adaptive_hold_target_bias.json']
REMOTE='/home/nvidia/hive_robot/DM_Control_Python'
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def save(path,value):
 path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def download(label):
 row=report[label];p=OUT/(label+'_evidence.zip')
 if not p.exists() or p.stat().st_size!=row['bytes'] or sha(p)!=row['sha256']:
  partial=p.with_suffix('.zip.partial')
  r=subprocess.run(['scp']+OPTS+['nvidia@192.168.201.199:'+row['path'],str(partial)],capture_output=True,timeout=400)
  assert r.returncode==0,r.stderr.decode(errors='replace')
  assert partial.stat().st_size==row['bytes'] and sha(partial)==row['sha256']
  partial.replace(p)
 extract=OUT/'control_evidence'
 with zipfile.ZipFile(p) as z:
  assert z.testzip() is None
  manifest=json.loads(z.read('manifest.json'))
  assert set(z.namelist())==set(manifest)|{'manifest.json'}
  for name,expected in manifest.items():
   rel=PurePosixPath(name)
   assert not rel.is_absolute() and '..' not in rel.parts and ':' not in name
   assert stat.S_IFMT(z.getinfo(name).external_attr>>16)!=stat.S_IFLNK
   h=hashlib.sha256();n=0
   with z.open(name) as f:
    for b in iter(lambda:f.read(1024*1024),b''):h.update(b);n+=len(b)
   assert n==expected['bytes'] and h.hexdigest()==expected['sha256'],name
   if label=='control':
    target=extract.joinpath(*rel.parts);target.parent.mkdir(parents=True,exist_ok=True)
    with z.open(name) as source,target.open('wb') as dest:
     for b in iter(lambda:source.read(1024*1024),b''):dest.write(b)
  if label=='control':save(extract/'manifest.json',manifest)
 result=dict(row,local_path=p.relative_to(ROOT).as_posix(),local_sha256=sha(p),
  byte_count_verified=True,zip_crc_verified=True,every_member_sha256_verified=True,manifest_files=len(manifest))
 print('Verified '+label+' '+str(p.stat().st_size)+' bytes / '+str(len(manifest))+' files',flush=True)
 return label,result
with ThreadPoolExecutor(max_workers=2) as pool:
 verified=dict(pool.map(download,('control','rgbd')))
extract=OUT/'control_evidence'
ctx=extract/'data/board_workspace_trials/a8_reference_home_20261108'
declared=json.loads((ctx/'source_hashes.json').read_text())
frozen_checks={}
for name,expected in declared.items():
 p=extract/('models' if name.startswith('data/') else 'source')/Path(name).name
 assert p.is_file() and sha(p)==expected,name
 frozen_checks[name]={'sha256':expected,'verified':True}
save(OUT/'download_verification.json',{'motion_issued':False,'archives':verified,
 'runtime_source_snapshot_verified':frozen_checks,'control_extracted':True,
 'rgbd_extracted':False,'operator_observation_independent_of_software':True})

def remote_learning():
 code="""
from pathlib import Path
import base64,hashlib,json
root=Path(REMOTE)
rows={}
for name in NAMES:
 p=root/'data'/name;before=p.stat();data=p.read_bytes();after=p.stat()
 assert (before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns),name
 rows[name]={'path':str(p),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'mtime_ns':after.st_mtime_ns,'data_base64':base64.b64encode(data).decode()}
print(json.dumps({'motion_issued':False,'files':rows}))
""".replace('REMOTE',repr(REMOTE)).replace('NAMES',repr(LEARNING))
 r=subprocess.run(['ssh']+OPTS+['nvidia@192.168.201.199','python3 -'],input=code.encode(),capture_output=True,timeout=45)
 assert r.returncode==0,r.stderr.decode(errors='replace')
 return json.loads(r.stdout)
sync=OUT/'learning_sync';sync.mkdir(exist_ok=True)
remote=remote_learning();save(sync/'remote_before_sync.json',remote)
audit=[]
for name in LEARNING:
 row=remote['files'][name];data=base64.b64decode(row['data_base64'])
 assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256']
 archived=extract/'after_learning'/name
 assert archived.is_file() and sha(archived)==row['sha256'], 'Runtime learning changed after Home archive: '+name
 dest=ROOT/'scripts/data'/name
 old=dest.read_bytes() if dest.exists() else None
 before_sha=hashlib.sha256(old).hexdigest() if old is not None else None
 if old is not None:(sync/('local_before_'+name)).write_bytes(old)
 (sync/('remote_new_'+name)).write_bytes(data)
 audit.append({'name':name,'local_path':dest.relative_to(ROOT).as_posix(),
  'old_bytes':len(old) if old is not None else None,'old_sha256':before_sha,
  'new_bytes':len(data),'new_sha256':row['sha256'],
  'equals_archived_Home_after_learning':True,'changed':old!=data,'remote_source':row['path']})
# Audit is durable before any working-tree learning file is replaced.
save(sync/'old_new_hash_audit.json',{'motion_issued':False,'rows':audit,'synchronization_complete':False})
for row in audit:
 name=row['name'];dest=ROOT/row['local_path']
 assert (sha(dest) if dest.exists() else None)==row['old_sha256'],'Local learning changed during audit: '+name
 temp=sync/(name+'.new.tmp');temp.write_bytes(base64.b64decode(remote['files'][name]['data_base64']))
 os.replace(temp,dest)
 assert sha(dest)==row['new_sha256']
after=remote_learning();save(sync/'remote_after_sync.json',after)
assert all(after['files'][n]['sha256']==remote['files'][n]['sha256'] for n in LEARNING),'Runtime learning changed during sync'
save(sync/'old_new_hash_audit.json',{'motion_issued':False,'rows':audit,
 'synchronization_complete':True,'remote_before_after_consistent':True,
 'scope':'Only the three Home-learning JSON files in scripts/data; no source/model/robot commands changed'})
save(OUT/'operator_feedback.json',{'response':'Home全程正常，目前稳定',
 'source':'User confirmation relayed by root agent','accepted_scope':'ordinary Home return and following hold',
 'physical_observation':True,'software_certifies_absence_of_jitter':False,
 'additional_motion_issued_by_evidence_sync':False})
print(json.dumps({'archives_verified':2,'frozen_runtime_files_verified':len(frozen_checks),
 'learning_synced':[{'name':r['name'],'changed':r['changed'],'old_sha256':r['old_sha256'],'new_sha256':r['new_sha256']} for r in audit],
 'remote_learning_unchanged_during_sync':True,'motion_issued':False},ensure_ascii=False))
