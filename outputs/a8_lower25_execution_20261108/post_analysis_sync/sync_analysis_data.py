"""Freeze and sync post-trial analysis records; never install runtime code or configuration."""
from pathlib import Path
import hashlib,json,subprocess,zipfile

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
OUT=HERE.parent
REGISTRY='scripts/data/board_workspace_trials/validation_20261108_a8_lower25.json'
REGISTRY_SHA='a087dde587a297824c7d374cff9ae5170509e4b6a6e2851d8cdf3c88e0bde759'
REMOTE='/home/nvidia/hive_robot/DM_Control_Python'
DEST=REMOTE+'/data/board_workspace_trials/a8_lower25_execution_20261108/analysis_evidence'
OPTS=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def ssh(code,timeout=45):
 r=subprocess.run(['ssh']+OPTS+['nvidia@192.168.201.199','python3 -'],input=code.encode(),capture_output=True,timeout=timeout)
 assert r.returncode==0,r.stderr.decode(errors='replace')
 return json.loads(r.stdout)

registry=read(ROOT/REGISTRY);assert sha(ROOT/REGISTRY)==REGISTRY_SHA
assert registry['physical_validation_accepted'] is False and registry['height_passed'] is False
assert registry['fitting_eligible'] is False and registry['successful_path_library_updated'] is False
assert registry['motion_stability_accepted'] is True and registry['additional_motion_after_this_trial'] is False
files=[p for p in OUT.iterdir() if p.is_file() and p.suffix in {'.json','.md','.txt','.py','.log'}]
for folder in ['source_snapshot','reproducibility']:
 files.extend(p for p in (OUT/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc')
for folder in OUT.iterdir():
 if folder.is_dir() and ('diagnostic' in folder.name or 'next10' in folder.name):
  files.extend(p for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc')
# Exact executed plans and model bytes supplement the pre-trial source snapshot;
# omit the continuous trace/large control ZIP because those raw archives persist.
frozen=OUT/'control_evidence';manifest0=read(frozen/'manifest.json')
for name in manifest0:
 if name.startswith('models/') or name=='source/record_left_arm_rgbd_audit.py' or name=='result_summary.json' or (Path(name).suffix=='.json' and 'approach_' in Path(name).name):
  files.append(frozen/name)
files.extend([ROOT/REGISTRY,Path(__file__).resolve(),OUT/'learning_sync/old_new_hash_audit.json'])
assert all(p.is_file() for p in files)
entries={p.relative_to(ROOT).as_posix():p for p in files}
manifest={n:{'bytes':p.stat().st_size,'sha256':sha(p)} for n,p in sorted(entries.items())}
bundle=HERE/'analysis_evidence.zip'
with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for n,p in sorted(entries.items()):z.write(p,n)
 z.writestr('manifest.json',json.dumps(manifest,indent=2))
with zipfile.ZipFile(bundle) as z:assert z.testzip() is None
bundle_sha=sha(bundle);save(HERE/'analysis_evidence_manifest.json',manifest)
protected=dict(read(OUT/'preflight.json')['source_hashes'])
learning=read(OUT/'learning_sync/old_new_hash_audit.json')
assert learning['synchronization_complete']
for row in learning['rows']:protected['data/'+row['name']]=row['new_sha256']
assert protected['data/left_arm_v2_8_clearance_bias.json']=='52f1b83bc1a267e673cb1abcfa6981b25465111200daadade69b7ee7f31acd68'
remote_partial=DEST+'/analysis_evidence.zip.partial-'+bundle_sha[:12]
ssh("from pathlib import Path;import json;p=Path("+repr(DEST)+");p.mkdir(parents=True,exist_ok=True);print(json.dumps({'directory':str(p)}))")
r=subprocess.run(['scp']+OPTS+[str(bundle),'nvidia@192.168.201.199:'+remote_partial],capture_output=True,timeout=90)
assert r.returncode==0,r.stderr.decode(errors='replace')
print('UPLOADED '+json.dumps({'bytes':bundle.stat().st_size,'files':len(manifest),'sha256':bundle_sha}),flush=True)
code=r'''
from pathlib import Path,PurePosixPath
import hashlib,json,os,stat,time,zipfile
ROOT=Path(ROOT_VALUE).resolve();DEST=Path(DEST_VALUE).resolve();PENDING=Path(PENDING_VALUE)
assert ROOT in DEST.parents and DEST==ROOT/'data/board_workspace_trials/a8_lower25_execution_20261108/analysis_evidence'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def state(n):
 p=(ROOT/n).resolve();assert ROOT in p.parents
 return sha(p) if p.is_file() else None
protected=PROTECTED_VALUE
assert all(state(n)==h for n,h in protected.items()),'Runtime changed before data sync'
assert sha(PENDING)==SHA_VALUE and PENDING.stat().st_size==BYTES_VALUE
with zipfile.ZipFile(PENDING) as z:
 assert z.testzip() is None
 manifest=json.loads(z.read('manifest.json'));assert set(z.namelist())==set(manifest)|{'manifest.json'}
 assert manifest[REGISTRY_VALUE]['sha256']==REGISTRY_SHA_VALUE
 for n,row in manifest.items():
  rel=PurePosixPath(n);assert not rel.is_absolute() and '..' not in rel.parts and ':' not in n
  assert stat.S_IFMT(z.getinfo(n).external_attr>>16)!=stat.S_IFLNK
  data=z.read(n);assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],n
 # These are copied as evidence, never imported or installed in runtime paths.
 for n,row in manifest.items():
  target=(DEST/n).resolve();assert DEST in target.parents
  target.parent.mkdir(parents=True,exist_ok=True)
  if target.exists():assert sha(target)==row['sha256'],'Existing analysis evidence differs: '+n
  else:
   pending=target.with_name(target.name+'.data-sync.pending');pending.write_bytes(z.read(n));os.replace(pending,target)
  assert sha(target)==row['sha256']
 registry_data=z.read(REGISTRY_VALUE)
 record=json.loads(registry_data)
 assert record['physical_validation_accepted'] is False and record['height_passed'] is False and record['fitting_eligible'] is False
 assert record['successful_path_library_updated'] is False and record['motion_stability_accepted'] is True
 assert record['additional_motion_after_this_trial'] is False
 registry=ROOT/'data/board_workspace_trials/validation_20261108_a8_lower25.json'
 assert ROOT in registry.resolve().parents
 if registry.exists() and sha(registry)!=REGISTRY_SHA_VALUE:
  backup=DEST/('previous_registry_'+str(time.time_ns())+'.json');backup.write_bytes(registry.read_bytes())
 if not registry.exists() or sha(registry)!=REGISTRY_SHA_VALUE:
  pending=registry.with_suffix('.json.data-sync.pending');pending.write_bytes(registry_data);os.replace(pending,registry)
 assert sha(registry)==REGISTRY_SHA_VALUE
 assert all(state(n)==h for n,h in protected.items()),'Runtime changed during data sync'
 archive=DEST/'analysis_evidence.zip'
 if archive.exists():assert sha(archive)==SHA_VALUE
 else:os.replace(PENDING,archive)
 assert sha(archive)==SHA_VALUE
 report={'ok':True,'data_only':True,'motion_issued':False,'runtime_sources_or_configs_installed':False,'web_restarted':False,'remote_directory':str(DEST),'remote_archive':str(archive),'archive_bytes':BYTES_VALUE,'archive_sha256':SHA_VALUE,'zip_crc_verified':True,'every_member_sha256_verified':True,'member_files':len(manifest),'registry_path':str(registry),'registry_sha256':sha(registry),'protected_runtime_files_unchanged':len(protected),'motion_stability_accepted':True,'height_passed':False,'fitting_eligible':False,'successful_path_library_updated':False,'prepared_execution_stamp_created':False,'physical_validation_complete':False,'GitHub_accessed':False}
 report['verified_analysis_file_sha256']={n:r['sha256'] for n,r in manifest.items() if Path(n).name in {'README.md','trial_summary.json','operator_feedback.json','execution_audit.json'} or '_diagnostic/' in n}
 (DEST/'data_sync_receipt.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report))
'''
replace={'ROOT_VALUE':repr(REMOTE),'DEST_VALUE':repr(DEST),'PENDING_VALUE':repr(remote_partial),'PROTECTED_VALUE':repr(protected),'SHA_VALUE':repr(bundle_sha),'BYTES_VALUE':repr(bundle.stat().st_size),'REGISTRY_SHA_VALUE':repr(REGISTRY_SHA),'REGISTRY_VALUE':repr(REGISTRY)}
# Longer placeholder names must be replaced before their suffixes.
for token in sorted(replace,key=len,reverse=True):code=code.replace(token,replace[token])
report=ssh(code,90)
assert report['ok'] and report['registry_sha256']==REGISTRY_SHA and report['archive_sha256']==bundle_sha
report.update({'local_archive':bundle.relative_to(ROOT).as_posix(),'local_archive_sha256':bundle_sha,'local_manifest':'analysis_evidence_manifest.json'})
save(HERE/'data_sync_receipt.json',report)
print(json.dumps(report,indent=2))
