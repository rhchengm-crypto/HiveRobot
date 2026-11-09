"""Retrieve v33 trial evidence; sync only five byte-audited learning JSONs."""
from pathlib import Path,PurePosixPath
from concurrent.futures import ThreadPoolExecutor
import base64,hashlib,json,os,stat,subprocess,zipfile

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
REPORT=json.loads((OUT/'collection_report.json').read_text(encoding='utf-8-sig'))
REMOTE='/home/nvidia/hive_robot/DM_Control_Python'
OPTS=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3']
LEARNING=['left_arm_v2_8_clearance_bias.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_6_adaptive_hold_target_bias.json','left_arm_v2_6_adaptive_hold_tau.json','left_arm_v2_6_adaptive_active_tau.json']
PREPARED={'rigid_model':'data/left_arm_general_dynamics.json','board_model':'data/left_arm_board_workspace_candidate.json',
 'planner':'left_arm_board_grasp.py','controller':'left_arm_v2_6.py','motion_adapter':'left_arm_v3_1_motion_dynamics.py',
 'fault_support':'left_arm_fault_support.py','kinematics':'left_arm_board_kinematics.py','collision':'left_arm_board_collision.py',
 'trace':'left_arm_motion_trace.py','task_pose':'left_arm_board_task_pose.py','static_support':'left_arm_static_support.py',
 'trajectory_support':'left_arm_trajectory_support.py','general_motion':'left_arm_general_motion.py'}
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,x):
 p.parent.mkdir(parents=True,exist_ok=True)
 pending=p.with_suffix(p.suffix+'.tmp');pending.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');os.replace(pending,p)
def download(label):
 row=REPORT[label];p=OUT/(label+'_evidence.zip')
 if not p.exists() or p.stat().st_size!=row['bytes'] or sha(p)!=row['sha256']:
  partial=p.with_suffix('.zip.partial')
  r=subprocess.run(['scp']+OPTS+['nvidia@192.168.201.199:'+row['path'],str(partial)],capture_output=True,timeout=900)
  assert r.returncode==0,r.stderr.decode(errors='replace')
  assert partial.stat().st_size==row['bytes'] and sha(partial)==row['sha256']
  partial.replace(p)
 extracted=OUT/'control_evidence'
 with zipfile.ZipFile(p) as z:
  assert z.testzip() is None
  manifest=json.loads(z.read('manifest.json'))
  assert set(z.namelist())==set(manifest)|{'manifest.json'}
  for name,expected in manifest.items():
   rel=PurePosixPath(name);assert not rel.is_absolute() and '..' not in rel.parts and ':' not in name
   assert stat.S_IFMT(z.getinfo(name).external_attr>>16)!=stat.S_IFLNK
   h=hashlib.sha256();count=0
   with z.open(name) as f:
    for b in iter(lambda:f.read(1024*1024),b''):h.update(b);count+=len(b)
   assert count==expected['bytes'] and h.hexdigest()==expected['sha256'],name
   if label=='control':
    dest=extracted.joinpath(*rel.parts);dest.parent.mkdir(parents=True,exist_ok=True)
    with z.open(name) as source,dest.open('wb') as target:
     for b in iter(lambda:source.read(1024*1024),b''):target.write(b)
  if label=='control':save(extracted/'manifest.json',manifest)
 result=dict(row,local_path=p.relative_to(ROOT).as_posix(),local_sha256=sha(p),
  byte_count_verified=True,zip_crc_verified=True,every_member_sha256_verified=True,manifest_files=len(manifest),extracted=label=='control')
 save(OUT/(label+'_download_verification.json'),result)
 print(label.upper()+'_READY '+json.dumps({'path':str(extracted if label=='control' else p),'bytes':row['bytes'],'sha256':row['sha256'],'manifest_files':len(manifest)}),flush=True)
 return result

# Control is deliberately first so analysis need not wait for the RGB-D transfer.
control=download('control')
pool=ThreadPoolExecutor(max_workers=1);rgbd_future=pool.submit(download,'rgbd')
try:
 extracted=OUT/'control_evidence';ctx=extracted/'data/board_workspace_trials/a8_lower25_execution_20261108'
 summary=read(extracted/'result_summary.json');expected=summary['prepared_source_hashes_verified']
 paths=summary['prepared_source_file_paths']
 assert set(expected)==set(PREPARED)==set(paths) and len(expected)==13
 fingerprint={}
 for key,name in paths.items():
  p=extracted/('models' if name.startswith('data/') else 'source')/Path(name).name
  assert p.is_file() and sha(p)==expected[key],key
  fingerprint[key]={'path':p.relative_to(ROOT).as_posix(),'sha256':expected[key],'verified':True}
 preprotected={}
 for name,row in summary['pre_motion_protected_snapshots'].items():
  p=ctx/row['snapshot_path'];assert p.is_file() and sha(p)==row['sha256'],name
  preprotected[name]={'sha256':row['sha256'],'verified':True,'frozen_path':p.relative_to(ROOT).as_posix()}
 extra_math={}
 for name,expected_sha in summary.get('extra_math_source_hashes',{}).items():
  q=extracted/'source'/name;assert q.is_file() and sha(q)==expected_sha,name
  extra_math[name]={'sha256':expected_sha,'verified':True,'path':q.relative_to(ROOT).as_posix()}
 binding=summary['prepared_source_collection_binding']['board_model']
 current_board=extracted/'current_runtime_models'/Path(paths['board_model']).name
 assert sha(current_board)==binding['current_runtime_sha256']
 initial_row=summary['pre_motion_protected_snapshots'][paths['board_model']]
 if binding['temporary_board_model_restored_before_collection']:
  assert sha(current_board)==initial_row['sha256']
 board_provenance={'prepared_board_model_sha256':expected['board_model'],
  'before_trial_original_sha256':initial_row['sha256'],
  'current_runtime_at_collection_sha256':sha(current_board),
  'temporary_board_model_restored_before_collection':binding['temporary_board_model_restored_before_collection'],
  'prepared_model_distinguished_from_original':True}
 learning_evidence={}
 for name in LEARNING:
  before=extracted/'learning_before'/name;after=extracted/'learning_after'/name
  info=summary['learning_snapshots'][name]
  assert before.is_file() and after.is_file() and sha(before)==info['before_sha256'] and sha(after)==info['after_sha256'],name
  learning_evidence[name]={'before_sha256':sha(before),'after_sha256':sha(after),'changed_in_trial':sha(before)!=sha(after),'before_and_after_preserved':True}
 save(OUT/'source_and_learning_verification.json',{'motion_issued':False,'prepared_source_count':13,
  'prepared_sources':fingerprint,'prepared_source_file_paths':paths,'extra_math_sources':extra_math,
  'board_model_provenance':board_provenance,'pre_motion_protected_files':preprotected,'learning':learning_evidence,
  'physical_accepted':False,'operator_feedback_after_collection_kept_separately':True,
  'qualification':'Authorized lower25 trial; physical acceptance remains pending separate operator observation. Software completion alone is not experimental acceptance.'})
 print('FINGERPRINTS_READY '+json.dumps({'prepared_sources':13,'pre_motion_protected_files':len(preprotected),'learning_before_after_files':10,'physical_accepted':False}),flush=True)
 def remote_learning():
  code="""
from pathlib import Path
import base64,hashlib,json
root=Path(REMOTE);rows={}
for name in NAMES:
 p=root/'data'/name;s0=p.stat();data=p.read_bytes();s1=p.stat()
 assert (s0.st_size,s0.st_mtime_ns)==(s1.st_size,s1.st_mtime_ns),name
 rows[name]={'path':str(p),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'mtime_ns':s1.st_mtime_ns,'data_base64':base64.b64encode(data).decode()}
print(json.dumps({'motion_issued':False,'files':rows}))
""".replace('REMOTE',repr(REMOTE)).replace('NAMES',repr(LEARNING))
  r=subprocess.run(['ssh']+OPTS+['nvidia@192.168.201.199','python3 -'],input=code.encode(),capture_output=True,timeout=45)
  assert r.returncode==0,r.stderr.decode(errors='replace')
  return json.loads(r.stdout)
 sync=OUT/'learning_sync';sync.mkdir(exist_ok=True)
 remote=remote_learning();save(sync/'remote_before_sync.json',remote)
 audit=[]
 # Verify all five against the collected after-state before replacing any local file.
 for name in LEARNING:
  row=remote['files'][name];data=base64.b64decode(row['data_base64'])
  assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256']
  assert row['sha256']==learning_evidence[name]['after_sha256'],'Runtime moved beyond archived lower25 trial: '+name
  dest=ROOT/'scripts/data'/name;old=dest.read_bytes() if dest.exists() else None
  old_sha=hashlib.sha256(old).hexdigest() if old is not None else None
  if old is not None:(sync/('local_before_'+name)).write_bytes(old)
  (sync/('remote_new_'+name)).write_bytes(data)
  audit.append({'name':name,'local_path':dest.relative_to(ROOT).as_posix(),'old_bytes':len(old) if old is not None else None,
   'old_sha256':old_sha,'new_bytes':len(data),'new_sha256':row['sha256'],'changed_locally':old!=data,
   'remote_current_equals_archive_after':True,'remote_source':row['path']})
 save(sync/'old_new_hash_audit.json',{'motion_issued':False,'synchronization_complete':False,'rows':audit})
 for row in audit:
  dest=ROOT/row['local_path'];assert (sha(dest) if dest.exists() else None)==row['old_sha256'],'Local file changed during sync: '+row['name']
  pending=sync/(row['name']+'.new.tmp');pending.write_bytes(base64.b64decode(remote['files'][row['name']]['data_base64']))
  os.replace(pending,dest);assert sha(dest)==row['new_sha256']
 remote_after=remote_learning();save(sync/'remote_after_sync.json',remote_after)
 assert all(remote_after['files'][name]['sha256']==remote['files'][name]['sha256'] for name in LEARNING),'Runtime learning changed during synchronization'
 save(sync/'old_new_hash_audit.json',{'motion_issued':False,'synchronization_complete':True,'remote_before_after_consistent':True,
  'rows':audit,'scope':'Only five scripts/data learning JSON files; no control code, geometry, Web state or motor process changed.'})
 print('LEARNING_SYNC_READY '+json.dumps({'files':len(audit),'changed':[r['name'] for r in audit if r['changed_locally']],'remote_freshness_verified_twice':True}),flush=True)
 rgbd=rgbd_future.result()
 save(OUT/'download_verification.json',{'motion_issued':False,'archives':{'control':control,'rgbd':rgbd},
  'prepared_source_count_verified':13,'extra_math_sources_verified':len(extra_math),
  'board_model_provenance':board_provenance,'pre_motion_protected_files_verified':len(preprotected),
  'learning_before_after_files_verified':10,'local_learning_synced':5,'physical_accepted':False,
  'operator_feedback_separate_from_archive':True,'operator_feedback_path':'outputs/a8_lower25_execution_20261108/operator_feedback.json',
  'source_receipt':'source_and_learning_verification.json','learning_sync_receipt':'learning_sync/old_new_hash_audit.json'})
 print('COMPLETE '+json.dumps({'archives_verified':2,'prepared_fingerprints':13,'learning_synced':5,'physical_accepted':False,'motion_issued':False}),flush=True)
finally:pool.shutdown(wait=True)
