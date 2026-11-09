"""Idle-only task-pose height follow-up deployment. No controller construction, motion or Web restart."""
from pathlib import Path
import datetime,hashlib,json,os,stat,subprocess,sys,urllib.request,zipfile

ROOT=Path('/home/nvidia/hive_robot/DM_Control_Python').resolve()
BUNDLE=Path('/tmp/hiverobot_a8_height_followup_20261108.zip')
EXPECTED_BUNDLE_SHA='__BUNDLE_SHA__'
RUNTIME_ORDER=['left_arm_board_task_pose.py']
TESTS=['test_left_arm_board_task_pose.py']
PREPARED_FILES={'rigid_model':'data/left_arm_general_dynamics.json',
 'board_model':'data/left_arm_board_workspace_candidate.json','planner':'left_arm_board_grasp.py',
 'controller':'left_arm_v2_6.py','motion_adapter':'left_arm_v3_1_motion_dynamics.py',
 'fault_support':'left_arm_fault_support.py','kinematics':'left_arm_board_kinematics.py',
 'collision':'left_arm_board_collision.py','trace':'left_arm_motion_trace.py',
 'task_pose':'left_arm_board_task_pose.py','static_support':'left_arm_static_support.py',
 'trajectory_support':'left_arm_trajectory_support.py','general_motion':'left_arm_general_motion.py'}
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def idle():
 with client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5) as response:s=json.load(response)
 r=s.get('run') or {};assert not r.get('running') and not r.get('support_running'),'Web task/support active'
 active=[]
 arm_names={'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v2_6.py','left_arm_v3_1_model_control.py','left_arm_general_move.py'}
 for p in Path('/proc').iterdir():
  if not p.name.isdigit() or int(p.name)==os.getpid():continue
  try:argv=[x.decode(errors='replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
  except OSError:continue
  if any(Path(v).name in arm_names for v in argv[:4]):active.append({'pid':int(p.name),'argv':argv})
 assert not active,'Arm process still active: '+str(active)
 return {k:r.get(k) for k in ('action','running','support_running','returncode','finished_at')}
def root_state(name):
 p=(ROOT/name).resolve();assert ROOT in p.parents
 return sha(p) if p.is_file() else None
assert sha(BUNDLE)==EXPECTED_BUNDLE_SHA,'Uploaded bundle differs'
before_run=idle()
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
stage=ROOT/'data/deployment_staging'/('a8_height_followup_'+stamp);stage.mkdir(parents=True,exist_ok=False)
with zipfile.ZipFile(BUNDLE) as z:
 manifest=json.loads(z.read('manifest.json'))
 assert manifest['runtime_order']==RUNTIME_ORDER and manifest['test_files']==TESTS
 assert manifest['baseline_provenance']['old_task_pose_sha256']==manifest['expected_remote_hashes']['left_arm_board_task_pose.py']
 assert manifest['baseline_provenance']['old_task_pose_sha256']=='04972339e770869b478eda2b32d7137c2be67e363472c686cf7e5f086a2e0271'
 assert set(z.namelist())==set(manifest['files'])|{'manifest.json'}
 for name,expected in manifest['files'].items():
  p=(stage/name).resolve();assert stage in p.parents,'Bundle traversal rejected'
  data=z.read(name);assert hashlib.sha256(data).hexdigest()==expected,name
  p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
  if name.endswith('.py'):compile(data,str(p),'exec')
(stage/'manifest.json').write_text(json.dumps(manifest,indent=2))
expected=manifest['expected_remote_hashes']
for name,h in expected.items():assert root_state(name)==h,'Frozen baseline differs: '+name
install=RUNTIME_ORDER+TESTS
replace_before={name:root_state(name) for name in install}
optional_protected={name:root_state(name) for name in manifest['optional_protected_paths']}
preflight={'before_run':before_run,'expected_remote_hashes':expected,'replacement_file_preflight_sha256':replace_before,'optional_protected_sha256':optional_protected,'baseline_provenance':manifest['baseline_provenance']}
(stage/'preflight.json').write_text(json.dumps(preflight,indent=2))
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
check=subprocess.run([sys.executable,'-B',str(stage/'guarded_task_pose_tests.py'),str(stage),str(ROOT)],
 cwd=stage,env=env,capture_output=True,text=True,timeout=180)
(stage/'offline_tests.stdout').write_text(check.stdout);(stage/'offline_tests.stderr').write_text(check.stderr)
assert check.returncode==0,'Staged hardware-free tests failed; see staging test logs'
rows=[line for line in check.stdout.splitlines() if line.lstrip().startswith('{')]
tests=json.loads(rows[-1]);assert tests['ok'] and not tests['serial_opened'] and not tests['sdk_constructed'] and not tests['motion_issued']
idle()
for name,h in expected.items():assert root_state(name)==h,'Baseline changed during tests: '+name
for name,h in replace_before.items():assert root_state(name)==h,'Replacement input changed during tests: '+name
for name,h in optional_protected.items():assert root_state(name)==h,'Protected optional file changed during tests: '+name
backup=ROOT/'data/code_backups'/('a8_height_followup_'+stamp);backup.mkdir(parents=True,exist_ok=False)
for name,h in replace_before.items():
 if h is not None:
  p=backup/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((ROOT/name).read_bytes());assert sha(p)==h
(backup/'before.json').write_text(json.dumps(preflight,indent=2))
for name in install:
 target=ROOT/name;pending=ROOT/(name+'.height-followup-'+stamp+'.pending')
 with pending.open('xb') as f:f.write((stage/name).read_bytes());f.flush();os.fsync(f.fileno())
 pending.chmod(stat.S_IMODE(target.stat().st_mode) if target.exists() else 0o644)
 os.replace(str(pending),str(target));assert sha(target)==manifest['files'][name]
deployed={name:root_state(name) for name in install}
protected={name:root_state(name) for name in expected if name not in install}
for name,h in protected.items():assert h==expected[name],'Protected source/model changed: '+name
for name,h in optional_protected.items():assert root_state(name)==h,'Protected optional file changed: '+name
prepared={key:root_state(name) for key,name in PREPARED_FILES.items()}
assert len(prepared)==13 and all(prepared.values())
report={'ok':True,'release':'a8_height_followup_20261108','motion_issued':False,'serial_opened':False,'sdk_constructed':False,
 'web_restarted':False,'stage':str(stage),'backup':str(backup),'before_run':before_run,'after_run':idle(),
 'installation_order':install,'replacement_file_preflight_sha256':replace_before,'deployed_sha256':deployed,
 'protected_sha256':protected,'optional_protected_sha256':optional_protected,
 'offline_hardware_free_tests':tests,'prepared_source_hashes':prepared,'prepared_source_file_paths':PREPARED_FILES,
 'prepared_hash_count':len(prepared),'old_prepared_plans_invalidated':True,'hardware_validation_complete':False}
(stage/'deployment_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
