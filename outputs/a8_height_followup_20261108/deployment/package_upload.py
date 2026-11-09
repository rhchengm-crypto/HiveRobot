"""Build/upload a two-source update from frozen execution/Home baselines; never moves."""
from pathlib import Path
import argparse,ast,hashlib,json,subprocess,zipfile
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
RUNTIME=['left_arm_board_task_pose.py'];TESTS=['test_left_arm_board_task_pose.py']
REMOTE_BUNDLE='/tmp/hiverobot_a8_height_followup_20261108.zip'
REMOTE_DEPLOY='/tmp/deploy_a8_height_followup_20261108.py'
LEARNING=['left_arm_v2_8_clearance_bias.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_6_adaptive_hold_target_bias.json','left_arm_v2_6_adaptive_hold_tau.json','left_arm_v2_6_adaptive_active_tau.json']
PREPARED={'rigid_model':'data/left_arm_general_dynamics.json','board_model':'data/left_arm_board_workspace_candidate.json','planner':'left_arm_board_grasp.py','controller':'left_arm_v2_6.py','motion_adapter':'left_arm_v3_1_motion_dynamics.py','fault_support':'left_arm_fault_support.py','kinematics':'left_arm_board_kinematics.py','collision':'left_arm_board_collision.py','trace':'left_arm_motion_trace.py','task_pose':'left_arm_board_task_pose.py','static_support':'left_arm_static_support.py','trajectory_support':'left_arm_trajectory_support.py','general_motion':'left_arm_general_motion.py'}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def package():
 home=ROOT/'outputs/a8_v33_home_20261108/control_evidence';homectx=home/'data/board_workspace_trials/a8_v33_home_20261108'
 trial=ROOT/'outputs/a8_v33_execution_20261108/control_evidence';trialctx=trial/'data/board_workspace_trials/a8_v33_execution_20261108'
 hs=read(homectx/'source_hashes.json');summary=read(trial/'result_summary.json');expected={};anchors={}
 def anchor(name,p,h):
  assert sha(p)==h,name
  assert name not in expected or expected[name]==h,'Baseline provenance conflict: '+name
  expected[name]=h;anchors[name]=p.relative_to(ROOT).as_posix()
 assert len(hs)==13 and 'left_arm_board_task_pose.py' not in hs
 for name,h in hs.items():anchor(name,home/('models' if name.startswith('data/') else 'source')/Path(name).name,h)
 for key,name in PREPARED.items():anchor(name,trial/('models' if name.startswith('data/') else 'source')/Path(name).name,summary['prepared_source_hashes_verified'][key])
 assert expected['left_arm_board_task_pose.py']=='04972339e770869b478eda2b32d7137c2be67e363472c686cf7e5f086a2e0271'
 for name,row in summary['pre_motion_protected_snapshots'].items():
  if name.startswith('data/') and Path(name).name in LEARNING:continue
  anchor(name,trialctx/row['snapshot_path'],row['sha256'])
 for name in LEARNING:
  p=home/'after_learning'/name;anchor('data/'+name,p,sha(p))
 inputs={name:ROOT/'scripts'/name for name in RUNTIME+TESTS}
 # Extra regression module and data are staging-only fixtures, never installed.
 inputs['test_left_arm_board_kinematics.py']=ROOT/'scripts/test_left_arm_board_kinematics.py'
 inputs['guarded_task_pose_tests.py']=HERE/'guarded_task_pose_tests.py'
 for name in ['left_arm_general_dynamics.json','left_arm_board_workspace_continuous_candidate.json','left_arm_board_workspace_candidate.json','left_arm_v2_table_clearance.json']:
  p=ROOT/'scripts/data'/name;inputs['data/'+name]=p
  if 'data/'+name in expected:assert sha(p)==expected['data/'+name],name
  else:assert name=='left_arm_board_workspace_continuous_candidate.json'
 for name,p in inputs.items():
  if name.endswith('.py'):ast.parse(p.read_text(encoding='utf-8'),filename=name,feature_version=(3,8))
 manifest={'release':'a8_height_followup_20261108','runtime_order':RUNTIME,'test_files':TESTS,
 'offline_modules':['test_left_arm_board_task_pose','test_left_arm_board_kinematics'],
 'files':{name:sha(p) for name,p in inputs.items()},'expected_remote_hashes':expected,'baseline_anchor_files':anchors,
 'baseline_provenance':{'Home_frozen_source_files':13,'Home_hashes_exclude_task_pose':True,'old_task_pose_sha256':expected['left_arm_board_task_pose.py'],
 'task_pose_source':'v33 completed execution archive source == prepared-source SHA == checked ZIP manifest',
 'learning_source':'v33 return-Home archive after_learning; five files verified remote twice',
 'remote_current_used_as_task_pose_baseline':False},'optional_protected_paths':['data/left_arm_board_workspace_continuous_candidate.json'],
 'staging_fixture_policy':'Local continuous_candidate is an offline test fixture only; runtime same-name file is SHA protected unchanged, never installed',
 'installation_scope':RUNTIME+TESTS,'models_or_configurations_installed':False,'motion_issued':False}
 bundle=HERE/'a8_height_followup_bundle.zip'
 with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for name,p in inputs.items():z.write(p,name)
  z.writestr('manifest.json',json.dumps(manifest,indent=2))
 save(HERE/'release_manifest.json',manifest)
 digest=sha(bundle);template=(HERE/'deploy_remote_template.py').read_text(encoding='utf-8')
 rendered=HERE/'deploy_remote.py';rendered.write_text(template.replace('__BUNDLE_SHA__',digest),encoding='utf-8')
 receipt={'package_only':True,'uploaded':False,'deployed':False,'motion_issued':False,
 'bundle':bundle.relative_to(ROOT).as_posix(),'bytes':bundle.stat().st_size,'sha256':digest,
 'rendered_deploy':rendered.relative_to(ROOT).as_posix(),'rendered_deploy_sha256':sha(rendered),
 'source_file_sha256':{name:manifest['files'][name] for name in RUNTIME+TESTS},'protected_baseline_files':len(expected),
 'baseline_provenance':manifest['baseline_provenance']}
 save(HERE/'package_receipt.json',receipt);print(json.dumps(receipt,indent=2))
def upload():
 receipt=read(HERE/'package_receipt.json');bundle=ROOT/receipt['bundle'];script=ROOT/receipt['rendered_deploy']
 assert sha(bundle)==receipt['sha256'] and sha(script)==receipt['rendered_deploy_sha256']
 opts=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5']
 for source,dest in [(bundle,REMOTE_BUNDLE),(script,REMOTE_DEPLOY)]:
  r=subprocess.run(['scp']+opts+[str(source),'nvidia@192.168.201.199:'+dest],capture_output=True,timeout=90)
  assert r.returncode==0,r.stderr.decode(errors='replace')
 code="import hashlib,json;from pathlib import Path;ps="+repr([REMOTE_BUNDLE,REMOTE_DEPLOY])+";print(json.dumps({p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in ps}))"
 r=subprocess.run(['ssh']+opts+['nvidia@192.168.201.199','python3 -'],input=code.encode(),capture_output=True,timeout=30)
 assert r.returncode==0,r.stderr.decode(errors='replace');remote=json.loads(r.stdout)
 assert remote[REMOTE_BUNDLE]==receipt['sha256'] and remote[REMOTE_DEPLOY]==receipt['rendered_deploy_sha256']
 result={'uploaded':True,'deployed':False,'motion_issued':False,'web_restarted':False,'remote_sha256':remote,'root_next_step':'python3 '+REMOTE_DEPLOY}
 save(HERE/'upload_receipt.json',result);print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['package','upload']);a=p.parse_args()
 package() if a.mode=='package' else upload()
