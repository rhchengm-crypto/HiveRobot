"""Package frozen v33 files; upload is explicit and never deploys or restarts."""
from pathlib import Path
import argparse,ast,hashlib,json,subprocess,zipfile

HERE=Path(__file__).resolve().parent;OUT=HERE.parent;ROOT=OUT.parents[1]
RUNTIME=['left_arm_static_support.py','left_arm_board_grasp.py']
TESTS=['test_left_arm_static_support.py','test_left_arm_board_static_support.py']
REMOTE_BUNDLE='/tmp/hiverobot_a8_v33_20261108.zip'
REMOTE_DEPLOY='/tmp/deploy_a8_v33_20261108.py'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def package():
 home=ROOT/'outputs/a8_reference_home_20261108/control_evidence'
 homectx=home/'data/board_workspace_trials/a8_reference_home_20261108'
 a8=ROOT/'outputs/a8_reference_execution_20261108/control_evidence'
 a8ctx=a8/'data/board_workspace_trials/a8_reference_execution_20261108'
 frozen_home=read(homectx/'source_hashes.json');expected={};anchors={}
 for name,h in frozen_home.items():
  p=home/('models' if name.startswith('data/') else 'source')/Path(name).name
  assert sha(p)==h,name;expected[name]=h;anchors[name]=p.relative_to(ROOT).as_posix()
 oldplan=read(a8ctx/'prepared_plan.json')['prepared_execution']['source_hashes']
 oldmap={'rigid_model':'data/left_arm_general_dynamics.json','board_model':'data/left_arm_board_workspace_candidate.json',
 'planner':'left_arm_board_grasp.py','controller':'left_arm_v2_6.py','motion_adapter':'left_arm_v3_1_motion_dynamics.py',
 'fault_support':'left_arm_fault_support.py','kinematics':'left_arm_board_kinematics.py','collision':'left_arm_board_collision.py',
 'trace':'left_arm_motion_trace.py','task_pose':'left_arm_board_task_pose.py','trajectory_support':'left_arm_trajectory_support.py',
 'general_motion':'left_arm_general_motion.py'}
 old_manifest=read(a8/'manifest.json')
 for key,name in oldmap.items():
  logical=('models/' if name.startswith('data/') else 'source/')+Path(name).name;p=a8/logical
  assert sha(p)==oldplan[key]==old_manifest[logical]['sha256'],name
  assert name not in expected or expected[name]==oldplan[key],name
  expected[name]=oldplan[key];anchors[name]=p.relative_to(ROOT).as_posix()
 before=read(homectx/'before_status.json')['run'];before_log=before.get('full_log_path')
 a8log=read(ROOT/'outputs/a8_reference_execution_20261108/collection_report.json')['phase_logs']['dynamics:board-grasp']['primary_log']
 assert before_log==a8log,'Frozen old planner is not bound to the run immediately preceding Home'
 assert expected['left_arm_board_grasp.py']=='b0a4b9a1353e4198a112712e492139b96450ad7be89e17f7b7e4d4efccc4d7ac'
 for name in ['left_arm_v2_8_clearance_bias.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_6_adaptive_hold_target_bias.json']:
  p=home/'after_learning'/name;expected['data/'+name]=sha(p);anchors['data/'+name]=p.relative_to(ROOT).as_posix()
 for name in ['left_arm_validated_path_library.json','left_arm_v2_6_saved_moves.json']:
  p=a8/'models'/name;expected['data/'+name]=sha(p);anchors['data/'+name]=p.relative_to(ROOT).as_posix()
 driver=ROOT/'outputs/a8_reference_home_20261108/vendor_DM_CAN.py'
 if driver.exists():expected['DM_CAN.py']=sha(driver);anchors['DM_CAN.py']=driver.relative_to(ROOT).as_posix()
 inputs={name:ROOT/'scripts'/name for name in RUNTIME+TESTS}
 inputs['guarded_v33_tests.py']=HERE/'guarded_v33_tests.py'
 for name,p in inputs.items():ast.parse(p.read_text(encoding='utf-8'),filename=name,feature_version=(3,8))
 for name in ['README.md','regression_tests.log','recorded_window_audit.json']:
  p=OUT/name
  if p.is_file():inputs['evidence/'+name]=p
 manifest={'release':'a8_v33_20261108','runtime_order':RUNTIME,'test_files':TESTS,
 'files':{name:sha(p) for name,p in inputs.items()},'expected_remote_hashes':expected,
 'baseline_anchor_files':anchors,'baseline_provenance':{'Home_frozen_source_files':len(frozen_home),
 'Home_before_status_run_log':before_log,'matched_A8_frozen_execution_run_log':a8log,
 'board_grasp_sha256':expected['left_arm_board_grasp.py'],
 'board_grasp_validation':'frozen A8 source bytes == its prepared-plan SHA == archive manifest SHA; run is Home immediately preceding status',
 'remote_current_hash_used_as_planner_baseline':False},
 'optional_protected_paths':['data/left_arm_v2_6_adaptive_hold_tau.json','data/left_arm_v2_6_adaptive_active_tau.json'],
 'motion_issued_by_package':False,'hardware_validation_complete':False}
 bundle=OUT/'a8_v33_static_support_bundle.zip'
 with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for name,p in inputs.items():z.write(p,name)
  z.writestr('manifest.json',json.dumps(manifest,indent=2))
 save(OUT/'release_manifest.json',manifest)
 digest=sha(bundle);(OUT/'bundle_sha256.txt').write_text(digest+'\n')
 template=(HERE/'deploy_v33_remote_template.py').read_text(encoding='utf-8')
 rendered=HERE/'deploy_v33_remote.py';rendered.write_text(template.replace('__BUNDLE_SHA__',digest),encoding='utf-8')
 receipt={'package_only':True,'uploaded':False,'deployed':False,'motion_issued':False,
 'bundle':bundle.relative_to(ROOT).as_posix(),'bytes':bundle.stat().st_size,'sha256':digest,
 'rendered_deploy':rendered.relative_to(ROOT).as_posix(),'rendered_deploy_sha256':sha(rendered),
 'source_file_sha256':{name:manifest['files'][name] for name in RUNTIME+TESTS},
 'protected_baseline_files':len(expected),'baseline_provenance':manifest['baseline_provenance']}
 save(OUT/'package_receipt.json',receipt);print(json.dumps(receipt,indent=2))
def upload():
 receipt=read(OUT/'package_receipt.json');bundle=ROOT/receipt['bundle'];script=ROOT/receipt['rendered_deploy']
 assert sha(bundle)==receipt['sha256'] and sha(script)==receipt['rendered_deploy_sha256']
 # Root explicitly runs this only after reviewing the final local tests.
 opts=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199',
 '-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5']
 for source,dest in [(bundle,REMOTE_BUNDLE),(script,REMOTE_DEPLOY)]:
  r=subprocess.run(['scp']+opts+[str(source),'nvidia@192.168.201.199:'+dest],capture_output=True,timeout=90)
  assert r.returncode==0,r.stderr.decode(errors='replace')
 code="import hashlib,json;from pathlib import Path; ps="+repr([REMOTE_BUNDLE,REMOTE_DEPLOY])+";print(json.dumps({p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in ps}))"
 r=subprocess.run(['ssh']+opts+['nvidia@192.168.201.199','python3 -'],input=code.encode(),capture_output=True,timeout=30)
 assert r.returncode==0,r.stderr.decode(errors='replace');remote=json.loads(r.stdout)
 assert remote[REMOTE_BUNDLE]==receipt['sha256'] and remote[REMOTE_DEPLOY]==receipt['rendered_deploy_sha256']
 result={'uploaded':True,'deployed':False,'motion_issued':False,'web_restarted':False,'remote_sha256':remote,
 'next_step_root_only':'python3 '+REMOTE_DEPLOY}
 save(OUT/'upload_receipt.json',result);print(json.dumps(result,indent=2))
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['package','upload']);args=parser.parse_args()
 package() if args.mode=='package' else upload()
