"""Read-only motor status and source freeze for the authorized full v33 test."""
from pathlib import Path
import hashlib,json,math,os,subprocess,time,urllib.request
ROOT=Path('/home/nvidia/hive_robot/DM_Control_Python')
OUT=ROOT/'data/board_workspace_trials/a8_lower25_execution_20261108'
assert not OUT.exists(),'This experiment context already exists; inspect before repeating'
OUT.mkdir(parents=True)
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
status=json.load(client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5))
run=status.get('run') or {}
assert not run.get('running') and not run.get('support_running')
assert status['execution_enabled'] and status['auto_model_hold_enabled'] and status['rigid_body_mode']=='apply'
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid():continue
    try:argv=[v.decode(errors='replace') for v in (p/'cmdline').read_bytes().split(b'\0') if v]
    except OSError:continue
    assert not any(Path(v).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py'} for v in argv[:3]),argv
deployment=json.loads((ROOT/'data/deployment_staging/a8_height_followup_20261109T015644Z/deployment_report.json').read_text())
expected=dict(deployment['protected_sha256'],**deployment['deployed_sha256'])
expected.update(deployment['optional_protected_sha256'])
snap=OUT/'runtime_snapshot';snap.mkdir()
for name,digest in expected.items():
    if digest is None:continue
    data=(ROOT/name).read_bytes();assert hashlib.sha256(data).hexdigest()==digest,name
    dest=snap/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
names=list(json.loads((ROOT/'data/left_arm_v2_home.json').read_text())['joints'])
names=[n for n in names if n!='claw']
assert len(names)==7
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
read=subprocess.run(['python3','-u','left_arm_v2_8.py','status','--joints',','.join(names)],cwd=ROOT,env=env,capture_output=True,text=True,timeout=20)
assert read.returncode==0,read.stderr
(OUT/'before_status.txt').write_text(read.stdout)
feedback=json.loads(read.stdout[read.stdout.index('{'):])
home=json.loads((ROOT/'data/left_arm_v2_home.json').read_text())['joints']
home_errors={n:math.degrees(feedback[n]['pos']-home[n]) for n in names}
assert all(abs(v)<5 for v in home_errors.values()),home_errors
assert all(abs(feedback[n]['vel'])<.1 for n in names),feedback
for n in names:
    assert all(math.isfinite(float(feedback[n][k])) for k in ('pos','vel','tau'))
learning={}
for name in ('left_arm_v2_8_clearance_bias.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_6_adaptive_hold_target_bias.json','left_arm_v2_6_adaptive_hold_tau.json','left_arm_v2_6_adaptive_active_tau.json'):
    data=(ROOT/'data'/name).read_bytes();(OUT/('before_'+name)).write_bytes(data)
    learning[name]=hashlib.sha256(data).hexdigest()
(OUT/'before_web_status.json').write_text(json.dumps(status,indent=2))
report=dict(motion_issued=False,feedback=feedback,home_errors_deg=home_errors,
    source_hashes={n:d for n,d in expected.items() if d is not None},
    prepared_source_hashes=deployment['prepared_source_hashes'],learning_hashes=learning,
    saved_A8_pose=json.loads((ROOT/'data/left_arm_v2_6_saved_moves.json').read_text())['moves']['A8_take']['pose'],
    ordinary_clearance=json.loads((ROOT/'data/left_arm_v2_table_clearance.json').read_text())['joints'],
    last_action={k:run.get(k) for k in ('action','running','returncode','full_log_path')},
    settings={k:status.get(k) for k in ('auto_model_hold_enabled','rigid_body_mode','execution_enabled','board_model')},
    authorization='user: 现在实测; confirmed onsite E-stop, A8/channel empty, board/base/table unchanged; explicit new full frame12.5deg/front extra9.4deg; full descent and same-serial hold, no claw',
    captured_wall_s=time.time())
(OUT/'preflight.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
