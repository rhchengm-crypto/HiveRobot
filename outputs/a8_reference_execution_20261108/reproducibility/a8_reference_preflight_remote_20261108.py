from pathlib import Path
import json,hashlib,urllib.request,os,subprocess,time
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
out=root/'data/board_workspace_trials/a8_reference_execution_20261108'
out.mkdir(parents=True,exist_ok=True)
c=urllib.request.build_opener(urllib.request.ProxyHandler({}))
s=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5))
r=s.get('run') or {}
assert not r.get('running') and not r.get('support_running'),'active motion/support process'
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid():continue
    try:argv=[v.decode(errors='replace') for v in (p/'cmdline').read_bytes().split(b'\0') if v]
    except OSError:continue
    assert not any(Path(v).name in {'left_arm_v2_8.py','left_arm_v2_6.py','left_arm_v3_1_model_control.py','left_arm_board_grasp.py','left_arm_general_move.py'} for v in argv[:3]),argv
snapshot=json.loads((root/'data/board_workspace_trials/a8_saved_reference_20261108/saved_moves_snapshot.json').read_text())
names=list(snapshot['sources'][0]['data']['moves']['A8_take']['pose'])
result=subprocess.run(['python3','-u','left_arm_v2_8.py','status','--joints',','.join(names)],cwd=root,capture_output=True,text=True,timeout=20)
assert result.returncode==0,result.stderr
(out/'before_status.txt').write_text(result.stdout)
feedback=json.loads(result.stdout[result.stdout.index('{'):])
(out/'before_web_status.json').write_text(json.dumps(s,indent=2))
prior=json.loads((root/'data/board_workspace_trials/a8_test_20261108/preflight.json').read_text())
hashes={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in prior['source_hashes']}
assert hashes==prior['source_hashes'],'runtime source/model changed since inverse diagnostic'
learning={}
for name in ['left_arm_v2_8_clearance_bias.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_6_adaptive_hold_target_bias.json']:
    path=root/'data'/name
    if path.exists():
        data=path.read_bytes();(out/('before_'+name)).write_bytes(data)
        learning[name]=hashlib.sha256(data).hexdigest()
report=dict(motion_issued=False,feedback=feedback,source_hashes=hashes,learning_hashes=learning,
    saved_A8_pose=json.loads((root/'data/left_arm_v2_6_saved_moves.json').read_text())['moves']['A8_take']['pose'],
    last_action={k:r.get(k) for k in ('action','running','returncode','full_log_path')},
    settings={k:s.get(k) for k in ('auto_model_hold_enabled','rigid_body_mode','execution_enabled','board_model')},
    captured_wall_s=time.time())
(out/'preflight.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
