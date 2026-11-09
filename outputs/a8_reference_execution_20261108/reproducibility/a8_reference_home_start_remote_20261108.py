from pathlib import Path
import json,hashlib,urllib.request,subprocess,shlex,time
root=Path('/home/nvidia/hive_robot/DM_Control_Python');out=root/'data/board_workspace_trials/a8_reference_execution_20261108'
verified=json.loads((out/'verified_plan.json').read_text())
assert hashlib.sha256(Path(verified['prepared_plan_path']).read_bytes()).hexdigest()==verified['plan_sha256']
assert hashlib.sha256((root/'data/left_arm_board_workspace_candidate.json').read_bytes()).hexdigest()==verified['common_config_sha256']
c=urllib.request.build_opener(urllib.request.ProxyHandler({}));base='http://127.0.0.1:8100'
s=json.load(c.open(base+'/api/v3/dynamics/status',timeout=5));r=s.get('run') or {}
assert not r.get('running') and not r.get('support_running')
assert s['execution_enabled'] and s['auto_model_hold_enabled'] and s['rigid_body_mode']=='apply'
assert not (out/'home_start.json').exists(),'Home was already submitted; inspect state before repetition'
folder=root/'data/rgbd_audits'/('a8_reference_20261108_'+str(time.time_ns()))
with folder.with_suffix('.log').open('ab') as log:
    rec=subprocess.Popen(['bash','-c','source /opt/ros/noetic/setup.bash && source /home/nvidia/ascam_ws/devel/setup.bash && exec python3 -u record_left_arm_rgbd_audit.py --seconds 600 --fps 2 --output '+shlex.quote(str(folder))],cwd=root,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(out/'recording.json').write_text(json.dumps(dict(recorder_pid=rec.pid,recording_folder=str(folder))))
for _ in range(15):
    if list(folder.glob('*.jpg')) and list(folder.glob('*.npy')):break
    assert rec.poll() is None,'recorder exited before motion'
    time.sleep(1)
else:raise RuntimeError('no RGB-D pair; no motion sent')
req=urllib.request.Request(base+'/api/action/home',data=b'{}',headers={'Content-Type':'application/json'},method='POST')
result=json.load(c.open(req,timeout=10))
(out/'home_start.json').write_text(json.dumps(result,indent=2))
print(json.dumps(dict(recording_folder=str(folder),run={k:result.get(k) for k in ('ok','action','running','error','full_log_path','command')})))
