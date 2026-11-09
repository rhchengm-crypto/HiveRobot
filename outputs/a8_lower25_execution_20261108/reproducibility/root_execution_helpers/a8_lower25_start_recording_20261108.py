from pathlib import Path
import json,subprocess,shlex,time,urllib.request
ROOT=Path('/home/nvidia/hive_robot/DM_Control_Python')
OUT=ROOT/'data/board_workspace_trials/a8_lower25_execution_20261108'
assert not (OUT/'recording.json').exists(),'Recorder already registered'
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
status=json.load(client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5))
run=status.get('run') or {}
assert not run.get('running') and not run.get('support_running')
folder=ROOT/'data/rgbd_audits'/('a8_lower25_execution_20261108_'+str(time.time_ns()))
with folder.with_suffix('.log').open('ab') as log:
    recorder=subprocess.Popen(['bash','-c','source /opt/ros/noetic/setup.bash && source /home/nvidia/ascam_ws/devel/setup.bash && exec python3 -u record_left_arm_rgbd_audit.py --seconds 900 --fps 2 --output '+shlex.quote(str(folder))],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
record=dict(recorder_pid=recorder.pid,recording_folder=str(folder),motion_issued=False)
(OUT/'recording.json').write_text(json.dumps(record,indent=2))
for _ in range(15):
    if list(folder.glob('*.jpg')) and list(folder.glob('*.npy')):break
    assert recorder.poll() is None,'RGB-D recorder exited'
    time.sleep(1)
else:raise RuntimeError('No RGB-D frame pair; no motor command issued')
print(json.dumps(record))
