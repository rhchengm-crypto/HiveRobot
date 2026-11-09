"""Restore the shared pre-trial model after the serial-owning task has finished."""
from pathlib import Path
import hashlib,json,os,stat,time,urllib.request
ROOT=Path('/home/nvidia/hive_robot/DM_Control_Python')
OUT=ROOT/'data/board_workspace_trials/a8_lower25_execution_20261108'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
receipt=json.loads((OUT/'activation_receipt.json').read_text())
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
state=json.load(client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5))
run=state.get('run') or {}
assert not run.get('running') and not run.get('support_running'),'No restore during live motion/support'
assert run.get('action')=='dynamics:board-grasp'
target=Path(receipt['active_path']);original=OUT/'original_board_workspace_candidate.json'
assert target==ROOT/'data/left_arm_board_workspace_candidate.json'
assert sha(target)==receipt['trial_sha256'] and sha(original)==receipt['original_sha256']
pending=target.with_name(target.name+'.restore-'+str(time.time_ns())+'.pending')
with pending.open('xb') as stream:stream.write(original.read_bytes());stream.flush();os.fsync(stream.fileno())
pending.chmod(stat.S_IMODE(target.stat().st_mode));os.replace(str(pending),str(target))
assert sha(target)==receipt['original_sha256']
report=dict(motion_issued=False,serial_opened=False,web_restarted=False,
    action=run.get('action'),returncode=run.get('returncode'),running=False,
    original_sha256=sha(target),trial_sha256=receipt['trial_sha256'],trial_exact_bytes_retained_in=str(OUT/'board_workspace_trial.json'),
    restored_wall_s=time.time(),holding_motor_commands_unchanged=True)
(OUT/'config_restore_receipt.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
