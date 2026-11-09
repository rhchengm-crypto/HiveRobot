from pathlib import Path
import urllib.request,json
root=Path('/home/nvidia/hive_robot/DM_Control_Python');out=root/'data/board_workspace_trials/a8_reference_execution_20261108'
c=urllib.request.build_opener(urllib.request.ProxyHandler({}))
s=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5));r=s.get('run') or {}
assert r.get('action')=='dynamics:board-grasp' and r.get('ok') and r.get('returncode')==0
assert not r.get('running') and not r.get('support_running')
assert r.get('post_move_model_hold',{}).get('state')=='complete'
assert r.get('post_move_model_hold',{}).get('same_serial_session') is True
(out/'execution_complete_status.json').write_text(json.dumps(s,indent=2))
print(json.dumps(dict(action=r['action'],returncode=0,running=False,
    duration_s=r.get('duration_s'),full_log_path=r['full_log_path'],same_serial_hold=True)))
