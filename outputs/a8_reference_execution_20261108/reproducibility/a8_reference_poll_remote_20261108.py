from pathlib import Path
import json,urllib.request,re,time
root=Path('/home/nvidia/hive_robot/DM_Control_Python');out=root/'data/board_workspace_trials/a8_reference_execution_20261108'
c=urllib.request.build_opener(urllib.request.ProxyHandler({}))
s=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5));r=s.get('run') or {}
(out/({'home':'home_motion_status.json','dynamics:model-hold-after:home':'home_hold_status.json',
       'table-clearance':'clearance_motion_status.json','dynamics:model-hold-after:table-clearance':'clearance_hold_status.json',
       'dynamics:board-grasp':'board_motion_status.json'}.get(r.get('action'),'other_status.json'))).write_text(json.dumps(s,indent=2))
(out/'latest_status.json').write_text(json.dumps(s,indent=2))
text=Path(r['full_log_path']).read_text(errors='replace') if r.get('full_log_path') and Path(r['full_log_path']).exists() else r.get('stdout','')
traces=re.findall(r'trace_\d+\.jsonl',text);trace=None
if traces:
    path=root/'data/board_motion_traces'/traces[-1]
    if path.exists():
        with path.open('rb') as f:f.seek(max(0,path.stat().st_size-15000));lines=f.read().splitlines()
        for line in reversed(lines):
            try:row=json.loads(line)
            except ValueError:continue
            if row.get('event')=='motor_frame':
                trace=dict(path=str(path),wall_s=row.get('wall_s'),context=row.get('context'),
                    feedback=row.get('feedback'),shoulder_rotate_command=row.get('commands',{}).get('shoulder_rotate'));break
result={k:r.get(k) for k in ('action','ok','running','support_running','returncode','duration_s','post_move_model_hold','full_log_path','error')}
result.update(command=r.get('command'),trace=trace,
    fault_markers=[marker for marker in ('confirmed encoder oscillation','board fault support active','MotionStop','Traceback (most recent call last)') if marker in text],
    tail=[v[:500] for v in text.splitlines()[-2:]],polled_wall_s=time.time())
(out/'latest_poll.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
