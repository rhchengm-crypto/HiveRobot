import json,math
from pathlib import Path
out=Path(__file__).parent;root=out.parents[1];reports=[]
for version in ('v15a','v16','v17','v18'):
 source=out/(version+'_full_test_report.json')
 if not source.exists():continue
 report=json.loads(source.read_text(encoding='utf8'));p=root/'scripts/data/left_arm_v2_8_full_runs'/Path(report['logs_synced'][-1]).name;s=p.read_text();waypoints=[]
 for line in s.splitlines():
  if line.startswith('board waypoint= '):waypoints.append(json.loads(line.split('= ',1)[1]))
  elif line.startswith('v2.2 continuous group step') and 'targets= ' in line and waypoints:waypoints[-1]['encoder_rad']=json.loads(line.split('targets= ',1)[1])
 first=next(v for v in waypoints if v['stage']=='contact');idx=waypoints.index(first)
 desired_start=waypoints[idx-1]['encoder_rad'];measured_start={n:v['pos'] for n,v in first['entry_feedback'].items()};e=report['endpoint']
 audits=[json.loads(l.split('= ',1)[1]) for l in s.splitlines() if l.startswith('board tracking audit= ')];audits=[v for v in audits if v['stage']=='contact']
 rows={}
 for n in ('shoulder_side','shoulder_rotate','wrist_side'):
  delta=math.degrees(e['measured_pose_rad'][n]-measured_start[n])
  labels=('outward','inward') if n!='shoulder_rotate' else ('inward','outward')
  rows[n]={'commanded_total_deg':math.degrees(e['nominal_target_rad'][n]-desired_start[n]),'actual_total_deg':delta,'actual_direction_web':labels[0] if delta>0 else labels[1], 'peak_abs_reference_error_deg':max(abs(v['error_deg'][n]) for v in audits),'terminal_error_deg':e['nominal_error_deg'][n], 'kp_range':[min(v['last_sent_commands'][n]['kp'] for v in audits),max(v['last_sent_commands'][n]['kp'] for v in audits)]}
 reports.append({'version':version,'source_log':str(p),'descent_axes':rows,'operator_observation':report.get('operator_observation')})
(out/'three_axis_descent_comparison.json').write_text(json.dumps(reports,indent=2),encoding='utf8')
for r in reports:print(json.dumps(r))
