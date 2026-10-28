import sys,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'scripts'))
from left_arm_board_grasp import RigidBodyModel,load_board_config,tcp_pose
m=RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json');c=load_board_config(root/'scripts/data/left_arm_board_grasp.json',m)
R=np.array(c['board_to_cad_rotation']);t=np.array(c['board_to_cad_translation_m']);tcp=np.array(c['tcp_in_wrist_frame_m'])
def xyz(q):return (tcp_pose(m,q,tcp)[:3,3]-t)@R*1000
def vec(q):return np.array([q[n] for n in m.names])
def attribution(path):
 total=np.zeros((3,m.n));nodes,weights=np.polynomial.legendre.leggauss(4)
 for a,b in zip(path,path[1:]):
  for node,w in zip(nodes,weights):
   q=a+(b-a)*(node+1)/2
   j,_=m.point_jacobian(m.encoder_to_model(q),m.n-1,tcp)
   total+=(R.T@j*m.signs[None,:])*(b-a)[None,:]*w/2*1000
 return total
reports=[]
for name in ['arm_run_1793157033682_73140.txt','arm_run_1793158636951_76399.txt','arm_run_1793160008611_79906.txt','arm_run_1793161074626_82286.txt']:
 s=(root/'scripts/data/left_arm_v2_8_full_runs'/name).read_text();dec=json.JSONDecoder()
 waypoints=[]
 for line in s.splitlines():
  if line.startswith('board waypoint= '):waypoints.append(json.loads(line.split('= ',1)[1]))
  elif line.startswith('v2.2 continuous group step') and 'targets= ' in line and waypoints:
   waypoints[-1]['encoder_rad']=json.loads(line.split('targets= ',1)[1])
 contact=[r for r in waypoints if r['stage']=='contact']
 first=contact[0];idx=waypoints.index(first)
 measured_start=vec({n:v['pos'] for n,v in first['entry_feedback'].items()})
 e=dec.raw_decode(s.rsplit('board endpoint report= ',1)[1])[0]
 audits=[json.loads(l.split('= ',1)[1]) for l in s.splitlines() if l.startswith('board tracking audit= ')];audits=[r for r in audits if r['stage']=='contact']
 actual=[measured_start]+[vec({n:v['pos'] for n,v in r['feedback'].items()}) for r in audits]+[vec(e['measured_pose_rad'])]
 nominal=[vec(waypoints[idx-1]['encoder_rad'])]+[vec(r['encoder_rad']) for r in contact]
 at=attribution(actual);nt=attribution(nominal)
 report={'file':name,'measured_contact_start_model_mm':xyz(actual[0]).tolist(),'measured_contact_end_model_mm':xyz(actual[-1]).tolist(),'nominal_contact_delta_model_mm':(xyz(nominal[-1])-xyz(nominal[0])).tolist(),'measured_contact_delta_model_mm':(xyz(actual[-1])-xyz(actual[0])).tolist(),'physical_observation_is_independent':True,'axes':[]}
 for k,n in enumerate(m.names):
  report['axes'].append({'joint':n,'nominal_descent_deg':float(np.degrees(nominal[-1][k]-nominal[0][k])),'actual_descent_deg':float(np.degrees(actual[-1][k]-actual[0][k])), 'planned_model_X_mm':float(nt[0,k]),'actual_model_X_mm':float(at[0,k]),'max_abs_tracking_error_deg':max(abs(r['error_deg'][n]) for r in audits)})
 reports.append(report)
Path(__file__).with_name('full_descent_axis_comparison.json').write_text(json.dumps(reports,indent=2))
for r in reports:
 print(r['file'],'measured XYZ delta',r['measured_contact_delta_model_mm'])
 for a in r['axes']:print(a)
