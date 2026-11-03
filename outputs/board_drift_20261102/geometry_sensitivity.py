"""Offline perturbation audit; candidates are not calibrations or deployed fixes."""
import sys,json
from pathlib import Path
import numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
c=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());R=np.array(c['board_to_cad_rotation']);tcp=np.array(c['tcp_in_wrist_frame_m'])
log=ROOT/'scripts/data/left_arm_v2_8_full_runs/arm_run_1793164847303_93849.txt'
q0=None;audits=[]
for line in log.read_text().splitlines():
 if line.startswith('board waypoint='):
  w=json.loads(line.split('=',1)[1])
  if w['stage']=='contact' and q0 is None:q0=np.array([w['entry_feedback'][n]['pos'] for n in m.names])
 if line.startswith('board tracking audit='):
  a=json.loads(line.split('=',1)[1])
  if a['stage']=='contact':audits.append(a)
q1=np.array([audits[-1]['feedback'][n]['pos'] for n in m.names])
def delta(model,point):return R.T@(tcp_pose(model,q1,point)[:3,3]-tcp_pose(model,q0,point)[:3,3])*1000
baseline=delta(m,tcp);rows=[]
for i,n in enumerate(m.names):
 ref=m.reference.copy();m.reference[i]+=np.deg2rad(1)
 d=delta(m,tcp)-baseline;m.reference=ref
 rows.append({'parameter':n+'_encoder_zero','perturbation_degrees':1,'delta_xyz_change_mm':d.tolist()})
for i,n in enumerate('xyz'):
 p=tcp.copy();p[i]+=.01
 rows.append({'parameter':'tcp_local_'+n,'perturbation_mm':10,'delta_xyz_change_mm':(delta(m,p)-baseline).tolist()})
rot0=tcp_pose(m,q0,tcp)[:3,:3];rot1=tcp_pose(m,q1,tcp)[:3,:3]
angle=float(np.degrees(np.arccos(np.clip((np.trace(rot0.T@rot1)-1)/2,-1,1))))
report={'motion_issued':False,'production_modified':False,'baseline_delta_xyz_mm':baseline.tolist(),'wrist_orientation_change_deg':angle,'perturbations':rows,'interpretation':'Sensitivity alone cannot identify actual calibration error. No parameter should be changed to fit the reported 8 mm without independent evidence.'}
(OUT/'geometry_sensitivity.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
