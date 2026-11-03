"""Offline exact-pose descent with one redundant joint fixed at entry."""
import sys, json
from pathlib import Path
import numpy as np
OUT=Path(__file__).resolve().parent; ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'left_arm_general_model/python_deps'))
sys.path.insert(0,str(ROOT/'scripts'))
from scipy.optimize import least_squares
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose, rotation_error
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
c=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text())
R=np.array(c['board_to_cad_rotation']);t=np.array(c['board_to_cad_translation_m']);tcp=c['tcp_in_wrist_frame_m']
lines=(OUT/'jetson_snapshot/arm_run_1793671770732_5069.txt').read_text().splitlines()
w=next(json.loads(l.split('=',1)[1]) for l in lines if l.startswith('board waypoint=') and json.loads(l.split('=',1)[1])['stage']=='contact')
q0=np.array([w['entry_feedback'][n]['pos'] for n in m.names]);pose=tcp_pose(m,q0,tcp);start=(pose[:3,3]-t)@R*1000
lim=np.array(c['joint_limits_encoder_rad']);results=[]
for lock in ['shoulder_side','shoulder_rotate','wrist_side']:
 j=m.names.index(lock);free=np.array([k for k in range(m.n) if k!=j]);q=q0.copy();path=[q.copy()];pe=[];re=[]
 for z in np.linspace(start[2],50.2,24)[1:]:
  target=R@(np.array([start[0],start[1],z])/1000)+t
  def unpack(x):
   a=q0.copy();a[free]=x;return a
  def residual(x):
   p=tcp_pose(m,unpack(x),tcp)
   return np.r_[target-p[:3,3],.15*rotation_error(pose[:3,:3],p[:3,:3])]
  fit=least_squares(residual,q[free],bounds=(lim[free,0],lim[free,1]),xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=100)
  q=unpack(fit.x);e=residual(fit.x);pe.append(np.linalg.norm(e[:3])*1000);re.append(np.degrees(np.linalg.norm(e[3:])/.15));path.append(q.copy())
 path=np.array(path)
 results.append({'fixed_joint':lock,'peak_position_error_mm':max(pe),'peak_orientation_error_deg':max(re),'joint_delta_deg':dict(zip(m.names,np.degrees(path[-1]-q0).tolist())),'max_step_deg':float(np.max(np.abs(np.degrees(np.diff(path,axis=0))))),'path_encoder_rad':path.tolist()})
report={'status':'offline diagnostic, not deployed','start_board_mm':start.tolist(),'candidates':results}
(OUT/'redundancy_constraint_evaluation.json').write_text(json.dumps(report,indent=2))
print(json.dumps({**report,'candidates':[{k:v for k,v in r.items() if k!='path_encoder_rad'} for r in results]}))
