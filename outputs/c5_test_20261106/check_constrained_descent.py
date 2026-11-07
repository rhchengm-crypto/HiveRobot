"""Offline geometry diagnostic only. Never imports CAN or executes motion."""
import json,sys
from pathlib import Path
import numpy as np
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose,rotation_error
model=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
cfg=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text())
result=json.loads((BASE/'run_result.json').read_text())
q0=np.array([result['model_hold']['final'][n]['pos'] for n in model.names])
R=np.array(cfg['board_to_cad_rotation']);t=np.array(cfg['board_to_cad_translation_m']);tcp=np.array(cfg['tcp_in_wrist_frame_m'])
limits=np.array(cfg['joint_limits_encoder_rad']);frame=tcp_pose(model,q0,tcp);start=(frame[:3,3]-t)@R*1000
free=[i for i,n in enumerate(model.names) if n not in ['shoulder_side','shoulder_rotate']]

def solve(target,q,rotation_tolerance):
 def residual(a):
  f=tcp_pose(model,a,tcp);p=target-f[:3,3];w=rotation_error(frame[:3,:3],f[:3,:3]);return np.r_[p,.15*w],np.linalg.norm(p),np.linalg.norm(w)
 for it in range(350):
  err,pe,re=residual(q)
  if pe<.0005 and re<rotation_tolerance:break
  jv,jw=model.point_jacobian(model.encoder_to_model(q),model.n-1,tcp)
  jac=np.vstack([jv,.15*jw])*model.signs[None,:];jac=jac[:,free]
  dq=jac.T@np.linalg.solve(jac@jac.T+.002**2*np.eye(6),err)
  dq*=min(1,.03/max(np.max(np.abs(dq)),1e-12));accepted=False
  for scale in [1,.5,.25,.125,.0625]:
   a=q.copy();a[free]=np.clip(q[free]+scale*dq,limits[free,0],limits[free,1])
   if np.linalg.norm(residual(a)[0])<np.linalg.norm(err)-1e-12:q=a;accepted=True;break
  if not accepted:break
 err,pe,re=residual(q)
 return q,dict(success=bool(pe<.0005 and re<rotation_tolerance),position_error_mm=pe*1000,rotation_error_deg=float(np.degrees(re)))

trials=[]
for tolerance_deg in [.3,1,3,5]:
 q=q0.copy();steps=[]
 for depth in range(5,31,5):
  p=start+[0,0,-depth];q,a=solve(R@(p/1000)+t,q,np.radians(tolerance_deg))
  steps.append(dict(requested_descent_mm=depth,**a,joint_delta_deg=dict(zip(model.names,np.degrees(q-q0).tolist()))))
  if not a['success']:break
 trials.append(dict(rotation_tolerance_deg=tolerance_deg,steps=steps))
report={'motion_issued':False,'source':'last C5 model-hold feedback; not a fresh serial read','start_model_TCP_board_mm':start.tolist(),'fixed_joints':['shoulder_side','shoulder_rotate'],'checks':trials,'torso_collision_checked':False,'executable':False,'warning':'No torso mesh or verified clearance. A geometric solution is not authorization or collision clearance.'}
(BASE/'constrained_descent_check.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
