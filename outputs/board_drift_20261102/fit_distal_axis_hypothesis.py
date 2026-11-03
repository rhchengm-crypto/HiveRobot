"""Fit distal axis/zero alternatives on approach, with descent held out.

Offline only. No production writes and no robot commands.
"""
import json,sys
from pathlib import Path
import numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'left_arm_general_model/python_deps'))
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
cam=json.loads((OUT.parent/'left_arm_general_model/v18_camera_board_registration.json').read_text());cfg=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text())
K=np.array(cam['camera_matrix']);rc0=np.array(cam['board_to_camera_rotation'])@np.array(cfg['board_to_cad_rotation']).T;tc0=np.array(cam['board_to_camera_translation_mm'])/1000-rc0@np.array(cfg['board_to_cad_translation_m'])
d=np.load(OUT/'approach_camera_fit_without_feature5.npz');q=d['q'];pixels=d['pixels'];valid=d['valid'];train=d['train'];n=5
pb=np.array(json.loads((OUT/'approach_camera_fit_without_feature5.json').read_text())['point_bounds_m']);mask=valid&train[:,None];test=valid&~train[:,None]
reports=[]
for hypothesis in ['wrist_side_zero','wrist_axis_tilt']:
 count=1 if hypothesis=='wrist_side_zero' else 2
 def frames(parameters):
  a=m.signs[None,:]*(q-m.reference);T=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
  if hypothesis=='wrist_side_zero':a[:,5]-=m.signs[5]*parameters[0]
  for j,parent in enumerate(m.parents):
   parent=parent.copy()
   if j==6 and hypothesis=='wrist_axis_tilt':parent[:3,:3]=parent[:3,:3]@Rotation.from_rotvec([*parameters,0]).as_matrix()
   z=np.broadcast_to(np.eye(4),(len(q),4,4)).copy();c=np.cos(a[:,j]);s=np.sin(a[:,j]);z[:,0,0]=c;z[:,0,1]=-s;z[:,1,0]=s;z[:,1,1]=c
   T=T@parent@z
  return T
 def predict(x):
  fk=frames(x[-count:]);p=x[6:-count].reshape(n,3);rc=Rotation.from_rotvec(x[:3]).as_matrix()@rc0;tc=tc0+x[3:6]
  world=np.einsum('nij,kj->nki',fk[:,:3,:3],p)+fk[:,:3,3][:,None,:];camera=world@rc.T+tc;uv=camera@K.T
  return uv[:,:,:2]/uv[:,:,2:]
 lower=np.r_[np.full(3,-.6),np.full(3,-.15),np.tile(pb[0],n),np.full(count,-np.radians(30))]
 upper=np.r_[np.full(3,.6),np.full(3,.15),np.tile(pb[1],n),np.full(count,np.radians(30))]
 fit=least_squares(lambda x:(predict(x)-pixels)[mask].ravel(),np.r_[d['x'],np.zeros(count)],bounds=(lower,upper),loss='soft_l1',f_scale=.5,x_scale='jac',max_nfev=800,ftol=1e-10,xtol=1e-10,gtol=1e-9)
 err=np.linalg.norm(predict(fit.x)-pixels,axis=-1)
 def stats(b):return {'median_px':float(np.median(err[b])),'p95_px':float(np.quantile(err[b],.95)),'max_px':float(np.max(err[b]))}
 report={'hypothesis':hypothesis,'train':stats(mask),'heldout_descent':stats(test),'parameters_deg':np.degrees(fit.x[-count:]).tolist(),'bound_margin_deg':float(30-np.max(np.abs(np.degrees(fit.x[-count:])))),'status':'not deployed, requires independent validation','x':fit.x.tolist()}
 reports.append(report);print(json.dumps({k:v for k,v in report.items() if k!='x'}),flush=True)
(OUT/'distal_axis_hypotheses.json').write_text(json.dumps(reports,indent=2))
