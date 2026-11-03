"""Known-CAD hand-eye/zero fitting on original run, V22 entirely held out."""
import json,sys,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'left_arm_general_model/python_deps'));sys.path.insert(0,str(ROOT/'scripts'))
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json');K=np.array(json.loads((OUT/'camera_board_registration.json').read_text())['camera_matrix'])
audits=json.loads((OUT/'plate_relative_pose_audit.json').read_text());datasets=[]
for audit in audits:
 tracks=json.loads((OUT/('independent_plate_tracks_'+audit['session']+'.json')).read_text());idx={r['index']:r for r in tracks['frames']}
 obj=np.array(tracks['object_points_plane_m']);obj[:,2]=tracks['plane_station_mm']/1000
 datasets.append({'q':np.array([r['q'] for r in audit['frames']]),'pixels':np.array([idx[r['frame']]['points'] for r in audit['frames']]),'obj':obj})
initial=json.loads((OUT/'existing_plate_pose.json').read_text())['candidates'][0];R0=np.array(initial['camera_rotation']);t0=np.array(initial['camera_translation_m']);reports=[]
for active in [[],[3],[3,4],[1,2,3,4,5]]:
 def predict(x,d):
  offsets=np.zeros(7)
  if active:offsets[active]=x[6:]
  angles=m.signs[None,:]*(d['q']-m.reference-offsets);T=np.broadcast_to(np.eye(4),(len(angles),4,4)).copy()
  for j,parent in enumerate(m.parents[:6]):
   z=np.broadcast_to(np.eye(4),(len(angles),4,4)).copy();co=np.cos(angles[:,j]);si=np.sin(angles[:,j]);z[:,0,0]=co;z[:,0,1]=-si;z[:,1,0]=si;z[:,1,1]=co;T=T@parent@z
  p=np.einsum('nij,kj->nki',T[:,:3,:3],d['obj'])+T[:,:3,3][:,None,:];R=Rotation.from_rotvec(x[:3]).as_matrix()@R0;c=p@R.T+t0+x[3:6];uv=c@K.T
  return uv[:,:,:2]/uv[:,:,2:]
 fit=least_squares(lambda x:(predict(x,datasets[0])-datasets[0]['pixels']).ravel(),np.zeros(6+len(active)),bounds=(np.r_[np.full(3,-.5),np.full(3,-.15),np.full(len(active),-np.radians(10))],np.r_[np.full(3,.5),np.full(3,.15),np.full(len(active),np.radians(10))]),loss='soft_l1',f_scale=.35,x_scale='jac',max_nfev=1500,ftol=1e-11,xtol=1e-11,gtol=1e-9)
 stats=[]
 for d in datasets:
  uv=predict(fit.x,d);e=np.linalg.norm(uv-d['pixels'],axis=-1);motion=(uv-uv[0])-(d['pixels']-d['pixels'][0]);me=np.linalg.norm(motion,axis=-1)
  stats.append({'median_px':float(np.median(e)),'p95_px':float(np.quantile(e,.95)),'motion_median_px':float(np.median(me)),'motion_p95_px':float(np.quantile(me,.95))})
 report={'active_zero_joints':[m.names[j] for j in active],'zero_deltas_deg':np.degrees(fit.x[6:]).tolist(),'training_original':stats[0],'independent_v22':stats[1],'camera_rotation':(Rotation.from_rotvec(fit.x[:3]).as_matrix()@R0).tolist(),'camera_translation_m':(t0+fit.x[3:6]).tolist(),'jacobian_singular_values':np.linalg.svd(fit.jac,compute_uv=False).tolist(),'status':'offline, not deployed','nfev':fit.nfev}
 reports.append(report);print(json.dumps({k:v for k,v in report.items() if k not in ['camera_rotation','camera_translation_m','jacobian_singular_values']}),flush=True)
(OUT/'known_plate_calibration_candidates.json').write_text(json.dumps(reports,indent=2))
