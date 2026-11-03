"""Validate known CAD holes across approach/descent, not just one fitted frame."""
import sys,json,numpy as np,cv2
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];OLD=OUT.parent/'left_arm_general_model'
sys.path.insert(0,str(OLD/'python_deps'))
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json');pose=json.loads((OUT/'existing_plate_pose.json').read_text());tracks=json.loads((OUT/'plate_feature_tracks.json').read_text());K=np.array(json.loads((OLD/'v18_camera_board_registration.json').read_text())['camera_matrix'])
text=(OUT/'jetson_snapshot/arm_run_1793671770732_5069.txt').read_text();aud=[json.loads(l.split('=',1)[1]) for l in text.splitlines() if l.startswith('board tracking audit= ')]
ts=np.array([(a['feedback_read_started_wall_s']+a['feedback_read_finished_wall_s'])/2 for a in aud]);q=np.array([[a['feedback'][n]['pos'] for n in m.names] for a in aud])
rows=[r for r in tracks['frames'] if ts[0]<=r['rgb_stamp']<=ts[-1]];frames=np.array([r['index'] for r in rows]);pix=np.array([r['points'] for r in rows]);valid=np.array([r['valid'] for r in rows]);enc=np.array([[np.interp(r['rgb_stamp'],ts,q[:,j]) for j in range(m.n)] for r in rows]);train=frames<430;mask=valid&train[:,None];hold=valid&~train[:,None]
points=np.array(tracks['object_points_plane_m']);points[:,2]=tracks['plane_station_mm']/1000
initial=pose['candidates'][0];rc0=np.array(initial['camera_rotation']);tc0=np.array(initial['camera_translation_m'])
reports=[]
for attachment in ('after_wrist_side','before_wrist_side'):
 fk=[]
 for qq in enc:
  model_q=m.encoder_to_model(qq)
  if attachment=='before_wrist_side':model_q[5]=0
  fk.append(m.fk(model_q)[5])
 fk=np.array(fk);world=np.einsum('nij,kj->nki',fk[:,:3,:3],points)+fk[:,:3,3][:,None,:]
 def predict(x):
  rc=Rotation.from_rotvec(x[:3]).as_matrix()@rc0;tc=tc0+x[3:];camera=world@rc.T+tc;uv=camera@K.T
  return uv[:,:,:2]/uv[:,:,2:]
 def residual(x):return (predict(x)-pix)[mask].ravel()
 fit=least_squares(residual,np.zeros(6),loss='soft_l1',f_scale=.3,x_scale='jac',max_nfev=300)
 e=np.linalg.norm(predict(fit.x)-pix,axis=-1)
 def stats(s):return {'median_px':float(np.median(e[s])),'p95_px':float(np.quantile(e[s],.95)),'max_px':float(e[s].max())}
 rc=Rotation.from_rotvec(fit.x[:3]).as_matrix()@rc0;tc=tc0+fit.x[3:]
 reports.append({'attachment':attachment,'training':stats(mask),'held_out_descent':stats(hold),'camera_rotation':rc.tolist(),'camera_translation_m':tc.tolist(),'training_feature_observations':int(mask.sum()),'heldout_feature_observations':int(hold.sum())})
print(json.dumps({'status':'offline, no production modifications','candidates':reports}));(OUT/'plate_motion_validation.json').write_text(json.dumps({'status':'offline, no production modifications','candidates':reports},indent=2))
