"""Compare measured relative plate poses; fixed camera transform cancels out."""
import cv2,json,sys,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel,invert_transform
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
K=np.array(json.loads((OUT/'camera_board_registration.json').read_text())['camera_matrix'])
reports=[]
for session,log in [('original','arm_run_1793671770732_5069.txt'),('v22','arm_run_1793679218917_10319.txt')]:
 d=json.loads((OUT/('independent_plate_tracks_'+session+'.json')).read_text());obj=np.array(d['object_points_plane_m'])
 a=[json.loads(l.split('=',1)[1]) for l in (OUT/'jetson_snapshot'/log).read_text().splitlines() if l.startswith('board tracking audit=')]
 ts=np.array([(v['feedback_read_started_wall_s']+v['feedback_read_finished_wall_s'])/2 for v in a]);qs=np.array([[v['feedback'][n]['pos'] for n in m.names] for v in a]);rows=[]
 for f in d['frames']:
  if not ts[0]<=f['rgb_stamp']<=ts[-1]:continue
  pix=np.array(f['points']);ok,rv,tv=cv2.solvePnP(obj,pix,K,None,flags=cv2.SOLVEPNP_ITERATIVE)
  if not ok:continue
  uv=cv2.projectPoints(obj,rv,tv,K,None)[0][:,0];err=np.linalg.norm(uv-pix,axis=1)
  observed=np.eye(4);observed[:3,:3]=cv2.Rodrigues(rv)[0];observed[:3,3]=tv.ravel()
  q=np.array([np.interp(f['rgb_stamp'],ts,qs[:,j]) for j in range(m.n)])
  nominal=m.fk(m.encoder_to_model(q))[5];nominal[:3,3]+=nominal[:3,:3]@np.array([0,0,d['plane_station_mm']/1000])
  rows.append({'frame':f['index'],'q':q.tolist(),'observed_plane_in_camera':observed.tolist(),'nominal_plane_in_CAD':nominal.tolist(),'reprojection_p95_px':float(np.quantile(err,.95))})
 first,last=rows[0],rows[-1];O=invert_transform(np.array(first['observed_plane_in_camera']))@np.array(last['observed_plane_in_camera']);N=invert_transform(np.array(first['nominal_plane_in_CAD']))@np.array(last['nominal_plane_in_CAD']);E=O[:3,:3]@N[:3,:3].T
 r={'session':session,'frames':rows,'first_to_last_observed_delta_in_start_plate_mm':(O[:3,3]*1000).tolist(),'first_to_last_model_delta_in_start_plate_mm':(N[:3,3]*1000).tolist(),'relative_rotation_disagreement_deg':float(np.degrees(np.arccos(np.clip((np.trace(E)-1)/2,-1,1)))),'maximum_reprojection_p95_px':max(v['reprojection_p95_px'] for v in rows),'method':'camera extrinsic cancels; assumes correct six-hole CAD correspondence and camera intrinsics'}
 reports.append(r);print(json.dumps({k:v for k,v in r.items() if k!='frames'}))
(OUT/'plate_relative_pose_audit.json').write_text(json.dumps(reports,indent=2))
