"""Held-out rigid-feature epipolar test: no fitted landmark depth or TCP needed."""
import sys,json,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];old=OUT.parent/'left_arm_general_model'
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
c=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());cam=json.loads((old/'v18_camera_board_registration.json').read_text());fit=json.loads((old/'v19_axis_camera_fit_none.json').read_text())
K=np.array(cam['camera_matrix']);ki=np.linalg.inv(K)
rc=np.array(cam['board_to_camera_rotation'])@np.array(c['board_to_cad_rotation']).T
tc=np.array(cam['board_to_camera_translation_mm'])/1000-rc@np.array(c['board_to_cad_translation_m'])
tracks=np.load(old/'v21_rgb_descent_tracks.npz');ids=np.where(tracks['alive'])[0];a=tracks['points'][0,ids];b=tracks['points'][-1,ids]
sel=json.loads((old/'rgbd_v21_restored/selection.json').read_text());frames={r['index']:r for r in (json.loads(l) for l in (old/'rgbd_v21_restored/frames.jsonl').read_text().splitlines())}
text=(ROOT/'scripts/data/left_arm_v2_8_full_runs/arm_run_1793164847303_93849.txt').read_text()
audits=[json.loads(l.split('=',1)[1]) for l in text.splitlines() if l.startswith('board tracking audit= ')]
ts=np.array([(r['feedback_read_started_wall_s']+r['feedback_read_finished_wall_s'])/2 for r in audits]);qs=np.array([[r['feedback'][n]['pos'] for n in m.names] for r in audits])
def pose(i):
 stamp=frames[i]['rgb_stamp'];assert ts[0]<=stamp<=ts[-1], 'Image outside recorded feedback times'
 q=np.array([np.interp(stamp,ts,qs[:,j]) for j in range(m.n)])
 return m.fk(m.encoder_to_model(q))[-1]
end_frame=max(i for i in range(sel['contact_min'],sel['contact_max']+1) if frames[i]['rgb_stamp']<=ts[-1])
b=tracks['points'][end_frame-sel['contact_min'],ids]
T0=pose(sel['contact_min']);T1=pose(end_frame)
def skew(v):
 x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
def errors(rot,trans):
 C=np.eye(4);C[:3,:3]=rot;C[:3,3]=trans
 A=C@T0;B=C@T1;rel=B@np.linalg.inv(A)
 F=ki.T@skew(rel[:3,3])@rel[:3,:3]@ki
 aa=np.column_stack([a,np.ones(len(a))]);bb=np.column_stack([b,np.ones(len(b))])
 lines=aa@F.T;res=np.abs(np.sum(bb*lines,axis=1))/np.linalg.norm(lines[:,:2],axis=1)
 # Check the convention against synthetic correspondences from this same transform.
 local=np.array([[0.,.1,.01,1],[.02,.15,0,1],[-.02,.2,.02,1]])
 xx=(A@local.T).T[:,:3]@K.T;yy=(B@local.T).T[:,:3]@K.T;xx/=xx[:,2,None];yy/=yy[:,2,None]
 assert np.max(np.abs(np.sum(yy*(xx@F.T),axis=1)))<1e-8
 return {'median_epipolar_distance_px':float(np.median(res)),'max_px':float(res.max()),'per_feature_px':res.tolist()}
report={'status':'offline held-out motion validation; no calibration deployed','feature_count':len(ids),'feature_ids':ids.tolist(),'nominal_camera_from_board':errors(rc,tc),'axis_fit_camera_candidate':errors(np.array(fit['camera_rotation']),np.array(fit['camera_translation_m'])),'warning':'Rigid-feature and fixed-camera assumptions apply; failure identifies inconsistency, not a unique bad joint or parameter.'}
(OUT/'camera_hypothesis_crosscheck.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
