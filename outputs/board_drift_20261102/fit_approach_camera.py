"""Fit camera geometry on approach only; reserve descent as independent holdout.

Does not write a production model, coordinate offset, or joint replay.
"""
import sys,json,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'left_arm_general_model/python_deps'))
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
cfg=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());cam=json.loads((OUT.parent/'left_arm_general_model/v18_camera_board_registration.json').read_text())
K=np.array(cam['camera_matrix']);rc0=np.array(cam['board_to_camera_rotation'])@np.array(cfg['board_to_cad_rotation']).T
tc0=np.array(cam['board_to_camera_translation_mm'])/1000-rc0@np.array(cfg['board_to_cad_translation_m'])
tracks=json.loads((OUT/'approach_feature_tracks.json').read_text())
text=(OUT/'jetson_snapshot/arm_run_1793671770732_5069.txt').read_text()
audits=[json.loads(l.split('=',1)[1]) for l in text.splitlines() if l.startswith('board tracking audit= ')]
ts=np.array([(a['feedback_read_started_wall_s']+a['feedback_read_finished_wall_s'])/2 for a in audits]);qs=np.array([[a['feedback'][n]['pos'] for n in m.names] for a in audits])
records=[f for f in tracks['frames'] if ts[0]<=f['rgb_stamp']<=ts[-1]]
frames=np.array([f['index'] for f in records]);q=np.array([[np.interp(f['rgb_stamp'],ts,qs[:,j]) for j in range(m.n)] for f in records])
fk=np.array([m.fk(m.encoder_to_model(qq))[-1] for qq in q]);wr=fk[:,:3,:3];wt=fk[:,:3,3]
pixels=np.array([f['points'] for f in records]);valid=np.array([f['valid'] for f in records],bool);n=pixels.shape[1]
train=frames<tracks['contact_start'];train_mask=valid&train[:,None];test_mask=valid&~train[:,None]
verts=np.array(json.loads((ROOT/'outputs/left_arm_step_analysis/revised_2/step_vertex_preview.json').read_text(encoding='utf8'))[7]['vertices_mm'])/1000
local=(verts-m.home[-1,:3,3])@m.home[-1,:3,:3];lower_point=local.min(0)-.005;upper_point=local.max(0)+.005
# Start at the middle of the CAD body, not the distal TCP plane.
mid=(lower_point+upper_point)/2
z=(rc0@(wt[-1]+wr[-1]@mid)+tc0)[2]
rays=np.column_stack([pixels[-1],np.ones(n)])@np.linalg.inv(K).T
points=((rays*z-tc0)@rc0-wt[-1])@wr[-1]
def predict(x):
 rc=Rotation.from_rotvec(x[:3]).as_matrix()@rc0;tc=tc0+x[3:6];p=x[6:].reshape(n,3)
 world=np.einsum('nij,kj->nki',wr,p)+wt[:,None,:];camera=world@rc.T+tc;uv=camera@K.T
 return uv[:,:,:2]/uv[:,:,2:],camera[:,:,2]
def residual(x):return (predict(x)[0]-pixels)[train_mask].ravel()
lower=np.r_[np.full(3,-.6),np.full(3,-.15),np.tile(lower_point,n)]
upper=np.r_[np.full(3,.6),np.full(3,.15),np.tile(upper_point,n)]
x=np.clip(np.r_[np.zeros(6),points.ravel()],lower+1e-6,upper-1e-6)
print(json.dumps({'frames':len(records),'training_frames':int(train.sum()),'holdout_frames':int((~train).sum()),'training_feature_observations':int(train_mask.sum()),'holdout_feature_observations':int(test_mask.sum()),'joint_span_degrees':dict(zip(m.names,np.degrees(np.ptp(q[train],axis=0)).tolist()))}),flush=True)
fit=least_squares(residual,x,bounds=(lower,upper),loss='soft_l1',f_scale=.5,x_scale='jac',max_nfev=800,ftol=1e-10,xtol=1e-10,gtol=1e-9)
uv,depth=predict(fit.x);err=np.linalg.norm(uv-pixels,axis=-1)
def stats(mask):
 e=err[mask]
 return {'median_px':float(np.median(e)),'p95_px':float(np.quantile(e,.95)),'max_px':float(e.max()),'rmse_px':float(np.sqrt(np.mean(e**2)))}
rc=Rotation.from_rotvec(fit.x[:3]).as_matrix()@rc0;tc=tc0+fit.x[3:6]
sv=np.linalg.svd(fit.jac,compute_uv=False);p=fit.x[6:].reshape(n,3);distance,_=cKDTree(local).query(p)
restored=json.loads((OUT/'restored_board_camera_registration.json').read_text());rb=rc.T@np.array(restored['board_to_camera_rotation']);tb=rc.T@(np.array(restored['board_to_camera_translation_mm'])/1000-tc)
axis_local=m.home[-1,:3,:3].T@np.array(cfg['tcp_datum']['axis_in_cad'])
axis_board=rb.T@fk[-1,:3,:3]@axis_local
inclination=float(np.degrees(np.arccos(np.clip(abs(axis_board[2])/np.linalg.norm(axis_board),-1,1))))
report={'status':'offline candidate; NOT deployed','training_motion':'approach before contact; descent excluded from fitting','success':bool(fit.success),'nfev':fit.nfev,'cost':float(fit.cost),'training':stats(train_mask),'held_out_descent':stats(test_mask),'camera_rotation':rc.tolist(),'camera_translation_m':tc.tolist(),'camera_delta_rotation_vector_deg':np.degrees(fit.x[:3]).tolist(),'camera_delta_translation_m':fit.x[3:6].tolist(),'feature_local_m':p.tolist(),'feature_distance_to_nearest_CAD_vertex_mm':(distance*1000).tolist(),'point_bounds_m':[lower_point.tolist(),upper_point.tolist()],'minimum_bound_margin_m':float(np.min(np.minimum(p-lower_point,upper_point-p))),'minimum_camera_depth_m':float(depth[valid].min()),'jacobian_singular_values':sv.tolist(),'board_rotation_candidate':rb.tolist(),'board_translation_candidate_m':tb.tolist(),'candidate_gripper_axis_inclination_deg':inclination,'independently_validated':False}
(OUT/'approach_camera_fit.json').write_text(json.dumps(report,indent=2));np.savez(OUT/'approach_camera_fit.npz',frames=frames,pixels=pixels,predicted=uv,valid=valid,train=train,q=q,x=fit.x)
print(json.dumps(report))
