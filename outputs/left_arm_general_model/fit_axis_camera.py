"""Offline image/encoder consistency fit; never writes a robot configuration."""
import sys,json
from pathlib import Path
import numpy as np
out=Path(__file__).parent;sys.path.insert(0,str(out/'python_deps'))
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
root=out.parents[1];sys.path.insert(0,str(root/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json');cfg=json.loads((root/'scripts/data/left_arm_board_grasp.json').read_text());cam=json.loads((out/'v18_camera_board_registration.json').read_text());K=np.array(cam['camera_matrix']);rc0=np.array(cam['board_to_camera_rotation'])@np.array(cfg['board_to_cad_rotation']).T;tc0=np.array(cam['board_to_camera_translation_mm'])/1000-rc0@np.array(cfg['board_to_cad_translation_m'])
folder=out/'rgbd_axis_audit';rows=[json.loads(l) for l in (folder/'frames.jsonl').read_text().splitlines()];commands=[json.loads(l) for l in (folder/'axis_commands.jsonl').read_text().splitlines() if json.loads(l)['execute']];tracks=np.load(out/'v19_axis_rgb_tracks.npz');ids=np.where(tracks['alive'])[0];image_points=tracks['points'][:,ids];lo=int(tracks['frames'][0]);observations=[]
for cmd in commands:
 text=(folder/Path(cmd['run']['full_log_path']).name).read_text();audit=[json.loads(l.split('= ',1)[1]) for l in text.splitlines() if l.startswith('board tracking audit= ')];ts=np.array([(a['feedback_read_started_wall_s']+a['feedback_read_finished_wall_s'])/2 for a in audit]);q=np.array([[a['feedback'][n]['pos'] for n in m.names] for a in audit]);mask=[r for r in rows if r.get('full_log_path')==cmd['run']['full_log_path'] and ts[0]<=r['rgb_stamp']<=ts[-1]]
 for r in mask:
  if not lo<=r['index']<lo+len(image_points):continue
  qq=np.array([np.interp(r['rgb_stamp'],ts,q[:,j]) for j in range(m.n)]);fk=m.fk(m.encoder_to_model(qq))[-1];observations.append((cmd['joint'],r['index'],image_points[r['index']-lo],fk))
assert len(observations)>30
wr=np.array([v[3][:3,:3] for v in observations]);wt=np.array([v[3][:3,3] for v in observations]);pixels=np.array([v[2] for v in observations]);labels=np.array([v[0] for v in observations]);holdout=sys.argv[1] if len(sys.argv)>1 else 'none';train=labels!=holdout
z=(rc0@(wt[0]+wr[0]@np.array(cfg['tcp_in_wrist_frame_m']))+tc0)[2];rays=np.column_stack([pixels[0],np.ones(len(ids))])@np.linalg.inv(K).T;points=((rays*z-tc0)@rc0-wt[0])@wr[0]
def predict(x):
 rc=Rotation.from_rotvec(x[:3]).as_matrix()@rc0;tc=tc0+x[3:6];p=x[6:].reshape(-1,3);world=np.einsum('nij,kj->nki',wr,p)+wt[:,None,:];camera=world@rc.T+tc;uv=camera@K.T;return uv[:,:,:2]/uv[:,:,2:]
def residual(x):return (predict(x)[train]-pixels[train]).ravel()
x=np.r_[np.zeros(6),points.ravel()];lower=np.r_[np.full(3,-.6),np.full(3,-.15),np.tile([-.1,.03,-.06],len(ids))];upper=np.r_[np.full(3,.6),np.full(3,.15),np.tile([.1,.26,.10],len(ids))];x=np.clip(x,lower+1e-6,upper-1e-6)
print(json.dumps({'observations':len(observations),'features':len(ids),'holdout':holdout}),flush=True)
fit=least_squares(residual,x,bounds=(lower,upper),loss='soft_l1',f_scale=.5,x_scale='jac',max_nfev=500,ftol=1e-9);uv=predict(fit.x);err=np.linalg.norm(uv-pixels,axis=-1);s=np.linalg.svd(fit.jac,compute_uv=False);rc=Rotation.from_rotvec(fit.x[:3]).as_matrix()@rc0;tc=tc0+fit.x[3:6];rb=rc.T@np.array(cam['board_to_camera_rotation']);tb=rc.T@(np.array(cam['board_to_camera_translation_mm'])/1000-tc)
verts=np.array(json.loads((root/'outputs/left_arm_step_analysis/revised_2/step_vertex_preview.json').read_text())[7]['vertices_mm'])/1000;local=(verts-m.home[-1,:3,3])@m.home[-1,:3,:3];dist,_=cKDTree(local).query(fit.x[6:].reshape(-1,3))
result={'status':'offline hypothesis; NOT deployed','holdout_joint':holdout,'success':bool(fit.success),'nfev':fit.nfev,'observations':len(observations),'feature_ids':ids.tolist(),'pixel_errors':{n:{'median':float(np.median(err[labels==n])),'p95':float(np.quantile(err[labels==n],.95)),'max':float(err[labels==n].max())} for n in set(labels)},'camera_delta_rotation_vector_deg':np.degrees(fit.x[:3]).tolist(),'camera_delta_translation_m':fit.x[3:6].tolist(),'camera_rotation':rc.tolist(),'camera_translation_m':tc.tolist(),'board_rotation_candidate':rb.tolist(),'board_translation_candidate_m':tb.tolist(),'feature_local_m':fit.x[6:].reshape(-1,3).tolist(),'feature_distance_to_CAD_vertices_mm':(dist*1000).tolist(),'jacobian_singular_values':s.tolist()}
(out/('v19_axis_camera_fit_'+holdout+'.json')).write_text(json.dumps(result,indent=2));np.savez(out/('v19_axis_camera_fit_'+holdout+'.npz'),pixels=pixels,predicted=uv,frames=[v[1] for v in observations],labels=labels,x=fit.x);print(json.dumps(result))
