"""Compare sign hypotheses against recorded images; never edit robot models."""
import sys,json,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];OLD=OUT.parent/'left_arm_general_model'
sys.path.insert(0,str(OLD/'python_deps'))
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json');cfg=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());cam=json.loads((OLD/'v18_camera_board_registration.json').read_text());K=np.array(cam['camera_matrix']);ki=np.linalg.inv(K)
rc0=np.array(cam['board_to_camera_rotation'])@np.array(cfg['board_to_cad_rotation']).T;tc0=np.array(cam['board_to_camera_translation_mm'])/1000-rc0@np.array(cfg['board_to_cad_translation_m'])
d=np.load(OUT/'approach_camera_fit.npz');q=d['q'];pix=d['pixels'];valid=d['valid'];train=d['train'];n=pix.shape[1];mask=valid&train[:,None];test=valid&~train[:,None]
pbound=np.array(json.loads((OUT/'approach_camera_fit.json').read_text())['point_bounds_m']);lower=np.r_[np.full(3,-.6),np.full(3,-.15),np.tile(pbound[0],n)];upper=np.r_[np.full(3,.6),np.full(3,.15),np.tile(pbound[1],n)]
def batch_fk(enc,signs):
 angles=signs[None,:]*(enc-m.reference);T=np.broadcast_to(np.eye(4),(len(enc),4,4)).copy()
 for j,parent in enumerate(m.parents):
  z=np.broadcast_to(np.eye(4),(len(enc),4,4)).copy();c=np.cos(angles[:,j]);s=np.sin(angles[:,j]);z[:,0,0]=c;z[:,0,1]=-s;z[:,1,0]=s;z[:,1,1]=c
  T=T@parent@z
 return T
# Independent old descent, never used for camera fitting.
sel=json.loads((OLD/'rgbd_v21_restored/selection.json').read_text());meta={r['index']:r for r in (json.loads(l) for l in (OLD/'rgbd_v21_restored/frames.jsonl').read_text().splitlines())}
text=(ROOT/'scripts/data/left_arm_v2_8_full_runs/arm_run_1793164847303_93849.txt').read_text();aud=[json.loads(l.split('=',1)[1]) for l in text.splitlines() if l.startswith('board tracking audit= ')]
ts=np.array([(r['feedback_read_started_wall_s']+r['feedback_read_finished_wall_s'])/2 for r in aud]);qs=np.array([[r['feedback'][k]['pos'] for k in m.names] for r in aud]);end=max(i for i in range(sel['contact_min'],sel['contact_max']+1) if meta[i]['rgb_stamp']<=ts[-1]);idx=[sel['contact_min'],end]
q_old=np.array([[np.interp(meta[i]['rgb_stamp'],ts,qs[:,j]) for j in range(m.n)] for i in idx]);tr=np.load(OLD/'v21_rgb_descent_tracks.npz');ids=np.where(tr['alive'])[0];a=tr['points'][0,ids];b=tr['points'][end-sel['contact_min'],ids];aa=np.column_stack([a,np.ones(len(a))]);bb=np.column_stack([b,np.ones(len(b))])
def skew(v):
 x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
def epi(rc,tc,signs):
 C=np.eye(4);C[:3,:3]=rc;C[:3,3]=tc;fk=batch_fk(q_old,signs);rel=C@fk[1]@np.linalg.inv(C@fk[0]);F=ki.T@skew(rel[:3,3])@rel[:3,:3]@ki;line=aa@F.T;err=np.abs(np.sum(bb*line,axis=1))/np.linalg.norm(line[:,:2],axis=1)
 return float(np.median(err))
reports=[]
for change in (None,'shoulder_side','shoulder_rotate','arm_roll','wrist_side','wrist'):
 signs=m.signs.copy()
 if change:signs[m.names.index(change)]*=-1
 fk=batch_fk(q,signs)
 def predict(x):
  rc=Rotation.from_rotvec(x[:3]).as_matrix()@rc0;tc=tc0+x[3:6];p=x[6:].reshape(n,3);world=np.einsum('nij,kj->nki',fk[:,:3,:3],p)+fk[:,:3,3][:,None,:];camera=world@rc.T+tc;uv=camera@K.T
  return uv[:,:,:2]/uv[:,:,2:]
 def residual(x):return (predict(x)-pix)[mask].ravel()
 # Same initialization and bounds for each discrete hypothesis.
 x=np.array(d['x']);fit=least_squares(residual,x,bounds=(lower,upper),loss='soft_l1',f_scale=.5,x_scale='jac',max_nfev=600,ftol=1e-9)
 uv=predict(fit.x);e=np.linalg.norm(uv-pix,axis=-1);rc=Rotation.from_rotvec(fit.x[:3]).as_matrix()@rc0;tc=tc0+fit.x[3:6]
 def stat(s):return {'median_px':float(np.median(e[s])),'p95_px':float(np.quantile(e[s],.95)),'rmse_px':float(np.sqrt(np.mean(e[s]**2)))}
 row={'sign_flip_hypothesis':change,'success':bool(fit.success),'nfev':fit.nfev,'training':stat(mask),'heldout_current_descent':stat(test),'heldout_Oct27_descent_epipolar_median_px':epi(rc,tc,signs),'camera_rotation':rc.tolist(),'camera_translation_m':tc.tolist(),'feature_local_m':fit.x[6:].reshape(n,3).tolist(),'minimum_bound_margin':float(np.min(np.minimum(fit.x-lower,upper-fit.x)))}
 reports.append(row);print(json.dumps({k:v for k,v in row.items() if k not in ('camera_rotation','camera_translation_m','feature_local_m')}),flush=True)
(OUT/'direction_hypotheses.json').write_text(json.dumps({'status':'offline hypotheses, no production change','candidates':reports},indent=2))
