"""Joint-zero identification with measured depth, using approach-only training.

Descent data is held out. No production model, motor state or target is changed.
"""
import sys,json,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];OLD=OUT.parent/'left_arm_general_model'
sys.path.insert(0,str(OLD/'python_deps'))
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json');cfg=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());cam=json.loads((OLD/'v18_camera_board_registration.json').read_text());K=np.array(cam['camera_matrix'])
rc0=np.array(cam['board_to_camera_rotation'])@np.array(cfg['board_to_cad_rotation']).T;tc0=np.array(cam['board_to_camera_translation_mm'])/1000-rc0@np.array(cfg['board_to_cad_translation_m'])
d=np.load(OUT/'approach_camera_fit.npz');q=d['q'];pix=d['pixels'];valid=d['valid'];train=d['train'];frames=d['frames'];n=pix.shape[1]
depth={r['index']:r for r in json.loads((OUT/'rgbd_feature_motion.json').read_text())['frames']}
zobs=np.full(valid.shape,np.nan)
for i,frame in enumerate(frames):
 for j,o in enumerate(depth[int(frame)]['observations']):
  if o is not None:zobs[i,j]=o['depth_mm']
depth_valid=np.isfinite(zobs)&valid;ztrain=depth_valid&train[:,None];ztest=depth_valid&~train[:,None]
pbound=np.array(json.loads((OUT/'approach_camera_fit.json').read_text())['point_bounds_m']);np_=6+3*n;indices=np.arange(1,6)
def fk_batch(offset):
 angles=m.signs[None,:]*(q-m.reference-offset);T=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
 for j,parent in enumerate(m.parents):
  z=np.broadcast_to(np.eye(4),(len(q),4,4)).copy();c=np.cos(angles[:,j]);s=np.sin(angles[:,j]);z[:,0,0]=c;z[:,0,1]=-s;z[:,1,0]=s;z[:,1,1]=c
  T=T@parent@z
 return T
def predict(x):
 rc=Rotation.from_rotvec(x[:3]).as_matrix()@rc0;tc=tc0+x[3:6];p=x[6:np_].reshape(n,3);offset=np.zeros(7);offset[indices]=x[np_:]
 fk=fk_batch(offset);world=np.einsum('nij,kj->nki',fk[:,:3,:3],p)+fk[:,:3,3][:,None,:];camera=world@rc.T+tc;uv=camera@K.T
 return uv[:,:,:2]/uv[:,:,2:],camera,fk
mask=valid&train[:,None];test=valid&~train[:,None]
def residual(x):
 uv,xyz,_=predict(x)
 return np.r_[(uv-pix)[mask].ravel()/.7,(xyz[:,:,2]*1000-zobs)[ztrain]/2.]
lower=np.r_[np.full(3,-.6),np.full(3,-.15),np.tile(pbound[0],n),np.full(5,-np.radians(30))]
upper=np.r_[np.full(3,.6),np.full(3,.15),np.tile(pbound[1],n),np.full(5,np.radians(30))]
x=np.r_[d['x'],np.zeros(5)]
print(json.dumps({'train_pixels':int(mask.sum()),'train_depths':int(ztrain.sum()),'holdout_pixels':int(test.sum()),'holdout_depths':int(ztest.sum())}),flush=True)
fit=least_squares(residual,x,bounds=(lower,upper),loss='soft_l1',f_scale=1.,x_scale='jac',max_nfev=1500,ftol=1e-10,xtol=1e-10,gtol=1e-9)
uv,xyz,fk=predict(fit.x);pe=np.linalg.norm(uv-pix,axis=-1);ze=xyz[:,:,2]*1000-zobs
def stats(mask,zm):
 e=pe[mask];z=ze[zm]
 return {'pixel_median':float(np.median(e)),'pixel_p95':float(np.quantile(e,.95)),'pixel_rmse':float(np.sqrt(np.mean(e**2))),'depth_abs_median_mm':float(np.median(np.abs(z))),'depth_rmse_mm':float(np.sqrt(np.mean(z**2))),'depth_bias_mm':float(np.mean(z))}
offset=np.zeros(7);offset[indices]=fit.x[np_:];rc=Rotation.from_rotvec(fit.x[:3]).as_matrix()@rc0;tc=tc0+fit.x[3:6];p=fit.x[6:np_].reshape(n,3)
result={'status':'offline RGB-D hypothesis; NOT deployed','success':bool(fit.success),'nfev':fit.nfev,'training':stats(mask,ztrain),'heldout_descent':stats(test,ztest),'reference_encoder_delta_deg':dict(zip(m.names,np.degrees(offset).tolist())),'camera_rotation':rc.tolist(),'camera_translation_m':tc.tolist(),'feature_local_m':p.tolist(),'minimum_zero_bound_margin_deg':float(30-np.max(np.abs(np.degrees(offset)))),'minimum_feature_bound_margin_m':float(np.min(np.minimum(p-pbound[0],pbound[1]-p))),'jacobian_singular_values':np.linalg.svd(fit.jac,compute_uv=False).tolist(),'calibration_validated':False,'sensor_depth_scale_independently_verified':False,'first_and_last_zero_fixed_for_gauge':True}
(OUT/'rgbd_zero_hypothesis.json').write_text(json.dumps(result,indent=2));np.savez(OUT/'rgbd_zero_hypothesis.npz',q=q,frames=frames,predicted=uv,pixels=pix,valid=valid,train=train,zobs=zobs,zpred=xyz[:,:,2]*1000,x=fit.x);print(json.dumps(result))
