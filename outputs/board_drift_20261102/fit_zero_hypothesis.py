"""Offline intermediate-joint zero hypothesis with a descent holdout.

First and last joint zeros are fixed because camera/feature gauges absorb them.
No production file is changed by this experiment.
"""
import sys,json,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'left_arm_general_model/python_deps'))
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
cam=json.loads((OUT.parent/'left_arm_general_model/v18_camera_board_registration.json').read_text());cfg=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());K=np.array(cam['camera_matrix'])
rc0=np.array(cam['board_to_camera_rotation'])@np.array(cfg['board_to_cad_rotation']).T;tc0=np.array(cam['board_to_camera_translation_mm'])/1000-rc0@np.array(cfg['board_to_cad_translation_m'])
data=np.load(OUT/'approach_camera_fit.npz');q=data['q'];pixels=data['pixels'];valid=data['valid'];train=data['train'];frames=data['frames'];n=pixels.shape[1]
previous=json.loads((OUT/'approach_camera_fit.json').read_text());lower_point,upper_point=np.array(previous['point_bounds_m']);indices=np.arange(1,6);np_=6+3*n
def fk_batch(enc,offset):
 angles=m.signs[None,:]*(enc-m.reference[None,:]-offset[None,:]);current=np.broadcast_to(np.eye(4),(len(enc),4,4)).copy()
 for j,t in enumerate(m.parents):
  rz=np.broadcast_to(np.eye(4),(len(enc),4,4)).copy();c=np.cos(angles[:,j]);s=np.sin(angles[:,j]);rz[:,0,0]=c;rz[:,0,1]=-s;rz[:,1,0]=s;rz[:,1,1]=c
  current=current@t@rz
 return current
assert np.max(np.abs(fk_batch(q,np.zeros(7))[0]-m.fk(m.encoder_to_model(q[0]))[-1]))<1e-12
def unpack(x):
 offset=np.zeros(m.n);offset[indices]=x[np_:]
 return Rotation.from_rotvec(x[:3]).as_matrix()@rc0,tc0+x[3:6],x[6:np_].reshape(n,3),offset
def predict(x):
 rc,tc,p,offset=unpack(x);fk=fk_batch(q,offset)
 world=np.einsum('nij,kj->nki',fk[:,:3,:3],p)+fk[:,:3,3,None].transpose(0,2,1)
 camera=world@rc.T+tc;uv=camera@K.T
 return uv[:,:,:2]/uv[:,:,2:],fk
mask=train[:,None]&valid;test=(~train)[:,None]&valid
def residual(x):return (predict(x)[0]-pixels)[mask].ravel()
lower=np.r_[np.full(3,-.6),np.full(3,-.15),np.tile(lower_point,n),np.full(5,-np.radians(10))]
upper=np.r_[np.full(3,.6),np.full(3,.15),np.tile(upper_point,n),np.full(5,np.radians(10))]
x=np.r_[data['x'],np.zeros(5)]
print(json.dumps({'parameters':len(x),'training_observations':int(mask.sum()),'heldout_observations':int(test.sum())}),flush=True)
fit=least_squares(residual,x,bounds=(lower,upper),loss='soft_l1',f_scale=.5,x_scale='jac',max_nfev=1000,ftol=1e-10,xtol=1e-10,gtol=1e-9)
uv,fk=predict(fit.x);err=np.linalg.norm(uv-pixels,axis=-1)
def stats(select):
 e=err[select];return {'median_px':float(np.median(e)),'p95_px':float(np.quantile(e,.95)),'max_px':float(e.max()),'rmse_px':float(np.sqrt(np.mean(e**2)))}
rc,tc,p,offset=unpack(fit.x)
restored=json.loads((OUT/'restored_board_camera_registration.json').read_text());rb=rc.T@np.array(restored['board_to_camera_rotation']);tb=rc.T@(np.array(restored['board_to_camera_translation_mm'])/1000-tc)
axis_local=m.home[-1,:3,:3].T@np.array(cfg['tcp_datum']['axis_in_cad']);axis_board=rb.T@fk[-1,:3,:3]@axis_local
sv=np.linalg.svd(fit.jac,compute_uv=False)
report={'status':'offline hypothesis; NOT deployed','success':bool(fit.success),'nfev':fit.nfev,'training':stats(mask),'held_out_descent':stats(test),'reference_encoder_delta_deg':dict(zip(m.names,np.degrees(offset).tolist())),'camera_rotation':rc.tolist(),'camera_translation_m':tc.tolist(),'feature_local_m':p.tolist(),'board_rotation_candidate':rb.tolist(),'board_translation_candidate_m':tb.tolist(),'candidate_gripper_axis_inclination_deg':float(np.degrees(np.arccos(np.clip(abs(axis_board[2])/np.linalg.norm(axis_board),-1,1)))),'minimum_zero_bound_margin_deg':float(10-np.max(np.abs(np.degrees(offset)))),'minimum_feature_bound_margin_m':float(np.min(np.minimum(p-lower_point,upper_point-p))),'jacobian_singular_values':sv.tolist(),'independently_validated':False,'gauge_constraints':['shoulder_front zero fixed','wrist zero fixed'],'zero_search_bounds_degrees':[-10,10]}
(OUT/'joint_zero_hypothesis.json').write_text(json.dumps(report,indent=2));np.savez(OUT/'joint_zero_hypothesis.npz',frames=frames,pixels=pixels,predicted=uv,valid=valid,train=train,q=q,x=fit.x);print(json.dumps(report))
