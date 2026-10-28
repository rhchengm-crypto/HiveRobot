import sys,json,numpy as np,cv2
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent/'python_deps'))
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
out=Path(__file__).parent;root=out.parents[1];sys.path.insert(0,str(root/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json');cfg=json.loads((root/'scripts/data/left_arm_board_grasp.json').read_text());cam=json.loads((out/'v18_camera_board_registration.json').read_text());K=np.array(cam['camera_matrix']);rc0=np.array(cam['board_to_camera_rotation'])@np.array(cfg['board_to_cad_rotation']).T;tc0=np.array(cam['board_to_camera_translation_mm'])/1000-rc0@np.array(cfg['board_to_cad_translation_m']);back=np.load(out/'v18_rgb_backward_tracks.npz');forward=np.load(out/'v18_rgb_descent_tracks.npz');ids=np.where(back['alive'])[0];frames=np.arange(480,614,2);pixels=np.concatenate([back['points'][:-1],forward['points']],axis=0)[frames-480][:,ids]
rows=[json.loads(x) for x in (out/'rgbd_descent/frames.jsonl').read_text().splitlines()];times=[];qs=[];last=None
for row in rows:
 a=row.get('tracking') or {};p=a.get('feedback')
 if not p:continue
 q=[p[n]['pos'] for n in m.names]
 if q==last:continue
 times.append(row['wall_time']-.125);qs.append(q);last=q
qs=np.array(qs);ft=np.array([rows[i]['rgb_stamp'] for i in frames]);qimg=np.stack([np.interp(ft,times,qs[:,j]) for j in range(m.n)],axis=-1);fk=np.array([m.fk(m.encoder_to_model(q))[-1] for q in qimg]);wr=fk[:,:3,:3];wt=fk[:,:3,3];z=(rc0@(wt[0]+wr[0]@np.array(cfg['tcp_in_wrist_frame_m']))+tc0)[2];rays=np.column_stack([pixels[0],np.ones(len(ids))])@np.linalg.inv(K).T;cad=(rays*z-tc0)@rc0;points=(cad-wt[0])@wr[0];print('feature initial local m',points.tolist(),flush=True)
def predict(x):
 rc=Rotation.from_rotvec(x[:3]).as_matrix()@rc0;tc=tc0+x[3:6];p=x[6:].reshape(-1,3);world=np.einsum('nij,kj->nki',wr,p)+wt[:,None,:];camera=world@rc.T+tc;uv=camera@K.T;return uv[:,:,:2]/uv[:,:,2:]
def residual(x):return (predict(x)-pixels).ravel()
x=np.r_[np.zeros(6),points.ravel()];fit=least_squares(residual,x,loss='soft_l1',f_scale=1,x_scale='jac',max_nfev=500,diff_step=1e-5,ftol=1e-9);uv=predict(fit.x);err=np.linalg.norm(uv-pixels,axis=-1);s=np.linalg.svd(fit.jac,compute_uv=False);rc=Rotation.from_rotvec(fit.x[:3]).as_matrix()@rc0;newR=rc.T@np.array(cam['board_to_camera_rotation']);down=rc0@(-np.array(cfg['board_to_cad_rotation'])[:,2]);newdown=rc@(-np.array(cfg['board_to_cad_rotation'])[:,2]);print('fit done',flush=True)
result={'purpose':'offline camera/feature calibration hypothesis only; no robot parameter update','features':ids.tolist(),'frames':frames.tolist(),'success':bool(fit.success),'cost':float(fit.cost),'pixel_error_median':float(np.median(err)),'pixel_error_p95':float(np.quantile(err,.95)),'pixel_error_max':float(err.max()),'camera_rotation_change_deg':Rotation.from_rotvec(fit.x[:3]).as_euler('xyz',degrees=True).tolist(),'camera_translation_change_m':fit.x[3:6].tolist(),'fitted_feature_local_m':fit.x[6:].reshape(-1,3).tolist(),'jacobian_singular_values':s.tolist(),'inferred_board_to_cad_rotation_hypothesis':newR.tolist(),'current_model_down_in_camera':down.tolist(),'fitted_model_down_in_camera':newdown.tolist(),'camera_board_true_down':(-np.array(cam['board_to_camera_rotation'])[:,2]).tolist(),'physical_3d_pose_verified':False}
(out/'v18_camera_bundle_hypothesis.json').write_text(json.dumps(result,indent=2));np.savez(out/'v18_camera_bundle_fit.npz',pixels=pixels,predicted=uv,frames=frames,feature_ids=ids,q=qimg);print(json.dumps(result))
