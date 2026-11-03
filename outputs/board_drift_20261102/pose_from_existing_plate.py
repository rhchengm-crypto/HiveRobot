"""Independent six-hole plate pose from approved CAD; no robot modifications."""
import cv2,json,numpy as np,sys
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
im=cv2.imdecode(np.fromfile(OUT/'raw_end_plate.jpg',dtype=np.uint8),1);gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
seeds=np.array([[87,313],[104,329],[97,352],[71,363],[54,347],[63,323]],float)
pixels=[];contours_used=[]
for seed in seeds:
 x,y=np.rint(seed).astype(int);x0=x-7;y0=y-8;patch=gray[y0:y+9,x0:x+8]
 _,binary=cv2.threshold(patch,0,255,cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)
 contours,_=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
 options=[]
 for cnt in contours:
  area=cv2.contourArea(cnt)
  if not 8<area<160:continue
  mom=cv2.moments(cnt)
  if mom['m00']<=0:continue
  p=np.array([mom['m10']/mom['m00']+x0,mom['m01']/mom['m00']+y0]);options.append((np.linalg.norm(p-seed),p,cnt+np.array([[[x0,y0]]])))
 assert options,'No hole contour at '+str(seed)
 _,p,cnt=min(options,key=lambda r:r[0]);pixels.append(p);contours_used.append(cnt)
pixels=np.array(pixels);center=pixels.mean(0);pixels=pixels[np.argsort(np.arctan2(*(pixels-center)[:,::-1].T))]
planes=json.loads((OUT/'wrist_side_plate_candidates.json').read_text());plate=next(r for r in planes if r['part']==9 and abs(r['z_mm']-14.37)<.01)
points=np.array([[x,y,0.] for x,y,r in plate['circles'] if abs(r-3)<.001])/1000
center=points[:,:2].mean(0);points=points[np.argsort(np.arctan2(*(points[:,:2]-center)[:,::-1].T))]
assert len(points)==6
data=np.load(OUT/'approach_camera_fit.npz');q=data['q'][-1];assert int(data['frames'][-1])==501
fk=m.fk(m.encoder_to_model(q));cad_plane=fk[5].copy();cad_plane[:3,3]+=cad_plane[:3,:3]@np.array([0,0,plate['z_mm']/1000])
cfg=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());cam=json.loads((OUT.parent/'left_arm_general_model/v18_camera_board_registration.json').read_text());K=np.array(cam['camera_matrix'])
rc0=np.array(cam['board_to_camera_rotation'])@np.array(cfg['board_to_cad_rotation']).T;tc0=np.array(cam['board_to_camera_translation_mm'])/1000-rc0@np.array(cfg['board_to_cad_translation_m'])
candidates=[]
for reverse in (False,True):
 pp=pixels[::-1] if reverse else pixels
 for shift in range(6):
  uv=np.roll(pp,shift,axis=0).copy()
  ok,rvs,tvs,errs=cv2.solvePnPGeneric(points,uv,K,None,flags=cv2.SOLVEPNP_IPPE)
  if not ok:continue
  for rv,tv in zip(rvs,tvs):
   T=np.eye(4);T[:3,:3]=cv2.Rodrigues(rv)[0];T[:3,3]=tv.ravel();C=T@np.linalg.inv(cad_plane)
   repro=cv2.projectPoints(points,rv,tv,K,None)[0][:,0];e=np.linalg.norm(repro-uv,axis=1)
   angle=float(np.degrees(np.arccos(np.clip((np.trace(C[:3,:3]@rc0.T)-1)/2,-1,1))))
   distance=float(np.linalg.norm(C[:3,3]-tc0))
   row={'reverse':reverse,'shift':shift,'median_reprojection_px':float(np.median(e)),'max_reprojection_px':float(e.max()),'rotation_difference_from_nominal_deg':angle,'camera_translation_difference_m':distance,'camera_rotation':C[:3,:3].tolist(),'camera_translation_m':C[:3,3].tolist(),'score':angle+distance*100,'ordered_pixels':uv.tolist()}
   candidates.append(row)
candidates.sort(key=lambda r:r['score'])
result={'status':'single-frame CAD landmark hypothesis; NOT deployed','cad_part_index':9,'cad_plane_station_mm':plate['z_mm'],'hole_centers_px':pixels.tolist(),'object_points_plane_m':points.tolist(),'source_frame':501,'encoder_rad':dict(zip(m.names,q.tolist())),'candidates':candidates[:4],'physical_correspondence_independently_confirmed':False}
(OUT/'existing_plate_pose.json').write_text(json.dumps(result,indent=2))
annot=im.copy()
for i,p in enumerate(pixels):
 cv2.circle(annot,tuple(np.rint(p).astype(int)),6,(0,0,255),1);cv2.putText(annot,str(i),tuple(np.rint(p+[3,-3]).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.3,(0,0,255),1)
cv2.imencode('.jpg',annot)[1].tofile(OUT/'plate_landmarks_detected.jpg')
print(json.dumps({**{k:v for k,v in result.items() if k!='candidates'},'best_candidates':[{k:v for k,v in r.items() if k not in ('camera_rotation','camera_translation_m','ordered_pixels')} for r in candidates[:4]]}))
