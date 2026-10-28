import cv2,numpy as np,json
from pathlib import Path
out=Path('outputs/left_arm_general_model');folder=out/'rgbd_descent';sel=json.loads((folder/'selection.json').read_text());lo,hi=sel['contact_min'],sel['contact_max'];files=[folder/('%06d.jpg'%i) for i in range(lo,hi+1)];im=cv2.imread(str(files[0]));dep=np.load(str(files[0].with_suffix('.npy')));hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);roi=np.zeros(dep.shape,np.uint8);roi[90:185,205:288]=255;mask=np.uint8((dep==0)&(hsv[:,:,1]<110)&(hsv[:,:,2]>100))*roi;p=cv2.goodFeaturesToTrack(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),maxCorners=60,qualityLevel=.01,minDistance=3,mask=mask);assert p is not None
p0=p[:,0].copy();ids=np.arange(len(p));alive=np.ones(len(p),bool);tracks=[p[:,0].copy()];prev=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
for f in files[1:]:
 cur=cv2.imread(str(f),0);q,st,err=cv2.calcOpticalFlowPyrLK(prev,cur,p,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.001));back,sb,_=cv2.calcOpticalFlowPyrLK(cur,prev,q,None,winSize=(21,21),maxLevel=3);good=(st[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back[:,0]-p[:,0],axis=1)<.5)&(err[:,0]<25);alive &=good;p=q;tracks.append(p[:,0].copy());prev=cur
end=p[:,0];good=np.where(alive)[0];a=p0[good];b=end[good];cam=json.loads((out/'v18_camera_board_registration.json').read_text());K=np.array(cam['camera_matrix']);R=np.array(cam['board_to_camera_rotation']);ep=K@R[:,2];ep=ep/ep[2];ue=a[:,0]+(b[:,1]-a[:,1])*(ep[0]-a[:,0])/(ep[1]-a[:,1]);du=b[:,0]-ue
# Independent image trajectories: expected line for pure board-normal translation.
report={'start_frame':lo,'end_frame':hi,'initial_feature_count':len(p0),'surviving_feature_count':len(good),'vertical_vanishing_point_px':ep[:2].tolist(),'tracks':[{'id':int(i),'start_px':p0[i].tolist(),'end_px':end[i].tolist(),'horizontal_residual_to_board_normal_px':float(d)} for i,d in zip(good,du)],'median_horizontal_residual_px':float(np.median(du)),'meaning':'negative is image-left / toward B after removing perspective of pure board-normal translation; rotation can also contribute','physical_3d_orientation_verified':False}
# Fit a common translation direction only as a hypothesis; expose residual rather than assuming rigidity.
ra=np.column_stack([a,np.ones(len(a))])@np.linalg.inv(K).T;rb=np.column_stack([b,np.ones(len(b))])@np.linalg.inv(K).T;lines=np.cross(ra,rb);lines/=np.linalg.norm(lines,axis=1)[:,None];u,s,vt=np.linalg.svd(lines);direction=vt[-1];direction=R.T@direction
if direction[2]>0:direction=-direction
report['translation_only_hypothesis']={'board_direction_unit':direction.tolist(),'X_per_down_Z':float(direction[0]/-direction[2]),'singular_values':s.tolist(),'warning':'invalid if tool rotates or tracked surfaces are not one rigid body'}
(out/'v18_rgb_descent_tracking.json').write_text(json.dumps(report,indent=2));np.savez(out/'v18_rgb_descent_tracks.npz',initial=p0,points=np.array(tracks),alive=alive)
last=cv2.imread(str(files[-1]));first=im.copy()
for j,i in enumerate(good):
 aa=tuple(np.rint(p0[i]).astype(int));bb=tuple(np.rint(end[i]).astype(int));cv2.circle(first,aa,2,(0,255,0),1);cv2.putText(first,str(i),aa,cv2.FONT_HERSHEY_SIMPLEX,.3,(0,255,0),1);cv2.circle(last,bb,2,(0,255,0),1);cv2.line(last,tuple(np.rint([ue[j],end[i,1]]).astype(int)),bb,(0,0,255),1);cv2.putText(last,str(i),bb,cv2.FONT_HERSHEY_SIMPLEX,.3,(0,255,0),1)
cv2.imwrite(str(out/'v18_descent_feature_comparison.jpg'),np.hstack([first,last]));print(json.dumps(report))
