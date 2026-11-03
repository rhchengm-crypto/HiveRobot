"""Track visible parts independently of FK; pixel residuals are not TCP mm."""
import cv2,json,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;folder=OUT/'rgbd'
sel=json.loads((folder/'selection.json').read_text());lo,hi=sel['contact_min'],sel['contact_max']
cam=json.loads((OUT/'camera_board_registration.json').read_text());ep=np.array(cam['vertical_vanishing_point_px'])
def read(i):return cv2.imdecode(np.fromfile(folder/('%06d.jpg'%i),dtype=np.uint8),cv2.IMREAD_COLOR)
first=read(lo);last=read(hi);reports=[];annot0=first.copy();annot1=last.copy()
rois={'gripper_fixed_body':[205,75,300,205],'wrist_motor_cap':[90,65,220,215],'proximal_wrist_bracket':[0,200,155,430]}
for label,(x0,y0,x1,y1) in rois.items():
 mask=np.zeros(first.shape[:2],np.uint8);mask[y0:y1,x0:x1]=255
 p=cv2.goodFeaturesToTrack(cv2.cvtColor(first,cv2.COLOR_BGR2GRAY),100,.02,5,mask=mask)
 if p is None:continue
 start=p[:,0].copy();alive=np.ones(len(p),bool);prev=cv2.cvtColor(first,cv2.COLOR_BGR2GRAY)
 for i in range(lo+1,hi+1):
  cur=cv2.cvtColor(read(i),cv2.COLOR_BGR2GRAY)
  q,st,err=cv2.calcOpticalFlowPyrLK(prev,cur,p,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.001))
  back,sb,_=cv2.calcOpticalFlowPyrLK(cur,prev,q,None,winSize=(21,21),maxLevel=3)
  alive&=(st[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back[:,0]-p[:,0],axis=1)<.5)&(err[:,0]<25)
  p=q;prev=cur
 end=p[:,0];moved=np.linalg.norm(end-start,axis=1)>8
 ids=np.where(alive&moved)[0];a=start[ids];b=end[ids]
 if len(ids)<3:
  reports.append({'part_roi':label,'survivors':len(ids),'insufficient_features':True});continue
 predicted_x=a[:,0]+(b[:,1]-a[:,1])*(ep[0]-a[:,0])/(ep[1]-a[:,1])
 residual=b[:,0]-predicted_x
 row={'part_roi':label,'survivors':len(ids),'median_pixel_flow':np.median(b-a,axis=0).tolist(),'median_residual_to_board_normal_px':float(np.median(residual)),'residual_IQR_px':np.quantile(residual,[.25,.75]).tolist(),'feature_start_px':a.tolist(),'feature_end_px':b.tolist()}
 reports.append(row)
 for j,idx in enumerate(ids):
  color=(0,255,0) if label=='gripper_fixed_body' else ((255,180,0) if label=='wrist_motor_cap' else (0,200,255))
  aa=tuple(np.rint(start[idx]).astype(int));bb=tuple(np.rint(end[idx]).astype(int))
  cv2.circle(annot0,aa,2,color,1);cv2.circle(annot1,bb,2,color,1)
  cv2.line(annot1,tuple(np.rint([predicted_x[j],b[j,1]]).astype(int)),bb,(0,0,255),1)
report={'source_folder':sel['source_folder'],'start_frame':lo,'end_frame':hi,'parts':reports,'warning':'Visible feature trajectories are independent image evidence. ROI may contain more than one body; nonzero residual may include tool rotation. Pixels must not be reported as TCP millimeters.'}
(OUT/'visual_motion_audit.json').write_text(json.dumps(report,indent=2));cv2.imencode('.jpg',np.hstack([annot0,annot1]))[1].tofile(OUT/'descent_comparison.jpg');print(json.dumps(report))
