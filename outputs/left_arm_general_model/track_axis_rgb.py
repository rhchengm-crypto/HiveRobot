import cv2,numpy as np,json
from pathlib import Path
def read_image(path, flag=cv2.IMREAD_COLOR): return cv2.imdecode(np.fromfile(path,dtype=np.uint8),flag)
out=Path(__file__).parent;folder=out/'rgbd_axis_audit';sel=json.loads((folder/'selection.json').read_text());lo,hi=sel['first_included'],sel['last_included'];im=read_image(str(folder/('%06d.jpg'%lo)));dep=np.load(folder/('%06d.npy'%lo));hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);roi=np.zeros(dep.shape,np.uint8);roi[65:165,185:270]=255;mask=np.uint8((dep==0)&(hsv[:,:,1]<110)&(hsv[:,:,2]>100))*roi;p=cv2.goodFeaturesToTrack(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),maxCorners=60,qualityLevel=.01,minDistance=3,mask=mask);assert p is not None;initial=p[:,0].copy();alive=np.ones(len(p),bool);tracks=[p[:,0].copy()];prev=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
for i in range(lo+1,hi+1):
 cur=read_image(str(folder/('%06d.jpg'%i)),0);q,s,e=cv2.calcOpticalFlowPyrLK(prev,cur,p,None,winSize=(21,21),maxLevel=3);bk,sb,_=cv2.calcOpticalFlowPyrLK(cur,prev,q,None,winSize=(21,21),maxLevel=3);alive&=(s[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(bk[:,0]-p[:,0],axis=1)<.5)&(e[:,0]<25);p=q;prev=cur;tracks.append(p[:,0].copy())
np.savez(out/'v19_axis_rgb_tracks.npz',frames=np.arange(lo,hi+1),initial=initial,points=np.array(tracks),alive=alive);print('initial/surviving',len(alive),alive.sum(),np.where(alive)[0].tolist())
for i,p in enumerate(initial):
 color=(0,255,0) if alive[i] else (0,0,255);xy=tuple(np.rint(p).astype(int));cv2.circle(im,xy,2,color,1);cv2.putText(im,str(i),xy,cv2.FONT_HERSHEY_SIMPLEX,.3,color,1)
cv2.imencode('.jpg',im)[1].tofile(out/'v19_axis_features.jpg')
