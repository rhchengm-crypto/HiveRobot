"""Validate old-run fits against V22 camera observations without refitting."""
import json,subprocess,shlex,base64,zlib,sys
from pathlib import Path
import numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'left_arm_general_model/python_deps'))
sys.path.insert(0,str(ROOT/'scripts'))
from scipy.spatial.transform import Rotation
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
old=json.loads((OUT/'capture.json').read_text());new=json.loads((OUT/'capture_v22.json').read_text());d=np.load(OUT/'approach_camera_fit_without_feature5.npz');seed=d['pixels'][-1].tolist()
code=r'''
import cv2,numpy as np,json,zlib,base64
from pathlib import Path
def read(folder,i):return cv2.imdecode(np.fromfile(str(Path(folder)/('%06d.jpg'%i)),np.uint8),cv2.IMREAD_GRAYSCALE)
old=read(OLD,501);prev=read(NEW,407);p=np.array(SEED,np.float32).reshape(-1,1,2)
q,st,e=cv2.calcOpticalFlowPyrLK(old,prev,p,None,winSize=(21,21),maxLevel=3)
back,sb,_=cv2.calcOpticalFlowPyrLK(prev,old,q,None,winSize=(21,21),maxLevel=3)
alive=(st[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back[:,0]-p[:,0],axis=1)<.4)&(e[:,0]<25);p=q
meta={r['index']:r for r in map(json.loads,(Path(NEW)/'frames.jsonl').read_text().splitlines())};rows=[]
for i in range(407,337,-1):
 if i!=407:
  cur=read(NEW,i);q,st,e=cv2.calcOpticalFlowPyrLK(prev,cur,p,None,winSize=(21,21),maxLevel=3)
  back,sb,_=cv2.calcOpticalFlowPyrLK(cur,prev,q,None,winSize=(21,21),maxLevel=3)
  alive&=(st[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back[:,0]-p[:,0],axis=1)<.4)&(e[:,0]<25);p=q;prev=cur
 if i%2==0 or i==407:rows.append({'index':i,'rgb_stamp':meta[i]['rgb_stamp'],'points':p[:,0].tolist(),'valid':alive.tolist()})
print(base64.b64encode(zlib.compress(json.dumps(rows[::-1]).encode(),9)).decode())
'''.replace('OLD',repr(old['output'])).replace('NEW',repr(new['output'])).replace('SEED',repr(seed))
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(code.encode(),9)),capture_output=True,timeout=40)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
rows=json.loads(zlib.decompress(base64.b64decode(r.stdout)));(OUT/'v22_corresponding_feature_tracks.json').write_text(json.dumps({'frames':rows,'source_folder':new['output'],'contact_end':407},indent=2))
aud=[json.loads(l.split('=',1)[1]) for l in (OUT/'jetson_snapshot/arm_run_1793679218917_10319.txt').read_text().splitlines() if l.startswith('board tracking audit=')]
ts=np.array([(a['feedback_read_started_wall_s']+a['feedback_read_finished_wall_s'])/2 for a in aud]);qs=np.array([[a['feedback'][n]['pos'] for n in m.names] for a in aud])
q=np.array([[np.interp(f['rgb_stamp'],ts,qs[:,j]) for j in range(m.n)] for f in rows]);pix=np.array([f['points'] for f in rows]);valid=np.array([f['valid'] for f in rows])
c=json.loads((OUT.parent/'left_arm_general_model/v18_camera_board_registration.json').read_text());cfg=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());K=np.array(c['camera_matrix']);rc0=np.array(c['board_to_camera_rotation'])@np.array(cfg['board_to_cad_rotation']).T;tc0=np.array(c['board_to_camera_translation_mm'])/1000-rc0@np.array(cfg['board_to_cad_translation_m'])
tests=[{'hypothesis':'baseline','x':d['x'].tolist()}]+json.loads((OUT/'distal_axis_hypotheses.json').read_text());results=[]
for test in tests:
 x=np.array(test['x']);h=test['hypothesis'];count=0 if h=='baseline' else (1 if h=='wrist_side_zero' else 2)
 p=x[6:len(x)-count if count else None].reshape(5,3);angles=m.signs[None,:]*(q-m.reference);T=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
 if h=='wrist_side_zero':angles[:,5]-=m.signs[5]*x[-1]
 for j,parent in enumerate(m.parents):
  parent=parent.copy()
  if j==6 and h=='wrist_axis_tilt':parent[:3,:3]=parent[:3,:3]@Rotation.from_rotvec([*x[-2:],0]).as_matrix()
  z=np.broadcast_to(np.eye(4),(len(q),4,4)).copy();co=np.cos(angles[:,j]);si=np.sin(angles[:,j]);z[:,0,0]=co;z[:,0,1]=-si;z[:,1,0]=si;z[:,1,1]=co;T=T@parent@z
 world=np.einsum('nij,kj->nki',T[:,:3,:3],p)+T[:,:3,3][:,None,:];rc=Rotation.from_rotvec(x[:3]).as_matrix()@rc0;camera=world@rc.T+tc0+x[3:6];uv=camera@K.T;uv=uv[:,:,:2]/uv[:,:,2:];err=np.linalg.norm(uv-pix,axis=-1)[valid]
 relative=[];endpoint=[]
 for j in range(5):
  indices=np.where(valid[:,j])[0]
  if len(indices)<2:continue
  a,b=indices[0],indices[-1]
  errors=(uv[indices,j]-uv[a,j])-(pix[indices,j]-pix[a,j])
  relative.extend(np.linalg.norm(errors,axis=1).tolist())
  endpoint.append({'feature':j,'motion_prediction_minus_observed_px':errors[-1].tolist()})
 results.append({'hypothesis':h,'independent_run_median_px':float(np.median(err)),'independent_run_p95_px':float(np.quantile(err,.95)),'observations':int(valid.sum()),'motion_error_median_px':float(np.median(relative)),'motion_error_p95_px':float(np.quantile(relative,.95)),'endpoint_motion_errors':endpoint})
(OUT/'distal_axis_independent_validation.json').write_text(json.dumps(results,indent=2));print(json.dumps(results))
