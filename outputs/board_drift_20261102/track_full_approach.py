"""Reuse recorded natural features; read-only analysis, not robot control."""
import subprocess,shlex,json,base64,zlib
from pathlib import Path
OUT=Path(__file__).resolve().parent
v=json.loads((OUT/'visual_motion_audit.json').read_text());part=next(p for p in v['parts'] if p['part_roi']=='gripper_fixed_body')
code=r'''
import cv2,numpy as np,json,base64,zlib
from pathlib import Path
folder=Path(FOLDER)
rows={r['index']:r for r in (json.loads(l) for l in (folder/'frames.jsonl').read_text().splitlines())}
lo=LO;hi=HI;seed=np.array(SEED,dtype=np.float32).reshape(-1,1,2)
def read(i):return cv2.imdecode(np.fromfile(str(folder/('%06d.jpg'%i)),dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
records={lo:{'points':seed[:,0].tolist(),'valid':[True]*len(seed)}}
for indexes in (range(lo-1,-1,-1),range(lo+1,hi+1)):
 p=seed.copy();alive=np.ones(len(p),bool);prev=read(lo)
 for i in indexes:
  cur=read(i)
  if cur is None:break
  q,st,err=cv2.calcOpticalFlowPyrLK(prev,cur,p,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.001))
  back,sb,_=cv2.calcOpticalFlowPyrLK(cur,prev,q,None,winSize=(21,21),maxLevel=3)
  alive&=(st[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back[:,0]-p[:,0],axis=1)<.5)&(err[:,0]<25)&(q[:,0,0]>3)&(q[:,0,0]<637)&(q[:,0,1]>3)&(q[:,0,1]<477)
  if alive.sum()<3:break
  if i%2==0 or i in (lo,hi):records[i]={'points':q[:,0].tolist(),'valid':alive.tolist()}
  p=q;prev=cur
out={'source_folder':str(folder),'contact_start':lo,'contact_end':hi,'feature_initial_points_px':SEED,'frames':[{'index':i,'rgb_stamp':rows[i]['rgb_stamp'],'stage':(rows[i].get('tracking') or {}).get('stage'),**records[i]} for i in sorted(records)]}
print(base64.b64encode(zlib.compress(json.dumps(out).encode(),9)).decode())
'''.replace('FOLDER',repr(v['source_folder'])).replace('LO',str(v['start_frame'])).replace('HI',str(v['end_frame'])).replace('SEED',repr(part['feature_start_px']))
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(code)],capture_output=True,timeout=45)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(zlib.decompress(base64.b64decode(r.stdout)));(OUT/'approach_feature_tracks.json').write_text(json.dumps(d,indent=2));print(json.dumps({'frames':len(d['frames']),'first_frame':d['frames'][0]['index'],'last_frame':d['frames'][-1]['index'],'train_frames':sum(f['index']<d['contact_start'] for f in d['frames']),'min_surviving_features':min(sum(f['valid']) for f in d['frames'])}))
