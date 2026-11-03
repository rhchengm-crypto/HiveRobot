"""Track the six existing CAD holes in archived frames; no motor control."""
import json,subprocess,shlex,base64,zlib
from pathlib import Path
OUT=Path(__file__).resolve().parent
pose=json.loads((OUT/'existing_plate_pose.json').read_text());meta=json.loads((OUT/'capture.json').read_text())
code=r'''
import cv2,json,numpy as np,zlib,base64
from pathlib import Path
folder=Path(FOLDER);lo=501;rows={r['index']:r for r in (json.loads(l) for l in (folder/'frames.jsonl').read_text().splitlines())}
def read(i):return cv2.imdecode(np.fromfile(str(folder/('%06d.jpg'%i)),dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
p=np.array(SEED,dtype=np.float32).reshape(-1,1,2);alive=np.ones(len(p),bool);prev=read(lo);records=[]
for i in range(lo,-1,-1):
 if i!=lo:
  cur=read(i)
  if cur is None:break
  q,st,e=cv2.calcOpticalFlowPyrLK(prev,cur,p,None,winSize=(17,17),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.001))
  back,sb,_=cv2.calcOpticalFlowPyrLK(cur,prev,q,None,winSize=(17,17),maxLevel=3)
  alive&=(st[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back[:,0]-p[:,0],axis=1)<.4)&(e[:,0]<25)&(q[:,0,0]>6)&(q[:,0,0]<634)&(q[:,0,1]>6)&(q[:,0,1]<474)
  if alive.sum()<3:break
  p=q;prev=cur
 if i%2==0 or i==lo:records.append({'index':i,'rgb_stamp':rows[i]['rgb_stamp'],'points':p[:,0].tolist(),'valid':alive.tolist()})
print(base64.b64encode(zlib.compress(json.dumps({'frames':records[::-1],'object_points_plane_m':OBJECT,'plane_station_mm':STATION,'source_folder':str(folder)}).encode(),9)).decode())
'''.replace('FOLDER',repr(meta['output'])).replace('SEED',repr(pose['candidates'][0]['ordered_pixels'])).replace('OBJECT',repr(pose['object_points_plane_m'])).replace('STATION',repr(pose['cad_plane_station_mm']))
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(code)],capture_output=True,timeout=45)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(zlib.decompress(base64.b64decode(r.stdout)));(OUT/'plate_feature_tracks.json').write_text(json.dumps(d,indent=2));print(json.dumps({'frames':len(d['frames']),'first_frame':d['frames'][0]['index'],'last_frame':d['frames'][-1]['index'],'minimum_valid':min(sum(r['valid']) for r in d['frames'])}))
