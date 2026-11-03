"""Re-detect known holes in every archived frame instead of trusting LK drift."""
import json,subprocess,shlex,zlib,base64
from pathlib import Path
OUT=Path(__file__).resolve().parent
tracks=json.loads((OUT/'plate_feature_tracks.json').read_text())
code=r'''
import cv2,numpy as np,json,base64,zlib
from pathlib import Path
d=TRACKS;folder=Path(d['source_folder'])
for f in d['frames']:
 im=cv2.imdecode(np.fromfile(str(folder/('%06d.jpg'%f['index'])),np.uint8),cv2.IMREAD_GRAYSCALE)
 centers=[];valid=[];changes=[]
 for uv,alive in zip(f['points'],f['valid']):
  x,y=np.rint(uv).astype(int);r=7
  if not alive or x<r or y<r or x+r>=im.shape[1] or y+r>=im.shape[0]:centers.append(uv);valid.append(False);changes.append(None);continue
  patch=im[y-r:y+r+1,x-r:x+r+1];_,binary=cv2.threshold(patch,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
  contours,_=cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE);options=[]
  for co in contours:
   area=cv2.contourArea(co);M=cv2.moments(co)
   if not 5<area<90 or M['m00']==0:continue
   center=np.array([M['m10']/M['m00']+x-r,M['m01']/M['m00']+y-r]);distance=np.linalg.norm(center-uv)
   if distance<4:options.append((distance,center))
  if not options:centers.append(uv);valid.append(False);changes.append(None);continue
  dist,p=min(options,key=lambda a:a[0]);centers.append(p.tolist());valid.append(True);changes.append(float(dist))
 f['LK_points']=f['points'];f['points']=centers;f['valid']=valid;f['refinement_distance_px']=changes
d['method']='Independent local dark-hole contour centroid in every frame; reject unmatched patches'
print(base64.b64encode(zlib.compress(json.dumps(d).encode(),9)).decode())
'''.replace('TRACKS',repr(tracks))
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(code.encode(),9)),capture_output=True,timeout=40)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(zlib.decompress(base64.b64decode(r.stdout)));(OUT/'plate_centroid_tracks.json').write_text(json.dumps(d,indent=2));print(json.dumps({'frames':len(d['frames']),'valid':sum(sum(f['valid']) for f in d['frames'])}))
