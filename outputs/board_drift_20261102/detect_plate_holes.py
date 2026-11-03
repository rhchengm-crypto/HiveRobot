"""Independently detect the existing hexagonal hole pattern in raw RGB frames."""
import json,shlex,subprocess,zlib,base64
from pathlib import Path
OUT=Path(__file__).resolve().parent
meta=json.loads((OUT/'capture_v22.json').read_text())
code=r'''
import cv2,numpy as np,json,base64,zlib,itertools
from pathlib import Path
folder=Path(FOLDER);reports=[]
for frame in [338,350,365,380,395,407]:
 im=cv2.imdecode(np.fromfile(str(folder/('%06d.jpg'%frame)),np.uint8),1);gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);roi=gray[250:425,0:140]
 _,binary=cv2.threshold(roi,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
 contours,hierarchy=cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE);blobs=[]
 for co in contours:
  area=cv2.contourArea(co);M=cv2.moments(co);per=cv2.arcLength(co,True)
  if not 8<area<2500 or not M['m00'] or not per:continue
  xy=[M['m10']/M['m00'],M['m01']/M['m00']+250];circularity=4*np.pi*area/per**2
  if circularity>.3:blobs.append({'xy':xy,'area':float(area),'circularity':float(circularity)})
 reports.append({'frame':frame,'blobs':blobs})
print(base64.b64encode(zlib.compress(json.dumps(reports).encode(),9)).decode())
'''.replace('FOLDER',repr(meta['output']))
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(code.encode(),9)),capture_output=True,timeout=30)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(zlib.decompress(base64.b64decode(r.stdout)));(OUT/'plate_independent_blob_candidates.json').write_text(json.dumps(d,indent=2));print(json.dumps(d))
