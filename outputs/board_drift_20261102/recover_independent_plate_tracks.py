"""Detect six CAD ring holes independently; retain only a consistent ring."""
import json,subprocess,shlex,zlib,base64
from pathlib import Path
OUT=Path(__file__).resolve().parent
pose=json.loads((OUT/'existing_plate_pose.json').read_text())
sessions=[('original',json.loads((OUT/'capture.json').read_text())['output'],370,501),('v22',json.loads((OUT/'capture_v22.json').read_text())['output'],300,407)]
code=r'''
import cv2,numpy as np,json,base64,zlib,itertools
from pathlib import Path
reports={};obj=np.array(OBJECT);design=np.column_stack([obj[:,:2],np.ones(6)])
for name,directory,lo,hi in SESSIONS:
 folder=Path(directory);meta={r['index']:r for r in map(json.loads,(folder/'frames.jsonl').read_text().splitlines())};frames=[]
 for frame in range(lo,hi+1):
  if frame%2 and frame!=hi:continue
  im=cv2.imdecode(np.fromfile(str(folder/('%06d.jpg'%frame)),np.uint8),0);roi=im[240:440,:150]
  _,binary=cv2.threshold(roi,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU);contours,_=cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE);blobs=[]
  for co in contours:
   a=cv2.contourArea(co);p=cv2.arcLength(co,True);M=cv2.moments(co)
   if not 45<a<180 or not p or not M['m00'] or 4*np.pi*a/p**2<.72:continue
   blobs.append([M['m10']/M['m00'],M['m01']/M['m00']+240])
  if not 6<=len(blobs)<=12:continue
  best=None
  for indices in itertools.combinations(range(len(blobs)),6):
   points=np.array([blobs[i] for i in indices]);center=points.mean(0)
   points=points[np.argsort(np.arctan2((points-center)[:,1],(points-center)[:,0]))]
   if REVERSE:points=points[::-1]
   points=np.roll(points,SHIFT,axis=0)
   affine=np.linalg.lstsq(design,points,rcond=None)[0];error=np.linalg.norm(design@affine-points,axis=1)
   # Reject six unrelated features that happen to lie on the metal palm.
   ratio=np.linalg.svd(affine[:2],compute_uv=False);radius=np.mean(np.linalg.norm(points-center,axis=1))
   if not 18<radius<40 or ratio[-1]/ratio[0]<.35:continue
   score=float(np.mean(error**2))
   if best is None or score<best[0]:best=(score,points,error)
  if best is None or max(best[2])>1.5:continue
  frames.append({'index':frame,'rgb_stamp':meta[frame]['rgb_stamp'],'points':best[1].tolist(),'valid':[True]*6,'ring_affine_error_px':best[2].tolist()})
 reports[name]={'frames':frames,'object_points_plane_m':OBJECT,'plane_station_mm':STATION,'source_folder':directory,'method':'independent six-hole centroid detection with CAD ring consistency in each frame'}
print(base64.b64encode(zlib.compress(json.dumps(reports).encode(),9)).decode())
'''.replace('SESSIONS',repr(sessions)).replace('OBJECT',repr(pose['object_points_plane_m'])).replace('STATION',repr(pose['cad_plane_station_mm'])).replace('REVERSE',repr(pose['candidates'][0]['reverse'])).replace('SHIFT',repr(pose['candidates'][0]['shift']))
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(code.encode(),9)),capture_output=True,timeout=40)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(zlib.decompress(base64.b64decode(r.stdout)))
for name,value in d.items():(OUT/('independent_plate_tracks_'+name+'.json')).write_text(json.dumps(value,indent=2))
print(json.dumps({k:{'frames':len(v['frames']),'first':v['frames'][0]['index'] if v['frames'] else None,'last':v['frames'][-1]['index'] if v['frames'] else None} for k,v in d.items()}))
