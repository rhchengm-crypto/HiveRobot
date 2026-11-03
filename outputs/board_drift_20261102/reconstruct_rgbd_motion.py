"""Independent RGB-D feature motion; do not infer TCP truth from encoder FK."""
import subprocess,shlex,json,zlib,base64
from pathlib import Path
OUT=Path(__file__).resolve().parent
tracks=json.loads((OUT/'approach_feature_tracks.json').read_text());camera=json.loads((OUT/'camera_board_registration.json').read_text())
code=r'''
import numpy as np,json,zlib,base64
from pathlib import Path
tracks=TRACKS;K=np.array(CAMERA['camera_matrix']);R=np.array(CAMERA['board_to_camera_rotation']);ki=np.linalg.inv(K);folder=Path(tracks['source_folder'])
def sample(d,uv):
 x,y=np.rint(uv).astype(int);radius=4
 if x<radius or y<radius or x+radius>=640 or y+radius>=480:return None
 a=d[y-radius:y+radius+1,x-radius:x+radius+1].astype(float)
 yy,xx=np.mgrid[y-radius:y+radius+1,x-radius:x+radius+1]
 good=(a>180)&(a<550)
 if good.sum()<8:return None
 z=a[good];u=xx[good];v=yy[good]
 # Separate foreground surfaces from farther structure; don't fill missing depth with board depth.
 bins=np.floor(z/10).astype(int);best=max(set(bins),key=lambda b:sum(abs(bins-b)<=1));keep=abs(bins-best)<=1
 z=z[keep];u=u[keep];v=v[keep]
 if len(z)<8:return None
 A=np.column_stack([u-x,v-y,np.ones(len(z))])
 if np.linalg.matrix_rank(A)<3:return None
 co=np.linalg.lstsq(A,1/z,rcond=None)[0];pred=1/(A@co);res=z-pred
 keep=np.abs(res-np.median(res))<3
 if keep.sum()<8:return None
 co=np.linalg.lstsq(A[keep],1/z[keep],rcond=None)[0];z0=1/(np.array([uv[0]-x,uv[1]-y,1.])@co)
 rms=float(np.sqrt(np.mean((z[keep]-1/(A[keep]@co))**2)))
 if rms>2.5 or not 180<z0<550:return None
 point=ki@np.r_[uv,1.]*z0
 return {'camera_point_mm':point.tolist(),'depth_mm':float(z0),'local_depth_fit_rmse_mm':rms,'support_pixels':int(keep.sum())}
out=[]
for frame in tracks['frames']:
 if frame['index']>tracks['contact_end']:continue
 d=np.load(str(folder/('%06d.npy'%frame['index'])))
 observations=[sample(d,p) if valid else None for p,valid in zip(frame['points'],frame['valid'])]
 out.append({'index':frame['index'],'rgb_stamp':frame['rgb_stamp'],'observations':observations})
report={'status':'independent depth diagnostic only','assumptions':['depth units interpreted as millimeters; checked only for order of magnitude against CAD hole spacing','depth and RGB use the recorded color optical frame','nearby local surface fitted at a tracked feature; not a measured TCP'],'frames':out}
print(base64.b64encode(zlib.compress(json.dumps(report).encode(),9)).decode())
'''.replace('TRACKS',repr(tracks)).replace('CAMERA',repr(camera))
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(code.encode(),9)),capture_output=True,timeout=40)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(zlib.decompress(base64.b64decode(r.stdout)));(OUT/'rgbd_feature_motion.json').write_text(json.dumps(d,indent=2))
print(json.dumps({'frames':len(d['frames']),'valid_observations':sum(sum(o is not None for o in f['observations']) for f in d['frames']),'first_valid_count':sum(o is not None for o in d['frames'][0]['observations']),'last_valid_count':sum(o is not None for o in d['frames'][-1]['observations'])}))
