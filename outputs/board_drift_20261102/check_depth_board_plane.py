"""Check whether RGB chessboard pose and archived depth describe the same plane."""
import json,subprocess,shlex
from pathlib import Path
OUT=Path(__file__).resolve().parent
meta=json.loads((OUT/'capture.json').read_text());cam=json.loads((OUT/'camera_board_registration.json').read_text())
code=r'''
import json,numpy as np
from pathlib import Path
K=np.array(CAMERA['camera_matrix']);R=np.array(CAMERA['board_to_camera_rotation']);t=np.array(CAMERA['board_to_camera_translation_mm']);n=R[:,2];plane=float(n@t)
d=np.load(str(Path(FOLDER)/'000501.npy'));yy,xx=np.mgrid[70:350:3,315:540:3];uv=np.column_stack([xx.ravel(),yy.ravel(),np.ones(xx.size)])
z=d[yy,xx].ravel();rays=uv@np.linalg.inv(K).T;p=rays*z[:,None];good=(z>350)&(z<850)
for _ in range(6):
 a=p[good];center=np.median(a,axis=0);_,_,v=np.linalg.svd(a-center,full_matrices=False);nn=v[-1];offset=np.median(p@nn);res=p@nn-offset;good=(z>350)&(z<850)&(abs(res)<3)
if nn@n<0:nn=-nn;offset=-offset
print(json.dumps({'valid_points':int(good.sum()),'normal_difference_deg':float(np.degrees(np.arccos(np.clip(nn@n,-1,1)))),'depth_plane_normal':nn.tolist(),'rgb_plane_normal':n.tolist(),'depth_plane_offset_mm':float(offset),'rgb_plane_offset_mm':plane,'depth_residual_p95_mm':float(np.quantile(abs(res[good]),.95)),'status':'sensor cross-check only; no motor commands'}))
'''.replace('CAMERA',repr(cam)).replace('FOLDER',repr(meta['output']))
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(code)],capture_output=True,timeout=25)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(r.stdout);(OUT/'depth_board_plane_check.json').write_text(json.dumps(d,indent=2));print(json.dumps(d))
