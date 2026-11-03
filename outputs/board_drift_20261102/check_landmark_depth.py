"""Read archived depth only; no commands to the robot."""
import subprocess,shlex,json
from pathlib import Path
OUT=Path(__file__).resolve().parent;folder=json.loads((OUT/'capture.json').read_text())['output']
code=r'''
import numpy as np,json
d=np.load(FOLDER+'/000501.npy');out={}
rois={'wrist_side_plate':(40,300,115,370),'gripper_body':(225,150,280,210),'board_far':(350,60,450,100),'board_near':(350,300,450,350)}
for name,(x0,y0,x1,y1) in rois.items():
 a=d[y0:y1,x0:x1];v=a[a>0]
 out[name]={'valid_fraction':float((a>0).mean()),'quantiles_raw':np.quantile(v,[0,.1,.5,.9,1]).tolist() if len(v) else []}
print(json.dumps(out))
'''.replace('FOLDER',repr(folder))
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(code)],capture_output=True,timeout=15)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
(OUT/'depth_landmark_check.json').write_bytes(r.stdout);print(r.stdout.decode())
