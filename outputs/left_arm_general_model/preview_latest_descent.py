import json,subprocess,shlex,sys
from pathlib import Path
d=json.loads(Path(__file__).with_name('latest_descent_logs.json').read_text(encoding='utf8'))
pose={n:v['pos'] for n,v in d[0]['hold']['final'].items()}
code=r'''
import json,sys,numpy as np
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python');sys.path.insert(0,str(root))
from left_arm_board_grasp import RigidBodyModel,load_board_config,tcp_pose,plan
model=RigidBodyModel.load(root/'data/left_arm_general_dynamics.json')
cfg=load_board_config(root/'data/left_arm_board_grasp.json',model)
pose=POSE
q=np.array([pose[n] for n in model.names]);r=np.array(cfg['board_to_cad_rotation']);t=np.array(cfg['board_to_cad_translation_m'])
xyz=(tcp_pose(model,q,np.array(cfg['tcp_in_wrist_frame_m']))[:3,3]-t)@r*1000
x,y,z=round(float(xyz[0]),1),round(float(xyz[1]),1),round(float(xyz[2])-10,1)
trajectory=plan(model,cfg,pose,x,y,'white_knight','touch',z)
out={'motion_issued':False,'start_tcp_board_mm':xyz.tolist(),'x_mm':x,'y_mm':y,'height_mm':z,'descent_mm':float(xyz[2])-z,'waypoint_count':len(trajectory['waypoints']),'stages':[v.get('stage') for v in trajectory['waypoints']],'last_waypoint':trajectory['waypoints'][-1]}
print(json.dumps(out))
'''.replace('POSE',repr(pose)).replace('float(xyz[2])-10','float(xyz[2])-'+str(float(sys.argv[1]) if len(sys.argv)>1 else 10.0))
cmd=['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(code)]
r=subprocess.run(cmd,capture_output=True)
if r.returncode:raise RuntimeError(r.stderr.decode('utf8',errors='replace'))
print(r.stdout.decode('utf8'))
Path(__file__).with_name('latest_descent_preview.json').write_bytes(r.stdout)
