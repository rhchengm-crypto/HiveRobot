import json,subprocess,shlex
from pathlib import Path
d=json.loads(Path(__file__).with_name('latest_descent_logs.json').read_text(encoding='utf8'))
code=r'''
import json,sys,numpy as np
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python');sys.path.insert(0,str(root))
from left_arm_board_grasp import RigidBodyModel,load_board_config,tcp_pose
data=DATA
m=RigidBodyModel.load(root/'data/left_arm_general_dynamics.json');c=load_board_config(root/'data/left_arm_board_grasp.json',m)
r=np.array(c['board_to_cad_rotation']);t=np.array(c['board_to_cad_translation_m'])
def xyz(q):return ((tcp_pose(m,np.array([q[n] for n in m.names]),np.array(c['tcp_in_wrist_frame_m']))[:3,3]-t)@r*1000)
s=Path(data[1]['path']).read_text();request=json.JSONDecoder().raw_decode(s.split('board planning input= ',1)[1])[0];start=request['start_rad']
e=data[1]['endpoint'];h=data[0]['hold']
points={'start':xyz(start),'nominal_end':xyz(e['nominal_target_rad']),'motion_end':xyz(e['measured_pose_rad']),'hold_end':xyz({n:v['pos'] for n,v in h['final'].items()})}
out={'request':request,'hold_mode':h['seamless_takeover']['mode'],'max_hold_error_deg':h['max_abs_error_deg'],'tcp_board_mm':{k:v.tolist() for k,v in points.items()},'motion_delta_mm':(points['motion_end']-points['start']).tolist(),'hold_delta_mm':(points['hold_end']-points['motion_end']).tolist(),'tracking_delta_mm':(points['motion_end']-points['nominal_end']).tolist()}
print(json.dumps(out))
'''.replace('DATA',repr(d))
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(code)],capture_output=True)
if r.returncode:raise RuntimeError(r.stderr.decode('utf8',errors='replace'))
print(r.stdout.decode('utf8'));Path(__file__).with_name('latest_touch_diagnosis.json').write_bytes(r.stdout)
