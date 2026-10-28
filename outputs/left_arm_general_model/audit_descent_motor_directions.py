"""Read-only comparison of actual motor directions across failed descents."""
import json,base64,zlib,shlex,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parents[2];out=Path(__file__).parent
files=['arm_run_1793146973371_58478.txt','arm_run_1793147478126_59137.txt',
       'arm_run_1793149223472_62768.txt','arm_run_1793149361434_63277.txt',
       'arm_run_1793149694827_64243.txt']
# The first file is hold-only; locate its preceding motion separately below.
files[0]='arm_run_1793146963927_58442.txt'
code="""
import json,base64,zlib
from pathlib import Path
r=Path('/home/nvidia/hive_robot/DM_Control_Python/data/left_arm_v2_8_full_runs')
data={n:(r/n).read_text(errors='replace') for n in FILES}
print(base64.b64encode(zlib.compress(json.dumps(data).encode())).decode())
""".replace('FILES',repr(files))
result=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes',
 '-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(code)],capture_output=True,timeout=40)
if result.returncode:raise RuntimeError(result.stderr.decode('utf8',errors='replace'))
logs=json.loads(zlib.decompress(base64.b64decode(result.stdout)))
(out/'descent_direction_source_logs.json').write_text(json.dumps(logs),encoding='utf8')
sys.path.insert(0,str(root/'scripts'))
import numpy as np
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose
m=RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json')
c=json.loads((root/'scripts/data/left_arm_board_grasp.json').read_text())
R=np.array(c['board_to_cad_rotation']);t=np.array(c['board_to_cad_translation_m']);tcp=np.array(c['tcp_in_wrist_frame_m'])
def xyz(q):return (tcp_pose(m,q,tcp)[:3,3]-t)@R*1000
decoder=json.JSONDecoder();reports=[]
for name,text in logs.items():
 def record(prefix):return decoder.raw_decode(text.rsplit(prefix,1)[1].lstrip())[0]
 request=record('board planning input= ');end=record('board endpoint report= ')
 start=np.array([request['start_rad'][n] for n in m.names]);target=np.array([end['nominal_target_rad'][n] for n in m.names])
 actual=np.array([end['measured_pose_rad'][n] for n in m.names]);mid=(start+actual)/2
 j=np.empty((3,m.n));step=1e-5
 for k in range(m.n):
  plus=mid.copy();minus=mid.copy();plus[k]+=step;minus[k]-=step
  j[:,k]=(xyz(plus)-xyz(minus))/(2*step)
 audits=[]
 for line in text.splitlines():
  if line.startswith('board tracking audit= '):audits.append(json.loads(line.split('= ',1)[1]))
 near=audits[-2:] if audits else []
 rows=[]
 for k,n in enumerate(m.names):
  late=[float(a['feedback'][n]['pos']) for a in near]+[actual[k]]
  rows.append({'joint':n,'planned_encoder_delta_deg':float(np.degrees(target[k]-start[k])),
   'actual_encoder_delta_deg':float(np.degrees(actual[k]-start[k])),
   'terminal_error_deg':float(np.degrees(actual[k]-target[k])),
   'last_sample_delta_deg':float(np.degrees(late[-1]-late[-2])) if len(late)>1 else None,
   'last_sample_velocity_rad_s':near[-1]['feedback'][n]['vel'] if near else None,
   'model_X_mm_per_encoder_rad':float(j[0,k]),
   'model_X_contribution_planned_mm':float(j[0,k]*(target[k]-start[k])),
   'model_X_contribution_actual_mm':float(j[0,k]*(actual[k]-start[k]))})
 reports.append({'file':name,'build':request['build'],'axes':rows,
  'model_path_delta_mm':(xyz(actual)-xyz(start)).tolist(),
  'nominal_path_delta_mm':(xyz(target)-xyz(start)).tolist(),
  'last_two_audit_frames':near})
(out/'descent_motor_direction_audit.json').write_text(json.dumps(reports,indent=2),encoding='utf8')
for report in reports:
 print(report['file'],report['build'])
 for r in report['axes']:
  print(r['joint'],'target',round(r['planned_encoder_delta_deg'],3),'actual',round(r['actual_encoder_delta_deg'],3),
        'late',round(r['last_sample_delta_deg'],3) if r['last_sample_delta_deg'] is not None else None,
        'X planned/actual mm',round(r['model_X_contribution_planned_mm'],2),round(r['model_X_contribution_actual_mm'],2))
