"""Compare TCP datum with the full CAD gripper envelope at reported contact."""
import json,sys,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json');c=json.loads((OUT/'board_model_failed_optical.json').read_text())
text=(OUT/'jetson_snapshot/arm_run_1793680858524_14500.txt').read_text();lines=text.splitlines()
result=next(json.loads(l) for l in reversed(lines) if l.startswith('{"ok":') and 'post_move_model_hold' in l)
R=np.array(c['board_to_cad_rotation']);t=np.array(c['board_to_cad_translation_m'])
mesh=json.loads((ROOT/'outputs/left_arm_step_analysis/revised_2/step_vertex_preview.json').read_text(encoding='utf8'));vertices=np.array(mesh[7]['vertices_mm'])/1000
local=(vertices-m.home[-1,:3,3])@m.home[-1,:3,:3];reports=[]
poses={'motion_end':result['endpoint']['measured_pose_rad'],'hold_end':{n:result['model_hold']['final'][n]['pos'] for n in m.names}}
for name,pose in poses.items():
 q=np.array([pose[n] for n in m.names]);T=m.fk(m.encoder_to_model(q))[-1];world=local@T[:3,:3].T+T[:3,3];board=(world-t)@R*1000;tcp=(tcp_pose(m,q,c['tcp_in_wrist_frame_m'])[:3,3]-t)@R*1000;index=int(np.argmin(board[:,2]))
 reports.append({'pose':name,'TCP_board_mm':tcp.tolist(),'minimum_CAD_gripper_height_mm':float(board[index,2]),'lowest_CAD_vertex_board_mm':board[index].tolist(),'lowest_point_relative_TCP_mm':(board[index]-tcp).tolist(),'gripper_height_percentiles_mm':np.quantile(board[:,2],[0,.01,.05,.5,1]).tolist(),'assumption':'CAD gripper opening matches physical opening; vertex envelope only, not verified physical fingertip contact.'})
(OUT/'contact_mesh_height_audit.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports))
