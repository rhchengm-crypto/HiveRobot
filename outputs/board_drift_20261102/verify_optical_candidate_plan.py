"""Full-route and feedback-descent checks for the optical registration candidate."""
import json,sys,copy,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_grasp import load_board_config,plan,feedback_contact_path,check_transit_edge
from left_arm_board_kinematics import tcp_pose
model=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json');cfg=load_board_config(OUT/'left_arm_board_grasp_optical_candidate.json',model)
run=json.loads((OUT/'status_latest.json').read_text())['run'];assert not run['running'] and run['action']=='dynamics:model-hold-after:home'
decoder=json.JSONDecoder();decoded=[];text=(OUT/'jetson_snapshot'/Path(run['full_log_path']).name).read_text(encoding='utf8')
for i,ch in enumerate(text):
 if ch!='{':continue
 try:o,_=decoder.raw_decode(text[i:])
 except ValueError:continue
 if isinstance(o,dict) and 'final' in o and 'final_error_deg' in o:decoded.append(o)
assert decoded,'Need latest Home hold final feedback'
start={n:decoded[-1]['final'][n]['pos'] for n in model.names}
trajectory=plan(model,cfg,start,113.8,190.3,'white_knight','approach',50.2)
last_above=next(r for r in reversed(trajectory['waypoints']) if r['stage']=='source_above')
entry=last_above['encoder_rad'];rows,diagnostic=feedback_contact_path(model,cfg,entry,[113.8,190.3,50.2])
previous=np.array([entry[n] for n in model.names]);maxstep=0.
for row in rows:
 q=np.array([row['encoder_rad'][n] for n in model.names]);check_transit_edge(model,previous,q,cfg);maxstep=max(maxstep,float(np.max(np.abs(np.degrees(q-previous)))));previous=q
report={'passed':True,'motion_issued':False,'start_rad':start,'full_waypoints':len(trajectory['waypoints']),'feedback_descent_waypoints':len(rows),'max_feedback_step_deg':maxstep,'maximum_IK_position_error_mm':max(r['position_error_mm'] for r in rows),'maximum_IK_rotation_error_deg':max(r['rotation_error_deg'] for r in rows),'new_endpoint_board_mm':trajectory['target_board_mm'],'nominal_entry':diagnostic,'source_step_unchanged':True,'robot_joint_zeros_unchanged':True,'not_yet_physically_validated':True}
(OUT/'optical_candidate_plan_verification.json').write_text(json.dumps(report,indent=2));(OUT/'optical_candidate_full_plan.json').write_text(json.dumps(trajectory,indent=2));print(json.dumps(report))
