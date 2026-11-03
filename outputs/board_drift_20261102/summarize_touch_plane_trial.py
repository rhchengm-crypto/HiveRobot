"""Record control completion separately from physical height validation."""
import json,hashlib,sys,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json');c=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());assert 'touch_plane_datum' in c
log=OUT/'jetson_snapshot/arm_run_1793681494917_16715.txt';lines=log.read_text(encoding='utf8').splitlines()
result=next(json.loads(l) for l in reversed(lines) if l.startswith('{"ok":') and 'post_move_model_hold' in l)
mesh=json.loads((ROOT/'outputs/left_arm_step_analysis/revised_2/step_vertex_preview.json').read_text(encoding='utf8'));v=np.array(mesh[7]['vertices_mm'])/1000;local=(v-m.home[-1,:3,3])@m.home[-1,:3,:3]
R=np.array(c['board_to_cad_rotation']);t=np.array(c['board_to_cad_translation_m']);minimum=float('inf');count=0
for l in lines:
 if not l.startswith('board tracking audit='):continue
 a=json.loads(l.split('=',1)[1])
 if a['stage']!='contact':continue
 q=np.array([a['feedback'][n]['pos'] for n in m.names]);f=m.fk(m.encoder_to_model(q))[-1];z=((local@f[:3,:3].T+f[:3,3]-t)@R)[:,2]*1000;minimum=min(minimum,float(z.min()));count+=1
record={'build':c['build'],'target_mm':[113.8,190.3,50.2],'source_log':str(log),'source_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256((ROOT/'scripts/data/left_arm_board_grasp.json').read_bytes()).hexdigest(),'change':'retain optical XY/normal and set absolute board height using user-confirmed light fingertip touch','datum':c['touch_plane_datum'],'contact_audit_count':count,'model_predicted_minimum_gripper_height_during_contact_mm':minimum,'post_move_model_hold':result['post_move_model_hold'],'endpoint_settle_attempts':result['endpoint'].get('endpoint_settle_attempts'),'maximum_terminal_joint_error_deg':result['endpoint']['max_nominal_error_deg'],'nominal_joint_target_reached':result['endpoint']['nominal_joint_target_reached'],'physical_validation':'pending operator observation; no encoder or CAD computation is proof of physical clearance','claw_closed':False,'further_motion_issued':False}
(OUT/'touch_plane_trial_record.json').write_text(json.dumps(record,indent=2));print(json.dumps({k:v for k,v in record.items() if k not in ['datum','source_log']}))
