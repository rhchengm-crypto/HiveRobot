"""Offline STEP geometry projection from the last recorded encoder pose.

No robot connection, motor commands, image alteration, or calibration writes.
"""
import json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'scripts'))
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose
out=Path(__file__).parent
logs=json.loads((out/'latest_descent_logs.json').read_text(encoding='utf8'))
pose={n:row['pos'] for n,row in logs[0]['hold']['final'].items()}
model=RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json')
cfg=json.loads((root/'scripts/data/left_arm_board_grasp.json').read_text())
frames=model.fk(model.encoder_to_model([pose[n] for n in model.names]))
r=np.array(cfg['board_to_cad_rotation']);t=np.array(cfg['board_to_cad_translation_m'])
def board(points):return (np.asarray(points)-t)@r*1000
tcp=board(tcp_pose(model,np.array([pose[n] for n in model.names]),cfg['tcp_in_wrist_frame_m'])[:3,3])
parts=json.loads((root/'outputs/left_arm_step_analysis/revised_2/step_vertex_preview.json').read_text())
points=np.array(next(p['vertices_mm'] for p in parts if p['part_index']==7))/1000
local=(points-model.home[-1,:3,3])@model.home[-1,:3,:3]
world=local@frames[-1,:3,:3].T+frames[-1,:3,3]
gripper=board(world)
axis0=-np.array(cfg['tcp_datum']['axis_in_cad'])
axis=(axis0@model.home[-1,:3,:3])@frames[-1,:3,:3].T@r
angle=float(np.degrees(np.arccos(np.clip(axis@np.array([0.,0.,-1.]),-1,1))))
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1380" height="760" viewBox="0 0 1380 760">',
 '<rect width="1380" height="760" fill="white"/>',
 '<g font-family="Arial" fill="#25374a"><text x="30" y="36" font-size="24">Last recorded pose: articulated revised STEP gripper</text>',
 '<text x="30" y="68" font-size="17" fill="#a63829">MODEL PREDICTION ONLY: physical TCP and board registration unverified</text>']
for panel,(i,j,title) in enumerate([(0,2,'Board X / Z'),(1,2,'Board Y / Z'),(0,1,'Top view: board X / Y')]):
 lo=gripper[:,[i,j]].min(0)-12;hi=gripper[:,[i,j]].max(0)+12
 if j==2:lo[1]=min(-10,lo[1])
 else:lo=np.minimum(lo,[98,153]);hi=np.maximum(hi,[177,232])
 scale=min(400/(hi[0]-lo[0]),470/(hi[1]-lo[1]))
 x0=30+panel*460;y0=130
 def pixel(a,b):return x0+20+(a-lo[0])*scale,y0+470-(b-lo[1])*scale
 svg.append(f'<text x="{x0}" y="111" font-size="20">{title}</text>')
 for a,b in gripper[:,[i,j]][::2]:
  x,y=pixel(a,b);svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r=".7" fill="#465d76" opacity=".65"/>')
 x,y=pixel(tcp[i],tcp[j]);svg.append(f'<path d="M{x-9},{y}h18 M{x},{y-9}v18" stroke="#dc382d" stroke-width="2.5"/>')
 if j==2:
  x,y=pixel(lo[0],0);xx,_=pixel(hi[0],0)
  svg.append(f'<path d="M{x},{y}H{xx}" stroke="#999" stroke-width="1"/>')
 else:
  x,y=pixel(110,220);xx,yy=pixel(165,165)
  svg.append(f'<rect x="{x}" y="{y}" width="{xx-x}" height="{yy-y}" fill="none" stroke="#b49763" stroke-width="2"/>')
  svg.append(f'<text x="{x+5}" y="{y+20}" font-size="13" fill="#8f733e">Nominal C4</text>')
 svg.append(f'<text x="{x0}" y="636" font-size="14">Red cross: nominal finger-tip TCP</text>')
 svg.append(f'<text x="{x0}" y="657" font-size="13">{("X","Y","Z")[i]}: {lo[0]:.1f} .. {hi[0]:.1f} mm; {("X","Y","Z")[j]}: {lo[1]:.1f} .. {hi[1]:.1f} mm</text>')
svg.extend(['<text x="30" y="710" font-size="16">Orthographic views; not a camera-matched overlay. CAD jaw opening may differ from the photo.</text>',
 '</g></svg>'])
(out/'current_gripper_model_audit.svg').write_text('\n'.join(svg),encoding='utf8')
report={'motion_issued':False,'pose_source_log':logs[0]['path'],'live_status_read':False,
 'encoder_pose_rad':pose,'nominal_tcp_board_mm':tcp.tolist(),
 'nominal_gripper_axis_tilt_from_board_down_deg':angle,
 'nominal_gripper_vertices_board_bounds_mm':[gripper.min(0).tolist(),gripper.max(0).tolist()],
 'registration_fit_residuals_mm':cfg['fit_errors_mm'],
 'physical_tcp_verified':False,'physical_board_registration_verified':False,
 'photo_limitations':['shoulders and upper arm cropped','single oblique view; no metric 3D recovery',
                      'jaw articulation differs from fixed CAD jaw state'],
 'model_or_learning_json_modified':False}
(out/'current_gripper_model_audit.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report))
