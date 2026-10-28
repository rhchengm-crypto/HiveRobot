"""Offline kinematic/STEP comparison figure; does not edit the supplied photos."""
import json,sys,tempfile,os
from pathlib import Path
deps=Path(tempfile.gettempdir())/'hiverobot_pose_render_deps'
sys.path.insert(0,str(deps));os.environ.setdefault('MPLCONFIGDIR',str(deps/'mpl_config'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'scripts'))
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose
out=Path(__file__).parent
d=json.loads((out/'latest_descent_logs.json').read_text(encoding='utf8'))
model=RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json')
cfg=json.loads((root/'scripts/data/left_arm_board_grasp.json').read_text())
encoder=np.array([d[0]['hold']['final'][n]['pos'] for n in model.names])
frames=model.fk(model.encoder_to_model(encoder))
r=np.array(cfg['board_to_cad_rotation']);t=np.array(cfg['board_to_cad_translation_m'])
def board(points):return (np.asarray(points)-t)@r*1000
axes=board(frames[:,:3,3]);tcp=board(tcp_pose(model,encoder,cfg['tcp_in_wrist_frame_m'])[:3,3])
rows=json.loads((root/'outputs/left_arm_step_analysis/revised_2/step_vertex_preview.json').read_text())
points=np.array(rows[7]['vertices_mm'])/1000
gripper=board(((points-model.home[-1,:3,3])@model.home[-1,:3,:3])@frames[-1,:3,:3].T+frames[-1,:3,3])
chain=np.vstack([axes,tcp]);fig=plt.figure(figsize=(15,8))
labels=['1 front','2 side','3 shoulder rotate','4 elbow','5 arm roll','6 wrist side','7 wrist','TCP']
for p,(i,j,title) in enumerate([(0,2,'Board X / Z'),(1,2,'Board Y / Z')]):
 ax=fig.add_subplot(1,3,p+1)
 ax.plot(chain[:,i],chain[:,j],'-o',color='#23597e',lw=2,ms=5)
 ax.scatter(gripper[::2,i],gripper[::2,j],s=.35,color='#526375',alpha=.5)
 for k,pt in enumerate(chain):
  ax.annotate(labels[k],pt[[i,j]],xytext=(5,3 if k%2 else -12),textcoords='offset points',fontsize=8)
 ax.plot([0,440],[0,0],c='#b49763',lw=3)
 ax.scatter(tcp[i],tcp[j],c='#c33425',marker='+',s=140)
 ax.set_xlabel(('X','Y','Z')[i]+' (mm)');ax.set_ylabel('Z (mm)');ax.set_title(title)
 ax.axis('equal');ax.grid(alpha=.25)
ax=fig.add_subplot(1,3,3,projection='3d')
ax.plot(*chain.T,'-o',color='#23597e',lw=2,ms=4)
ax.scatter(*gripper[::5].T,s=.3,color='#526375',alpha=.35)
square=np.array([[0,0,0],[440,0,0],[440,440,0],[0,440,0],[0,0,0]])
ax.plot(*square.T,c='#b49763',lw=2)
ax.scatter(*tcp,c='#c33425',marker='+',s=100)
ax.set_xlabel('X (mm)');ax.set_ylabel('Y (mm)');ax.set_zlabel('Z (mm)')
ax.view_init(elev=15,azim=-65);ax.set_title('Oblique model view')
limits=np.vstack([chain,square,gripper]);span=limits.max(0)-limits.min(0)
ax.set_box_aspect(span)
fig.suptitle('Model at the last recorded encoder pose — NOT fitted to the photographs',fontsize=16)
fig.text(.03,.025,'Blue: model joint axes / link chain. Grey: articulated revised-STEP gripper vertices. Red: nominal finger-tip TCP.\nBoard registration, physical zero offsets and gripping-centre TCP remain unverified; CAD jaw opening is fixed.',fontsize=10)
fig.tight_layout(rect=[0,.10,1,.94]);fig.savefig(out/'current_arm_pose_review.png',dpi=160)
report={'source_log':d[0]['path'],'serial_read_or_motion_issued':False,
 'encoder_pose_rad':dict(zip(model.names,encoder.tolist())),
 'joint_origins_board_mm':dict(zip(model.names,axes.tolist())),
 'nominal_tcp_board_mm':tcp.tolist(),
 'model_joint_displacements_deg':dict(zip(model.names,np.degrees(model.encoder_to_model(encoder)).tolist()))}
(out/'current_arm_pose_review.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report))
