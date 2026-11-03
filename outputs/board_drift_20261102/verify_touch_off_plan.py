from pathlib import Path
source=(Path(__file__).resolve().parent/'verify_optical_candidate_plan.py').read_text()
source=source.replace('left_arm_board_grasp_optical_candidate.json','left_arm_board_grasp_touch_candidate.json').replace('optical_candidate_plan_verification.json','touch_candidate_plan_verification.json').replace('optical_candidate_full_plan.json','touch_candidate_full_plan.json')
exec(compile(source,__file__,'exec'))
# Check CAD gripper vertices, not just the TCP datum, along sampled route edges.
mesh=json.loads((ROOT/'outputs/left_arm_step_analysis/revised_2/step_vertex_preview.json').read_text(encoding='utf8'))
vertices=np.array(mesh[7]['vertices_mm'])/1000
local=(vertices-model.home[-1,:3,3])@model.home[-1,:3,:3]
R=np.array(cfg['board_to_cad_rotation']);t=np.array(cfg['board_to_cad_translation_m'])
q=np.array([start[n] for n in model.names]);minimum=float('inf');samples=0
for row in trajectory['waypoints']:
 nextq=np.array([row['encoder_rad'][n] for n in model.names])
 count=max(1,int(np.ceil(np.max(np.abs(nextq-q))/np.radians(1))))
 for alpha in np.linspace(0,1,count+1):
  f=model.fk(model.encoder_to_model(q+(nextq-q)*alpha))[-1]
  xyz=(local@f[:3,:3].T+f[:3,3]-t)@R*1000
  inside=(xyz[:,0]>=0)&(xyz[:,0]<=440)&(xyz[:,1]>=0)&(xyz[:,1]<=440)
  if inside.any():minimum=min(minimum,float(xyz[inside,2].min()))
  samples+=1
 q=nextq
assert minimum>15,'CAD gripper envelope approaches board too closely'
report['CAD_gripper_mesh_minimum_height_over_board_mm']=minimum;report['CAD_gripper_mesh_sampled_poses']=samples
(OUT/'touch_candidate_plan_verification.json').write_text(json.dumps(report,indent=2));print(json.dumps({'mesh_minimum_height_mm':minimum,'sampled_poses':samples,'passed':True,'scope':'CAD envelope at unchanged opening; physical gap still requires observation'}))
