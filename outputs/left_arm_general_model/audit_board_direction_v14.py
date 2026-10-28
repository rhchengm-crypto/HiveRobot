import sys,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'scripts'))
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose,board_point_to_cad
m=RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json')
c=json.loads((root/'scripts/data/left_arm_board_grasp.json').read_text())
registration=json.loads((root/'outputs/left_arm_step_analysis/revised_2/encoder_registration.json').read_text())
d=json.loads((root/'outputs/left_arm_general_model/latest_descent_logs.json').read_text())
q=np.array([d[1]['endpoint']['measured_pose_rad'][n] for n in m.names])
R=np.array(c['board_to_cad_rotation']);t=np.array(c['board_to_cad_translation_m']);tcp=np.array(c['tcp_in_wrist_frame_m'])
jv,_=m.point_jacobian(m.encoder_to_model(q),m.n-1,tcp)
analytic=R.T@jv*m.signs[None,:]*1000
finite=np.empty_like(analytic)
def independent_xyz(encoder):
 f=m.fk_poe(m.encoder_to_model(encoder))[-1]
 return R.T@(f[:3,3]+f[:3,:3]@tcp-t)*1000
for k in range(m.n):
 step=np.zeros(m.n);step[k]=1e-6
 finite[:,k]=(independent_xyz(q+step)-independent_xyz(q-step))/2e-6
np.testing.assert_allclose(analytic,finite,atol=1e-5,rtol=1e-6)
for n in m.names:
 assert m.rows[n]['encoder_sign']==registration['joints'][n]['encoder_to_model_sign']
 assert abs(m.rows[n]['reference_encoder_rad']-registration['joints'][n]['encoder_at_reference_rad'])<1e-10
xyz=independent_xyz(q)
np.testing.assert_allclose(board_point_to_cad(xyz,R,t),tcp_pose(m,q,tcp)[:3,3],atol=1e-10)
r={'encoder_pose_rad':dict(zip(m.names,q.tolist())),'board_axis_definition':'X toward H, Y toward rank 8, Z away from board',
   'units':'mm per encoder rad','jacobian_encoder_to_board':analytic.tolist(),
   'max_independent_poe_finite_difference_error':float(np.max(np.abs(analytic-finite))),
   'registration_signs_and_references_match':True,'board_transform_roundtrip_matches':True,
   'runtime_signs_changed':False,'physical_direction_verified':False,
   'physical_limitation':'Four historical anchors and coupled-motion observations do not uniquely identify a wrong physical axis, TCP or board registration.'}
Path(__file__).with_name('board_v14_direction_mapping_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))
