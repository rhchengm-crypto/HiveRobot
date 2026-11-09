import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import json,sys,hashlib
import numpy as np
sys.path.insert(0,'scripts')
import left_arm_board_grasp as b
out=Path('outputs/a8_saved_reference_20261108')
remote=json.loads((out/'saved_moves_remote.json').read_text(encoding='utf-8'))
source=next(s for s in remote['sources'] if s['name']=='data/left_arm_v2_6_saved_moves.json')
move=source['data']['moves']['A8_take']
m=b.RigidBodyModel.load('scripts/data/left_arm_general_dynamics.json')
cfg=b.load_board_config('scripts/data/left_arm_board_workspace_candidate.json',m)
q=np.array([move['pose'][n] for n in m.names]);bounds=np.array(cfg['joint_limits_encoder_rad'])
pose=b.tcp_pose(m,q,cfg['tcp_in_wrist_frame_m'])
r=np.array(cfg['board_to_cad_rotation']);t=np.array(cfg['board_to_cad_translation_m'])
point=(pose[:3,3]-t)@r*1000
try:b.check_board_intersection(m,q,cfg);collision={'passed':True}
except b.BoardPlanningError as exc:collision={'passed':False,'error':str(exc),'diagnostics':exc.diagnostics}
result=dict(motion_issued=False,saved_move=move,source_sha256=source['sha256'],
    saved_pose_tcp_board_mm=point.tolist(),saved_pose_wrist_rotation_in_board=(r.T@pose[:3,:3]).tolist(),
    target_board_mm=[27.5,412.5,50.2],
    difference_from_current_request_mm=(point-np.array([27.5,412.5,50.2])).tolist(),
    operating_bound_violations_deg={n:dict(below_min=float(np.degrees(max(0,bounds[i,0]-q[i]))),
        above_max=float(np.degrees(max(0,q[i]-bounds[i,1])))) for i,n in enumerate(m.names)
        if q[i]<bounds[i,0] or q[i]>bounds[i,1]},
    body_and_board_check=collision,latest_web_action=remote['web'],
    source_hashes={n:hashlib.sha256((Path('scripts')/n).read_bytes()).hexdigest() for n in
        ('data/left_arm_general_dynamics.json','data/left_arm_board_workspace_candidate.json')})
(out/'saved_pose_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
