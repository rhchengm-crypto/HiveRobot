"""Separate encoder tracking, nominal Cartesian paths and physical observations.

Offline only. Uses recorded feedback and the existing unmodified geometry;
does not turn internal FK agreement into physical validation.
"""
import sys, json, hashlib
from pathlib import Path
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose

model=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json')
cfg=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text(encoding='utf8'))
R=np.array(cfg['board_to_cad_rotation']);t=np.array(cfg['board_to_cad_translation_m']);tcp=np.array(cfg['tcp_in_wrist_frame_m'])
def vec(q):return np.array([q[n] for n in model.names])
def xyz(q):return (tcp_pose(model,q,tcp)[:3,3]-t)@R*1000
def attribution(path):
    answer=np.zeros((3,model.n));nodes,weights=np.polynomial.legendre.leggauss(4)
    for a,b in zip(path,path[1:]):
        delta=b-a
        for node,w in zip(nodes,weights):
            q=a+delta*(node+1)/2
            j,_=model.point_jacobian(model.encoder_to_model(q),model.n-1,tcp)
            answer+=(R.T@j*model.signs[None,:])*delta[None,:]*w/2*1000
    return answer
def analyze(name,observed):
    p=ROOT/'scripts/data/left_arm_v2_8_full_runs'/name
    text=p.read_text(encoding='utf8',errors='replace')
    samples=[];first=None;endpoint=None
    for line in text.splitlines():
        if line.startswith('board waypoint='):
            w=json.loads(line.split('=',1)[1])
            if w['stage']=='contact' and first is None:first=w
        elif line.startswith('board tracking audit='):
            a=json.loads(line.split('=',1)[1])
            if a['stage']=='contact':samples.append(a)
        elif line.startswith('board endpoint report='):
            endpoint=json.JSONDecoder().raw_decode(line.split('=',1)[1].lstrip())[0]
    if not samples or first is None:raise ValueError('No separately identified contact descent')
    initial=vec({n:s['pos'] for n,s in first['entry_feedback'].items()})
    actual=[initial]+[vec({n:s['pos'] for n,s in a['feedback'].items()}) for a in samples]
    reference=[initial]+[vec(a['reference_rad']) for a in samples]
    actual=np.array(actual);reference=np.array(reference)
    actual_xyz=np.array([xyz(q) for q in actual]);reference_xyz=np.array([xyz(q) for q in reference])
    error_xyz=actual_xyz-reference_xyz
    aa=attribution(actual);ra=attribution(reference)
    residual_actual=(actual_xyz[-1]-actual_xyz[0])-aa.sum(axis=1)
    residual_reference=(reference_xyz[-1]-reference_xyz[0])-ra.sum(axis=1)
    if max(np.max(np.abs(residual_actual)),np.max(np.abs(residual_reference)))>1e-4:
        raise ValueError('Joint attribution failed independent FK displacement check')
    command_error=np.array([[a['last_sent_commands'][n]['q']-a['reference_rad'][n] for n in model.names] for a in samples])
    rows=[]
    for j,n in enumerate(model.names):
        rows.append({'joint':n,'reference_descent_deg':float(np.degrees(reference[-1,j]-reference[0,j])),
                     'measured_descent_deg':float(np.degrees(actual[-1,j]-actual[0,j])),
                     'reference_model_X_mm':float(ra[0,j]),'measured_model_X_mm':float(aa[0,j]),
                     'tracking_model_X_difference_mm':float(aa[0,j]-ra[0,j]),
                     'max_tracking_error_deg':float(np.max(np.abs(np.degrees(actual[:,j]-reference[:,j])))),
                     'max_command_minus_reference_deg':float(np.max(np.abs(np.degrees(command_error[:,j])))),
                     'kp_range':[min(a['last_sent_commands'][n]['kp'] for a in samples),max(a['last_sent_commands'][n]['kp'] for a in samples)],
                     'kd_range':[min(a['last_sent_commands'][n]['kd'] for a in samples),max(a['last_sent_commands'][n]['kd'] for a in samples)]})
    final_tracking=error_xyz[-1]
    report={'source_log':name,'source_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
            'contact_sample_count':len(samples),'physical_observation_B_drift_mm':observed,
            'physical_observation_source':'operator report; estimate, not independent metrology',
            'reference_contact_model_delta_mm':(reference_xyz[-1]-reference_xyz[0]).tolist(),
            'measured_contact_model_delta_mm':(actual_xyz[-1]-actual_xyz[0]).tolist(),
            'tracking_model_X_error_range_mm':[float(error_xyz[:,0].min()),float(error_xyz[:,0].max())],
            'end_tracking_model_xyz_error_mm':final_tracking.tolist(),
            'max_command_minus_reference_deg':float(np.max(np.abs(np.degrees(command_error)))),
            'joint_attribution_sum_error_mm':max(float(np.max(np.abs(residual_actual))),float(np.max(np.abs(residual_reference)))),
            'joints':rows,'endpoint_settle_attempts':endpoint.get('endpoint_settle_attempts') if endpoint else None,
            'physical_cause_identified':False,
            'warning':'All model-mm quantities use the same unverified CAD-to-board/TCP mapping; they cannot validate physical accuracy.'}
    return report

if len(sys.argv)>1:
    report=analyze(str(Path(sys.argv[1]).resolve()),None)
    (OUT/'latest_tracking_attribution.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report))
    raise SystemExit(0)
reports=[analyze('arm_run_1793160008611_79906.txt',8.),
         analyze('arm_run_1793161074626_82286.txt',15.),
         analyze('arm_run_1793164847303_93849.txt',10.)]
up=-model.configured_gravity();up/=np.linalg.norm(up)
summary={'motion_issued':False,'production_files_modified':False,'physical_fix_verified':False,
         'board_X_direction':'positive D, negative B','trials':reports,
         'nominal_gravity_board_up_components':(R.T@up).tolist(),
         'board_normal_vs_nominal_gravity_degrees':float(np.degrees(np.arccos(np.clip(R[:,2]@up,-1,1)))),
         'gravity_direction_independently_measured':False}
(OUT/'tracking_attribution.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
for trial in reports:
    print(json.dumps({k:trial[k] for k in ['source_log','physical_observation_B_drift_mm','reference_contact_model_delta_mm',
          'measured_contact_model_delta_mm','tracking_model_X_error_range_mm','end_tracking_model_xyz_error_mm',
          'max_command_minus_reference_deg','endpoint_settle_attempts']}))
print('LATEST_AXIS_BREAKDOWN',json.dumps(reports[-1]['joints']))
