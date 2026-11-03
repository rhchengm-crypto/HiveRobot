"""Offline metrics with explicitly frozen models. Never opens serial or imports CAN.

Usage: python outputs/board_drift_20261102/reproduce_trial_metrics.py --trial V24
"""
import argparse,json,sys,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',choices=['original','V22','V23','V24'],default='V24');args=p.parse_args()
case=next(v for v in json.loads((OUT/'experiment_trials.json').read_text())['trials'] if v['id']==args.trial)
m=RigidBodyModel.load(OUT/'jetson_snapshot_v24_accepted/left_arm_general_dynamics.json');cfg=json.loads((OUT/case['model_file']).read_text());R=np.array(cfg['board_to_cad_rotation']);t=np.array(cfg['board_to_cad_translation_m'])
lines=(OUT/case['source_log']).read_text(encoding='utf8').splitlines();waypoint=next(json.loads(l.split('=',1)[1]) for l in lines if l.startswith('board waypoint=') and json.loads(l.split('=',1)[1])['stage']=='contact')
audits=[json.loads(l.split('=',1)[1]) for l in lines if l.startswith('board tracking audit=') and json.loads(l.split('=',1)[1])['stage']=='contact']
q0=np.array([waypoint['entry_feedback'][n]['pos'] for n in m.names]);actual=np.array([q0]+[[a['feedback'][n]['pos'] for n in m.names] for a in audits]);ref=np.array([q0]+[[a['reference_rad'][n] for n in m.names] for a in audits])
def xyz(q):return (tcp_pose(m,q,cfg['tcp_in_wrist_frame_m'])[:3,3]-t)@R*1000
av=np.array([xyz(q) for q in actual]);rv=np.array([xyz(q) for q in ref]);qerr=np.degrees(actual-ref)
print(json.dumps({'trial':case['id'],'frozen_board_model':case['model_file'],'contact_audits':len(audits),'encoder_FK_delta_board_mm':(av[-1]-av[0]).tolist(),'reference_FK_delta_board_mm':(rv[-1]-rv[0]).tolist(),'maximum_tracking_error_deg_by_joint':dict(zip(m.names,np.max(np.abs(qerr),axis=0).tolist())),'operator_B_drift_mm':case['B_drift_mm'],'height_result':case['height_result'],'warning':'FK values are model-dependent internal metrics, not independent physical TCP measurements. Operator drift is an estimate.'},indent=2))
