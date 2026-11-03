"""Keep rejected hypotheses and operator evidence distinct from a released fix."""
import json
from pathlib import Path
out=Path(__file__).resolve().parent
status=json.loads((out/'status_latest.json').read_text());run=status['run']
assert run['ok'] and not run['running'] and run['action']=='dynamics:model-hold-after:home'
report={
 'operator_observation':{'B_drift_mm':10,'when':'during descent','oscillation':False,'improvement':False},
 'v22_result':'rejected and rolled back on both local and Jetson',
 'production_version':'V21, same hashes as before the V22 trial',
 'distal_zero_or_axis_candidates':'not deployed; independent motion prediction did not improve',
 'new_evidence':{'distal_axis_validation':'distal_axis_independent_validation.json','depth_vs_RGB_board_normal_difference_deg':1.7811609457152748,'plate_centroid_visibility':'No reliable six-hole centroids before frame448; only late descent visible'},
 'robot_state':{'home_complete':True,'post_home_hold_complete':True,'running':False,'log':run['full_log_path']},
 'unresolved':'Incorrect physical Cartesian mapping not yet identified uniquely; no verified model correction.',
 'next_required_input':'Whether camera view can temporarily include board and six-hole wrist plate throughout approach and descent; do not move camera until agreed.',
 'next_motion':'No new descent issued after Home.',
 'preserved':['model hold takeover','adaptive learning','ordinary Clearance shoulder_rotate skip','shoulder_rotate cap36, kd4.5','no fixed Cartesian offset','no replay']
}
(out/'model_identification_status.json').write_text(json.dumps(report,indent=2));print(json.dumps({'saved':True,'home_complete':True,'verified_fix':False}))
