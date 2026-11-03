"""Restore pretrial registration after fingertip contact; no motor commands."""
import json
from pathlib import Path
OUT=Path(__file__).resolve().parent
report=json.loads((OUT/'optical_trial_record.json').read_text())
report['physical_validation']={'result':'FAILED absolute height; do not reuse candidate as a validated registration','B_drift_mm':5,'contact':'operator reports fingertip lightly touched board','operator_authorized_home':True,'causal_limit':'Known-plate image agreement did not independently validate absolute fingertip height.'}
report['next_motion']='None after Home; model/TCP height must be resolved before further descent.'
(OUT/'optical_trial_record.json').write_text(json.dumps(report,indent=2))
source=(OUT/'deploy_optical_board_candidate.py').read_text()
source=source.replace('board_model_before_optical.json','board_model_failed_optical.json')
source=source.replace('left_arm_board_grasp_optical_candidate.json','board_model_before_optical.json')
source=source.replace('optical_registration_deployment.json','optical_registration_rollback.json').replace('v23_optical_','v23_rollback_')
source=source.replace('board registration only; no motor-controller or learning-file change','restore pretrial registration after failed absolute-height test; no motor commands')
exec(compile(source,__file__,'exec'))
