import json,hashlib
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
report={
 'change':'Board-to-CAD optical registration, derived from known STEP six-hole plate, fixed RGB camera, and current board intersections.',
 'camera_moved':False,'joint_zero_changes':False,'CAD_dimension_changes':False,
 'evidence':json.loads((OUT/'optical_board_candidate_audit.json').read_text()),
 'held_out_validation':json.loads((OUT/'known_plate_calibration_candidates.json').read_text())[0]['independent_v22'],
 'deployment':json.loads((OUT/'optical_registration_deployment.json').read_text()),
 'offline_plan':json.loads((OUT/'optical_candidate_plan_verification.json').read_text()),
 'target_mm':[113.8,190.3,50.2],'piece':'white_knight','stage':'approach, empty board, no claw closure',
 'model_board_sha256':hashlib.sha256((ROOT/'scripts/data/left_arm_board_grasp.json').read_bytes()).hexdigest(),
 'physical_validation':'pending trial and operator observation',
 'correction_to_prior_diagnosis':'Plate was visible near descent entry; LK points drifted off hole centers. Independent contour/ring detection replaced LK for this calibration.',
 'rejected_alternatives':['V22 fixed shoulder_rotate (operator reports10mm B, no improvement); rolled back','Distal axis tilt: independent motion error worsened; never deployed','Five-joint zero fit: independent motion prediction not improved; never deployed'],
 'preserved':['same-process same-serial model hold','adaptive learning','ordinary Clearance skips shoulder_rotate','shoulder_rotate cap36/kd4.5','no fixed Cartesian offset','no angle replay'],
}
(OUT/'optical_trial_record.json').write_text(json.dumps(report,indent=2));print(json.dumps({'saved':True,'physical_validation':report['physical_validation']}))
