"""Preserve trial evidence separately from unverified physical accuracy."""
import hashlib,json
from pathlib import Path
out=Path(__file__).resolve().parent
log=out/'jetson_snapshot/arm_run_1793679218917_10319.txt'
lines=log.read_text(encoding='utf8').splitlines()
result=next(json.loads(l) for l in reversed(lines) if l.startswith('{"ok":') and 'post_move_model_hold' in l)
audits=[json.loads(l.split('=',1)[1]) for l in lines if l.startswith('board tracking audit=')]
contact=[a for a in audits if a['stage']=='contact']
constraint=next(json.loads(l.split('=',1)[1]) for l in lines if l.startswith('board descent constraint='))
entry=constraint['fixed_encoder_rad']['shoulder_rotate']
refs=[a['reference_rad']['shoulder_rotate'] for a in contact]
feedback=[a['feedback']['shoulder_rotate']['pos'] for a in contact]
report={
 'build':'seven-axis-board-grasp-v22-fixed-swivel-descent',
 'change':'Fix shoulder_rotate reference at measured descent entry; solve six remaining joints for unchanged Cartesian pose.',
 'rationale':'Controlled test removes approximately 1.5deg of previously planned swivel cancellation; does not claim geometry is calibrated.',
 'target_mm':[113.8,190.3,50.2], 'stage':'approach; no claw closure',
 'start':'User confirmed Home before execution',
 'production_files':['scripts/left_arm_board_grasp.py','scripts/left_arm_board_kinematics.py'],
 'unchanged':['model hold serial handoff','adaptive learning','ordinary Clearance skips shoulder_rotate','shoulder_rotate cap36/kd4.5','board transform','TCP','no fixed Cartesian compensation','no replay'],
 'full_log':str(log),'sha256':hashlib.sha256(log.read_bytes()).hexdigest(),
 'contact_audit_count':len(contact),'constraint':constraint,
 'reference_peak_deviation_rad':max(abs(v-entry) for v in refs),
 'feedback_peak_deviation_deg':max(abs(v-entry) for v in feedback)*180/3.141592653589793,
 'post_move_model_hold':result['post_move_model_hold'],
 'physical_observation':'pending user feedback; encoder FK alone is not proof of no drift',
 'next_action':'No further motion until observation is incorporated.',
}
(out/'v22_trial_record.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
