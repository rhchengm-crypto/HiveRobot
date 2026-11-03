"""Build publication-support tables and a traceable experiment sequence figure."""
import csv,hashlib,json,html
from pathlib import Path
OUT=Path(__file__).resolve().parent
archive=json.loads((OUT/'raw_evidence/archive_manifest.json').read_text())
source={m['path']:m for m in archive['members']}
cases=[
 dict(id='original',model='board_model_before_optical.json',log='arm_run_1793671770732_5069.txt',capture='capture.json',intervention='Unmodified V21 replication',B_drift_mm=None,height_result='Not quantified',oscillation=None,accepted=False,notes='Initial25mm observation confounded by board displacement before motion; later8mm was endpoint reobservation after board repositioning, NOT another trajectory.'),
 dict(id='V22',model='board_model_before_optical.json',log='arm_run_1793679218917_10319.txt',capture='capture_v22.json',intervention='Fixed shoulder_rotate during descent',B_drift_mm=10,height_result='Not separately quantified',oscillation=False,accepted=False,notes='No improvement; rolled back both Python files. Kp cap36, Kd4.5.'),
 dict(id='V23',model='board_model_failed_optical.json',log='arm_run_1793680858524_14500.txt',capture='capture_v23.json',intervention='Known-CAD plate optical board registration',B_drift_mm=5,height_result='FAILED: light fingertip-board contact',oscillation=None,accepted=False,notes='Lateral improvement, absolute height invalid. Operator authorized Home; candidate rolled back.'),
 dict(id='V24',model='jetson_snapshot_v24_accepted/left_arm_board_grasp.json',log='arm_run_1793681494917_16715.txt',capture='capture_v24.json',intervention='Optical normal/XY + contact-referenced plane height',B_drift_mm=2,height_result='PASS: clearly visible fingertip clearance',oscillation=False,accepted=True,notes='Operator accepts2mm as working range. Empty-load only, same gripper opening. No claw closure or loaded lift.'),
]
for case in cases:
 case.update(target_X_mm=113.8,target_Y_mm=190.3,target_TCP_height_mm=50.2,stage='approach',gripper_closed=False,B_measurement_method='operator visual estimate; uncertainty not instrumented',repetition_count=1)
 case['model_file']=case.pop('model');case['source_log']='jetson_snapshot/'+case.pop('log')
 raw='data/left_arm_v2_8_full_runs/'+Path(case['source_log']).name
 case['raw_log_SHA256']=source[raw]['sha256'];case['raw_log_archive_part']=source[raw]['archive_part']
 cap=json.loads((OUT/case['capture']).read_text());case['raw_RGBD_source_directory']=cap['output']
 case['model_snapshot_byte_SHA256']=hashlib.sha256((OUT/case['model_file']).read_bytes()).hexdigest()
 text=(OUT/case['source_log']).read_text(encoding='utf8');result=next(json.loads(l) for l in reversed(text.splitlines()) if l.startswith('{"ok":') and 'post_move_model_hold' in l)
 case['post_hold_same_serial']=result['post_move_model_hold']['same_serial_session'];case['post_hold_seconds']=result['post_move_model_hold']['seconds']
 case['endpoint_settle_attempts']=result['endpoint'].get('endpoint_settle_attempts');case['maximum_terminal_joint_error_deg']=result['endpoint']['max_nominal_error_deg'];case['nominal_joint_target_reached']=result['endpoint']['nominal_joint_target_reached']
(OUT/'experiment_trials.json').write_text(json.dumps({'local_date':'2026-11-02','timezone':'America/Los_Angeles','design':'Exploratory sequential tests, not randomized repeated trials. Model height mapping changes between cases.','trials':cases},indent=2))
with (OUT/'experiment_trials.csv').open('w',encoding='utf-8-sig',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(cases[0]));w.writeheader();w.writerows(cases)
inventory=json.loads((OUT/'final_readonly_inventory.json').read_text());times={v['path']:v['mtime_utc'] for v in inventory['files']}
with (OUT/'raw_evidence/catalog.csv').open('w',encoding='utf-8-sig',newline='') as f:
 w=csv.DictWriter(f,fieldnames=['path','size','sha256','archive_part','source_mtime_utc']);w.writeheader()
 for v in archive['members']:w.writerow({**v,'source_mtime_utc':times.get(v['path'],'')})
fig=OUT/'figures';fig.mkdir(exist_ok=True)
cards=[('V22','Fixed shoulder swivel','B drift: ~10 mm','No improvement','#fbe6e5','#ac2d2d'),('V23','Optical registration','B drift: ~5 mm','Fingertip contact: height FAILED','#fff0d2','#956000'),('V24','Touch-referenced height','B drift: ~2 mm','Visible gap; operator ACCEPTED','#e4f3eb','#226343')]
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="360" viewBox="0 0 1200 360">','<rect width="1200" height="360" fill="white"/>','<g font-family="Arial, sans-serif">','<text x="30" y="38" font-size="24" fill="#13293d">2026-11-02: model-registration experiment sequence</text>']
for i,(version,title,drift,status,bg,ink) in enumerate(cards):
 x=30+i*395
 svg.extend([f'<rect x="{x}" y="65" width="350" height="190" rx="12" fill="{bg}"/>',f'<text x="{x+18}" y="98" font-size="16" font-weight="bold" fill="{ink}">{version}</text>',f'<text x="{x+18}" y="132" font-size="20" fill="#13293d">{html.escape(title)}</text>',f'<text x="{x+18}" y="174" font-size="27" fill="#13293d">{drift}</text>',f'<text x="{x+18}" y="220" font-size="15" fill="{ink}">{html.escape(status)}</text>'])
 if i<2:svg.append(f'<path d="M{x+359} 155 h26 m-9 -7 l9 7 -9 7" stroke="#64748b" fill="none" stroke-width="2"/>')
svg.extend(['<text x="30" y="294" font-size="15" fill="#475569">Operator-estimated drift; one observation per configuration; empty-load, no gripper closure.</text>','<text x="30" y="320" font-size="15" fill="#475569">Sequential interventions changed absolute height. No statistical or isolated-causal claim is implied.</text>','</g></svg>'])
(fig/'experiment_sequence.svg').write_text('\n'.join(svg),encoding='utf8')
print(json.dumps({'trial_rows':len(cases),'raw_file_rows':len(archive['members']),'archive_parts':len(archive['parts']),'figure':'figures/experiment_sequence.svg'}))
