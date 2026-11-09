from pathlib import Path
import json,datetime,hashlib,shutil
root=Path.cwd();out=root/'outputs/a8_reference_execution_20261108'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
audit=read(out/'execution_audit.json');collection=read(out/'collection_report.json')
prep=read(out/'preparation.json');download=read(out/'download_verification.json')
feedback=dict(accepted=False,motion_accepted=False,trajectory_smoothness_accepted=True,
    original_response='无抖动，无接触，最终高度太高，可以下降15mm，有横移向边界外横移了20mm',
    stage_confirmation='下降过程中逐渐发生',jitter=False,contact=False,
    gripping_height_acceptable=False,height_too_high_mm=15.,
    approximate_lateral_drift_mm=20.,lateral_direction='outward beyond A-column boundary',
    qualitative_visual_estimate=True,external_precise_metrology=False)
(out/'operator_feedback.json').write_text(json.dumps(feedback,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
audit['operator_feedback']=feedback;audit['physical_validation_accepted']=False;audit['fitting_eligible']=False
(out/'execution_audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
trace=out/'control_evidence/data/board_motion_traces/trace_1794185516975282199.jsonl'
last={}
for line in trace.open(encoding='utf-8'):
    row=json.loads(line)
    if row.get('event')=='motor_frame':last[row.get('context',{}).get('stage')]=row
contact=last['contact'];pd={}
for n,cmd in contact['commands'].items():
    state=contact['feedback'][n]
    pd[n]=dict(target_rad=cmd['q'],feedback_rad=state['pos'],kp=cmd['kp'],kd=cmd['kd'],
        tau_ff_Nm=cmd['tau_ff'],measured_tau_Nm=state['tau'],
        calculated_pd_tau_Nm=cmd['kp']*(cmd['q']-state['pos'])+cmd['kd']*(cmd['dq']-state['vel']))
pred=audit['model_predictions_not_external_measurements']['board_mm']
analysis=dict(schema=1,motion_issued=False,accepted=False,reference_endpoint='A8_take',
    tracking_error_confirmed=True,planned_terminal_equals_saved=audit['checks']['planned_terminal_equals_saved_A8_take'],
    actual_endpoint_minus_saved_deg=audit['motion_endpoint_minus_saved_deg'],
    terminal_contact_commands_and_PD_estimates=pd,
    lateral_observation=feedback,
    model_predictions_not_external_measurements=audit['model_predictions_not_external_measurements'],
    model_predicted_contact_displacement_mm=audit['stage_and_waypoint_tracking']['stages']['contact']['model_predicted_displacement_mm'],
    model_to_observation_caveat='The model predicts contact-stage X increases by about3.61mm, while the operator observes about20mm outward. Also FK at measured endpoint is below FK at saved endpoint, while physical grip remains too high. These are different datums/observations and do not establish an external geometric calibration. Do not correct height or XY from encoder-FK alone.',
    support_takeover_stable=True,
    no_automatic_motion_after_operator_rejection=True,
    conclusion='Exact saved joint endpoint was planned but not reached in feedback. Multi-axis steady tracking/support discrepancy exists. It is not sufficient to attribute all observed drift to tracking alone; shared task-datum and joint-to-board mapping remain to be checked.',
    next_implementation_direction='Improve generic load/feedforward and steady tracking without square-specific XY/Z offsets or increasing shoulder_rotate Kp/Kd; verify against actual joint endpoint and independent field observation before accepting a new path.')
(out/'tracking_postmortem.json').write_text(json.dumps(analysis,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
record=dict(schema=1,experiment_id='20261108_a8_reference_inverse',date_local='2026-11-08',
    recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    status='controller-completed; operator-rejected-height-and-lateral-drift',
    request=prep['request'],request_kind='inverse-geometric path constrained by user-approved saved endpoint',
    physical_board_square='A8',original_nominal_A8_XYZ_solved=False,
    operator_validation=feedback,geometry_ground_truth=False,fitting_eligible=False,
    motion_issued=True,claw_closed=False,same_serial_hold_completed=True,
    encoder_endpoint_rad=audit['endpoint']['measured_pose_rad'],
    saved_endpoint_rad=audit['endpoint']['nominal_target_rad'],
    endpoint_minus_saved_deg=audit['motion_endpoint_minus_saved_deg'],
    evidence=dict(directory=out.relative_to(root).as_posix(),control_log='arm_run_1794185510062_30907.txt',
        control_archive=collection['control'],rgbd_archive=collection['rgbd'],rgbd_pairs=collection['rgb_depth_pairs'],
        local_download_verification=download),
    source_models=prep['source_hashes_before'],active_board_model_sha256=prep['deployed_candidate_sha256'],
    controller_audit=dict(motor_frames=audit['trace']['motor_frames'],
        trace_complete=audit['trace']['complete_without_recording_errors'],
        oscillation_events=audit['trace']['confirmed_oscillation_events'],
        shoulder_rotate_gain_pairs=audit['trace']['stages']['contact']['shoulder_rotate_gain_pairs']),
    learning_and_Clearance_policy_unchanged=True,
    next_work='Correct generic tracking and reconcile task-frame mapping before retesting; no new trajectory executed after rejection.')
registry=root/'scripts/data/board_workspace_trials/validation_20261108_a8_reference_inverse.json'
data=json.dumps(record,ensure_ascii=False,indent=2)+'\n'
registry.write_text(data,encoding='utf-8');(out/'trial_summary.json').write_text(data,encoding='utf-8')
repro=out/'reproducibility';repro.mkdir(exist_ok=True)
for p in (root/'tmp').glob('*a8_reference*20261108.py'):shutil.copyfile(p,repro/p.name)
print(json.dumps(dict(accepted=False,encoder_endpoint_minus_saved_deg=record['endpoint_minus_saved_deg'],
    rgbd_pairs=collection['rgb_depth_pairs'],fitting_eligible=False,registry=str(registry)),ensure_ascii=False))
