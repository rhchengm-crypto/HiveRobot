"""Local A8 lower25 archived-control audit; no network, SDK, or motor operations.

Run only after control_evidence has been downloaded/extracted. Reuses the
established A8 trace audit, adding the thirteenth sealed source and reproducing
accepted quasi-static windows with the frozen pure observer implementation.
"""
import argparse
from collections import Counter, deque
import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import runpy
from statistics import median
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[3]
BASE_PATH = Path(__file__).resolve().parent/'audit_a8_reference_execution_20261108.py'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeat', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    repeat = args.repeat.resolve()
    evidence = repeat/'control_evidence'
    namespace = runpy.run_path(str(BASE_PATH), run_name='a8_reference_audit_decoder')
    globals_ = namespace['main'].__globals__
    load, sha = globals_['load'], globals_['sha']
    resolve = globals_['local_evidence']
    status = load(evidence/'web_status.json')
    run = status.get('run', status)
    while isinstance(run.get('preceding_motion'), dict) and run.get('action') != 'dynamics:board-grasp':
        run = run['preceding_motion']
    # Bind this audit to the collected launch, not an arbitrary latest Web run.
    launches = list(evidence.rglob('execution_start.json'))+list(evidence.rglob('execute_start.json'))
    matching = []
    for path in launches:
        row = load(path)
        row = row.get('run', row)
        submitted, actual = row.get('command'), run.get('command')
        if actual == submitted or actual == (submitted or [])+['--inline-model-hold']:
            matching.append(path)
    if not matching:
        raise ValueError('Collected executed command does not match this trial launch')
    log_path = resolve(evidence, run['full_log_path'])
    log = log_path.read_text(encoding='utf-8-sig')
    headers = globals_['tagged'](log, 'board motor trace= ')
    if len(headers) != 1:
        raise ValueError('Expected one board motor trace')
    trace_path = resolve(evidence, headers[0]['path'])
    if log_path.name != 'arm_run_1794198443568_49465.txt' or trace_path.name != 'trace_1794198451214228486.jsonl':
        raise ValueError('Evidence is not the exact authorized lower25 motion/trace')
    globals_['EXPECTED_LOG'] = log_path.name
    globals_['EXPECTED_TRACE'] = trace_path.name
    globals_['SOURCE_NAMES']['static_support'] = 'left_arm_static_support.py'
    # Prefer the collector's explicit source/model freeze instead of matching
    # ambiguous same-name pre-motion snapshots elsewhere in the context.
    original_locator = globals_['locate_source']
    def locate_archived(folder, trial, remote, expected):
        basename = Path(str(remote).replace('\\', '/')).name
        for prefix in ('source', 'models'):
            path = folder/prefix/basename
            if path.is_file() and sha(path) == expected:
                return path, True
        return original_locator(folder, trial, remote, expected)
    globals_['locate_source'] = locate_archived
    destination = (args.output or repeat/'execution_audit.json').resolve()
    old_argv = sys.argv
    sys.argv = [str(BASE_PATH), '--repeat', str(repeat), '--output', str(destination)]
    captured = io.StringIO()
    try:
        with contextlib.redirect_stdout(captured):
            namespace['main']()
    finally:
        sys.argv = old_argv
    report = load(destination)
    prepared = load(Path(report['prepared_plan']['path']))
    report['build'] = prepared.get('build')
    if report['build'] != 'seven-axis-board-grasp-v33-quasistatic-support':
        raise ValueError('Collected plan is not the expected v33 build')
    frozen = evidence/'source/left_arm_static_support.py'
    source_check = report['source_file_verification']['static_support']
    if not source_check['match'] or sha(frozen) != source_check['expected_sha256']:
        raise ValueError('Frozen static observer differs from prepared execution source')
    spec = importlib.util.spec_from_file_location('archived_v33_static_support', frozen)
    observer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(observer)
    recent = deque(maxlen=128)
    fresh_frames = Counter()
    rejection_reasons = Counter()
    observations, reproduced = [], []
    sequence_counts = Counter()
    contact_gains = {}
    packet_age_maximum_s = {}
    freshness_field_errors = 0
    prior_sequences = {}
    for line in trace_path.open(encoding='utf-8-sig'):
        row = json.loads(line)
        event = row.get('event')
        if event == 'motor_frame':
            recent.append(row)
            for name, age in row.get('feedback_age_s', {}).items():
                if not isinstance(age, (int, float)) or not globals_['finite'](age):
                    freshness_field_errors += 1
                else:
                    packet_age_maximum_s[name] = max(packet_age_maximum_s.get(name, 0.), age)
            if row.get('context', {}).get('stage') == 'contact':
                for name, command in row.get('commands', {}).items():
                    contact_gains.setdefault(name, set()).add((command['kp'], command['kd']))
            stage = row.get('context', {}).get('stage', 'unknown')
            fresh_frames[(stage, bool(row.get('individual_feedback_freshness_verified')))] += 1
            for name, sequence in row.get('feedback_sequence', {}).items():
                if name in prior_sequences:
                    sequence_counts['regression' if sequence < prior_sequences[name] else
                                    'progress' if sequence > prior_sequences[name] else 'same-cache'] += 1
                prior_sequences[name] = sequence
        elif event == 'quasistatic_support_observation':
            diagnostic = row.get('diagnostics', {})
            obs = diagnostic.get('observation', {})
            item = {key: diagnostic.get(key) for key in ('stage', 'updated', 'reason',
                'bias_tau', 'previous_bias_tau', 'window_end_s', 'applied_to_motor', 'application')}
            item['accepted'] = obs.get('accepted')
            observations.append(item)
            if not obs.get('accepted'):
                rejection_reasons[obs.get('reason', 'unknown')] += 1
                continue
            names = tuple(obs['positions'])
            times = [obs['feedback_received_monotonic_s'][name]+obs['feedback_age_s'][name] for name in names]
            observation_now = median(times)
            targets = {name: recent[-1]['commands'][name]['q'] for name in names}
            again = observer.stable_support_observation(recent, names, targets, observation_now)
            fields_match = again.get('accepted') is True and all(
                again.get(field) == obs.get(field) for field in
                ('window_start_s', 'window_end_s', 'frames', 'positions', 'velocities', 'torques',
                 'feedback_sequence_start', 'feedback_sequence'))
            reproduced.append({'window_end_s': obs['window_end_s'], 'stage': diagnostic.get('stage'),
                'accepted_again': again.get('accepted'), 'reason': again.get('reason'),
                'same_observation_fields': fields_match})
    freshness = {}
    for (stage, fresh), count in fresh_frames.items():
        freshness.setdefault(stage, {'verified_frames': 0, 'unverified_frames': 0})[
            'verified_frames' if fresh else 'unverified_frames'] += count
    endpoint_support = report.get('endpoint', {}).get('quasistatic_support')
    ramp_logs = globals_['tagged'](log, 'board endpoint settle= ')+globals_['tagged'](log, 'board waypoint= ')
    ramp_timing = [row['timing'] for row in ramp_logs
                   if row.get('timing', {}).get('support_baseline_slew_limit_Nm_s') is not None]
    report['v33_quasistatic_support_audit'] = {
        'source': {'path': str(frozen), 'sha256': sha(frozen)},
        'monitor_and_collector_used_no_motor_commands': True,
        'feedback_freshness_by_stage': freshness,
        'packet_sequence_transition_counts': dict(sequence_counts),
        'observations': observations,
        'accepted_observations': sum(bool(v.get('accepted')) for v in observations),
        'bias_updates': sum(bool(v.get('updated')) for v in observations),
        'rejected_reasons': dict(rejection_reasons),
        'accepted_window_reproduction': reproduced,
        'every_accepted_window_reproduced': bool(reproduced) and all(v['same_observation_fields'] for v in reproduced),
        'endpoint_estimated_applied_pending': endpoint_support,
        'baseline_ramp_timings': ramp_timing,
        'slew_scope': 'Support-baseline smoothstep only; adapter gravity and dynamic residual retain their own slew.',
        'audit_does_not_change_persistent_models_or_geometry': True,
        'physical_improvement_certified': False,
        'operator_observation_still_required': True}
    # This trial has 21 contact waypoints, not the seven in the earlier inverse trial.
    report['contact_gain_pairs_all_axes'] = {n: sorted(list(v)) for n, v in contact_gains.items()}
    report['v33_quasistatic_support_audit']['packet_age_maximum_s'] = packet_age_maximum_s
    report['v33_quasistatic_support_audit']['freshness_field_errors'] = freshness_field_errors
    report['checks'].update(
        all_contact_axis_gains_frozen=len(contact_gains) == 7 and all(len(v) == 1 for v in contact_gains.values()),
        all_accepted_static_windows_reproduced=report['v33_quasistatic_support_audit']['every_accepted_window_reproduced'],
        packet_sequence_never_regressed=sequence_counts.get('regression', 0) == 0,
        static_observer_source_hash_verified=source_check['match'],
        exact_trial_board_config_sealed=report['source_file_verification']['board_model']['expected_sha256'] ==
            'b64370a7c0e4264c65b085b6b0db15fbe6f1ce1be66a489260e8640a988c1301')
    context = load(Path(__file__).resolve().parent/'reference_height_context.json')
    prediction = report['model_predictions_not_external_measurements']
    current = prediction.get('board_mm', {})
    old = context['previous_v33_model_predictions']['board_mm']
    requested = report['request']
    goal = [requested['x_mm'], requested['y_mm'], requested['grip_height_mm']]
    prediction['requested_target_board_mm'] = goal
    prediction['motion_endpoint_minus_requested_mm'] = globals_['mm_delta'](current.get('motion_endpoint_feedback'), goal)
    prediction['after_hold_minus_requested_mm'] = globals_['mm_delta'](current.get('after_inline_hold_feedback'), goal)
    prediction['previous_v33_model_height_context'] = context
    prediction['motion_endpoint_minus_previous_v33_endpoint_mm'] = globals_['mm_delta'](
        current.get('motion_endpoint_feedback'), old.get('motion_endpoint_feedback'))
    prediction['after_hold_minus_previous_v33_after_hold_mm'] = globals_['mm_delta'](
        current.get('after_inline_hold_feedback'), old.get('after_inline_hold_feedback'))
    # Independently transform the complete frozen CAD gripper vertices. This
    # is a model envelope for one STEP opening, not measured fingertip height.
    model = load(Path(report['source_file_verification']['rigid_model']['path']))
    cfg = load(Path(report['source_file_verification']['board_model']['path']))
    def lowest_vertex(encoder):
        current = [[float(i == j) for j in range(4)] for i in range(4)]
        for name in model['joint_order']:
            joint = model['joints'][name]
            if joint['axis_in_joint_frame'] != [0., 0., 1.]:
                raise ValueError('Independent envelope FK expects declared local-Z axes')
            angle = joint['encoder_sign']*(encoder[name]-joint['reference_encoder_rad'])
            cosine, sine = math.cos(angle), math.sin(angle)
            turn = [[cosine,-sine,0.,0.],[sine,cosine,0.,0.],
                    [0.,0.,1.,0.],[0.,0.,0.,1.]]
            current = globals_['matmul'](globals_['matmul'](
                current, joint['parent_to_joint_at_reference']), turn)
        normal = [row[2] for row in cfg['board_to_cad_rotation']]
        origin = cfg['board_to_cad_translation_m']
        heights = []
        for vertex in cfg['gripper_vertex_envelope']['vertices_in_wrist_frame_m']:
            point = [sum(current[i][j]*vertex[j] for j in range(3))+current[i][3]
                     for i in range(3)]
            heights.append(1000.*sum((point[i]-origin[i])*normal[i] for i in range(3)))
        index = min(range(len(heights)), key=heights.__getitem__)
        return {'minimum_vertex_height_mm':heights[index], 'vertex_index':index}
    vertex_poses = {
        'old_saved_reference_planned_pose': context['saved_A8_take_pose'],
        'new_planned_terminal': prepared['waypoints'][-1]['encoder_rad'],
        'motion_endpoint_feedback': report['endpoint']['measured_pose_rad'],
        'after_inline_hold_feedback': globals_['qpose'](report['inline_hold']['final'])}
    minima = {name:lowest_vertex(q) for name,q in vertex_poses.items()}
    old_min = minima['old_saved_reference_planned_pose']['minimum_vertex_height_mm']
    new_min = minima['new_planned_terminal']['minimum_vertex_height_mm']
    old_tcp = old['saved_A8_take'][2]
    new_tcp = current['planned_terminal'][2]
    prediction['gripper_vertex_model'] = {
        'external_measurement':False,
        'opening_scope':cfg['gripper_vertex_envelope']['opening_scope'],
        'qualifier':'Frozen complete-CAD gripper opening only; not measured finger or calibrated clamp height.',
        'pose_minima':minima,
        'new_plan_minus_old_reference_lowest_vertex_mm':new_min-old_min,
        'new_plan_minus_old_reference_TCP_height_mm':new_tcp-old_tcp,
        'new_orientation_relative_lowpoint_height_change_mm':
            (new_min-new_tcp)-(old_min-old_tcp)}
    feedback_path = repeat/'operator_feedback.json'
    if not feedback_path.is_file():
        raise ValueError('Expected the separately recorded operator feedback')
    feedback = load(feedback_path)
    report['operator_feedback'] = {
        'path':str(feedback_path),'sha256':sha(feedback_path),
        'verbatim':feedback['verbatim'],'followup_verbatim':feedback.get('followup_verbatim'),
        'physical_observation':True,'not_calibrated_physical_metrology':True,
        'motion_stability_accepted':feedback.get('motion_stability_accepted'),
        'lateral_drift_observed':feedback.get('lateral_drift_observed'),
        'jitter':feedback.get('jitter'),'contact':feedback.get('contact'),
        'orientation_acceptable':feedback.get('orientation_acceptable'),
        'hold_extra_movement':feedback.get('hold_extra_movement'),
        'full_motion_accepted':feedback.get('full_motion_accepted')}
    report['physical_height_acceptance'] = {
        'passed':False, 'physical_TCP_measured':False,
        'operator_reported_approximate_remaining_gap_mm':feedback['visual_gap_above_gripping_region_mm'],
        'observation':feedback['verbatim'],
        'other_physical_flags_confirmed':True,
        'no_measured_descent_inferred_from_prior_gap_lower_bound':True,
        'fitting_eligible':False,'reference_library_eligible':False}
    report['physical_validation_accepted'] = False
    report['fitting_eligible'] = False
    report['audit_helper_files'] = {str(path.relative_to(ROOT)): sha(path)
        for path in (Path(__file__).resolve(), BASE_PATH, BASE_PATH.parent/'audit_a7_repeat_20261108.py')}
    collection = evidence/'result_summary.json'
    if collection.exists():
        record = load(collection)
        report['collection_phase_and_learning_summary'] = {key: record.get(key) for key in
            ('phase_logs', 'prepared_source_count', 'pre_motion_protected_snapshot_count',
             'learning_snapshots', 'camera', 'ordinary_Home_Clearance_limitations')}
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'output': str(destination), 'checks': report['checks'],
        'motor_frames': report['trace']['motor_frames'],
        'trace_complete': report['trace']['complete_without_recording_errors'],
        'oscillation_events': report['trace']['confirmed_oscillation_events'],
        'fault_support_frames': report['trace']['fault_support_frames'],
        'static_observations': len(observations),
        'accepted_static_windows': len(reproduced),
        'all_accepted_windows_reproduced': report['v33_quasistatic_support_audit']['every_accepted_window_reproduced'],
        'endpoint_max_error_deg': report.get('endpoint', {}).get('max_nominal_error_deg'),
        'estimated_applied_pending_bias': endpoint_support,
        'hold_ok': report.get('inline_hold', {}).get('ok'),
        'physical_drift_solved_by_audit': False}, ensure_ascii=False, separators=(',', ':')))


if __name__ == '__main__':
    main()
