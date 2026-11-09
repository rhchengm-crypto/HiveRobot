"""Local archived-evidence decoder for the A8 lower25 geometric trial.

No network, subprocess, motor, or runtime-controller imports. This script uses
the existing stdlib A7 evidence decoder, plus independent plain-Python FK.
Run after root has downloaded and extracted control_evidence with its manifest.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import runpy

ROOT = Path(__file__).resolve().parents[3]
BASE_PATH = Path(__file__).resolve().parent / 'audit_a7_repeat_20261108.py'
BASE = runpy.run_path(str(BASE_PATH), run_name='a7_evidence_decoder')
load, sha, option, tagged = (BASE[n] for n in ('load', 'sha', 'option', 'tagged'))
local_evidence, finite = (BASE[n] for n in ('local_evidence', 'finite'))
EXPECTED_LOG = 'arm_run_1794198443568_49465.txt'
EXPECTED_TRACE = 'trace_1794198451214228486.jsonl'
SOURCE_NAMES = {
    'rigid_model': 'data/left_arm_general_dynamics.json',
    'board_model': 'data/left_arm_board_workspace_candidate.json',
    'planner': 'left_arm_board_grasp.py', 'controller': 'left_arm_v2_6.py',
    'motion_adapter': 'left_arm_v3_1_motion_dynamics.py',
    'fault_support': 'left_arm_fault_support.py',
    'kinematics': 'left_arm_board_kinematics.py',
    'collision': 'left_arm_board_collision.py', 'trace': 'left_arm_motion_trace.py',
    'task_pose': 'left_arm_board_task_pose.py',
    'trajectory_support': 'left_arm_trajectory_support.py',
    'general_motion': 'left_arm_general_motion.py',
}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def qpose(value):
    if not isinstance(value, dict):
        return None
    result = {n: row.get('pos') if isinstance(row, dict) else row for n, row in value.items()}
    return result if result and all(finite(v) for v in result.values()) else None


def differences(actual, target):
    if actual is None or target is None:
        return None
    return {n: math.degrees(float(actual[n])-float(target[n]))
            for n in target if n in actual}


def mm_delta(a, b):
    return [x-y for x, y in zip(a, b)] if a is not None and b is not None else None


def matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(len(b)))
             for j in range(len(b[0]))] for i in range(len(a))]


def fk_board(model, cfg, encoder):
    """Independent serial parent-transform FK; output is MODEL prediction only."""
    current = [[float(i == j) for j in range(4)] for i in range(4)]
    for name in model['joint_order']:
        row = model['joints'][name]
        if row['axis_in_joint_frame'] != [0., 0., 1.]:
            raise ValueError('Independent FK only supports declared local-Z serial axes')
        angle = row['encoder_sign']*(encoder[name]-row['reference_encoder_rad'])
        c, s = math.cos(angle), math.sin(angle)
        motion = [[c, -s, 0., 0.], [s, c, 0., 0.],
                  [0., 0., 1., 0.], [0., 0., 0., 1.]]
        current = matmul(matmul(current, row['parent_to_joint_at_reference']), motion)
    tcp = list(cfg['tcp_in_wrist_frame_m'])+[1.]
    cad = [sum(current[i][j]*tcp[j] for j in range(4)) for i in range(3)]
    delta = [v-t for v, t in zip(cad, cfg['board_to_cad_translation_m'])]
    rotation = cfg['board_to_cad_rotation']
    return [1000.*sum(delta[i]*rotation[i][j] for i in range(3)) for j in range(3)]


def locate_source(evidence, repeat, remote, expected):
    """Prefer archived exact-path bytes; explicitly label any local fallback."""
    basename = Path(str(remote).replace('\\', '/')).name
    for directory in ('source', 'models'):
        archived_path = evidence / directory / basename
        if archived_path.is_file() and sha(archived_path) == expected:
            return archived_path, True
    try:
        found = local_evidence(evidence, remote)
        return found, True
    except ValueError:
        relative = str(remote).replace('\\', '/').split('/DM_Control_Python/', 1)[-1]
        candidates = [repeat / 'board_workspace_candidate.json', ROOT/'scripts'/relative]
        for path in candidates:
            if path.is_file() and sha(path) == expected:
                return path, False
    return None, False


def stage_plan_tracking(path, plan, trace):
    stages, per_waypoint, started = {}, {}, []
    rows = plan['waypoints']
    expected_contact = [i for i, row in enumerate(rows) if row['stage'] == 'contact']
    expected_terminal = rows[-1]['encoder_rad']
    for line in path.open(encoding='utf-8-sig'):
        if not line.strip():
            continue
        row = json.loads(line)
        if row.get('event') == 'trace_started':
            started.append(row)
        if row.get('event') != 'motor_frame':
            continue
        stage = row.get('context', {}).get('stage', 'unknown')
        index = row.get('context', {}).get('waypoint_index')
        goal = rows[index]['encoder_rad'] if isinstance(index, int) and 0 <= index < len(rows) else None
        if stage in ('endpoint_settle', 'model_hold'):
            goal = expected_terminal
        feedback = qpose(row.get('feedback'))
        if goal is None or feedback is None:
            continue
        for key, destination in ((stage, stages), (str(index), per_waypoint)):
            item = destination.setdefault(key, {'frames': 0,
                'peak_distance_to_planned_goal_deg': {},
                'peak_abs_velocity_rad_s': {},
                'first_feedback_rad': feedback, 'first_context': row.get('context')})
            error = differences(feedback, goal)
            item['frames'] += 1
            item['last_feedback_rad'] = feedback
            item['last_context'] = row.get('context')
            item['last_feedback_minus_planned_goal_deg'] = error
            for name, value in error.items():
                item['peak_distance_to_planned_goal_deg'][name] = max(
                    item['peak_distance_to_planned_goal_deg'].get(name, 0.), abs(value))
                velocity = row.get('feedback', {}).get(name, {}).get('vel')
                if finite(velocity):
                    item['peak_abs_velocity_rad_s'][name] = max(
                        item['peak_abs_velocity_rad_s'].get(name, 0.), abs(velocity))
    contact_stage = trace['stages'].get('contact', {})
    observed = contact_stage.get('waypoint_indices', [])
    return {'stages': stages, 'waypoints': per_waypoint, 'trace_started': started,
            'expected_contact_indices': expected_contact,
            'observed_contact_indices': observed,
            'all_planned_contact_waypoints_observed': bool(expected_contact) and set(expected_contact) == set(observed),
            'goal_error_caveat': 'Distance to a segment endpoint includes intentional interpolation travel; sent-reference tracking error is separately reported in trace.stages.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeat', type=Path, default=ROOT/'outputs/a8_lower25_execution_20261108')
    parser.add_argument('--evidence', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--stdout-only', action='store_true')
    parser.add_argument('--allow-other-run', action='store_true',
                        help='Only for parser regression against another archived run')
    args = parser.parse_args()
    repeat = args.repeat.resolve()
    evidence = (args.evidence or repeat/'control_evidence').resolve()
    manifest = BASE['verify_manifest'](evidence)
    status = load(evidence/'web_status.json')
    run = status.get('run', status)
    while isinstance(run.get('preceding_motion'), dict) and run.get('action') != 'dynamics:board-grasp':
        run = run['preceding_motion']
    command = run.get('command', [])
    if not run.get('full_log_path') or '--execute' not in command or '--prepare-execution' in command:
        raise ValueError('Expected an executed board-grasp run, not preparation or an unrelated command')
    log_path = local_evidence(evidence, run['full_log_path'])
    if not args.allow_other_run and log_path.name != EXPECTED_LOG:
        raise ValueError('A8 reference audit was pointed at another motion log')
    log = log_path.read_text(encoding='utf-8-sig')
    headers = tagged(log, 'board motor trace= ')
    completion = tagged(log, 'board motor trace complete= ')
    paths = {v['path'] for v in headers+completion if v.get('path')}
    if len(paths) != 1:
        raise ValueError('Exactly one continuous board trace is required')
    trace_path = local_evidence(evidence, next(iter(paths)))
    if not args.allow_other_run and trace_path.name != EXPECTED_TRACE:
        raise ValueError('A8 reference audit was pointed at another continuous trace')
    trace = BASE['inspect_trace'](trace_path)
    trace['completion_log'] = completion[-1] if completion else None
    closed = trace['completion_log'] or {}
    trace['complete_without_recording_errors'] = bool(
        trace['footer_count'] == 1 and trace['trace_footer_count_matches'] and completion and
        closed.get('frames_written') == trace['records_excluding_close_footer'] and
        not closed.get('dropped_frames') and not closed.get('capture_errors') and
        not closed.get('error') and not trace['max_queue_dropped_frames_observed'] and
        not trace['events'].get('capture_error') and not trace['control_field_problems'])
    prepared_path = local_evidence(evidence, option(command, '--prepared-plan'))
    plan = load(prepared_path)
    metadata = plan.get('prepared_execution', {})
    unsealed = dict(plan)
    unsealed.pop('prepared_execution', None)
    expected_hashes = metadata.get('source_hashes', {})
    source_audits, source_paths = {}, {}
    source_options = {'rigid_model': '--model', 'board_model': '--board-model'}
    for name, expected in expected_hashes.items():
        remote = option(command, source_options[name]) if name in source_options else SOURCE_NAMES.get(name)
        path, archived = locate_source(evidence, repeat, remote, expected) if remote else (None, False)
        item = {'expected_sha256': expected, 'path': str(path) if path else None,
                'archived_source': archived, 'match': bool(path and sha(path) == expected)}
        if path:
            item['actual_sha256'] = sha(path)
            source_paths[name] = path
        source_audits[name] = item
    requests = {'x_mm': float(option(command, '--x-mm')),
                'y_mm': float(option(command, '--y-mm')),
                'piece': option(command, '--piece'), 'stage': option(command, '--stage'),
                'grip_height_mm': float(option(command, '--grip-height-mm'))}
    reports = tagged(log, 'board plan= ')
    actual_plan_report = reports[-1] if reports else {}
    planned_execution = tagged(log, 'board execution groups= ')
    executed_waypoints = tagged(log, 'board waypoint= ')
    plan_output = option(command, '--plan-output')
    actual_output = load(local_evidence(evidence, plan_output)) if plan_output else actual_plan_report
    tracking = stage_plan_tracking(trace_path, plan, trace)
    start_events = tracking['trace_started']
    runtime_hashes = start_events[-1].get('source_hashes', {}) if start_events else {}
    results = BASE['final_results'](log)
    final = results[-1] if results else {}
    endpoints = tagged(log, 'board endpoint report= ')
    endpoint = final.get('endpoint') or (endpoints[-1] if endpoints else {})
    begins = tagged(log, 'board same-session hold begin= ')
    completes = tagged(log, 'board same-session hold complete= ')
    held = final.get('model_hold') or (completes[-1] if completes else {})
    gains = trace['stages'].get('contact', {}).get('shoulder_rotate_gain_pairs', [])
    descent_logs = tagged(log, 'board descent gain continuation= ')
    saved_source = Path(__file__).resolve().parent/'reference_height_context.json'
    saved = load(saved_source)['saved_A8_take_pose']
    terminal = plan['waypoints'][-1]['encoder_rad']
    measured = endpoint.get('measured_pose_rad')
    after_hold = qpose(held.get('final'))
    model_predictions = {'external_measurement': False,
        'caveat': 'All board positions and displacements below are FK predictions from encoders, not camera/physical displacement measurements.'}
    if all(name in source_paths and source_audits[name]['match'] for name in ('rigid_model', 'board_model')):
        model, cfg = load(source_paths['rigid_model']), load(source_paths['board_model'])
        poses = {'planned_terminal': terminal, 'motion_endpoint_feedback': measured,
                 'after_inline_hold_feedback': after_hold, 'saved_A8_take': saved}
        positions = {name: fk_board(model, cfg, q) for name, q in poses.items() if q is not None}
        model_predictions['board_mm'] = positions
        model_predictions['hold_minus_motion_endpoint_mm'] = mm_delta(
            positions.get('after_inline_hold_feedback'), positions.get('motion_endpoint_feedback'))
        for stage, item in tracking['stages'].items():
            first = fk_board(model, cfg, item['first_feedback_rad'])
            last = fk_board(model, cfg, item['last_feedback_rad'])
            item['model_predicted_first_board_mm'] = first
            item['model_predicted_last_board_mm'] = last
            item['model_predicted_displacement_mm'] = mm_delta(last, first)
    else:
        model_predictions['unavailable_reason'] = 'Hash-matched rigid/board model bytes were not available'
    proof = {'plan_digest_matches': metadata.get('plan_sha256') == digest(unsealed),
        'request_matches_actual_command': metadata.get('request') == requests,
        'plan_target_matches_command_xyz': plan.get('target_board_mm') == [requests['x_mm'], requests['y_mm'], requests['grip_height_mm']],
        'all_source_bytes_match': len(source_audits) == len(SOURCE_NAMES) and all(v['match'] for v in source_audits.values()),
        'all_source_bytes_archived': bool(source_audits) and all(v['archived_source'] for v in source_audits.values()),
        'trace_runtime_sources_match_prepared': runtime_hashes == expected_hashes,
        'runtime_metadata_matches_prepared': actual_output.get('prepared_execution') == metadata,
        'runtime_precomputed_before_motion': actual_output.get('precomputed_before_motion') is True,
        'runtime_waypoint_geometry_unchanged': actual_output.get('waypoints') == plan.get('waypoints'),
        'runtime_no_feedback_descent_replan': not tagged(log, 'board feedback descent entry= '),
        'actual_execution_complete_precomputed': bool(final.get('ok') and run.get('returncode') == 0
            and not run.get('running') and actual_output.get('precomputed_before_motion') is True
            and tracking['all_planned_contact_waypoints_observed']),
        'planned_gravity_geometry_hash_matches': plan.get('planned_gravity_envelope', {}).get('path_sha256') == digest(
            {'start_rad': plan['start_rad'], 'joints': [r['encoder_rad'] for r in plan['waypoints']]}),
        'planned_terminal_is_new_solution_not_saved_pose': saved is not None and terminal != saved,
        'all_planned_contact_waypoints_observed': tracking['all_planned_contact_waypoints_observed'],
        'contact_shoulder_rotate_gain_frozen': len(gains) == 1,
        'contact_shoulder_rotate_Kp_at_most_36': bool(gains) and all(v[0] <= 36.+1e-8 for v in gains),
        'contact_shoulder_rotate_Kd_4_5': bool(gains) and all(abs(v[1]-4.5) <= 1e-8 for v in gains),
        'same_session_inline_hold_completed': bool(begins and completes and held.get('ok') and
            begins[-1].get('same_serial_session') and held.get('serial_session_reused'))}
    summary = {'audited_at_utc': datetime.now(timezone.utc).isoformat(),
        'motion_issued_by_audit': False, 'network_accessed': False, 'runtime_code_imported': False,
        'audit_helper_dependency': {'path': str(BASE_PATH), 'sha256': sha(BASE_PATH)},
        'manifest_verification': manifest, 'checks': proof,
        'source_file_verification': source_audits, 'prepared_plan': {'path': str(prepared_path), 'sha256': sha(prepared_path)},
        'request': requests, 'request_kind': 'explicit A8-reference model XYZ lower25 with newly requested full wrist frame',
        'original_nominal_A8_XYZ_solved': False,
        'original_nominal_A8_XYZ_mm': [27.5, 412.5, 50.2],
        'web_run': {k: run.get(k) for k in ('action', 'ok', 'running', 'support_running', 'returncode', 'duration_s', 'command', 'full_log_path')},
        'motion_log': {'path': str(log_path), 'sha256': sha(log_path)},
        'trace': trace, 'stage_and_waypoint_tracking': tracking,
        'execution_groups': planned_execution, 'executed_waypoints': executed_waypoints,
        'descent_gain_logs': descent_logs,
        'endpoint_settle_logs': tagged(log, 'board endpoint settle= '),
        'endpoint': endpoint,
        'saved_reference_source': {'path': str(saved_source), 'sha256': sha(saved_source)} if saved is not None else None,
        'motion_endpoint_minus_saved_deg': differences(measured, saved),
        'after_inline_hold_minus_saved_deg': differences(after_hold, saved),
        'model_predictions_not_external_measurements': model_predictions,
        'same_session_hold_begin': begins, 'inline_hold': held,
        'post_move_model_hold': final.get('post_move_model_hold') or run.get('post_move_model_hold'),
        'protective_hold_logs': tagged(log, 'board protective hold= '),
        'fault_support_event_count': sum(v for k, v in trace['events'].items() if 'fault' in k or 'oscillation' in k),
        'fault_support_frames': trace.get('fault_support_frames'),
        'operator_feedback': 'Pending separate operator observation; encoder data do not certify physical XY drift or absence of vibration/contact.',
        'physical_validation_accepted': False, 'fitting_eligible': False,
        'scope': 'A8 lower25 new-frame geometric motion plus inline same-serial hold only. Ordinary Web Home/Clearance and their separate holds must be audited separately.',
        'command_result_ok': final.get('ok')}
    destination = args.output or repeat/'execution_audit.json'
    if not args.stdout_only:
        destination.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'checks': proof, 'motor_frames': trace['motor_frames'],
        'trace_complete': trace['complete_without_recording_errors'],
        'oscillation_events': trace['confirmed_oscillation_events'],
        'fault_support_frames': trace['fault_support_frames'], 'contact_gain_pairs': gains,
        'endpoint_minus_saved_deg': summary['motion_endpoint_minus_saved_deg'],
        'model_hold_ok': held.get('ok'), 'command_result_ok': final.get('ok'),
        'output': None if args.stdout_only else str(destination)}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
