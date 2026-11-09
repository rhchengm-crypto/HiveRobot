"""Local evidence audit only: no motor imports, networking, or subprocesses.

Usage: python -X utf8 tmp/audit_a7_repeat_20261108.py
       python -X utf8 tmp/audit_a7_repeat_20261108.py --repeat DIR --stdout-only
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[3]
DECODER = json.JSONDecoder()


def load(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def local_evidence(folder, remote):
    """Map an archived Jetson path, rejecting traversal and ambiguous names."""
    remote = str(remote).replace('\\', '/')
    relative = remote.split('/DM_Control_Python/', 1)[-1]
    if relative.startswith('/'):
        relative = relative.split('/data/', 1)[-1]
        relative = 'data/' + relative if '/data/' in remote else Path(remote).name
    candidate = (folder / relative).resolve()
    if folder.resolve() in candidate.parents and candidate.is_file():
        return candidate
    matches = list(folder.rglob(Path(remote).name))
    matches = [p for p in matches if p.is_file()]
    if len(matches) != 1:
        raise ValueError('Evidence path missing or ambiguous: ' + remote)
    return matches[0]


def verify_manifest(folder):
    manifest_path = folder / 'manifest.json'
    manifest = load(manifest_path)
    files = manifest.get('files', manifest)
    verified = []
    for relative, expected in files.items():
        path = (folder / relative).resolve()
        if folder.resolve() not in path.parents:
            raise ValueError('Manifest traversal rejected')
        if isinstance(expected, str):
            expected = {'sha256': expected}
        if not path.is_file() or ('bytes' in expected and path.stat().st_size != expected['bytes']):
            raise ValueError('Manifest file missing or wrong length: ' + relative)
        if sha(path) != expected['sha256']:
            raise ValueError('Manifest SHA256 differs: ' + relative)
        verified.append(relative)
    return dict(path=str(manifest_path), sha256=sha(manifest_path),
                checked_files=len(verified), all_match=True, files=verified)


def option(command, name):
    return command[command.index(name)+1] if name in command else None


def tagged(log, prefix):
    result = []
    for line in log.splitlines():
        if line.startswith(prefix):
            value, _ = DECODER.raw_decode(line[len(prefix):])
            result.append(value)
    return result


def final_results(log):
    result = []
    offset = 0
    for line in log.splitlines(keepends=True):
        if line.lstrip().startswith('{'):
            try:
                value, _ = DECODER.raw_decode(log, offset + len(line)-len(line.lstrip()))
            except ValueError:
                value = None
            if isinstance(value, dict) and ('endpoint' in value or 'post_move_model_hold' in value):
                result.append(value)
        offset += len(line)
    return result


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered)-1)*fraction
    lo = int(index)
    hi = min(lo+1, len(ordered)-1)
    return ordered[lo]+(ordered[hi]-ordered[lo])*(index-lo)


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def inspect_trace(path):
    events = Counter()
    stages = {}
    footer = []
    problems = []
    records = 0
    motor_frames = 0
    max_dropped = 0
    first_contact = None
    previous_motor = None
    fault_diagnostics = []
    with path.open(encoding='utf-8-sig') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            records += 1
            event = row.get('event', 'unknown')
            events[event] += 1
            if event == 'trace_closed':
                footer.append(row)
            if event == 'confirmed_encoder_oscillation':
                fault_diagnostics.append(row)
            dropped = row.get('queue_dropped_frames_before_submit')
            if finite(dropped):
                max_dropped = max(max_dropped, dropped)
            if event != 'motor_frame':
                continue
            motor_frames += 1
            stage = row.get('context', {}).get('stage', 'unknown')
            current = stages.setdefault(stage, dict(frames=0, first_wall_s=row.get('wall_s'),
                last_wall_s=None, first_monotonic_s=row.get('monotonic_s'), last_monotonic_s=None,
                waypoint_indices=set(), gain_pairs={}, peak_abs_velocity_rad_s={},
                peak_abs_tracking_error_deg={}, frame_intervals=[],
                first_feedback=row.get('feedback'), last_feedback=None,
                first_commands=row.get('commands'), last_commands=None))
            current['frames'] += 1
            current['last_wall_s'] = row.get('wall_s')
            current['last_monotonic_s'] = row.get('monotonic_s')
            current['last_feedback'] = row.get('feedback')
            current['last_commands'] = row.get('commands')
            waypoint = row.get('context', {}).get('waypoint_index')
            if waypoint is not None:
                current['waypoint_indices'].add(waypoint)
            interval = row.get('frame_interval_ms')
            if finite(interval):
                current['frame_intervals'].append(interval)
            for name, feedback in row.get('feedback', {}).items():
                command = row.get('commands', {}).get(name, {})
                needed = [feedback.get('pos'), feedback.get('vel'), command.get('q'),
                          command.get('kp'), command.get('kd')]
                if not all(finite(v) for v in needed):
                    if len(problems) < 20:
                        problems.append(dict(line=line_number, joint=name, issue='missing-or-nonfinite-control-fields'))
                    continue
                current['gain_pairs'].setdefault(name, set()).add((command['kp'], command['kd']))
                current['peak_abs_velocity_rad_s'][name] = max(
                    current['peak_abs_velocity_rad_s'].get(name, 0.), abs(feedback['vel']))
                current['peak_abs_tracking_error_deg'][name] = max(
                    current['peak_abs_tracking_error_deg'].get(name, 0.),
                    abs(math.degrees(feedback['pos']-command['q'])))
            if stage == 'contact' and first_contact is None:
                first_contact = dict(first_contact_commands=row.get('commands'),
                                     preceding_motor_commands=None if previous_motor is None else previous_motor.get('commands'),
                                     preceding_context=None if previous_motor is None else previous_motor.get('context'))
            previous_motor = row
    for stage, row in stages.items():
        row['waypoint_indices'] = sorted(row['waypoint_indices'])
        row['gain_pairs'] = {n: sorted(values) for n,values in row['gain_pairs'].items()}
        row['shoulder_rotate_gain_pairs'] = row['gain_pairs'].get('shoulder_rotate', [])
        intervals = row.pop('frame_intervals')
        row['frame_interval_ms'] = dict(p50=percentile(intervals, .5),
            p99=percentile(intervals, .99), max=max(intervals) if intervals else None)
        a,b = row['first_monotonic_s'],row['last_monotonic_s']
        row['elapsed_s'] = b-a if finite(a) and finite(b) else None
    if first_contact:
        before = first_contact['preceding_motor_commands'] or {}
        first_contact['same_gains_at_contact_entry'] = {
            n: all(before.get(n, {}).get(k) == command.get(k) for k in ('kp', 'kd'))
            for n,command in first_contact['first_contact_commands'].items()}
    closed = footer[-1] if footer else None
    return dict(path=str(path), sha256=sha(path), json_records=records,
        records_excluding_close_footer=records-len(footer), motor_frames=motor_frames,
        events=dict(events), stages=stages, trace_closed=closed, footer_count=len(footer),
        trace_footer_count_matches=(closed is not None and closed.get('frames_written')==records-len(footer)),
        max_queue_dropped_frames_observed=max_dropped, control_field_problems=problems,
        confirmed_oscillation_events=events.get('confirmed_encoder_oscillation', 0),
        oscillation_diagnostics=fault_diagnostics, fault_support_frames=stages.get('fault_support', {}).get('frames', 0),
        contact_entry=first_contact)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeat', type=Path, default=ROOT/'outputs/a7_repeat_20261108')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--stdout-only', action='store_true')
    args = parser.parse_args()
    repeat = args.repeat.resolve()
    evidence = repeat/'control_evidence'
    manifest = verify_manifest(evidence)
    status = load(evidence/'web_status.json')
    run = status.get('run', status)
    while isinstance(run.get('preceding_motion'), dict) and run.get('action')!='dynamics:board-grasp':
        run = run['preceding_motion']
    command = run.get('command', [])
    if not run.get('full_log_path'):
        raise ValueError('web_status.json has no executed motion log path')
    if '--prepare-execution' in command or ('--execute' not in command and command):
        raise ValueError('Collected status refers to a preview rather than an executed motion')
    log_path = local_evidence(evidence, run['full_log_path'])
    log = log_path.read_text(encoding='utf-8-sig')
    trace_headers = tagged(log, 'board motor trace= ')
    trace_completes = tagged(log, 'board motor trace complete= ')
    paths = [x['path'] for x in trace_headers+trace_completes if x.get('path')]
    if not paths:
        paths = re.findall(r'[^\s"\x27]+/board_motion_traces/trace_[A-Za-z0-9_.-]+\.jsonl', log)
    if not paths or len({Path(x).name for x in paths})!=1:
        raise ValueError('Motion log must identify exactly one complete-control trace')
    trace = inspect_trace(local_evidence(evidence, paths[0]))
    verified_path = repeat/'verified_plan.json'
    verified = load(verified_path) if verified_path.exists() else {}
    plan_reference = option(command, '--prepared-plan')
    if plan_reference is None:
        for key in ('prepared_plan', 'prepared_plan_path', 'plan_path', 'plan_file'):
            if isinstance(verified.get(key), str):
                plan_reference = verified[key]
                break
    if plan_reference is None:
        plan_reference = option(command, '--plan-output')
    if plan_reference is not None:
        plan_path = local_evidence(evidence, plan_reference)
        plan = load(plan_path)
        plan_source = dict(path=str(plan_path), sha256=sha(plan_path))
    elif isinstance(verified.get('waypoints'), list):
        plan = verified
        plan_source = dict(path=str(verified_path), sha256=sha(verified_path))
    else:
        raise ValueError('Cannot derive the prepared geometric plan from verified_plan or command')
    results = final_results(log)
    final = results[-1] if results else {}
    endpoint_reports = tagged(log, 'board endpoint report= ')
    hold_begins = tagged(log, 'board same-session hold begin= ')
    hold_completes = tagged(log, 'board same-session hold complete= ')
    sessions = tagged(log, 'general motion complete= ')
    held = final.get('model_hold') or (hold_completes[-1] if hold_completes else None)
    post_hold = final.get('post_move_model_hold') or run.get('post_move_model_hold')
    completion = trace_completes[-1] if trace_completes else None
    trace['completion_log'] = completion
    trace['complete_without_recording_errors'] = bool(
        trace['footer_count']==1 and trace['trace_footer_count_matches'] and completion and
        completion.get('frames_written')==trace['records_excluding_close_footer'] and
        not completion.get('dropped_frames') and not completion.get('capture_errors') and
        not completion.get('error') and not trace['max_queue_dropped_frames_observed'] and
        not trace['events'].get('capture_error') and not trace['control_field_problems'])
    summary = dict(audited_at_utc=datetime.now(timezone.utc).isoformat(),
        motion_issued_by_audit=False, network_accessed=False, runtime_code_imported=False,
        build=plan.get('build'), request_board_mm=plan.get('target_board_mm'),
        prepared_plan=plan_source, prepared_request=plan.get('prepared_execution', {}).get('request'),
        prepared_source_hashes=plan.get('prepared_execution', {}).get('source_hashes'),
        web_run={k:run.get(k) for k in ('action','ok','running','support_running','returncode','duration_s','command','full_log_path')},
        motion_log=dict(path=str(log_path), sha256=sha(log_path)), manifest_verification=manifest,
        trace=trace, contact=trace['stages'].get('contact'),
        descent_gain_logs=tagged(log, 'board descent gain continuation= '),
        endpoint_settle_log=tagged(log, 'board endpoint settle= '),
        endpoint=final.get('endpoint') or (endpoint_reports[-1] if endpoint_reports else None),
        protective_hold_log=tagged(log, 'board protective hold= '),
        same_session_hold_begin=hold_begins, model_hold=held, post_move_model_hold=post_hold,
        same_serial_session_confirmed=bool(hold_begins and hold_begins[-1].get('same_serial_session')
            and held and held.get('ok') and hold_completes),
        command_result_ok=final.get('ok'),
        motion_sessions=[{k:s.get(k) for k in ('mode','frames','prediction_status',
            'worker_error_frames','stale_worker_frames','peak_applied_delta_tau_Nm')} for s in sessions],
        operator_feedback='pending; must be recorded separately by root from this repeat',
        physical_validation_accepted=False, fitting_eligible=False,
        measurement_caveat='Encoder and command tracking cannot certify physical XY drift, fingertip height, contact, or absence of mechanical vibration.')
    if not args.stdout_only:
        destination=args.output or repeat/'execution_audit.json'
        destination.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(build=summary['build'], request_board_mm=summary['request_board_mm'],
        motor_frames=trace['motor_frames'], trace_complete=trace['complete_without_recording_errors'],
        confirmed_oscillation_events=trace['confirmed_oscillation_events'],
        fault_support_frames=trace['fault_support_frames'],
        contact_shoulder_rotate_gain_pairs=None if summary['contact'] is None else summary['contact']['shoulder_rotate_gain_pairs'],
        model_hold_ok=None if held is None else held.get('ok'),
        same_serial_session_confirmed=summary['same_serial_session_confirmed'],
        command_result_ok=summary['command_result_ok'], output=None if args.stdout_only else str(destination)),
        ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
