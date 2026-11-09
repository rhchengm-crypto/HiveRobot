"""One compact motor-read-only v33 poll; HTTP GET only, no serial/commands.

Run on Jetson only when requested. Saves status and incremental trace counters
under this trial's context. It neither starts nor stops the camera or arm.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import time
import urllib.request

ROOT = Path('/home/nvidia/hive_robot/DM_Control_Python')
CONTEXT = ROOT/'data/board_workspace_trials/a8_lower25_execution_20261108'
DECODER = json.JSONDecoder()


def tagged(text, prefix):
    result = []
    for line in text.splitlines():
        if line.startswith(prefix):
            try:
                result.append(DECODER.raw_decode(line[len(prefix):])[0])
            except ValueError:
                pass
    return result


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def project_path(value):
    value = Path(value)
    path = (value if value.is_absolute() else ROOT/value).resolve()
    if ROOT.resolve() not in path.parents:
        raise ValueError('Evidence path escapes robot project')
    return path


def trace_increment(path, previous):
    state = previous if previous.get('path') == str(path) else {'path': str(path), 'offset': 0,
        'events': {}, 'stage_frames': {}, 'peak_velocity_rad_s': {}, 'peak_tracking_error_deg': {},
        'max_dropped': 0, 'writer_error': None, 'latest_frame': None, 'latest_static_update': None}
    if state.get('offset', 0) > path.stat().st_size:
        raise ValueError('Continuous trace was truncated; refuse to merge incompatible evidence')
    with path.open('rb') as stream:
        stream.seek(state.get('offset', 0))
        data = stream.read(8*1024*1024)
    boundary = data.rfind(b'\n')+1
    complete = data[:boundary]
    state['offset'] = state.get('offset', 0)+len(complete)
    for line in complete.splitlines():
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except ValueError:
            state['parse_errors'] = state.get('parse_errors', 0)+1
            continue
        event = row.get('event', 'unknown')
        state['events'][event] = state['events'].get(event, 0)+1
        if event == 'quasistatic_support_observation':
            diagnostic = row.get('diagnostics', {})
            observation = diagnostic.get('observation', {})
            state['latest_static_update'] = {k: diagnostic.get(k) for k in
                ('stage', 'updated', 'reason', 'bias_tau', 'applied_to_motor', 'application', 'window_end_s')}
            state['latest_static_update']['observation_accepted'] = observation.get('accepted')
            if diagnostic.get('updated'):
                state['accepted_bias_updates'] = state.get('accepted_bias_updates', 0)+1
        if event == 'trace_closed':
            state['footer'] = row
        if event != 'motor_frame':
            continue
        stage = row.get('context', {}).get('stage', 'unknown')
        state['stage_frames'][stage] = state['stage_frames'].get(stage, 0)+1
        error, speeds = {}, {}
        for name, feedback in row.get('feedback', {}).items():
            command = row.get('commands', {}).get(name, {})
            if finite(feedback.get('vel')):
                speeds[name] = feedback['vel']
                state['peak_velocity_rad_s'][name] = max(state['peak_velocity_rad_s'].get(name, 0.), abs(feedback['vel']))
            if finite(feedback.get('pos')) and finite(command.get('q')):
                error[name] = math.degrees(feedback['pos']-command['q'])
                state['peak_tracking_error_deg'][name] = max(state['peak_tracking_error_deg'].get(name, 0.), abs(error[name]))
        state['max_dropped'] = max(state['max_dropped'], row.get('queue_dropped_frames_before_submit', 0))
        state['writer_error'] = row.get('writer_error') or state['writer_error']
        state['latest_frame'] = {'wall_s': row.get('wall_s'), 'context': row.get('context'),
            'velocity_rad_s': speeds, 'feedback_minus_command_deg': error,
            'individual_feedback_freshness_verified': row.get('individual_feedback_freshness_verified'),
            'feedback_age_s': row.get('feedback_age_s'),
            'shoulder_rotate_gains': {k: row.get('commands', {}).get('shoulder_rotate', {}).get(k) for k in ('kp', 'kd')}}
    state['backlog_bytes'] = max(0, path.stat().st_size-state['offset'])
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--context', type=Path, default=CONTEXT)
    parser.add_argument('--status-json', type=Path, help='Offline parser check; do not request HTTP')
    args = parser.parse_args()
    out = args.context
    if not out.is_dir():
        raise ValueError('Trial context must already exist; polling never prepares a motion')
    if args.status_json:
        status = json.loads(args.status_json.read_text())
    else:
        client = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with client.open('http://127.0.0.1:8100/api/v3/dynamics/status', timeout=5) as response:
            status = json.load(response)
    run = status.get('run') or {}
    text = project_path(run['full_log_path']).read_text(errors='replace') if run.get('full_log_path') else run.get('stdout', '')
    name = {'home': 'home_motion_status.json', 'dynamics:model-hold-after:home': 'home_hold_status.json',
        'table-clearance': 'clearance_motion_status.json',
        'dynamics:model-hold-after:table-clearance': 'clearance_hold_status.json',
        'dynamics:board-grasp': 'board_motion_status.json'}.get(run.get('action'), 'other_status.json')
    (out/name).write_text(json.dumps(status, indent=2))
    (out/'latest_status.json').write_text(json.dumps(status, indent=2))
    state_path = out/'trace_monitor_state.json'
    previous = json.loads(state_path.read_text()) if state_path.exists() else {}
    trace_names = re.findall(r'trace_\d+\.jsonl', text)
    trace = trace_increment(project_path('data/board_motion_traces/'+trace_names[-1]), previous) if trace_names else None
    if trace is not None:
        state_path.write_text(json.dumps(trace, indent=2))
    endpoints = tagged(text, 'board endpoint report= ')
    endpoint = endpoints[-1] if endpoints else {}
    hold_begin = tagged(text, 'board same-session hold begin= ')
    hold_complete = tagged(text, 'board same-session hold complete= ')
    latest_hold = hold_complete[-1] if hold_complete else {}
    markers = [marker for marker in ('confirmed encoder oscillation', 'board fault support active',
        'MotionStop', 'Traceback (most recent call last)') if marker in text]
    result = {k: run.get(k) for k in ('action', 'ok', 'running', 'support_running', 'returncode', 'duration_s', 'full_log_path', 'error')}
    if trace:
        result['trace'] = {k: trace.get(k) for k in ('path', 'latest_frame', 'stage_frames', 'events',
            'peak_velocity_rad_s', 'peak_tracking_error_deg', 'latest_static_update', 'accepted_bias_updates',
            'max_dropped', 'writer_error', 'footer', 'backlog_bytes', 'parse_errors')}
    result.update(polled_wall_s=time.time(), fault_markers=markers,
        confirmed_oscillation=bool(trace and trace['events'].get('confirmed_encoder_oscillation')),
        fault_support=bool(run.get('support_running') or trace and trace['stage_frames'].get('fault_support')),
        endpoint_max_error_deg=endpoint.get('max_nominal_error_deg'),
        estimated_applied_pending_bias=endpoint.get('quasistatic_support'),
        same_serial_hold={'began': bool(hold_begin), 'same_serial_session': bool(hold_begin and hold_begin[-1].get('same_serial_session')),
            'complete': bool(hold_complete), 'ok': latest_hold.get('ok'), 'duration_s': latest_hold.get('duration_s'),
            'reason': latest_hold.get('reason')},
        physical_drift_or_vibration_certified=False)
    (out/'latest_poll.json').write_text(json.dumps(result, indent=2))
    with (out/'poll_history.jsonl').open('a') as history:history.write(json.dumps(result,separators=(',', ':'))+'\n')
    print(json.dumps(result, separators=(',', ':')))


if __name__ == '__main__':
    main()
