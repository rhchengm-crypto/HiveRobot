"""Freeze a completed authorized lower25 geometric trial; never imports/commands/stops any motor.

The ONLY process signal permitted here is SIGINT to this trial's explicitly
authorized recorder PID, after Web motion and hold completion and exact PID/
script/output verification. No Web POSTs, deployment, or arm signals exist.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import time
import urllib.request
import zipfile

ROOT = Path('/home/nvidia/hive_robot/DM_Control_Python').resolve()
CONTEXT = ROOT/'data/board_workspace_trials/a8_lower25_execution_20261108'
EXPECTED_RECORDER_PID = None
EXPECTED_MEDIA_NAME = None
PREPARED_SOURCES = {
    'rigid_model': 'data/left_arm_general_dynamics.json',
    'board_model': 'data/left_arm_board_workspace_candidate.json',
    'planner': 'left_arm_board_grasp.py', 'controller': 'left_arm_v2_6.py',
    'motion_adapter': 'left_arm_v3_1_motion_dynamics.py',
    'fault_support': 'left_arm_fault_support.py', 'kinematics': 'left_arm_board_kinematics.py',
    'collision': 'left_arm_board_collision.py', 'trace': 'left_arm_motion_trace.py',
    'task_pose': 'left_arm_board_task_pose.py', 'static_support': 'left_arm_static_support.py',
    'trajectory_support': 'left_arm_trajectory_support.py', 'general_motion': 'left_arm_general_motion.py'}
EXTRA_SOURCE = ['left_arm_board_transit.py', 'left_arm_v2_8.py', 'left_arm_rigid_body.py',
    'left_arm_v3_1_model_control.py', 'left_arm_board_home_handoff.py',
    'chessboard_vision_v3_0_web_control.py', 'record_left_arm_rgbd_audit.py']
MODELS = ['left_arm_general_dynamics.json', 'left_arm_board_workspace_candidate.json',
    'left_arm_v3_1_dynamics.json', 'left_arm_v2_home.json', 'left_arm_v2_table_clearance.json',
    'left_arm_validated_path_library.json', 'left_arm_v2_6_saved_moves.json']
LEARNING = ['left_arm_v2_8_clearance_bias.json', 'left_arm_v2_6_adaptive_target_bias.json',
    'left_arm_v2_6_adaptive_hold_target_bias.json', 'left_arm_v2_6_adaptive_hold_tau.json',
    'left_arm_v2_6_adaptive_active_tau.json']


def load(path):
    return json.loads(Path(path).read_text())


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda: stream.read(1048576), b''):
            h.update(data)
    return h.hexdigest()


def project_path(value):
    path = Path(value)
    path = (path if path.is_absolute() else ROOT/path).resolve()
    if ROOT not in path.parents:
        raise ValueError('Evidence path escapes project: '+str(value))
    return path


def run_of(document):
    return document['run'] if isinstance(document.get('run'), dict) else document


def chain(run):
    seen = set()
    while isinstance(run, dict) and id(run) not in seen:
        seen.add(id(run))
        yield run
        run = run.get('preceding_motion')


def completed(document, action):
    current = run_of(document)
    if not (current.get('ok') and current.get('returncode') == 0 and
            not current.get('running') and not current.get('support_running')):
        raise ValueError('Motion or hold has not completed successfully: '+action)
    matches = [row for row in chain(current) if row.get('action') == action]
    if len(matches) != 1:
        raise ValueError('Completed phase missing/ambiguous: '+action)
    primary = matches[0]
    if not (primary.get('ok') and primary.get('returncode') == 0 and
            not primary.get('running') and not primary.get('support_running')):
        raise ValueError('Primary motion did not complete: '+action)
    if current.get('post_move_model_hold', {}).get('state') != 'complete':
        raise ValueError('Post-motion hold incomplete: '+action)
    return current, primary


def verify_launch(primary, path):
    launch = run_of(load(path))
    submitted, actual = launch.get('command'), primary.get('command')
    inline = primary.get('action') == 'dynamics:board-grasp' and actual == (submitted or [])+['--inline-model-hold']
    if not (actual == submitted or inline):
        raise ValueError('Completed command differs from launch: '+path.name)
    if launch.get('started_at') is not None and primary.get('started_at') != launch['started_at']:
        raise ValueError('Completed start identity differs: '+path.name)
    if launch.get('full_log_path') and primary.get('full_log_path') != launch['full_log_path']:
        raise ValueError('Completed log identity differs: '+path.name)


def stop_authorized_recorder(recording):
    media = project_path(recording['recording_folder'])
    pid = int(recording['recorder_pid'])
    if (media.parent != ROOT/'data/rgbd_audits' or media.name != EXPECTED_MEDIA_NAME
            or pid != EXPECTED_RECORDER_PID or pid <= 1 or pid == os.getpid()):
        raise ValueError('Recorder identity differs from explicitly authorized capture; no signal sent')
    proc = Path('/proc')/str(pid)
    def state():
        try:
            return (proc/'stat').read_text().rsplit(')', 1)[1].split()[0]
        except FileNotFoundError:
            return None
    if state() in (None, 'Z'):
        return media, {'pid': pid, 'signal_sent': False, 'state': 'already-exited'}
    argv = [v.decode(errors='strict') for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    if not any(Path(v).name == 'record_left_arm_rgbd_audit.py' for v in argv[:4]):
        raise ValueError('PID is not the authorized camera recorder; no signal sent')
    if '--output' not in argv or Path(argv[argv.index('--output')+1]).resolve() != media:
        raise ValueError('Recorder output does not match; no signal sent')
    os.kill(pid, signal.SIGINT)
    for _ in range(50):
        if state() in (None, 'Z'):
            return media, {'pid': pid, 'signal_sent': True, 'state': 'matching-recorder-exited'}
        time.sleep(.2)
    raise RuntimeError('Recorder did not exit; no additional signal or arm operation attempted')


def workflow_ids(text):
    decoder, result = json.JSONDecoder(), set()
    for line in text.splitlines():
        if 'workflow_run_id' not in line or 'workflow_action' not in line:
            continue
        try:
            row = decoder.raw_decode(line[line.index('{'):])[0]
        except ValueError:
            continue
        if isinstance(row, dict) and row.get('workflow_action') and row.get('workflow_run_id'):
            result.add((row['workflow_action'], row['workflow_run_id']))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--context', type=Path, default=CONTEXT)
    parser.add_argument('--expected-recorder-pid', type=int, required=True)
    parser.add_argument('--expected-media-name', required=True)
    parser.add_argument('--expected-board-log', required=True)
    parser.add_argument('--expected-clearance-log', required=True)
    args = parser.parse_args()
    global EXPECTED_RECORDER_PID, EXPECTED_MEDIA_NAME
    EXPECTED_RECORDER_PID, EXPECTED_MEDIA_NAME = args.expected_recorder_pid, args.expected_media_name
    if not EXPECTED_MEDIA_NAME.startswith('a8_lower25_execution_20261108_'):
        raise ValueError('Wrong authorized lower25 capture prefix')
    context = project_path(args.context)
    stages = {}
    for filename, action in [('clearance_complete_status.json', 'table-clearance'),
                             ('execution_complete_status.json', 'dynamics:board-grasp')]:
        stages[action] = completed(load(context/filename), action)
    launched_home = (context/'home_start.json').exists()
    if launched_home and (context/'home_complete_status.json').exists():
        stages['home'] = completed(load(context/'home_complete_status.json'), 'home')
    for action, (_, primary) in stages.items():
        names = {'home': ['home_start.json'], 'table-clearance': ['clearance_start.json'],
                 'dynamics:board-grasp': ['execution_start.json', 'execute_start.json']}[action]
        launch = next((context/n for n in names if (context/n).exists()), None)
        if launch is None:
            raise ValueError('No launch identity for '+action)
        verify_launch(primary, launch)
    board_current, board_primary = stages['dynamics:board-grasp']
    if board_primary.get('full_log_path') != str(project_path(args.expected_board_log)):
        raise ValueError('Board log differs from explicit completion authorization')
    if stages['table-clearance'][1].get('full_log_path') != str(project_path(args.expected_clearance_log)):
        raise ValueError('Clearance log differs from explicit completion authorization')
    command = board_primary['command']
    if '--execute' not in command or '--prepared-plan' not in command:
        raise ValueError('Expected complete prepared motion')
    prepared_paths = dict(PREPARED_SOURCES)
    for key, flag in [('rigid_model', '--model'), ('board_model', '--board-model')]:
        if flag not in command:raise ValueError('Missing executed model option '+flag)
        prepared_paths[key] = project_path(command[command.index(flag)+1]).relative_to(ROOT).as_posix()
    prepared_path = project_path(command[command.index('--prepared-plan')+1])
    prepared = load(prepared_path)
    metadata = prepared.get('prepared_execution', {})
    expected = metadata.get('source_hashes', {})
    if set(expected) != set(PREPARED_SOURCES):
        raise ValueError('Expected all thirteen v33 prepared source hashes, including static_support')
    unsealed = dict(prepared)
    unsealed.pop('prepared_execution', None)
    encoded = json.dumps(unsealed, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    if hashlib.sha256(encoded).hexdigest() != metadata.get('plan_sha256'):
        raise ValueError('Prepared plan digest mismatch')
    preflight = load(context/'preflight.json')
    prepared_byte_sources = {}; prepared_collection_binding = {}
    for name, relative in prepared_paths.items():
        current_path = project_path(relative);current_sha = digest(current_path)
        chosen = current_path
        if current_sha != expected[name]:
            # A completed trial may have its temporary active board configuration
            # restored by the root hardware owner. Freeze the exact trial bytes
            # from its pre-existing context snapshot, never substitute original
            # common bytes for the executed prepared-board model.
            original_sha = preflight.get('source_hashes', {}).get(relative)
            candidates = [context/n for n in ('board_workspace_trial.json', 'active_board_model_trial.json', 'trial_board_config.json')]
            matches = [q for q in candidates if q.is_file() and digest(q)==expected[name]]
            if name != 'board_model' or current_sha != original_sha or not matches:
                raise ValueError('Prepared runtime source changed before collection: '+relative)
            chosen = matches[0]
        prepared_byte_sources[name] = chosen
        prepared_collection_binding[name] = {'runtime_relative_path':relative,
            'prepared_sha256':expected[name],'current_runtime_sha256':current_sha,
            'archive_byte_source':str(chosen),
            'temporary_board_model_restored_before_collection':name=='board_model' and current_sha!=expected[name]}
    client = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def status():
        with client.open('http://127.0.0.1:8100/api/v3/dynamics/status', timeout=5) as response:
            return json.load(response)
    def verify_same(document):
        _, primary = completed(document, 'dynamics:board-grasp')
        for key in ('command', 'started_at', 'full_log_path'):
            if primary.get(key) != board_primary.get(key):
                raise ValueError('Current run changed during collection: '+key)
    verify_same(status())
    media, recorder = stop_authorized_recorder(load(context/'recording.json'))
    final = status()
    verify_same(final)
    dest = ROOT/'data/experiment_archives'/('a8_lower25_execution_20261108_'+str(time.time_ns()))
    dest.mkdir(parents=True)
    files, missing = {}, []
    def add(value, logical=None, required=True, expected_sha=None):
        path = project_path(value)
        if not path.is_file():
            if required:
                raise FileNotFoundError(str(path))
            missing.append(str(path))
            return
        name = logical or path.relative_to(ROOT).as_posix()
        if Path(name).is_absolute() or '..' in Path(name).parts:
            raise ValueError('Unsafe archive path')
        data = path.read_bytes()
        if expected_sha and hashlib.sha256(data).hexdigest() != expected_sha:
            raise ValueError('Source changed during evidence freeze: '+str(path))
        target = dest/'snapshot'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        files[name] = target
    for path in sorted(context.rglob('*')):
        if path.is_file():
            add(path)
    texts, identities, phases = [], set(), {}
    for action, (current, primary) in stages.items():
        phases[action] = {'primary_log': primary['full_log_path'], 'hold_log': current['full_log_path'],
            'started_at': primary.get('started_at'), 'finished_at': current.get('finished_at'),
            'post_move_model_hold': current.get('post_move_model_hold')}
        for row in chain(current):
            if row.get('full_log_path'):
                add(row['full_log_path'])
                text = project_path(row['full_log_path']).read_text(errors='replace')
                texts.append(text)
                identities.update(workflow_ids(text))
            cmd = row.get('command') or []
            for flag in ('--plan-output', '--prepared-plan'):
                if flag in cmd:
                    add(cmd[cmd.index(flag)+1])
    traces = sorted(set(re.findall(r'trace_\d+\.jsonl', '\n'.join(texts))))
    if not traces:
        raise ValueError('Completed board logs do not identify a continuous control trace')
    for name in traces:
        add(ROOT/'data/board_motion_traces'/name)
    frozen_sources = {}
    for name, relative in prepared_paths.items():
        path = prepared_byte_sources[name]
        add(path, ('models/' if relative.startswith('data/') else 'source/')+Path(relative).name,
            expected_sha=expected[name])
        frozen_sources[name] = expected[name]
    add(project_path(prepared_paths['board_model']),
        'current_runtime_models/'+Path(prepared_paths['board_model']).name)
    extra_math_sources = {}
    for name in EXTRA_SOURCE:
        add(ROOT/name, 'source/'+name)
        extra_math_sources[name] = digest(files['source/'+name])
    for name in MODELS:
        if 'models/'+name not in files:
            add(ROOT/'data'/name, 'models/'+name)
    preflight = load(context/'preflight.json')
    before_sources = {}
    snapshot_roots = [context/'runtime_snapshot', context/'source_snapshot']
    for relative, expected_sha in preflight.get('source_hashes', {}).items():
        candidates = [p for base in snapshot_roots if base.is_dir()
                      for p in base.rglob(Path(relative).name) if p.is_file()]
        matches_before = [p for p in candidates if digest(p) == expected_sha]
        if not matches_before:
            raise ValueError('Missing hash-matched pre-motion protected snapshot: '+relative)
        before_sources[relative] = {'sha256': expected_sha,
            'snapshot_path': matches_before[0].relative_to(context).as_posix()}
    learning = {}
    for name in LEARNING:
        declared = before_sources.get('data/'+name)
        before = context/declared['snapshot_path'] if declared else context/('before_'+name)
        after = ROOT/'data'/name
        add(before, 'learning_before/'+name, required=True)
        add(after, 'learning_after/'+name, required=True)
        learning[name] = {'before_available': before.is_file(), 'after_available': after.is_file(),
            'before_sha256': digest(before) if before.is_file() else None,
            'after_sha256': digest(after) if after.is_file() else None}
        learning[name]['changed'] = learning[name]['before_sha256'] != learning[name]['after_sha256']
    matches, selected = [], []
    rolling = ROOT/'data/left_arm_general_motion_runs.jsonl'
    if rolling.exists():
        with rolling.open('rb') as stream:
            for number, line in enumerate(stream, 1):
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if (row.get('workflow_action'), row.get('workflow_run_id')) in identities:
                    selected.append(line)
                    matches.append({'source_line': number, 'workflow_action': row['workflow_action'],
                        'workflow_run_id': row['workflow_run_id'], 'frames': row.get('frames'),
                        'sha256': hashlib.sha256(line).hexdigest()})
    if not matches:
        missing.append('No general-motion records matched the exact workflow/run identities')
    scoped = dest/'general_motion_records.jsonl'
    scoped.write_bytes(b''.join(selected))
    add(scoped, scoped.name)
    # Model-hold rolling rows have local-time timestamps, not workflow IDs.
    # Preserve only this trial's phase window and report that weaker binding.
    window_start = min(row[1]['started_at'] for row in stages.values())
    window_end = max(row[0]['finished_at'] for row in stages.values())
    model_rows = []; model_meta = []
    model_log = ROOT/'data/left_arm_v3_1_model_runs.jsonl'
    if model_log.is_file():
        with model_log.open('rb') as stream:
            for number, line in enumerate(stream, 1):
                try:row = json.loads(line)
                except ValueError:continue
                stamp = row.get('time')
                if isinstance(stamp, str) and window_start <= stamp <= window_end:
                    model_rows.append(line);model_meta.append({'source_line':number,'time':stamp,'event':row.get('event'),'sha256':hashlib.sha256(line).hexdigest()})
    scoped_models = dest/'model_hold_records.jsonl'
    scoped_models.write_bytes(b''.join(model_rows));add(scoped_models,scoped_models.name)
    if not model_rows:missing.append('No time-window model-hold JSONL rows; full hold log/trace still preserved')
    for launch in context.glob('prepar*.json'):
        try:document = load(launch)
        except ValueError:continue
        def collect_preparation(value):
            if isinstance(value, dict):
                if value.get('full_log_path'):add(value['full_log_path'])
                cmd=value.get('command') or []
                if isinstance(cmd,list):
                    for flag in ('--plan-output','--prepared-plan'):
                        if flag in cmd:add(cmd[cmd.index(flag)+1])
                for child in value.values():collect_preparation(child)
            elif isinstance(value,list):
                for child in value:collect_preparation(child)
        collect_preparation(document)
    add(media.with_suffix('.log'), 'camera_recorder.log', required=False)
    media_files = sorted(path for path in media.rglob('*') if path.is_file())
    rgb = {path.stem for path in media_files if path.suffix.lower() == '.jpg'}
    depth = {path.stem for path in media_files if path.suffix.lower() == '.npy'}
    frame_log = media/'frames.jsonl'
    frame_rows = sum(bool(line.strip()) for line in frame_log.read_text().splitlines()) if frame_log.exists() else None
    summary = {'motion_issued_by_collector': False, 'arm_process_not_interrupted': True,
        'camera_recorder': recorder, 'phase_logs': phases, 'continuous_traces': traces,
        'antecedent_home_status_context_only': (context/'home_complete_status.json').exists() and not launched_home,
        'home_executed_as_part_of_this_trial': 'home' in stages,
        'prepared_source_hashes_verified': frozen_sources, 'prepared_source_count': len(frozen_sources),
        'prepared_source_file_paths': prepared_paths, 'extra_math_source_hashes':extra_math_sources,
        'prepared_source_collection_binding':prepared_collection_binding,
        'temporary_board_model_restored_at_collection':prepared_collection_binding['board_model']['temporary_board_model_restored_before_collection'],
        'model_hold_records':model_meta,'model_hold_record_binding':'phase local-time window; rows lack workflow IDs',
        'model_hold_record_window':{'start':window_start,'end':window_end},
        'pre_motion_protected_snapshots': before_sources,
        'pre_motion_protected_snapshot_count': len(before_sources),
        'learning_snapshots': learning, 'general_motion_records': matches,
        'workflow_identities': [list(pair) for pair in sorted(identities)],
        'request_kind': 'authorized lower25 target with explicitly approved frame and front-bound extension; model-computed path',
        'target_board_mm': prepared['target_board_mm'], 'original_nominal_A8_XYZ_solved': False,
        'camera': {'rgb_frames': len(rgb), 'depth_frames': len(depth), 'paired_frames': len(rgb & depth),
            'frame_log_rows': frame_rows, 'pairing_complete': bool(rgb) and rgb == depth and frame_rows == len(rgb)},
        'physical_drift_solved_by_software': False, 'physical_observation_required': True,
        'operator_feedback_included': (context/'operator_feedback.json').exists(),
        'ordinary_Home_Clearance_limitations': 'Separate Web phases lack per-frame board traces; general-motion references are commands, not measured feedback.',
        'missing_optional_items': missing}
    for name, value in [('web_status.json', final), ('result_summary.json', summary)]:
        path = dest/name
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2))
        add(path, name)
    report = {'motion_issued_by_collector': False, 'arm_process_not_interrupted': True,
        'run_identity_verified': True, 'archive_directory': str(dest),
        'antecedent_home_status_context_only': summary['antecedent_home_status_context_only'],
        'home_executed_as_part_of_this_trial': 'home' in stages,
        'prepared_source_count': len(frozen_sources), 'recorder': recorder,
        'pre_motion_protected_snapshot_count': len(before_sources),
        'continuous_traces': traces, 'scoped_general_motion_records': len(matches),
        'learning_files': len(learning), 'model_hold_records':len(model_meta),'camera': summary['camera'], 'missing_optional_items': missing}
    for label, entries in [('control', files), ('rgbd', {p.relative_to(media).as_posix(): p for p in media_files})]:
        manifest, archive = {}, dest/(label+'_evidence.zip')
        with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=3) as zipped:
            for name, path in sorted(entries.items()):
                data = path.read_bytes()
                manifest[name] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
                zipped.writestr(name, data)
            zipped.writestr('manifest.json', json.dumps(manifest, indent=2))
        (dest/(label+'_manifest.json')).write_text(json.dumps(manifest, indent=2))
        with zipfile.ZipFile(archive) as zipped:
            if zipped.testzip() is not None:
                raise ValueError('Archive CRC verification failed')
        report[label] = {'path': str(archive), 'bytes': archive.stat().st_size,
                         'sha256': digest(archive), 'zip_crc_verified': True}
    (dest/'collection_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, separators=(',', ':')))


if __name__ == '__main__':
    main()
