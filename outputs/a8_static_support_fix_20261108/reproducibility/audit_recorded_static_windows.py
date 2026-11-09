"""Reproduce the recorded A8 v32 stationary residual, without hardware.

The old trace has no verified individual receive clocks. It must be rejected
by the new observer. The numerical bias below is an explicitly ineligible
counterfactual, never a fabricated accepted observation or physical result.
"""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import ast
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from statistics import median
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = ROOT/'outputs/a8_reference_execution_20261108/control_evidence'
OUTPUT = Path(__file__).resolve().parents[1]/'recorded_window_audit.json'
DECODER = json.JSONDecoder()
sys.path.insert(0, str(ROOT/'scripts'))
from left_arm_static_support import stable_support_observation, bounded_support_update, slew_duration


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def load(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def tagged(log, prefix):
    return [DECODER.raw_decode(line[len(prefix):])[0]
            for line in log.splitlines() if line.startswith(prefix)]


def metadata(path):
    return {'path': path.relative_to(ROOT).as_posix(), 'bytes': path.stat().st_size,
            'sha256': sha(path)}


def verify_manifest():
    path = EVIDENCE/'manifest.json'
    manifest = load(path)
    files = manifest.get('files', manifest)
    count = 0
    for relative, expected in files.items():
        candidate = (EVIDENCE/relative).resolve()
        if EVIDENCE.resolve() not in candidate.parents:
            raise ValueError('Manifest traversal rejected')
        if isinstance(expected, str):
            expected = {'sha256': expected}
        if ('bytes' in expected and candidate.stat().st_size != expected['bytes']) or sha(candidate) != expected['sha256']:
            raise ValueError('Original evidence differs: '+relative)
        count += 1
    return {**metadata(path), 'checked_files': count, 'all_original_files_match': True}


def main():
    manifest = verify_manifest()
    log_path = EVIDENCE/'data/left_arm_v2_8_full_runs/arm_run_1794185510062_30907.txt'
    trace_path = EVIDENCE/'data/board_motion_traces/trace_1794185516975282199.jsonl'
    model_path = EVIDENCE/'models/left_arm_general_dynamics.json'
    gravity_code = EVIDENCE/'source/left_arm_rigid_body.py'
    v32_code = EVIDENCE/'source/left_arm_board_grasp.py'
    limits_code = EVIDENCE/'source/left_arm_v3_1_model_control.py'
    support_code = ROOT/'scripts/left_arm_static_support.py'
    log = log_path.read_text(encoding='utf-8-sig')
    planned = tagged(log, 'board planned load support= ')[0]
    initial_anchor = planned['entry_load_tau']
    initial_gravity = planned['initial_gravity_tau']
    trials = tagged(log, 'board endpoint settle= ')
    if [row['attempt'] for row in trials] != [1, 2, 3]:
        raise ValueError('Expected exactly the recorded three v32 settle retries')
    expected = trials[-1]['target_rad']
    names = tuple(expected)
    settle_frames = []
    for line in trace_path.open(encoding='utf-8-sig'):
        row = json.loads(line)
        if row.get('event') == 'motor_frame' and row.get('context', {}).get('stage') == 'endpoint_settle':
            settle_frames.append(row)
    attempts = Counter(row['context']['attempt'] for row in settle_frames)
    latest_attempt = max(attempts)
    latest_frames = [row for row in settle_frames if row['context']['attempt'] == latest_attempt]
    end = latest_frames[-1]['monotonic_s']
    window = [row for row in latest_frames if row['monotonic_s'] >= end-.4-1e-9]
    duration = window[-1]['monotonic_s']-window[0]['monotonic_s']
    if len(window) < 6 or duration < .25:
        raise ValueError('Recorded terminal window does not meet numerical duration/frame criteria')
    observed = stable_support_observation(window, names, expected, end)
    if observed['accepted'] or observed['reason'] != 'individual-feedback-freshness-unverified':
        raise ValueError('Old telemetry must not be promoted to a verified receive-clock observation')

    med_positions = {n: median(row['feedback'][n]['pos'] for row in window) for n in names}
    med_torques = {n: median(row['feedback'][n]['tau'] for row in window) for n in names}
    gains = {n: {key: window[-1]['commands'][n][key] for key in ('kp', 'kd')} for n in names}
    ff = {n: median(row['commands'][n]['tau_ff'] for row in window) for n in names}
    retry_summary = []
    for trial in trials:
        rows = [row for row in settle_frames if row['context']['attempt'] == trial['attempt']]
        retry_summary.append({'attempt': trial['attempt'], 'frames': len(rows),
            'reported_max_error_deg': trial['max_error_deg'],
            'entry_error_deg': {n: math.degrees(trial['entry_feedback'][n]['pos']-expected[n]) for n in names},
            'last_ff_Nm': {n: rows[-1]['commands'][n]['tau_ff'] for n in names}})
    invariants = {}
    for n in names:
        commands = [row['commands'][n] for row in settle_frames]
        invariants[n] = {'q_ref_span_rad': max(r['q'] for r in commands)-min(r['q'] for r in commands),
            'ff_span_Nm': max(r['tau_ff'] for r in commands)-min(r['tau_ff'] for r in commands),
            'gain_pairs': sorted({(r['kp'], r['kd']) for r in commands}),
            'max_abs_reference_dq_rad_s': max(abs(r['dq']) for r in commands)}
    same_control = all(v['q_ref_span_rad'] <= 1e-12 and v['ff_span_Nm'] <= 1e-12
                       and len(v['gain_pairs']) == 1 and v['max_abs_reference_dq_rad_s'] <= 1e-12
                       for v in invariants.values())
    window_stats = {n: {'measured_position_median_rad': med_positions[n],
        'target_rad': expected[n], 'position_error_deg': math.degrees(med_positions[n]-expected[n]),
        'position_span_deg': math.degrees(max(r['feedback'][n]['pos'] for r in window)-min(r['feedback'][n]['pos'] for r in window)),
        'peak_abs_velocity_rad_s': max(abs(r['feedback'][n]['vel']) for r in window),
        'torque_median_Nm': med_torques[n], 'ff_median_Nm': ff[n],
        'stationary_P_component_Nm': gains[n]['kp']*(expected[n]-med_positions[n])}
        for n in names}

    # Import only the archived pure rigid-body math, never a motor/controller.
    spec = importlib.util.spec_from_file_location('frozen_a8_rigid_math', gravity_code)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = module.RigidBodyModel.load(model_path)
    gravity = model.signs*model.gravity_torques(model.encoder_to_model([med_positions[n] for n in model.names]))
    gravity = dict(zip(model.names, map(float, gravity)))
    baseline = {n: initial_anchor[n]+gravity[n]-initial_gravity[n] for n in names}
    residual = {n: med_torques[n]-baseline[n] for n in names}
    tree = ast.parse(limits_code.read_text(encoding='utf-8-sig'))
    limit_assign = next(node for node in tree.body if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == 'MIT_TAU_MAX' for t in node.targets))
    limits = ast.literal_eval(limit_assign.value)
    counterfactual = bounded_support_update(dict.fromkeys(names, 0.), residual, gains, limits)
    counterfactual_ff = {n: ff[n]+counterfactual['bias'][n] for n in names}
    report = {'schema': 1, 'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'motion_issued': False, 'network_accessed': False, 'controller_or_sdk_imported': False,
        'evidence_preserved': True, 'manifest_verification': manifest,
        'files': {key: metadata(path) for key, path in (
            ('helper', Path(__file__).resolve()), ('recorded_v32_board_code', v32_code),
            ('recorded_gravity_code', gravity_code), ('recorded_model', model_path),
            ('recorded_limits_source', limits_code), ('recorded_motion_log', log_path),
            ('recorded_control_trace', trace_path), ('current_static_support_code', support_code))},
        'recorded_retries': retry_summary, 'all_three_retries_same_reference_gains_and_ff': same_control,
        'control_invariants_across_three_retries': invariants,
        'last_fixed_window': {'attempt': latest_attempt, 'frames': len(window),
            'start_s': window[0]['monotonic_s'], 'end_s': end, 'duration_s': duration,
            'frame_indices_are_original': True,
            'recorded_individual_freshness_verified_flags': sorted({r.get('individual_feedback_freshness_verified') for r in window}),
            'per_joint': window_stats},
        'actual_new_observer_result_on_unchanged_old_trace': observed,
        'counterfactual_only': {'actual_observation_eligible': False,
            'individual_receive_timestamps_or_sequences_fabricated': False,
            'applied_to_robot': False, 'new_hardware_validation_pending': True,
            'initial_entry_anchor_Nm': initial_anchor, 'initial_model_gravity_Nm': initial_gravity,
            'model_gravity_at_window_median_actual_q_Nm': gravity,
            'immutable_entry_baseline_at_actual_q_Nm': baseline,
            'potential_observed_absolute_static_residual_Nm': residual,
            'bounded_update_from_zero_alpha_half': counterfactual,
            'potential_baseline_smoothstep_duration_s_at_1Nm_per_s': slew_duration(ff, counterfactual_ff, 0., rate=1.),
            'scope': 'Numerical demonstration only. These old cached samples are ineligible for live learning. Bias is not installed in any model or controller.'},
        'root_cause_supported_by_record': 'Three settle retries repeated the same q-reference, gains and FF while stable encoder residual persisted; no additional correction law was applied.',
        'physical_drift_solved': False, 'physical_tracking_improvement_verified': False,
        'fitting_eligible': False,
        'measurement_caveat': 'Encoder stability and model gravity cannot establish physical XY drift, contact clearance, or uniquely identify gravity-versus-friction/load-model error.'}
    OUTPUT.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'output': str(OUTPUT), 'window_frames': len(window), 'duration_s': duration,
        'observer_accepted': observed['accepted'], 'observer_reason': observed['reason'],
        'same_control_all_three_retries': same_control,
        'potential_residual_Nm': {n: residual[n] for n in ('shoulder_front', 'shoulder_side', 'elbow')},
        'counterfactual_new_bias_Nm': {n: counterfactual['bias'][n] for n in ('shoulder_front', 'shoulder_side', 'elbow')},
        'new_hardware_validation_pending': True}, ensure_ascii=False))


if __name__ == '__main__':
    main()
