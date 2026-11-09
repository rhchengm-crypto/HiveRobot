"""Offline A8 accepted-reference torque-proxy curation and grouped fit.

No SDK, network, robot commands, or active-file writes. A held pose is one
independent group, not 7 independent poses or N camera/control-frame labels.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
sys.dont_write_bytecode = True
import numpy as np


def read(p):
    return json.loads(p.read_text(encoding='utf-8-sig'))


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def dump(p, row):
    p.write_text(json.dumps(row, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def nearby(a, b, deg=5.):
    return max(abs(np.degrees(float(a[n])-float(b[n]))) for n in b) <= deg


def stripped(history, test):
    result = copy.deepcopy(history)
    removed = {}
    for n, body in result['joints'].items():
        removed[n] = {}
        for field in ('gravity_samples', 'inertia_samples'):
            old = body[field]
            body[field] = [s for s in old if s.get('experiment_id') != test['id']
                           and not nearby(s['pose_rad'], test['encoder_pose_rad'])]
            removed[n][field] = len(old)-len(body[field])
    return result, removed


def add(history, rows):
    result = copy.deepcopy(history)
    for row in rows:
        for n in row['encoder_pose_rad']:
            result['joints'][n]['gravity_samples'].append(dict(
                pose_rad=row['encoder_pose_rad'], tau=row['measured_torque_Nm'][n],
                direction='stationary', source='quasi-static-hold-proxy',
                experiment_id=row['id'], uncalibrated_torque_proxy=True,
                independent_pose_group=row['id'], geometry_target_labels=False))
    return result


def evaluate(model_data, train, heldout, RigidBodyModel):
    model = RigidBodyModel(model_data)
    bias = {}
    # Train nuisance terms only on training data. They are not runtime torque.
    for j, n in enumerate(model.names):
        a, b = [], []
        for s in train['joints'][n]['gravity_samples']:
            q = model.encoder_to_model([s['pose_rad'][k] for k in model.names])
            pred = model.signs[j]*model.gravity_torques(q)[j]
            a.append([1., {'positive': 1., 'negative': -1.}.get(s.get('direction'), 0.)])
            b.append(s['tau']-pred)
        bias[n] = float(np.linalg.lstsq(a, b, rcond=1e-9)[0][0])
    q = model.encoder_to_model([heldout['encoder_pose_rad'][n] for n in model.names])
    pred = model.signs*model.gravity_torques(q)
    raw = {n: float(pred[j]) for j, n in enumerate(model.names)}
    with_offset = {n: raw[n]+bias[n] for n in model.names}
    measured = heldout['measured_torque_Nm']
    residual = {n: with_offset[n]-measured[n] for n in model.names}
    raw_residual = {n: raw[n]-measured[n] for n in model.names}
    return dict(prediction_without_nuisance_Nm=raw,
                train_only_nuisance_offset_Nm=bias,
                prediction_with_train_only_nuisance_Nm=with_offset,
                heldout_feedback_torque_proxy_Nm=measured,
                residual_without_nuisance_Nm=raw_residual,
                residual_Nm=residual,
                rmse_without_nuisance_Nm=float(np.sqrt(np.mean(np.square(list(raw_residual.values()))))),
                rmse_Nm=float(np.sqrt(np.mean(np.square(list(residual.values()))))),
                residual_definition='prediction minus heldout motor feedback torque proxy')


def run(root, out):
    out.mkdir(parents=True, exist_ok=True)
    frozen = out/'source_snapshot'
    frozen.mkdir(exist_ok=True)
    paths = {
        'geometry': root/'outputs/common_calibration_20261106/source_snapshot/outputs/left_arm_general_model/left_arm_cad_exact.json',
        'inspection': root/'outputs/common_calibration_20261106/source_snapshot/outputs/left_arm_step_analysis/revised_2/step_inspection.json',
        'history_original': root/'outputs/common_calibration_20261106/source_snapshot/outputs/board_workspace_20261106/baseline/left_arm_v3_1_dynamics.json',
        'history_three_groups': root/'outputs/common_calibration_20261106/hold_proxy_augmented_history.json',
        'previous_audit': root/'outputs/common_calibration_20261106/common_calibration_audit.json',
        'current_model': root/'scripts/data/left_arm_general_dynamics.json',
        'trace': root/'outputs/a8_lower25_execution_20261108/control_evidence/data/board_motion_traces/trace_1794198451214228486.jsonl',
        'execution_audit': root/'outputs/a8_lower25_execution_20261108/execution_audit.json',
        'fit_source': root/'scripts/fit_left_arm_general_dynamics.py',
        'rigid_source': root/'scripts/left_arm_rigid_body.py',
    }
    protected_names = ['left_arm_general_dynamics.json', 'left_arm_v3_1_dynamics.json',
                       'left_arm_board_workspace_candidate.json',
                       'left_arm_v2_8_clearance_bias.json',
                       'left_arm_v2_6_adaptive_target_bias.json',
                       'left_arm_v2_6_adaptive_hold_target_bias.json',
                       'left_arm_v2_6_adaptive_hold_tau.json',
                       'left_arm_v2_6_adaptive_active_tau.json']
    protected = {str(p.relative_to(root)).replace('\\', '/'): sha(p)
                 for p in [root/'scripts/data'/n for n in protected_names]+
                 list((root/'scripts').glob('*.py')) if p.is_file()}
    source = {k: {'path': str(p.relative_to(root)).replace('\\', '/'),
                  'sha256': sha(p), 'bytes': p.stat().st_size} for k, p in paths.items()}
    for key in ('geometry', 'inspection', 'history_original', 'history_three_groups',
                'previous_audit', 'current_model', 'fit_source', 'rigid_source'):
        shutil.copyfile(paths[key], frozen/paths[key].name)
    sys.path.insert(0, str(frozen))
    from fit_left_arm_general_dynamics import fit
    from left_arm_rigid_body import RigidBodyModel
    geometry, inspection = read(paths['geometry']), read(paths['inspection'])
    original, existing = read(paths['history_original']), read(paths['history_three_groups'])
    previous, active = read(paths['previous_audit']), read(paths['current_model'])
    names = list(RigidBodyModel(active).names)
    assert sha(paths['current_model']) == '83f5e4b240d48ce4aae1c8037922696c5f2f1ed120476c8ae7da9e6ca00d6fd7'
    assert all(sha(paths[k]) in active['identification_sources'].values()
               for k in ('geometry', 'inspection', 'history_original'))
    assert {s.get('experiment_id') for n in names for s in existing['joints'][n]['gravity_samples']
            if s.get('experiment_id')} == {'C4', 'C5', 'B3'}
    records = [json.loads(s) for s in paths['trace'].read_text(encoding='utf-8-sig').splitlines()]
    assert records[-1]['event'] == 'trace_closed' and records[-1]['queue_dropped_frames'] == 0
    frames = [r for r in records if r.get('event') == 'motor_frame'
              and r.get('context', {}).get('stage') == 'model_hold']
    raw_count = len(frames)
    cutoff = frames[0]['monotonic_s']+.75
    frames = [r for r in frames if r['monotonic_s'] >= cutoff]
    assert frames[-1]['monotonic_s']-frames[0]['monotonic_s'] >= 3.
    assert all(r.get('individual_feedback_freshness_verified') is True
               and not r.get('invalid_feedback_fields')
               and not r.get('writer_error') for r in frames)
    stats, final = {}, {}
    for n in names:
        seen, unique = set(), []
        prior_sequence, prior_received = -1, -float('inf')
        for r in frames:
            seq = r['feedback_sequence'][n]
            received = r['feedback_received_monotonic_s'][n]
            age = r['feedback_age_s'][n]
            assert seq >= prior_sequence and received >= prior_received
            assert r['individual_feedback_freshness'][n] is True and 0 <= age <= .1
            assert abs(r['monotonic_s']-received-age) < 1e-6
            prior_sequence, prior_received = seq, received
            if seq not in seen:
                seen.add(seq)
                unique.append(r)
        samples = np.array([[r['feedback'][n][k] for k in ('pos', 'vel', 'tau')] for r in unique])
        assert np.isfinite(samples).all()
        med = np.median(samples, axis=0)
        final[n] = dict(zip(('pos', 'vel', 'tau'), map(float, med)))
        stats[n] = dict(unique_feedback_packets=len(unique),
            duplicate_cached_rows_excluded=len(frames)-len(unique),
            peak_abs_velocity_rad_s=float(np.max(abs(samples[:, 1]))),
            position_span_deg=float(np.degrees(np.ptp(samples[:, 0]))),
            position_mad_deg=float(np.degrees(np.median(abs(samples[:, 0]-med[0])))),
            torque_span_Nm=float(np.ptp(samples[:, 2])),
            torque_mad_Nm=float(np.median(abs(samples[:, 2]-med[2]))),
            torque_std_Nm=float(np.std(samples[:, 2])),
            max_feedback_age_s=max(r['feedback_age_s'][n] for r in unique),
            sequence_start=unique[0]['feedback_sequence'][n],
            sequence_end=unique[-1]['feedback_sequence'][n])
        assert stats[n]['peak_abs_velocity_rad_s'] < .05
        assert stats[n]['position_span_deg'] < .25
    audit = read(paths['execution_audit'])
    assert all(audit['checks'].values()), 'Execution audit must pass before proxy curation'
    a8 = dict(id='A8_lower25_20261108', independent_fitting_pose_count=1,
        encoder_pose_rad={n: final[n]['pos'] for n in names},
        measured_torque_Nm={n: final[n]['tau'] for n in names},
        feedback_median=final, per_axis_stability=stats,
        stable_hold=dict(source='last same-serial Model Hold; first 0.75 s excluded',
            raw_model_hold_frames=raw_count, retained_complete_frames=len(frames),
            retained_duration_s=frames[-1]['monotonic_s']-frames[0]['monotonic_s'],
            max_feedback_age_limit_s=.1, peak_abs_velocity_limit_rad_s=.05,
            position_span_limit_deg=.25),
        operator_validation={'accepted': True,
            'original_response': '好的，那么我认为可以把此次运行标记为通过，并纳入参考库进行拟合，因为现在这个位置目测已到机械臂的极限，可以夹取除兵以外的其他全部棋子。',
            'scope': 'operator-approved empty-load motion/reference; grasping of non-pawn pieces considered feasible by visual observation',
            'closed_gripper_loaded_grasp_verified': False,
            'mechanical_limit_status': 'operator visual estimate; not a measured hard limit'},
        fitting_qualification=dict(geometry_target_labels=False,
            quasi_static_torque_proxy=True, calibrated_gravity_measurement=False,
            new_inertia_measurement=False, physical_TCP_measurement=False,
            nominal_requested_XYZ_not_used_as_ground_truth=True))
    old_rows = [{k: row[k] for k in ('id', 'encoder_pose_rad', 'measured_torque_Nm')}
                for row in previous['trials']]
    rows = old_rows+[a8]
    for n in names:
        assert len(existing['joints'][n]['gravity_samples']) == len(original['joints'][n]['gravity_samples'])+3
        for row in old_rows:
            match = [s for s in existing['joints'][n]['gravity_samples'] if s.get('experiment_id') == row['id']]
            assert len(match) == 1 and match[0]['pose_rad'] == row['encoder_pose_rad']
            assert match[0]['tau'] == row['measured_torque_Nm'][n]
    augmented = add(existing, [a8])
    augmented['offline_dataset_extension'] = dict(
        type='accepted-empty-load-quasistatic-hold-proxies',
        independent_hold_pose_groups=[r['id'] for r in rows],
        geometry_target_labels=False, new_independent_pose_groups=1,
        new_torque_proxy_records=7, source_trace=source['trace'],
        candidate_requires_holdout_review=True)
    dump(out/'a8_quasistatic_pose_group.json', a8)
    dump(out/'hold_proxy_augmented_history.json', augmented)
    baseline, base_report = fit(geometry, original, inspection)
    def param_vector(data):
        values = []
        for n in names:
            item = data['links'][n+'_link']['inertial']
            values += [item['mass_kg']]+list(item['com_m'])+list(np.ravel(item['inertia_com_kg_m2']))
        return np.r_[values, data['reflected_rotor_inertias_kg_m2']]
    baseline_error = float(np.max(abs(param_vector(baseline)-param_vector(active))))
    assert baseline_error < 1e-10, 'Original baseline not reproduced; stop candidate comparison/promotion'
    candidate, candidate_report = fit(geometry, augmented, inspection)
    candidate['identification_sources'] = {source[k]['path']: source[k]['sha256'] for k in
        ('geometry', 'inspection', 'history_original', 'history_three_groups', 'trace')}
    candidate['identification_sources']['curated_augmented_history'] = sha(out/'hold_proxy_augmented_history.json')
    candidate['identification_sources']['fit_source'] = source['fit_source']['sha256']
    candidate['identification_sources']['rigid_source'] = source['rigid_source']['sha256']
    candidate['identification']['added_hold_proxy_records'] = 28
    candidate['identification']['added_independent_pose_groups'] = 4
    candidate['identification']['hold_proxy_experiments'] = [r['id'] for r in rows]
    candidate['identification']['do_not_treat_as_measured_mass_properties'] = True
    candidate['offline_candidate_only'] = True
    candidate['runtime_apply_requires_explicit_opt_in'] = True
    dump(out/'left_arm_general_dynamics_hold_proxy_candidate.json', candidate)
    cv = []
    for test in rows:
        clean, removed = stripped(original, test)
        train_rows = [r for r in rows if r['id'] != test['id']
                      and not nearby(r['encoder_pose_rad'], test['encoder_pose_rad'])]
        train = add(clean, train_rows)
        prior_train_rows = [r for r in train_rows if r['id'] != a8['id']]
        prior_train = add(clean, prior_train_rows)
        assert all(s.get('experiment_id') != test['id'] and not nearby(s['pose_rad'], test['encoder_pose_rad'])
                   for body in train['joints'].values() for field in ('gravity_samples', 'inertia_samples')
                   for s in body[field])
        old, _ = fit(geometry, clean, inspection)
        prior_model, _ = fit(geometry, prior_train, inspection)
        new, _ = fit(geometry, train, inspection)
        cv.append(dict(heldout_experiment=test['id'], included_experiments=[r['id'] for r in train_rows],
            exclusion_rule='exclude complete heldout pose group and all historical seven-axis poses within 5 degrees max-axis distance',
            removed_original_history=removed,
            baseline=evaluate(old, clean, test, RigidBodyModel),
            prior_three_group_comparator=evaluate(prior_model, prior_train, test, RigidBodyModel),
            augmented=evaluate(new, train, test, RigidBodyModel)))
    after = {p: sha(root/p) for p in protected}
    assert after == protected, 'Protected active runtime/source/learning changed during offline study'
    summary = dict(schema=1, motion_issued=False, network_used=False,
        runtime_model_replaced=False, candidate_promoted=False,
        independent_hold_pose_groups_before=3, independent_hold_pose_groups_after=4,
        new_independent_pose_groups=1, new_joint_torque_proxy_records=7,
        total_added_joint_torque_proxy_records=28, geometry_target_labels=False,
        geometry_refitted=False, per_square_offsets_fitted=False,
        baseline_reproduction_max_parameter_abs_difference=baseline_error,
        baseline_training=base_report, candidate_training=candidate_report,
        a8_current_runtime_heldout=evaluate(active, original, a8, RigidBodyModel),
        grouped_cross_validation=cv,
        baseline_grouped_heldout_rmse_Nm=float(np.sqrt(np.mean([r['baseline']['rmse_Nm']**2 for r in cv]))),
        prior_three_group_heldout_rmse_Nm=float(np.sqrt(np.mean([r['prior_three_group_comparator']['rmse_Nm']**2 for r in cv]))),
        augmented_grouped_heldout_rmse_Nm=float(np.sqrt(np.mean([r['augmented']['rmse_Nm']**2 for r in cv]))),
        sources=source, protected_runtime_sha256=protected,
        protected_runtime_unchanged=True,
        decision=dict(accepted_reference_curated=True, offline_refit_completed=True,
            runtime_candidate_promoted=False,
            reason='Torque feedback proxies and four pose groups support an offline candidate, not automatic promotion. Entire-group holdout is reported; no independently measured XYZ, mass or load-grasp labels.',
            fitting_does_not_certify_pawn_grasp_or_physical_reach_limit=True))
    summary['grouped_rmse_change_percent'] = 100*(summary['augmented_grouped_heldout_rmse_Nm']/summary['baseline_grouped_heldout_rmse_Nm']-1)
    summary['folds_worse'] = [r['heldout_experiment'] for r in cv if r['augmented']['rmse_Nm'] > r['baseline']['rmse_Nm']]
    summary['incremental_a8_cv_comparison'] = dict(
        note='For A8 heldout, prior-three and augmented folds are the identical training set: A8 is never used to assess its own generalization.',
        grouped_rmse_change_percent=100*(summary['augmented_grouped_heldout_rmse_Nm']/summary['prior_three_group_heldout_rmse_Nm']-1),
        folds_worse=[r['heldout_experiment'] for r in cv if r['augmented']['rmse_Nm'] > r['prior_three_group_comparator']['rmse_Nm']])
    dump(out/'fit_summary.json', summary)
    print(json.dumps({k: summary[k] for k in ('independent_hold_pose_groups_after',
        'baseline_reproduction_max_parameter_abs_difference', 'baseline_grouped_heldout_rmse_Nm',
        'augmented_grouped_heldout_rmse_Nm', 'folds_worse', 'decision')}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    run(args.root.resolve(), args.output.resolve())
