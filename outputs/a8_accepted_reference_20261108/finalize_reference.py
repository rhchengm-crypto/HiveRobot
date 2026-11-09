"""Promote the accepted reference metadata locally; issue no robot command."""
from pathlib import Path
import hashlib
import json
import shutil
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from build_left_arm_path_reference_library import build
from left_arm_board_task_pose import reference_applies_to_piece

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def read(p):
    return json.loads(p.read_text(encoding='utf-8-sig'))

def save(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

def main():
    registry_path = ROOT / 'scripts/data/board_workspace_trials/validation_20261108_a8_lower25.json'
    active_path = ROOT / 'scripts/data/left_arm_board_workspace_candidate.json'
    library_path = ROOT / 'scripts/data/left_arm_validated_path_library.json'
    reference_name = 'validated:20261108_a8_v33_lower25_nonpawn'
    assert (OUT / 'fitting/fit_summary.json').is_file(), 'Complete the offline fit audit before changing its protected configuration'
    fitting = read(OUT / 'fitting/fit_summary.json')
    previous_candidate = read(OUT / 'reference_build/workspace_reference_candidate.json')
    assert any(r['name'] == reference_name for r in previous_candidate['approved_task_pose_references'])
    before = OUT / 'before_acceptance'
    for name, source in [('active_workspace_candidate.json', active_path), ('active_validated_path_library.json', library_path)]:
        if not (before / name).exists():
            shutil.copyfile(source, before / name)
    base_path = before / 'active_workspace_candidate.json'
    assert sha(base_path) == 'ff4d75df0ef85964ba153d267c5547ba036f63d075409ca0582174c6e836164d'
    record = read(registry_path)
    record['successful_path_library_updated'] = True
    record['path_library_reference_name'] = reference_name
    record['path_library_role'] = 'shared IK full-pose and branch seed; every new target/path is re-solved'
    record['offline_fitting_evidence'] = 'outputs/a8_accepted_reference_20261108/fitting/fit_summary.json'
    record['running_dynamics_model_replaced'] = False
    save(registry_path, record)
    save(ROOT / 'outputs/a8_lower25_execution_20261108/trial_summary.json', record)
    library, candidate = build(ROOT, base_path)
    found = [r for r in candidate['approved_task_pose_references'] if r['name'] == reference_name]
    assert len(found) == 1
    reference = found[0]
    assert reference['seed_encoder_rad'] == record['encoder_endpoint_rad']
    assert reference['grasp_verified'] is False
    for color in ['white', 'black']:
        assert not reference_applies_to_piece(reference, color + '_pawn')
        assert all(reference_applies_to_piece(reference, color + '_' + kind) for kind in ['king', 'queen', 'rook', 'bishop', 'knight'])
    row = next(r for r in library['records'] if r['name'] == reference_name)
    assert row['reference_eligible'] and row['frame_compatible'] and not row['geometry_ground_truth']
    assert row['registry_sha256'] == sha(registry_path)
    original = read(base_path)
    changed = {k for k in original.keys() | candidate.keys() if original.get(k) != candidate.get(k)}
    allowed = {'approved_task_pose_references', 'validated_path_reference_policy', 'reference_attempt_budget_s', 'validated_path_library_provenance'}
    assert changed <= allowed, changed
    # No gain, bound, TCP, board transform, clearance or hold parameter is changed.
    for k in original.keys() - allowed:
        assert candidate[k] == original[k], k
    save(OUT / 'reference_build/validated_path_library.json', library)
    save(OUT / 'reference_build/workspace_reference_candidate.json', candidate)
    shutil.copyfile(OUT / 'reference_build/validated_path_library.json', library_path)
    shutil.copyfile(OUT / 'reference_build/workspace_reference_candidate.json', active_path)
    prior_names = {r['name'] for r in read(before / 'active_validated_path_library.json')['records']}
    added = [r['name'] for r in library['records'] if r['name'] not in prior_names]
    summary = dict(
        motion_issued=False, motor_or_web_command_issued=False, accepted=True,
        operator_acceptance_scope='executed empty-load A8 path/posture/current non-pawn height',
        additional_descent_executed=False, mechanical_limits_measured=False,
        grasp_verified=False, applicable_pieces=reference['applicable_pieces'], excluded_pieces=reference['excluded_pieces'],
        path_library_reference_name=reference_name, reference_eligible=True,
        registry_sha256=sha(registry_path), library_sha256=sha(library_path),
        active_workspace_before_sha256=sha(base_path), active_workspace_after_sha256=sha(active_path),
        changed_workspace_keys=sorted(changed),
        library_record_count=len(library['records']), eligible_reference_count=library['eligible_reference_count'],
        added_records=added, geometry_ground_truth=False, numeric_xyz_fitting_label_eligible=False,
        fitting_summary='fitting/fit_summary.json', fitting_summary_sha256=sha(OUT / 'fitting/fit_summary.json'),
        dynamics_model_promoted=False, joint_angle_replay=False, per_square_offset_added=False,
        model_hold_preserved=True, adaptive_learning_preserved=True,
        ordinary_clearance_shoulder_rotate_skip_preserved=True,
    )
    save(OUT / 'acceptance_summary.json', summary)
    print(json.dumps(summary, ensure_ascii=False))

if __name__ == '__main__':
    main()
