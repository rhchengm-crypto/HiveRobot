"""Record verified Home, common search change and a separate inactive proposal."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    deployment = read(OUT/'deployment/deployment_receipt.json')
    proposal = read(OUT/'proposal/proposal_report.json')
    assert deployment['ok'] and not deployment['motion_issued']
    assert deployment['offline_hardware_free_tests']['ok']
    assert not proposal['executable'] and not proposal['motion_issued']
    config = ROOT/'scripts/data/left_arm_board_workspace_candidate.json'
    assert sha(config) == 'ff4d75df0ef85964ba153d267c5547ba036f63d075409ca0582174c6e836164d'
    for name, expected in deployment['deployed_sha256'].items():
        assert sha(ROOT/'scripts'/name) == expected
        frozen = OUT/'source_snapshot'/name
        frozen.parent.mkdir(parents=True, exist_ok=True)
        frozen.write_bytes((ROOT/'scripts'/name).read_bytes())
    snapshot_config = OUT/'source_snapshot/data/left_arm_board_workspace_candidate.json'
    snapshot_config.parent.mkdir(parents=True, exist_ok=True)
    snapshot_config.write_bytes(config.read_bytes())
    log = (OUT/'regression_tests.log').read_text(encoding='utf-8')
    assert 'Ran 157 tests in 34.797s' in log and '\nOK\n' in log
    home = read(ROOT/'outputs/a8_v33_home_20261108/latest_poll.json')
    assert home['action'] == 'dynamics:model-hold-after:home' and not home['running']
    assert home['returncode'] == 0 and home['post_move_model_hold']['state'] == 'complete'
    record = dict(schema=1, kind='common-geometric-search-fix-and-inactive-height-proposal',
        recorded_at_utc=datetime.now(timezone.utc).isoformat(),
        status='search fix deployed; lower-height new-frame proposal inactive and physically unvalidated',
        physical_validation_complete=False, fitting_eligible=False,
        successful_path_library_updated=False,
        Home=dict(completed=True, motion_log='arm_run_1794188641610_37144.txt',
            hold_log='arm_run_1794188707909_37371.txt', returncode=0,
            following_hold_seconds=5, operator_feedback=None,
            physical_vibration_free_claimed=False,
            evidence='outputs/a8_v33_home_20261108/download_verification.json'),
        fixed_defect='axis-only seed objective did not match strict complete-wrist terminal constraint; full-frame seed search and candidate refinement added',
        numerical_acceptance_tolerances_relaxed=False,
        geometry_TCP_or_board_transform_changed=False, per_square_offset_added=False,
        stored_joint_trajectory_replayed=False, Kp_Kd_changed=False,
        persistent_learning_changed_by_fix=False,
        same_serial_Model_Hold_preserved=True, v33_quasistatic_support_preserved=True,
        ordinary_Clearance_skip_shoulder_rotate_preserved=True,
        local_regression=dict(passed=True, tests=157, duration_s=34.797,
            log='outputs/a8_height_followup_20261108/regression_tests.log'),
        Jetson_hardware_free_regression=deployment['offline_hardware_free_tests'],
        deployed_source_sha256=deployment['deployed_sha256'],
        deployment_receipt='outputs/a8_height_followup_20261108/deployment/deployment_receipt.json',
        height_diagnosis=dict(operator_gap_approximate_lower_bound_mm=25,
            old_reference_endpoint_board_mm=[11.798050592563328,405.6006774366655,84.62267445748284],
            explicit_lower_height_target_board_mm=proposal['target_board_mm'],
            previous_motion_to_new_motion_predicted_height_change_mm=18.89253544,
            subsequent_same_serial_hold_predicted_height_change_mm=.443254,
            calibrated_physical_height_known=False,
            nominal_A8_XYZ_geometry_consistency_resolved=False,
            current_policy_lower_height_path_solved=False,
            mathematical_infeasibility_proven=False),
        inactive_proposal=dict(report='outputs/a8_height_followup_20261108/proposal/proposal_report.json',
            executable=False, deployed=False, executed=False,
            requested_new_full_frame_change_deg=proposal['new_full_frame_deviation_from_old_deg'],
            requested_new_tool_axis_change_deg=proposal['new_tool_axis_deviation_from_old_deg'],
            proposed_common_front_lower_extension_deg=12., other_six_bounds_unchanged=True,
            active_configuration_changed=False, mechanical_qualification_pending=True,
            same_geometry_and_strict_tracking_tolerances=True,
            independently_physically_validated=False),
        further_robot_motion_after_Home=False, GitHub_pushed_this_turn=False,
        documentation='outputs/a8_height_followup_20261108/README.md')
    destination = ROOT/'scripts/data/board_workspace_trials/fix_20261108_a8_full_frame_search.json'
    destination.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    (OUT/'fix_record.json').write_bytes(destination.read_bytes())
    print(json.dumps(dict(registry=str(destination.relative_to(ROOT)), sha256=sha(destination),
        local_tests=157, remote_tests=deployment['offline_hardware_free_tests']['tests_run'],
        proposal_active=False, motion_after_Home=False)))


if __name__ == '__main__':
    main()
