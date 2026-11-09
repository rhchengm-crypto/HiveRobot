"""Record the completed trial without upgrading its unaccepted grip height."""
from datetime import datetime, timezone
import hashlib,json
from pathlib import Path

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    if (OUT/'operator_acceptance_update.json').exists():
        raise RuntimeError('Later operator acceptance supersedes this initial record; use outputs/a8_accepted_reference_20261108/register_acceptance.py and finalize_reference.py instead.')
    audit=read(OUT/'execution_audit.json')
    feedback=read(OUT/'operator_feedback.json')
    prepared=read(OUT/'verified_plan.json')
    restore=read(OUT/'config_restore_receipt.json')
    further=read(OUT/'next10_diagnostic/diagnostic.json')
    relative=read(OUT/'feedback_relative10_diagnostic/diagnostic.json')
    assert len(audit['checks'])==23 and all(audit['checks'].values())
    assert audit['web_run']['returncode']==0 and not audit['web_run']['running']
    assert audit['inline_hold']['ok'] and not feedback['height_acceptable']
    assert feedback['jitter'] is False and feedback['contact'] is False
    assert not further['motion_issued'] and not further['endpoint_passed']
    assert not relative['motion_issued'] and not relative['endpoint_passed']
    assert sha(ROOT/'scripts/data/left_arm_board_workspace_candidate.json')==restore['original_sha256']
    record=dict(schema=1,kind='operator-observed-empty-load-validation',
        recorded_at_utc=datetime.now(timezone.utc).isoformat(),
        build='seven-axis-board-grasp-v33-quasistatic-support',
        status='stable complete new-frame approach/descent; grip height still approximately10mm high',
        physical_validation_accepted=False,height_passed=False,fitting_eligible=False,
        successful_path_library_updated=False,motion_stability_accepted=True,
        user_request='现在实测',request=prepared['request'],
        request_kind='explicit lower-height/new-frame task, not nominal A8-centre registration validation',
        original_nominal_A8_XYZ_solved=False,
        operator_feedback=feedback,
        motion=dict(action='dynamics:board-grasp',returncode=0,duration_s=124.782,
            clearance_log='arm_run_1794198389715_49164.txt',clearance_hold_log='arm_run_1794198415914_49291.txt',
            primary_log='arm_run_1794198443568_49465.txt',trace='trace_1794198451214228486.jsonl',
            ordinary_Clearance_executed=True,ordinary_Clearance_skip_shoulder_rotate_preserved=True,
            full_path_precomputed_before_Clearance=True,raw_waypoints=47,execution_groups=24,
            all_contact_points_executed=21,no_claw=True,same_serial_Model_Hold=True,
            hold_duration_s=audit['inline_hold']['duration_s'],
            motor_frames=audit['trace']['motor_frames'],
            event_records_including_close_footer=audit['trace']['json_records'],
            event_records_excluding_close_footer=audit['trace']['records_excluding_close_footer'],
            dropped_frames=0,confirmed_oscillations=0,fault_support_frames=0,
            static_windows_independently_reproduced=22,endpoint_correction_retries=0,
            all_descent_gains_frozen=True,shoulder_rotate_gains=[36.,4.5],
            endpoint_max_joint_error_deg=audit['endpoint']['max_nominal_error_deg']),
        model_predictions_not_physical_measurements=audit['model_predictions_not_external_measurements'],
        exact_prepared_source_hashes=prepared['source_hashes'],
        trial_board_model_sha256=prepared['common_config_sha256'],
        restored_board_model_sha256=restore['original_sha256'],
        original_shared_configuration_restored=True,
        config_restore_changes_motor_commands=False,
        geometry_TCP_board_transform_or_per_square_offsets_changed=False,
        saved_angles_replayed=False,
        persistent_learning_preserved=True,
        next10_diagnostic=dict(report='outputs/a8_lower25_execution_20261108/next10_diagnostic/diagnostic.json',
            target_basis='previous requested target minus10mm; not an actual-held-pose relative descent',
            endpoint_passed=False,full_path_produced=False,motion_issued=False,
            target_board_mm=further['target_board_mm'],bounds_or_orientation_relaxed=False,
            finite_failure_is_physical_unreachability_proof=False),
        feedback_relative10_diagnostic=dict(
            report='outputs/a8_lower25_execution_20261108/feedback_relative10_diagnostic/diagnostic.json',
            target_basis='actual final held encoder FK position/orientation; preserve model XY and lower model Z by10mm',
            target_board_mm=[9.639697000830376,406.65055148807653,52.17518852976162],
            endpoint_passed=False,short_path_produced=False,motion_issued=False,
            bounds_or_orientation_relaxed=False,calibrated_physical_board_datum=False,
            finite_failure_is_physical_unreachability_proof=False),
        independent_audit=dict(checks_passed=23,total_checks=23,
            path='outputs/a8_lower25_execution_20261108/execution_audit.json',
            sha256=sha(OUT/'execution_audit.json')),
        evidence_directory='outputs/a8_lower25_execution_20261108',
        documentation='outputs/a8_lower25_execution_20261108/README.md',
        additional_motion_after_this_trial=False,GitHub_pushed_this_turn=False)
    path=ROOT/'scripts/data/board_workspace_trials/validation_20261108_a8_lower25.json'
    path.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (OUT/'trial_summary.json').write_bytes(path.read_bytes())
    print(json.dumps(dict(registry=str(path.relative_to(ROOT)),sha256=sha(path),
        physical_height_passed=False,motion_stability_accepted=True)))


if __name__=='__main__':main()
