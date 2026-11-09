"""Offline logged-feedback height comparison; no hardware or runtime writes."""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import hashlib
import json
import math
from pathlib import Path
import sys
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(OUT/'source_snapshot/scripts'))
import numpy as np
from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose, rotation_error


def read(path):
    return json.loads(Path(path).read_text(encoding='utf8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def tagged(path, prefix):
    rows = [json.loads(line.split('=', 1)[1]) for line in Path(path).read_text(encoding='utf8').splitlines()
            if line.startswith(prefix+'=')]
    if not rows:
        raise ValueError((str(path), prefix))
    return rows[-1]


def main():
    old_out = ROOT/'outputs/a8_reference_execution_20261108'
    old_log = old_out/'control_evidence/data/left_arm_v2_8_full_runs/arm_run_1794185510062_30907.txt'
    latest = read(OUT/'latest_poll.json')
    new_log = OUT/'control_evidence/data/left_arm_v2_8_full_runs'/Path(latest['full_log_path']).name
    if not new_log.exists():
        raise SystemExit('Awaiting collector download: '+str(new_log))
    model_path = OUT/'source_snapshot/scripts/data/left_arm_general_dynamics.json'
    cfg_path = OUT/'source_snapshot/scripts/data/left_arm_board_workspace_candidate.json'
    model = RigidBodyModel.load(model_path)
    cfg, old_cfg = read(cfg_path), read(old_out/'board_workspace_candidate.json')
    common_fields = ('tcp_in_wrist_frame_m', 'board_to_cad_rotation', 'board_to_cad_translation_m',
                     'gripper_vertex_envelope', 'joint_limits_encoder_rad')
    conformance = {key: cfg[key] == old_cfg[key] for key in common_fields}
    assert all(conformance.values())
    names = model.names
    r, t, tcp = np.array(cfg['board_to_cad_rotation']), np.array(cfg['board_to_cad_translation_m']), np.array(cfg['tcp_in_wrist_frame_m'])
    vertices = np.array(cfg['gripper_vertex_envelope']['vertices_in_wrist_frame_m'])
    saved_path = ROOT/'outputs/a8_saved_reference_20261108/saved_pose_audit.json'
    saved = read(saved_path)['saved_move']['pose']
    q_saved = np.array([saved[name] for name in names])

    def pose(q):
        return tcp_pose(model, q, tcp)

    def xyz(q):
        return (pose(q)[:3, 3]-t)@r*1000

    def describe(q):
        frame = model.fk(model.encoder_to_model(q))[-1]
        independent = model.fk_poe(model.encoder_to_model(q))[-1]
        lowest = float(np.min((vertices@frame[:3, :3].T+frame[:3, 3]-t)@r[:, 2])*1000)
        return dict(encoder_rad=dict(zip(names, q.tolist())), predicted_TCP_board_mm=xyz(q).tolist(),
                    predicted_lowest_CAD_gripper_vertex_height_mm=lowest,
                    orientation_difference_from_saved_deg=float(np.degrees(np.linalg.norm(rotation_error(pose(q_saved)[:3, :3], pose(q)[:3, :3])))),
                    FK_vs_independent_POE_max_matrix_difference=float(np.max(np.abs(frame-independent))))

    observations = {}
    endpoints = {}
    qs = {}
    for label, log in (('v32_previous', old_log), ('v33_current', new_log)):
        endpoint = tagged(log, 'board endpoint report')
        hold = tagged(log, 'board same-session hold complete')
        actual = np.array([endpoint['measured_pose_rad'][name] for name in names])
        held = np.array([hold['final'][name]['pos'] for name in names])
        nominal = np.array([endpoint['nominal_target_rad'][name] for name in names])
        assert np.array_equal(nominal, q_saved)
        endpoints[label] = endpoint
        qs[label] = actual
        observations[label] = dict(log=str(log.relative_to(ROOT)).replace('\\', '/'), log_sha256=sha(log),
            same_exact_saved_joint_target=True, endpoint_max_joint_error_deg=endpoint['max_nominal_error_deg'],
            motion_endpoint=describe(actual), inline_hold_final=describe(held),
            hold_minus_motion_endpoint_predicted_mm=(xyz(held)-xyz(actual)).tolist(),
            terminal_command=endpoint['model_hold_motion_gains'],
            support_bias=endpoint.get('quasistatic_support'),
            hold_duration_s=hold['duration_s'], hold_ok=hold['ok'],
            hold_terminal_origin_rad={name:hold.get('motion_gain_floor', {}).get(name, {}).get('q') for name in names},
            physical_position_measured=False)

    # Exact Shapley decomposition averages each joint's marginal FK change over
    # all 7! update orders. 128 FK calls suffice; sum equals total nonlinear delta.
    old_q, new_q = qs['v32_previous'], qs['v33_current']
    values = {}
    for mask in range(1 << len(names)):
        value = old_q.copy()
        for k in range(len(names)):
            if mask & (1 << k):
                value[k] = new_q[k]
        values[mask] = xyz(value)
    contributions = {}
    for k, name in enumerate(names):
        change = np.zeros(3)
        for mask in range(1 << len(names)):
            if mask & (1 << k):
                continue
            size = mask.bit_count()
            weight = math.factorial(size)*math.factorial(len(names)-size-1)/math.factorial(len(names))
            change += weight*(values[mask | (1 << k)]-values[mask])
        contributions[name] = dict(feedback_change_deg=float(np.degrees(new_q[k]-old_q[k])),
            previous_error_from_saved_deg=float(np.degrees(old_q[k]-q_saved[k])),
            current_error_from_saved_deg=float(np.degrees(new_q[k]-q_saved[k])),
            exact_order_averaged_FK_displacement_contribution_mm=change.tolist(),
            height_contribution_mm=float(change[2]))
    total = xyz(new_q)-xyz(old_q)
    np.testing.assert_allclose(np.sum([row['exact_order_averaged_FK_displacement_contribution_mm'] for row in contributions.values()], axis=0), total, atol=1e-10)
    gain_match = {name: all(endpoints['v32_previous']['model_hold_motion_gains'][name][key] ==
        endpoints['v33_current']['model_hold_motion_gains'][name][key] for key in ('kp', 'kd')) for name in names}
    capture_log = ROOT/'outputs/a8_saved_reference_20261108/source_snapshot/arm_run_1794184334719_27716.txt'
    capture_source = ROOT/'scripts/left_arm_v2_6_move_library.py'
    read_source = ROOT/'scripts/left_arm_v2_6.py'
    replay_source = ROOT/'scripts/left_arm_v2_8_move_library.py'
    feedback_path = OUT/'operator_feedback.json'
    result = dict(schema=1, motion_issued=False, hardware_accessed=False, network_accessed=False,
        geometry_or_offsets_or_runtime_code_modified=False,
        observation_scope='Offline common-model encoder-FK predictions, not physical TCP/contact-height metrology',
        operator_observation=read(feedback_path),
        additional_user_height_observation=dict(verbatim='当前夹持区域至少还高25mm',
            approximate_visual_gap_lower_bound_mm=25., calibrated_board_plane_measurement=False,
            exact_TCP_Z_measurement=False, another_motion_authorized=False),
        reference_endpoint=describe(q_saved), trials=observations,
        current_minus_previous_predicted_TCP_board_mm=total.tolist(),
        exact_joint_contributions=contributions, same_endpoint_Kp_Kd=gain_match,
        geometry_conformance=conformance,
        expected_height_rise_if_previous_tracking_error_removed_mm=float(xyz(q_saved)[2]-xyz(old_q)[2]),
        current_predicted_height_error_from_saved_reference_mm=float(xyz(new_q)[2]-xyz(q_saved)[2]),
        capture_semantics=dict(stored_pose_is_encoder_feedback_not_command_reference=True,
            chain='Web capture-move -> v3.0/v2.8 non-replay delegation -> v2.6 capture_move -> arm.positions -> read_status -> reply-cache getPosition',
            capture_body='capture_move stores pose=arm.positions(DEFAULT_JOINTS); it does not command enable or a target',
            read_body='read_status refreshes seven motor statuses, waits20ms, receives replies, returns getPosition/getVelocity/getTorque; positions selects pos',
            capture_log_sha256=sha(capture_log), captured_source_log_matches_saved_pose=True,
            missing_capture_fields=['individual packet timestamps and sequences', 'per-axis freshness certificate',
                'velocity and torque snapshot', 'simultaneous stable-window evidence', 'actual MIT q/Kp/Kd/FF law',
                'gripper opening', 'identified physical gripping/contact point and measured Z'],
            physical_capture_pose_and_grasp_datum_independently_revalidated=False,
            source_version_caveat='Capture log and seven saved q are frozen. Capture implementation chain inspected in current local files; those move-library source bytes were not included in original capture evidence.',
            inspected_local_source_sha256={str(path.relative_to(ROOT)).replace('\\', '/'):sha(path) for path in (capture_source, read_source, replay_source)}),
        replay_semantics=dict(replay_is_distinct_from_capture=True,
            description='v2.6 biased_replay_target uses stored feedback pose plus adaptive bias; v2.8 combines inherited shared bias with pose-local bias and can alter held targets/torques. Thus successful legacy replay is not proof that its command q or final feedback equals stored pose.',
            these_board_trials_used_legacy_saved_angle_replay=False,
            these_board_trials_targeted_exact_saved_q_with_new_geometric_waypoints=True),
        support_and_hold_semantics=dict(v33_static_residual_is_session_local=True,
            last_learned_bias_is_not_necessarily_applied=True,
            actual_applied_bias_source='endpoint.quasistatic_support.last_completed_segment_bias_tau',
            pending_bias_source='endpoint.quasistatic_support.pending_unapplied_bias_tau',
            hold_transfer_requires_comparing_actual_motion_endpoint_and_final_hold_feedback=True),
        conclusion='Steady tracking improved toward the same saved feedback endpoint. The unchanged model predicts that removing the old tracking error raises this endpoint; the measured-joint FK comparison and exact per-axis decomposition quantify that rise. This explains the direction of the new higher observation without proving physical millimetres. The taught endpoint/contact-datum suitability remains unresolved; no Z50.2 or common-plane correction follows.',
        next_step_scope='Use the explicit observed height request only for a separately labelled offline geometric candidate. Do not adjust runtime board registration/TCP or hide a per-square compensation, and do not execute without a new user motion instruction.',
        source_sha256={str(path.relative_to(ROOT)).replace('\\', '/'):sha(path) for path in
            (model_path, cfg_path, old_out/'board_workspace_candidate.json', saved_path, old_log, new_log, feedback_path)},
        physical_grasp_contact_Z_mm=None, nominal_A8_XYZ_consistency_resolved=False)
    target = OUT/'height_diagnosis.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(dict(previous=observations['v32_previous']['motion_endpoint']['predicted_TCP_board_mm'],
        current=observations['v33_current']['motion_endpoint']['predicted_TCP_board_mm'],
        current_minus_previous_mm=total.tolist(), reference=xyz(q_saved).tolist(),
        current_hold_minus_motion_mm=observations['v33_current']['hold_minus_motion_endpoint_predicted_mm'],
        height_contributions={name:row['height_contribution_mm'] for name,row in contributions.items()},
        same_gains=all(gain_match.values()), output=str(target)), ensure_ascii=True, indent=2))


if __name__ == '__main__':
    main()
