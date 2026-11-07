"""Index frozen C4/C5 evidence and calibration eligibility; no robot access."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from left_arm_board_kinematics import tcp_pose
from left_arm_rigid_body import RigidBodyModel


def audit(root):
    def read(p):
        return json.loads((root/p).read_text(encoding='utf-8-sig'))
    def record(p):
        path = root/p
        return dict(path=p, bytes=path.stat().st_size,
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    c4 = read('outputs/board_drift_20261102/accepted_v24_sync_manifest.json')
    c5 = read('outputs/c5_test_20261106/accepted_summary.json')
    cfg = read('scripts/data/left_arm_board_grasp.json')
    m = RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json')
    geometry = read('scripts/data/chess_crown_geometry_v2_8.json')
    anchors = []
    for section in ('grasp_anchors', 'placement_anchors'):
        for key, row in geometry.get(section, {}).items():
            if row.get('board_tcp_mm') is None:
                continue
            q = [row['pose_rad'][n] for n in m.names]
            pose = tcp_pose(m, q, cfg['tcp_in_wrist_frame_m'])
            prediction = (pose[:3, 3]-cfg['board_to_cad_translation_m'])@np.array(cfg['board_to_cad_rotation'])*1000
            anchors.append(dict(id=section+'/'+key, board_tcp_mm=row['board_tcp_mm'],
                validated_flag=row.get('validated'),
                current_model_residual_mm=(prediction-row['board_tcp_mm']).tolist(),
                usable_as_numerical_seed=True, usable_as_exact_tcp_measurement=False,
                reason='historical taught contact convention is not independently matched to the current fingertip-plane TCP'))
    hold = read('scripts/data/left_arm_v3_1_dynamics.json')
    sample_poses = [sample['pose_rad'] for row in hold['joints'].values()
                    for field in ('gravity_samples', 'inertia_samples') for sample in row.get(field, [])
                    if all(name in sample.get('pose_rad', {}) for name in m.names)]
    recorded = np.array([[pose[name] for name in m.names] for pose in sample_poses])
    bounds = np.asarray(cfg['joint_limits_encoder_rad'])
    coverage = {name: dict(observed_min_rad=float(recorded[:, i].min()),
                          observed_max_rad=float(recorded[:, i].max()),
                          existing_bounds_rad=bounds[i].tolist(),
                          samples_outside_existing_bounds=int(np.sum((recorded[:, i] < bounds[i, 0])
                                                                     | (recorded[:, i] > bounds[i, 1]))))
                for i, name in enumerate(m.names)}
    frozen = [
        'outputs/board_drift_20261102/accepted_v24_sync_manifest.json',
        'outputs/board_drift_20261102/jetson_snapshot_v24_accepted/left_arm_board_grasp.json',
        'outputs/board_drift_20261102/jetson_snapshot_v24_accepted/left_arm_general_dynamics.json',
        'outputs/board_drift_20261102/jetson_snapshot_v24_accepted/left_arm_v3_1_dynamics.json',
        'outputs/c5_test_20261106/accepted_summary.json',
        'outputs/c5_test_20261106/accepted_run_result.json',
        'outputs/c5_test_20261106/accepted_collection_manifest.json',
        'outputs/c5_test_20261106/accepted_jetson_raw/data/left_arm_board_grasp.json',
        'outputs/c5_test_20261106/accepted_jetson_raw/data/left_arm_general_dynamics.json',
        'outputs/c5_test_20261106/accepted_jetson_raw/data/left_arm_v3_1_dynamics.json',
        'outputs/c5_test_20261106/source_snapshots/accepted_active_set/left_arm_board_grasp.py',
        'outputs/c5_test_20261106/source_snapshots/accepted_active_set/left_arm_board_kinematics.py']
    frozen += [str(p.relative_to(root)).replace('\\', '/') for p in
               (root/'outputs/board_workspace_20261106/baseline').glob('*') if p.is_file()]
    c4_target = None
    # Resolve the exact accepted raw logs by content hash, not a guessed date.
    for trial, checksum, folder in [('C4', c4['source_log_sha256'], 'outputs/board_drift_20261102/motion_control_logs'),
                                     ('C5', c5['log_sha256'], 'outputs/c5_test_20261106/accepted_jetson_raw')]:
        matches = [p for p in (root/folder).rglob('*.txt')
                   if hashlib.sha256(p.read_bytes()).hexdigest() == checksum]
        if not matches:
            raise ValueError('accepted '+trial+' log hash not found')
        frozen.append(str(matches[0].relative_to(root)).replace('\\', '/'))
        if trial == 'C4':
            lines = matches[0].read_text(encoding='utf-8').splitlines()
            plans = [json.loads(line.split('=', 1)[1]) for line in lines if line.startswith('board plan=')]
            c4_target = plans[-1]['target_board_mm']
    return dict(schema=1, motion_issued=False, runtime_files_replaced=False,
        trials=[dict(id='C4-V24', model_sha256=c4['model_sha256'], target_board_mm=c4_target,
                     observation='operator accepted about 2mm B drift, visible board gap, no oscillation',
                     exact_external_endpoint_measurement_available=False, loaded_grasp_validated=False),
                dict(id='C5-active-set', model_sha256=c5['board_model_sha256'],
                     target_board_mm=c5['target_board_mm'], observation=c5['operator_observation'],
                     exact_external_endpoint_measurement_available=False, loaded_grasp_validated=False)],
        frozen_files=[record(p) for p in sorted(set(frozen))],
        historical_correspondences=anchors,
        dynamics_data_audit=dict(raw_pose_records=len(sample_poses), operating_bound_coverage=coverage,
            existing_shared_parameter_fit=m.data.get('identification'),
            action='retain shared fit; existing bounds already contain all recorded gravity/inertia poses'),
        fitting_policy=dict(shared_geometry=True, per_square_bias_fit=False,
            optical_board_pose=cfg['optical_registration'], surface_datum=cfg['touch_plane_datum'],
            current_action='retain existing optical fit and touch datum; no new exact XYZ samples available',
            reason='encoder FK is a model prediction, not independent position ground truth; qualitative C5 acceptance is validation evidence'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(frozen_files=len(result['frozen_files']),
                         historical_correspondences=len(result['historical_correspondences']), motion_issued=False)))
