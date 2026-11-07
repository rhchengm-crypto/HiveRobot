"""Build a shared OFFLINE torso-aware candidate; never replace the live model.

The older chassis mesh has no confirmed registration to this seven-axis arm.
Its OBB is therefore a traceable nominal candidate, not a measured obstacle.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

from left_arm_rigid_body import RigidBodyModel


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def binary_stl_bounds(path):
    data = Path(path).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84+count*50:
        raise ValueError('expected binary STL with explicit triangle count')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    vertices = np.ndarray(count, dtype=dtype, buffer=data, offset=84)['vertices'].reshape(-1, 3)
    if not np.isfinite(vertices).all():
        raise ValueError('nonfinite STL vertices')
    return vertices.min(0).astype(float), vertices.max(0).astype(float)


def build(root, output):
    model_path = root/'scripts/data/left_arm_general_dynamics.json'
    board_path = root/'scripts/data/left_arm_board_grasp.json'
    hardware = root/'roboparty_rpo_hardware/rpo_hardware-main/V2.0/roboto_origin_mechanic/03_URDF'
    urdf_path = hardware/'urdf/roboto_origin.urdf'
    mesh_path = hardware/'meshes/torso_link.STL'
    model = RigidBodyModel.load(model_path)
    cfg = json.loads(board_path.read_text(encoding='utf-8'))
    urdf = ET.parse(urdf_path).getroot()
    joint = urdf.find("joint[@name='left_arm_pitch_joint']")
    if joint.find('origin').get('rpy') != '0 0 0' or joint.find('axis').get('xyz') != '0 1 0':
        raise ValueError('unexpected chassis shoulder convention; registration needs review')
    origin = np.fromstring(joint.find('origin').get('xyz'), sep=' ')
    # URDF Y is the front/pitch axis. Nominal CAD installation gravity fixes Z;
    # these two orthogonal directions fix X without using any square endpoint.
    y = model.axes[0]
    z = -model.configured_gravity()
    z /= np.linalg.norm(z)
    orthogonality_error = float(abs(y@z))
    z -= y*(y@z)
    z /= np.linalg.norm(z)
    axes = np.column_stack([np.cross(y, z), y, z])
    translation = model.points[0]-axes@origin
    low, high = binary_stl_bounds(mesh_path)
    center = axes@((low+high)/2)+translation
    # Check the second shoulder datum as independent evidence of mismatch.
    side = urdf.find("joint[@name='left_arm_roll_joint']")
    side_offset = np.fromstring(side.find('origin').get('xyz'), sep=' ')
    side_prediction = model.points[0]+axes@side_offset
    mismatch = model.points[1]-side_prediction
    # Motor axis origins are gauge-dependent along the shaft. An origin
    # mismatch is not a measured mounting error. Preserve the first axis LINE
    # and optimize its unconstrained axial translation against the second line.
    side_axis = model.axes[1]
    perpendicular = np.eye(3)-np.outer(side_axis, side_axis)
    projected_y = perpendicular@y
    shift = float(projected_y@(perpendicular@mismatch)/(projected_y@projected_y))
    translation += shift*y
    center = axes@((low+high)/2)+translation
    axis_line_residual = perpendicular@(mismatch-shift*y)
    provenance = {
        'source_files': {str(p.relative_to(root)): sha(p) for p in (model_path, board_path, urdf_path, mesh_path)},
        'method': 'align first shoulder axis line and nominal up; fit free axial translation to second shoulder axis line; mesh AABB becomes OBB',
        'frame_registration_physically_verified': False,
        'pitch_axis_nominal_up_dot_abs': orthogonality_error,
        'initial_coordinate_origin_difference_mm': (mismatch*1000).tolist(),
        'initial_coordinate_origin_difference_norm_mm': float(np.linalg.norm(mismatch)*1000),
        'origin_difference_is_not_physical_mounting_error': True,
        'first_axis_free_axial_translation_mm': shift*1000,
        'second_axis_line_residual_mm': (axis_line_residual*1000).tolist(),
        'second_axis_line_residual_norm_mm': float(np.linalg.norm(axis_line_residual)*1000),
        'second_axis_direction_disagreement_deg': float(np.degrees(np.arccos(np.clip(axes[:, 0]@side_axis, -1, 1)))),
        'mesh_bounds_in_torso_frame_m': [low.tolist(), high.tolist()],
        'torso_to_cad_rotation': axes.tolist(), 'torso_to_cad_translation_m': translation.tolist(),
        'limitations': ['old five-axis chassis mesh is not the approved seven-axis assembly',
                       'chassis mounting registration requires verification before hardware use',
                       '30mm link-axis capsules are simplified; motors, wires, supports and finger sweep are incomplete',
                       'no new exact Cartesian measurements were inferred from qualitative C4/C5 observations']}
    cfg['build'] = 'seven-axis-board-workspace-v5-full-tool-orientation-candidate'
    cfg.pop('c5_posture_trial', None)
    cfg.pop('ik_soft_posture_seed_tolerance_deg', None)
    cfg['workspace_planning_policy'] = 'shared-body-constrained-IK'
    cfg['motion_feedforward_policy'] = 'single-entry-gravity-anchor'
    cfg['lower_elbow_nullspace'] = True
    cfg['posture_cost_weights'] = {'elbow_height_per_m': 100.,'nominal_gravity_squared_per_Nm2': .02,
        'scope': 'same weights for every XY; lower elbow and estimated static load preferred; not measured motor ratings'}
    hold_path = root/'scripts/data/left_arm_v3_1_dynamics.json'
    hold = json.loads(hold_path.read_text(encoding='utf-8'))
    observed_elbow = [float(row['pose_rad']['elbow']) for joint in hold['joints'].values()
        for quantity in ('gravity_samples','inertia_samples') for row in joint.get(quantity,[])
        if len(row.get('pose_rad',{}))==model.n and 'elbow' in row['pose_rad']]
    if not observed_elbow:
        raise ValueError('accepted full-pose elbow evidence required')
    index = model.names.index('elbow')
    old_upper = cfg['joint_limits_encoder_rad'][index][1]
    cfg['joint_limits_encoder_rad'][index][1] = min(old_upper,max(observed_elbow))
    cfg['joint_bound_source'] += '; elbow upper provisionally restricted to accepted full-pose observations; NOT a measured mechanical limit'
    cfg['operating_bound_adjustments'] = {'elbow':{
        'old_upper_rad':old_upper,'upper_rad':cfg['joint_limits_encoder_rad'][index][1],
        'source_model_sha256':sha(hold_path),
        'reason':'provisional observed upper envelope after high-load uncontacted trial beyond observed poses',
        'mechanical_limit_independently_measured':False,'applies_to_all_xy':True}}
    clearance_path = root/'scripts/data/left_arm_v2_table_clearance.json'
    clearance = json.loads(clearance_path.read_text(encoding='utf-8'))['joints']
    cfg['required_clearance_entry'] = {
        'policy': 'existing-staged-clearance-before-every-approach',
        'reference_encoder_rad': {n:clearance[n] for n in model.names if n!='shoulder_rotate'},
        'source_sha256': sha(clearance_path), 'tolerance_deg': 5.,
        'preserve_ordinary_clearance_skip': ['shoulder_rotate'],
        'purpose': 'entry posture gate only; target remains continuous geometric IK'}
    cfg['support_surface'] = {'enabled': True, 'policy': 'global-board-height-halfspace',
        'source': 'conservative continuation of registered board plane across all XY; tabletop outline unknown',
        'entry_policy': 'existing staged Clearance first if starting below support surface',
        'physically_validated': False}
    cfg['planning_time_budget_s'] = 120.
    cfg['tool_orientation_constraint'] = 'full-wrist'
    cfg['orientation_constraint_reason'] = ('Preserve the complete CAD gripper frame, not only its axis; '
        'v4.1 B3 selected a 44deg free-axis rotation and operator rejected the physical gripper posture. '
        'Common orientation references remain solver task constraints for all XY, never joint replay.')
    tool_axis = model.home[-1, :3, :3].T@np.asarray(cfg['tcp_datum']['axis_in_cad'])
    cfg['tool_axis_in_wrist_frame'] = (tool_axis/np.linalg.norm(tool_axis)).tolist()
    cfg['workspace_scope'] = 'A1-D8 continuous XY; common model; offline candidate, no per-square bounds'
    cfg['collision_validation'] = 'whole link/tool segments against finite board; bounded joint-interval subdivision; declared envelopes only'
    cfg['body_collision'] = {
        'schema': 1, 'enabled': True, 'frame': 'cad_base', 'physically_validated': False,
        'registration_status': 'nominal-old-chassis-registration; offline-only',
        'provenance': provenance,
        'obstacles': [dict(name='torso_mesh_obb', center_m=center.tolist(), axes_in_cad=axes.tolist(),
                           half_extents_m=((high-low)/2).tolist())],
        # Exclude mounting/adjacent shoulder interfaces; distal arm and tool
        # remain checked at every pose and sampled edge.
        'capsules': [dict(name=model.names[i]+'-to-'+(model.names[i+1] if i+1 < model.n else 'tcp'),
                          start_index=i, end_index=i+1, radius_m=.03, margin_m=.005)
                     for i in range(2, model.n)]}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(cfg, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return dict(motion_issued=False, output=str(output), sha256=sha(output),
                runtime_model_replaced=False, registration=provenance)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.root, args.output), ensure_ascii=False, indent=2))
