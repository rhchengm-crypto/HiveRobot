"""Build a separate nominal board/TCP model from STEP and existing teaching."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from inspect_left_arm_step import StepInspection
from left_arm_board_kinematics import rigid_fit
from left_arm_rigid_body import RigidBodyModel

ROOT = Path(__file__).resolve().parents[1]
STEP_SHA = 'f7729567ef511d1d9dc38b250dcd276ab09b7aa84fff98f49c0e068971ef9718'


def build(step, root=ROOT):
    step = Path(step)
    if hashlib.sha256(step.read_bytes()).hexdigest() != STEP_SHA:
        raise ValueError('STEP differs from the approved revised assembly')
    reader = StepInspection(step, True)
    # Exact analytic cylinder of the gripper mount, not its bounding box.
    cylinder = 193498
    frame = reader.frame(reader.graph[cylinder][0])
    axis, origin = frame[:3, 2], frame[:3, 3]
    nodes = reader.reachable(63844)
    vertices = np.array([reader.xyz(reader.graph[n][0]) for n in nodes
                         if reader.entities[n].startswith('VERTEX_POINT(')])
    if len(vertices) < 1000 or reader.entities[cylinder].split('(')[0] != 'CYLINDRICAL_SURFACE':
        raise ValueError('approved gripper feature evidence missing')
    # The TCP is a declared datum: centreline at the distal finger-tip plane.
    # The STEP has no semantic "contact centre". Do not label this measured.
    station = float(np.min((vertices-origin) @ axis))
    tcp_cad = (origin + axis*station)/1000
    model_path = root/'scripts/data/left_arm_general_dynamics.json'
    geometry_path = root/'scripts/data/chess_crown_geometry_v2_8.json'
    saved_path = root/'scripts/data/left_arm_v2_6_saved_moves.json'
    model = RigidBodyModel.load(model_path)
    wrist = model.home[-1]
    tcp_local = wrist[:3, :3].T @ (tcp_cad-wrist[:3, 3])
    geometry = json.loads(geometry_path.read_text(encoding='utf8'))
    anchors = [(key, a) for section in ('grasp_anchors', 'placement_anchors')
               for key, a in geometry[section].items() if a.get('board_tcp_mm') is not None]
    board = np.array([a['board_tcp_mm'] for _, a in anchors])/1000
    poses = [model.fk(model.encoder_to_model([a['pose_rad'][n] for n in model.names]))[-1]
             for _, a in anchors]
    world = np.array([f[:3, 3]+f[:3, :3]@tcp_local for f in poses])
    r, t = rigid_fit(board, world)
    errors = np.linalg.norm(board@r.T+t-world, axis=1)*1000
    saved = json.loads(saved_path.read_text(encoding='utf8'))['moves']
    seeds = {name: m['pose'] for name, m in saved.items()}
    all_poses = np.array([[p[n] for n in model.names] for p in seeds.values()] + [model.reference.tolist()])
    for name in ('left_arm_v2_home.json', 'left_arm_v2_table_clearance.json'):
        path = root/'scripts/data'/name
        if path.exists():
            data = json.loads(path.read_text(encoding='utf8'))
            pose = data.get('joints', data.get('pose', data))
            if all(n in pose for n in model.names):
                all_poses = np.vstack([all_poses, [pose[n] for n in model.names]])
    bounds = np.column_stack([all_poses.min(0)-np.radians(10), all_poses.max(0)+np.radians(10)])
    clearance_path = root/'scripts/data/left_arm_v2_table_clearance.json'
    clearance_pose = json.loads(clearance_path.read_text(encoding='utf8'))['joints']
    profiles = {}
    for piece, key in [('white_bishop', 'white_bishop:d4'), ('white_knight', 'white_knight:c4'),
                       ('white_rook', 'white_knight:c4')]:
        a = geometry['grasp_anchors'][key]
        f = model.fk(model.encoder_to_model([a['pose_rad'][n] for n in model.names]))[-1]
        profiles[piece] = {'grip_height_mm': a['board_tcp_mm'][2],
                           'wrist_rotation_in_board': (r.T@f[:3, :3]).tolist(),
                           'seed_encoder_rad': a['pose_rad'],
                           'source': key if piece != 'white_rook' else 'existing C4 rook workflow uses knight grasp',
                           'height_independently_measured': piece == 'white_bishop'}
    return {'schema': 1, 'build': 'seven-axis-board-grasp-v1',
            'registration_status': 'nominal-step-and-historical-teaching; needs on-robot validation',
            'independently_validated': False,
            'source_step_sha256': STEP_SHA,
            'rigid_model_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
            'tcp_datum': {'definition': 'gripper mount centreline intersecting distal finger-tip plane',
                          'semantic_contact_centre_verified': False,
                          'mount_cylinder_step_entity': cylinder, 'gripper_representation': 63844,
                          'axis_in_cad': axis.tolist(), 'axis_origin_mm': origin.tolist(),
                          'distal_axial_station_mm': station, 'cad_point_m': tcp_cad.tolist(),
                          'wrist_axis_distance_mm': float(np.linalg.norm(tcp_cad-wrist[:3, 3])*1000)},
            'tcp_in_wrist_frame_m': tcp_local.tolist(),
            'board_to_cad_rotation': r.tolist(), 'board_to_cad_translation_m': t.tolist(),
            'fit_errors_mm': {key: float(e) for (key, _), e in zip(anchors, errors)},
            'joint_order': list(model.names), 'joint_limits_encoder_rad': bounds.tolist(),
            'joint_bound_source': 'observed saved-pose/reference envelope plus 10deg; NOT mechanical limits',
            'transit_references': [{'name': 'raised-entry',
                                    'source': 'left_arm_v2_table_clearance.json',
                                    'encoder_rad': clearance_pose,
                                    'preserve_start_joints': ['shoulder_rotate']}],
            'orientation_hints': [{'board_xy_mm': a['board_tcp_mm'][:2],
                                   'wrist_rotation_in_board': (r.T@f[:3, :3]).tolist(),
                                   'seed_encoder_rad': a['pose_rad']}
                                  for (_, a), f in zip(anchors, poses)],
            'seeds': seeds, 'profiles': profiles,
            'board_size_mm': 440, 'square_size_mm': 55,
            'default_clearance_mm': 140, 'default_test_height_mm': 160,
            'scope': 'nominal geometry testing on a cleared board; no occupied-board collision planner'}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--step', required=True, type=Path)
    p.add_argument('--output', type=Path, default=ROOT/'scripts/data/left_arm_board_grasp.json')
    args = p.parse_args()
    result = build(args.step)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf8')
    print(json.dumps({'tcp': result['tcp_datum'], 'fit_errors_mm': result['fit_errors_mm']}))
