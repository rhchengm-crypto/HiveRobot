"""Separate contact-IK failures from complete-path search failures, offline.

No operating bounds are enlarged and no coordinates are compensated. Failure
of these finite multistart solves is not a proof of physical unreachability.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from left_arm_board_collision import collision_scene
from left_arm_board_grasp import load_board_config
from left_arm_board_kinematics import board_point_to_cad, solve_ik
from left_arm_rigid_body import RigidBodyModel


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports', type=Path, nargs='+', required=True)
    parser.add_argument('--board-model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    model = RigidBodyModel.load(here/'data/left_arm_general_dynamics.json')
    cfg = load_board_config(args.board_model, model)
    bounds = np.asarray(cfg['joint_limits_encoder_rad'])
    scene = collision_scene(model, cfg)
    seeds = [model.reference, bounds.mean(1)]
    seeds += [np.array([pose[n] for n in model.names]) for pose in cfg['seeds'].values()]
    requested = {}
    for path in args.reports:
        for row in json.loads(path.read_text(encoding='utf-8'))['results']:
            if not row['success']:
                requested[row['point']] = row['target_board_mm']
    report = dict(motion_issued=False, physical_unreachable_proven=False,
        board_model_sha256=hashlib.sha256(args.board_model.read_bytes()).hexdigest(),
        source_sha256={name: hashlib.sha256((here/name).read_bytes()).hexdigest() for name in
            ['left_arm_board_kinematics.py', 'left_arm_board_collision.py']},
        interpretation='contact-pose diagnostics only; no transit, descent, finger collision or execution validation',
        results=[])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    reach = float(np.linalg.norm(np.diff(model.points, axis=0), axis=1).sum()
                  +np.linalg.norm(cfg['tcp_in_wrist_frame_m']))
    rotation = np.array(cfg['board_to_cad_rotation'])
    for label, xyz in requested.items():
        target = board_point_to_cad(xyz, rotation, cfg['board_to_cad_translation_m'])
        row = dict(point=label, target_board_mm=xyz,
            shoulder_distance_mm=float(np.linalg.norm(target-model.points[0])*1000),
            conservative_chain_length_upper_bound_mm=reach*1000, contact_solves={})
        started = time.monotonic()
        for mode in ('position_only', 'tool_axis_without_body', 'tool_axis_with_body'):
            best = None
            for hint in ([cfg['profiles']['white_knight']] if mode == 'position_only' else cfg['orientation_hints']):
                for seed in seeds:
                    if np.any(seed < bounds[:, 0]) or np.any(seed > bounds[:, 1]):
                        continue
                    answer = solve_ik(model, target, seed, cfg['tcp_in_wrist_frame_m'], bounds,
                        None if mode == 'position_only' else rotation@np.array(hint['wrist_rotation_in_board']),
                        max_iterations=80, position_tolerance_m=.0005, rotation_tolerance_rad=.005,
                        seed_projection_tolerance_rad=np.full(model.n, 1e-12),
                        tool_axis_local=None if mode == 'position_only' else cfg['tool_axis_in_wrist_frame'],
                        clearance_function=scene if mode == 'tool_axis_with_body' else None)
                    score = answer['position_error_mm']+2*answer['rotation_error_deg']
                    if best is None or (answer['success'], -score) > (best['success'], -best['score']):
                        best = dict(answer, score=score)
                    if answer['success']:
                        break
                if best['success']:
                    break
            q = np.array([best['encoder_rad'][n] for n in model.names])
            best['near_operating_bounds'] = [name for i, name in enumerate(model.names)
                if min(q[i]-bounds[i, 0], bounds[i, 1]-q[i]) < np.radians(.2)]
            row['contact_solves'][mode] = best
        row['elapsed_s'] = time.monotonic()-started
        report['results'].append(row)
        args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(dict(point=label, contact={key:value['success'] for key,value in row['contact_solves'].items()},
                              elapsed_s=row['elapsed_s'])), flush=True)


if __name__ == '__main__':
    main()
