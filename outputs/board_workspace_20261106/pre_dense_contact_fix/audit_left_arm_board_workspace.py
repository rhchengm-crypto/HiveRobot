"""Offline continuous-coordinate path coverage; imports no motor drivers.

Search failures are not proofs of physical unreachability. Every coordinate
uses the same model, constraints, seed bank and planner.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from left_arm_board_grasp import (load_board_config, plan, check_transit_edge,
                                  feedback_contact_path, PlanningBudgetExceeded)
from left_arm_board_collision import collision_scene
from left_arm_rigid_body import RigidBodyModel


def targets(files, ranks, include_between):
    points = [(file+str(rank), (ord(file)-97+.5)*55, (rank-.5)*55)
              for rank in ranks for file in files]
    if include_between:
        points += [('between-'+str(i), x, y) for i, (x, y) in enumerate(
            [(55., 55.), (110., 110.), (165., 165.), (55., 330.),
             (110., 385.), (165., 275.), (5., 5.), (215., 435.)])]
    return points


def main():
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', type=Path, default=here/'data/left_arm_general_dynamics.json')
    p.add_argument('--board-model', type=Path, default=here/'data/left_arm_board_grasp.json')
    p.add_argument('--start-file', type=Path, default=here/'data/left_arm_v2_home.json')
    p.add_argument('--piece', default='white_knight')
    p.add_argument('--files', default='abcd')
    p.add_argument('--ranks', default='1,2,3,4,5,6,7,8')
    p.add_argument('--height-mm', type=float, default=50.2)
    p.add_argument('--stage', choices=['above', 'approach', 'grasp'], default='grasp')
    p.add_argument('--include-between', action='store_true')
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    if not args.files or set(args.files)-set('abcdefgh'):
        p.error('--files must contain a..h')
    ranks = [int(v) for v in args.ranks.split(',')]
    if not ranks or any(not 1 <= v <= 8 for v in ranks):
        p.error('--ranks must contain 1..8')
    model = RigidBodyModel.load(args.model)
    cfg = load_board_config(args.board_model, model)
    start = json.loads(args.start_file.read_text(encoding='utf-8-sig'))
    start = start.get('joints', start.get('pose', start))
    if set(start) != set(model.names):
        raise ValueError('start file requires all seven encoder coordinates')
    scene = collision_scene(model, cfg)
    report = dict(schema=2, motion_issued=False, physical_validation_complete=False,
        model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
        board_model_sha256=hashlib.sha256(args.board_model.read_bytes()).hexdigest(),
        source_sha256={name: hashlib.sha256((here/name).read_bytes()).hexdigest() for name in
            ['left_arm_board_grasp.py', 'left_arm_board_kinematics.py',
             'left_arm_board_collision.py', 'left_arm_board_transit.py']},
        start_source=str(args.start_file), start_rad=start, piece=args.piece,
        requested_stage=args.stage, height_mm=args.height_mm,
        collision_scope='finite board plus configured body scene; not complete self/occupied-board collision',
        body_registration_status=cfg.get('body_collision', {}).get('registration_status'),
        search_failure_is_physical_unreachable_proof=False, results=[])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    def save():
        report['tested'] = len(report['results'])
        report['successes'] = sum(row['success'] for row in report['results'])
        report['failure_codes'] = dict(Counter(row.get('failure_code') for row in report['results'] if not row['success']))
        args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    save()
    for label, x, y in targets(args.files, ranks, args.include_between):
        began = time.monotonic()
        row = dict(point=label, target_board_mm=[x, y, args.height_mm], success=False)
        try:
            trajectory = plan(model, cfg, start, x, y, args.piece, args.stage, args.height_mm)
            previous = np.array([start[n] for n in model.names])
            gap, bound_margin = float('inf'), float('inf')
            bounds = np.asarray(cfg['joint_limits_encoder_rad'])
            for waypoint in trajectory['waypoints']:
                q = np.array([waypoint['encoder_rad'][n] for n in model.names])
                check_transit_edge(model, previous, q, cfg)
                bound_margin = min(bound_margin, float(np.min(q-bounds[:, 0])), float(np.min(bounds[:, 1]-q)))
                if scene is not None:
                    gap = min(gap, float(scene(q).min()))
                previous = q
            if args.stage == 'approach':
                above = [w for w in trajectory['waypoints'] if w['stage'] == 'source_above'][-1]
                rebuilt, _ = feedback_contact_path(model, cfg, above['encoder_rad'], row['target_board_mm'])
                row['nominal_feedback_descent_samples'] = len(rebuilt)
            row.update(success=True, samples=len(trajectory['waypoints']),
                       entry_route=trajectory['entry_route'], posture_selection=trajectory.get('posture_selection'),
                       waypoint_min_body_clearance_lower_bound_mm=gap*1000 if scene is not None else None,
                       joint_bound_margin_deg=float(np.degrees(bound_margin)))
        except (ValueError, PlanningBudgetExceeded) as exc:
            details = getattr(exc, 'diagnostics', {})
            row.update(message=str(exc), failure_code=details.get('code', 'numeric_path_failure'), diagnostics=details)
        row['elapsed_s'] = time.monotonic()-began
        report['results'].append(row)
        save()
        print(json.dumps({key: row.get(key) for key in ('point', 'success', 'failure_code', 'elapsed_s')}), flush=True)


if __name__ == '__main__':
    main()
