"""Offline actual path planning at 64 square centres, no motor imports."""
import argparse
import json
from pathlib import Path

from left_arm_board_grasp import load_board_config, plan
from left_arm_rigid_body import RigidBodyModel


def main():
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--piece', default='white_knight')
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    model = RigidBodyModel.load(here/'data/left_arm_general_dynamics.json')
    cfg = load_board_config(here/'data/left_arm_board_grasp.json', model)
    start = json.loads((here/'data/left_arm_v2_table_clearance.json').read_text())['joints']
    # Ordinary Clearance holds rotate at the Home branch rather than moving
    # to the recorded clearance rotate target. Use that branch for this audit.
    start['shoulder_rotate'] = model.reference[2]
    rows = []
    for rank in range(1, 9):
        for file in range(8):
            square = chr(97+file)+str(rank)
            try:
                result = plan(model, cfg, start, (file+.5)*55, (rank-.5)*55, args.piece, 'grasp')
                rows.append({'square': square, 'success': True, 'samples': len(result['waypoints'])})
            except ValueError as exc:
                rows.append({'square': square, 'success': False, 'reason': str(exc)})
        report = {'motion_issued': False, 'piece': args.piece,
                  'registration_status': cfg['registration_status'], 'results': rows,
                  'successes': sum(r['success'] for r in rows), 'tested': len(rows),
                  'note': 'Nominal model and observed operating bounds, not physical workspace certification.'}
        args.output.write_text(json.dumps(report, indent=2), encoding='utf8')
        print(json.dumps({'rank': rank, 'successes': report['successes'], 'tested': len(rows)}), flush=True)


if __name__ == '__main__':
    main()
