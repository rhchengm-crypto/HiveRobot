"""Offline changed-orientation retry; never imports a serial controller."""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import sys, json, time, hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0, 'scripts')
import left_arm_board_grasp as board
from left_arm_board_task_pose import endpoint_seeds
from left_arm_board_kinematics import tcp_pose, solve_ik, rotation_error
from left_arm_rigid_body import axis_rotation

out = Path('outputs/a8_test_20261108')
context = json.loads((out/'preflight.json').read_text())
for name, digest in context['source_hashes'].items():
    assert hashlib.sha256((Path('scripts')/name).read_bytes()).hexdigest() == digest, name
model = board.RigidBodyModel.load('scripts/data/left_arm_general_dynamics.json')
cfg = board.load_board_config('scripts/data/left_arm_board_workspace_candidate.json', model)
request = context['request']
point, _ = board.validate_request(request['x_mm'], request['y_mm'], request['piece'],
                                   request['stage'], request['grip_height_mm'], cfg)
br = np.array(cfg['board_to_cad_rotation'])
origin = np.array(cfg['board_to_cad_translation_m'])
target = br@(point/1000)+origin
tcp = np.array(cfg['tcp_in_wrist_frame_m'])
axis = np.array(cfg['tool_axis_in_wrist_frame']); axis /= np.linalg.norm(axis)
bounds = np.array(cfg['joint_limits_encoder_rad'])
reference = next(r for r in cfg['approved_task_pose_references']
                 if 'a7_v32_height50_x17' in r['name'])
preferred = br@np.array(reference['wrist_rotation_in_board'])
q_reference = np.array([reference['seed_encoder_rad'][n] for n in model.names])
position_diagnostic = json.loads((out/'orientation_retry_position_diagnostic.json').read_text())
q_best = np.clip([position_diagnostic['encoder_rad'][n] for n in model.names],
                 bounds[:,0], bounds[:,1])
q0 = np.array([context['forecast_clearance'][n] for n in model.names])
seeds = [q_best, q_reference, q0, bounds.mean(axis=1)]
seeds += endpoint_seeds(model, cfg, q0, 4)[-4:]
seeds = [np.clip(q, bounds[:,0], bounds[:,1]) for q in seeds]
scene = board.collision_scene(model, cfg)
candidates = [('unchanged-A7-control', preferred)]
for coordinate, name in ((0,'board-X'), (1,'board-Y'), (2,'board-Z')):
    for degrees in (-30., -15., 15., 30.):
        candidates.append((f'{name}-tilt-{degrees:+g}deg',
                           axis_rotation(br[:,coordinate], np.radians(degrees))@preferred))
for degrees in (-90., -60., -30., 30., 60., 90.):
    candidates.append((f'tool-axis-twist-{degrees:+g}deg',
                       axis_rotation(preferred@axis, np.radians(degrees))@preferred))
candidates.append(('free-position-best-FK-orientation', tcp_pose(model,q_best,tcp)[:3,:3]))
report = dict(request=request, target_board_mm=point.tolist(), target_cad_m=target.tolist(),
    baseline_reference=reference['name'], source_hashes=context['source_hashes'],
    motion_issued=False, serial_opened=False, network_accessed=False,
    runtime_modified=False, bounds_modified=False, geometry_modified=False,
    position_tolerance_mm=.5, full_rotation_tolerance_deg=float(np.degrees(.005)),
    solver='repository solve_ik; full wrist constraint, body clearance retained',
    seed_count=len(seeds), physical_unreachable_proven=False, candidates=[])
started = time.monotonic()
for name, desired in candidates:
    began = time.monotonic()
    answers = []
    for seed in seeds:
        answer = solve_ik(model, target, seed, tcp, bounds, desired,
            max_iterations=150, position_tolerance_m=.0005,
            rotation_tolerance_rad=.005, clearance_function=scene)
        answers.append(answer)
    best = min(answers, key=lambda a: (not a['success'],
        (a['position_error_mm']/1000)**2+(.15*np.radians(a['rotation_error_deg']))**2))
    accepted = [a for a in answers if a['success']]
    row = dict(name=name, requested_wrist_rotation_in_cad=desired.tolist(),
        full_rotation_change_from_A7_deg=float(np.degrees(np.linalg.norm(rotation_error(desired,preferred)))),
        axis_change_from_A7_deg=float(np.degrees(np.arccos(np.clip((desired@axis)@(preferred@axis),-1,1)))),
        endpoint_solutions=len(accepted), elapsed_s=time.monotonic()-began,
        best=best, attempts=answers)
    for answer in accepted:
        q = np.array([answer['encoder_rad'][n] for n in model.names])
        try:
            board.check_board_intersection(model,q,cfg)
            answer['full_board_intersection_passed'] = True
        except board.BoardPlanningError as exc:
            answer['full_board_intersection_passed'] = False
            answer['board_collision'] = exc.diagnostics
    report['candidates'].append(row)
    report['elapsed_s'] = time.monotonic()-started
    (out/'changed_orientation_retry.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(dict(name=name,position_error_mm=best['position_error_mm'],
        rotation_error_deg=best['rotation_error_deg'],solutions=len(accepted),
        elapsed_s=row['elapsed_s'])),flush=True)
report['successful_endpoints'] = sum(r['endpoint_solutions'] for r in report['candidates'])
report['full_path_generated'] = False
report['executable'] = False
report['source_hashes_still_match'] = all(
    hashlib.sha256((Path('scripts')/n).read_bytes()).hexdigest()==h
    for n,h in context['source_hashes'].items())
assert report['source_hashes_still_match']
(out/'changed_orientation_retry.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('DONE '+json.dumps(dict(orientations=len(candidates),successes=report['successful_endpoints'],
                             elapsed_s=report['elapsed_s'])),flush=True)
