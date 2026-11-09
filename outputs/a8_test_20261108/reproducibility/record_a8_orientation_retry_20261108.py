from pathlib import Path
import datetime, json, hashlib, zipfile, shutil

root = Path.cwd()
out = root/'outputs/a8_test_20261108'
read = lambda n: json.loads((out/n).read_text(encoding='utf-8'))
scan = read('changed_orientation_retry.json')
free = read('orientation_retry_position_diagnostic.json')
assert scan['successful_endpoints'] == 0 and not scan['motion_issued']
assert len(scan['candidates']) == 20 and scan['source_hashes_still_match']
summary = dict(schema=1, experiment_id='20261108_a8_changed_orientation_retry',
    recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    operator_request='改变朝向再试试', request=scan['request'],
    status='offline-endpoint-unsolved-after-changed-orientation-search',
    motion_issued=False, motor_enable_issued=False, a8_physically_validated=False,
    fitting_eligible=False, runtime_code_or_models_modified=False,
    position_tolerance_mm=.5, full_rotation_tolerance_deg=scan['full_rotation_tolerance_deg'],
    baseline_reference=scan['baseline_reference'], tested_orientations=20,
    changed_orientations=19, numerical_seeds_per_orientation=scan['seed_count'],
    total_ik_attempts=len(scan['candidates'])*scan['seed_count'],
    orientation_choices='A7 control; board X/Y/Z +/-15,+/-30deg; tool-axis +/-30,+/-60,+/-90deg; orientation at best position-only FK',
    search_elapsed_s=scan['elapsed_s'], successful_endpoints=0,
    best_position_error_mm=min(r['best']['position_error_mm'] for r in scan['candidates']),
    free_orientation_current_bounds_best_position_error_mm=free['best_residual_mm'],
    free_orientation_DE_evaluations=sum(r['population_size']*(r['generations']+1) for r in free['de_runs']),
    current_bound_active_axes={'lower':free['active_lower'],'upper':free['active_upper']},
    full_path_generated=False, executable=False, physical_unreachable_proven=False,
    target_height_audit={
        'validated_request_mm':[27.5,412.5,50.2],
        'runtime_height_offset_mm':0.,
        'existing_plane_datum_already_in_board_transform':True,
        'height_interpretation':'Model TCP Z in current calibrated board frame, not necessarily minimum fingertip physical clearance.'},
    conclusion='Changed orientation did not produce a qualifying endpoint under the unchanged common model and stored operating bounds. The numerical results do not prove physical unreachability.',
    source_hashes=scan['source_hashes'],
    next_required_work='Audit common geometric reach and operating-bound consistency. Do not execute approximate position-only or angle-replay candidates.')
(out/'orientation_retry_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
registry = root/'scripts/data/board_workspace_trials/planning_20261108_a8_v32_height50.json'
record = json.loads(registry.read_text(encoding='utf-8'))
record['orientation_retry'] = summary
record['recorded_at_utc'] = summary['recorded_at_utc']
for path in (registry,out/'trial_summary.json'):
    path.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
repro = out/'reproducibility'
repro.mkdir(exist_ok=True)
for name in ('retry_a8_orientations_20261108.py','record_a8_orientation_retry_20261108.py'):
    shutil.copyfile(root/'tmp'/name,repro/name)
excluded = {'offline_manifest.json','upload_manifest.json','orientation_retry_manifest.json',
            'orientation_retry_upload.json','sync_receipt.json'}
files = [p for p in out.rglob('*') if p.is_file() and p.suffix!='.zip' and p.name not in excluded]
manifest = {p.relative_to(out).as_posix():dict(bytes=p.stat().st_size,
    sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files}
manifest_path=out/'orientation_retry_manifest.json'
manifest_path.write_text(json.dumps(manifest,indent=2),encoding='utf-8')
archive=out/'orientation_retry_evidence.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=3) as z:
    for p in files: z.write(p,p.relative_to(out).as_posix())
    z.write(manifest_path,manifest_path.name)
upload=dict(bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
    archive=str(archive), remote_directory='/home/nvidia/hive_robot/DM_Control_Python/data/board_workspace_trials/a8_test_20261108')
(out/'orientation_retry_upload.json').write_text(json.dumps(upload,indent=2),encoding='utf-8')
print(json.dumps(dict(orientations=20,attempts=summary['total_ik_attempts'],
    min_position_error_mm=summary['best_position_error_mm'],archive_bytes=upload['bytes'])))
