from pathlib import Path
import copy,datetime,hashlib,json,shutil,zipfile

root=Path.cwd();out=root/'outputs/a8_saved_reference_20261108'
load=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
audit=load(out/'saved_pose_audit.json');inverse=load(out/'reference_inverse_path_audit.json')
fit=load(out/'common_geometry_fit_audit.json')
assert inverse['geometric_inverse_path_found'] and not inverse['nominal_request_solved']
assert inverse['source_hashes_still_match'] and fit['source_hashes_unchanged_at_finish']
active_path=root/'scripts/data/left_arm_board_workspace_candidate.json'
cfg=load(active_path);source=load(out/'saved_moves_remote.json')
record=dict(schema=1,experiment_id='20261108_a8_take_approved_endpoint',
    recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    reference_kind='operator_approved_saved_endpoint',
    physical_board_square='A8',nominal_physical_board_xy_mm=[27.5,412.5],piece='white_knight',
    physical_tcp_height_mm=None,external_TCP_measurement=False,
    operator_validation=dict(grasp_endpoint_accepted=True,
        original_request='参考saved moves里A8_take, 实测可以抓取，以此为答案反算路径',
        confirmation='白马，基本对准 A8 中心',qualitative_only=True),
    saved_move=audit['saved_move'],saved_moves_source_sha256=audit['source_sha256'],
    capture_log=source['web']['full_log_path'],
    encoder_endpoint_rad=audit['saved_move']['pose'],
    current_model_fk_board_mm=audit['saved_pose_tcp_board_mm'],
    current_model_wrist_rotation_in_board=audit['saved_pose_wrist_rotation_in_board'],
    requested_height_50_2_was_not_measured=True,
    complete_path_physically_validated=False,model_geometry_ground_truth=False,
    motion_issued=False,execution_eligible=False,runtime_model_modified=False,
    inverse_geometry_path=dict(found=True,waypoints=inverse['branches'][0]['waypoints'],
        file='outputs/a8_saved_reference_20261108/reference_inverse_plan_hover140.0.json',
        nominal_request_solved=False,endpoint_boundary='operator-approved saved configuration',
        intermediate_angles='new Cartesian IK and geometric transit',
        saved_angles_replayed_as_trajectory=False),
    source_models=audit['source_hashes'],
    common_fit_deployed=False,
    evidence_directory='outputs/a8_saved_reference_20261108')
registry=root/'scripts/data/board_workspace_trials/reference_20261108_a8_take.json'
registry.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(out/'reference_record.json').write_bytes(registry.read_bytes())
# A shared reference candidate, available to every XY. It is not active,
# and it does not replace target XYZ with FK or add a square-specific offset.
candidate=copy.deepcopy(cfg)
candidate['joint_limits_encoder_rad']=load(out/'reference_envelope_candidate.json')['joint_limits_encoder_rad']
candidate['joint_bound_source']='existing shared operating envelope union operator-demonstrated A8_take endpoint; physical travel not certified'
reference=dict(name='saved:A8_take',reference_kind='operator_approved_saved_endpoint',
    reference_board_xy_mm=[27.5,412.5],seed_encoder_rad=audit['saved_move']['pose'],
    wrist_rotation_in_board=audit['saved_pose_wrist_rotation_in_board'],
    path_seed_encoder_rad=[],source_saved_moves_sha256=audit['source_sha256'],
    qualification='User-approved grasp endpoint; numerical seed/full frame for all XY; newly solve and check every path; no complete trajectory acceptance claim')
candidate['approved_task_pose_references'].append(reference)
candidate.setdefault('seeds',{})['A8_take']=audit['saved_move']['pose']
candidate['pending_reference_provenance']=dict(record=registry.relative_to(root).as_posix(),
    record_sha256=hashlib.sha256(registry.read_bytes()).hexdigest(),
    runtime_enabled=False,nominal_A8_XYZ_consistency_resolved=False,
    per_square_offset_added=False,angle_replay_added=False)
(out/'shared_reference_candidate.json').write_text(json.dumps(candidate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
note='''# A8_take endpoint inverse-path evidence — 2026-11-08

The user confirms that `A8_take` can grasp a white knight near the A8 centre.
This is an accepted physical endpoint, not a previously accepted full motion.

- Exact Jetson Saved Moves source: `source_snapshot/left_arm_v2_6_saved_moves.json`.
- Nominal physical square centre: X27.5, Y412.5 mm; physical TCP Z was not measured.
- Current model FK at that endpoint: X11.798, Y405.601, Z84.623 mm.
- Shoulder-front is 2.052° below the previous software operating envelope.
- The earlier Z50.2 request was an assumption, not a measurement of this grasp.

`reference_inverse_plan_hover140.0.json` reconstructs 33 new geometric waypoints
from fixed ordinary Clearance to a 140 mm model hover and vertically to the
approved saved endpoint. Its intermediate poses are IK solutions and geometric
transit points, with full-frame descent and swept tool/body checks. The endpoint
is deliberately an explicit boundary condition. This is not a passing solution
for the old nominal `[27.5,412.5,50.2]` request, and is not an executable release.
The diagnostic envelope admits exactly the demonstrated shoulder-front angle;
it does not invent additional mechanical travel.

`shared_reference_candidate.json` adds the endpoint as a general numerical
reference for every board XY, with no per-square compensation or replay.
It is not installed as the runtime board model. Existing model hold, learning,
gain policy and ordinary Clearance shoulder-rotate skip are unchanged.

`common_geometry_fit_audit.json` checks approximate known square centres without
fitting their old requested Z values. The data do not yet support a robust new
shared TCP/board fit. In particular, shifting the common plane to force this
endpoint to Z50.2 contradicts existing non-contact trials. These approximate
observations must not be promoted to exact TCP metrology.

No A8 motion, motor enable, claw closure or runtime deployment occurred in this
inverse-path diagnostic. Read fresh actual state before any later execution;
the latest Web action when data were collected was `capture-move:A8_take`.
'''
(out/'REFERENCE_PATH_NOTES.md').write_text(note,encoding='utf-8')
repro=out/'reproducibility';repro.mkdir(exist_ok=True)
for name in ('audit_a8_saved_pose_20261108.py','reconstruct_a8_reference_path_20261108.py',
             'record_a8_saved_reference_20261108.py'):
    shutil.copyfile(root/'tmp'/name,repro/name)
files=[p for p in out.rglob('*') if p.is_file() and p.suffix!='.zip'
       and p.name not in ('evidence_manifest.json','upload_manifest.json','sync_receipt.json')]
manifest={p.relative_to(out).as_posix():dict(bytes=p.stat().st_size,
    sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files}
(out/'evidence_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
archive=out/'saved_reference_evidence.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=3) as z:
    for p in files:z.write(p,p.relative_to(out).as_posix())
    z.write(out/'evidence_manifest.json','evidence_manifest.json')
upload=dict(bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
    archive=str(archive),registry=str(registry),remote_directory='/home/nvidia/hive_robot/DM_Control_Python/data/board_workspace_trials/a8_saved_reference_20261108')
(out/'upload_manifest.json').write_text(json.dumps(upload,indent=2),encoding='utf-8')
print(json.dumps(dict(reference_saved=True,shared_candidate_active=False,
    geometric_inverse_waypoints=record['inverse_geometry_path']['waypoints'],archive_bytes=upload['bytes'])))
