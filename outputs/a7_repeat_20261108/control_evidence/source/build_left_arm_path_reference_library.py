"""Build a provenance-bound shared path library without opening robot hardware.

Accepted requests locate historical references; they are not measured TCP
labels and never replace the new target or become replay commands.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from left_arm_board_kinematics import tcp_pose
from left_arm_rigid_body import RigidBodyModel


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def frame_compatible(a, b):
    return (a.get('source_step_sha256') == b.get('source_step_sha256')
        and a.get('rigid_model_sha256') == b.get('rigid_model_sha256')
        and all(np.array_equal(a[k], b[k]) for k in
            ('tcp_in_wrist_frame_m', 'board_to_cad_rotation', 'board_to_cad_translation_m')))


def tagged(text, tag):
    return [json.loads(s.split('=', 1)[1]) for s in text.splitlines()
            if s.startswith(tag+'=')]


def build(root, active_path):
    root=Path(root).resolve();active=read(active_path)
    model=RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json')
    assert sha(root/'scripts/data/left_arm_general_dynamics.json') == active['rigid_model_sha256']
    outputs=root/'outputs'
    # Match frozen bytes, not a same-named latest model from a different trial.
    configs={}
    for pattern in ('left_arm_board_workspace_candidate.json','left_arm_board_grasp.json'):
        for p in outputs.rglob(pattern):
            configs.setdefault(sha(p),p)
    records=[]
    for p in sorted((root/'scripts/data/board_workspace_trials').glob('validation_*.json')):
        r=read(p);v=r.get('operator_validation',{})
        if not (v.get('accepted') or v.get('motion_accepted')):continue
        directory=root/r.get('evidence',{}).get('directory',r.get('local_evidence_directory',''))
        candidates=list(directory.rglob('arm_run_*.txt'))
        matches=[]
        expected=r.get('evidence',{}).get('manifest',r.get('evidence_manifest',{}))
        for log in candidates:
            known=next((x for n,x in expected.items() if Path(n).name==log.name),None)
            if known and sha(log)!=known['sha256']:continue
            if r.get('evidence',{}).get('control_log') and log.name!=r['evidence']['control_log']:continue
            text=log.read_text(encoding='utf-8')
            if tagged(text,'board same-session hold complete') and tagged(text,'board endpoint report'):
                matches.append((log,text))
        # B3 older registry has multiple logs but only its successful full
        # descent has a same-session hold and matching target request.
        req=r['request'];target=[req['x_mm'],req['y_mm'],req['grip_height_mm']]
        matches=[(l,s) for l,s in matches if any(
            np.allclose(x.get('target_board_mm',[float('inf')]*3),target)
            for x in tagged(s,'board plan'))]
        if len(matches)!=1:raise ValueError(f'{p.name}: expected one verified full-run log, found {len(matches)}')
        log,text=matches[0]
        model_sha=r.get('model_snapshots',{}).get('left_arm_board_workspace_candidate.json',
            r.get('sources',{}).get('data/left_arm_board_workspace_candidate.json'))
        frozen=configs.get(model_sha)
        records.append(dict(name='validated:'+r['experiment_id'],reference_kind='validated_path',
            reference_board_xy_mm=target[:2],requested_height_mm=target[2],
            seed_encoder_rad=r['encoder_endpoint_rad'],operator_validation=v,
            registry=p.relative_to(root).as_posix(),registry_sha256=sha(p),
            source_code_sha256=r.get('sources',{}),source_models=r.get('model_snapshots',r.get('protected_models',{})),
            source_log=log.relative_to(root).as_posix(),source_log_sha256=sha(log),
            frozen_board_model=frozen.relative_to(root).as_posix() if frozen else None,
            frozen_board_model_sha256=model_sha,_text=text,_frozen=read(frozen) if frozen else None,
            evidence_directory=directory.relative_to(root).as_posix()))
    # Earlier C4/C5 evidence predates the validation registry schema.
    historical=read(outputs/'common_calibration_20261106/common_calibration_audit.json')['trials']
    for r in historical:
        if r['id'] not in ('C4','C5'):continue
        log=root/r['log'];assert sha(log)==r['log_sha256']
        frozen=root/r['board_model'];assert sha(frozen)==r['board_model_sha256']
        records.append(dict(name='validated:historical-'+r['id'],reference_kind='validated_path',
            reference_board_xy_mm=r['requested_board_mm'][:2],requested_height_mm=r['requested_board_mm'][2],
            seed_encoder_rad=r['encoder_pose_rad'],operator_validation=dict(accepted=True,
                original_response=r['operator_acceptance'],qualitative_only=True),
            source_log=r['log'],source_log_sha256=sha(log),
            frozen_board_model=r['board_model'],frozen_board_model_sha256=sha(frozen),
            _text=log.read_text(encoding='utf-8'),_frozen=read(frozen),
            evidence_directory=log.parent.as_posix(),source_code_sha256={},source_models={}))
    references=[]
    for r in records:
        text=r.pop('_text');frozen=r.pop('_frozen')
        r['frame_compatible']=bool(frozen and frame_compatible(frozen,active))
        r['physical_registration_unchanged_independently_verified']=False
        samples=[]
        for row in tagged(text,'board waypoint'):
            feedback=row.get('entry_feedback',{})
            if all(n in feedback for n in model.names):
                samples.append(dict(stage=row.get('stage'),waypoint_index=row.get('index'),
                    encoder_rad={n:feedback[n]['pos'] for n in model.names}))
        r['historical_path_samples']=samples
        r['path_sample_kind']='logged measured waypoint-entry poses; full raw log retained'
        r['geometry_ground_truth']=False
        v=r['operator_validation'];eligible=(r['frame_compatible'] and v.get('accepted',False)
            and v.get('gripping_height_acceptable',True) and v.get('jitter') is not True and v.get('contact') is not True)
        r['reference_eligible']=bool(eligible)
        r['qualification']=('approved pose/branch seed; re-solve and re-check every new path'
            if eligible else 'archived motion-only or incompatible-frame record; not a grasp reference')
        if not eligible:continue
        q=np.array([r['seed_encoder_rad'][n] for n in model.names],dtype=float)
        if not np.isfinite(q).all():raise ValueError('non-finite accepted endpoint')
        frame=tcp_pose(model,q,active['tcp_in_wrist_frame_m'])
        r['wrist_rotation_in_board']=(np.asarray(active['board_to_cad_rotation']).T@frame[:3,:3]).tolist()
        indices=np.unique(np.linspace(0,len(samples)-1,min(6,len(samples)),dtype=int)) if samples else []
        r['path_seed_encoder_rad']=[samples[i]['encoder_rad'] for i in indices]
        # Runtime bank contains only initial guesses, frame and provenance.
        references.append({k:copy.deepcopy(r[k]) for k in ('name','reference_kind',
            'reference_board_xy_mm','seed_encoder_rad','wrist_rotation_in_board','path_seed_encoder_rad',
            'source_log_sha256','qualification')})
    library=dict(schema=1,motion_issued=False,selection_policy='primary-then-nearest-validated-path',
        distance='Euclidean board XY in mm; same rule for all targets',
        records=records,eligible_reference_count=len(references),
        limitations=['Accepted request XYZ is not external TCP ground truth',
            'Historical physical board placement not independently certified',
            'Empty-load acceptance does not certify claw closure or loaded lifting'],
        common_model_sha256=active['rigid_model_sha256'],active_board_model_sha256=sha(active_path))
    candidate=copy.deepcopy(active)
    candidate['approved_task_pose_references']=[r for r in active.get('approved_task_pose_references',[])
        if r.get('reference_kind')!='validated_path']+references
    candidate['validated_path_reference_policy']='primary-then-nearest-validated-path'
    candidate['reference_attempt_budget_s']=45.
    candidate['validated_path_library_provenance']=dict(records=len(records),eligible=len(references),
        primary_model_first=True,path_replay=False,per_square_parameters=False)
    return library,candidate


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root',type=Path,default=Path(__file__).resolve().parent.parent)
    ap.add_argument('--model',type=Path)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();active=args.model or args.root/'scripts/data/left_arm_board_workspace_candidate.json'
    library,candidate=build(args.root,active);args.output.mkdir(parents=True,exist_ok=True)
    for name,value in [('validated_path_library.json',library),('workspace_reference_candidate.json',candidate)]:
        (args.output/name).write_bytes((json.dumps(value,indent=2,ensure_ascii=False)+'\n').encode())
    print(json.dumps(dict(records=len(library['records']),eligible=library['eligible_reference_count'],
        motion_issued=False,runtime_model_replaced=False)))


if __name__=='__main__':main()
