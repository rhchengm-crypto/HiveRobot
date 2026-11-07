"""Offline provenance, common-frame audit and grouped hold-torque fit study.

No motor driver or robot access. Output models are research candidates only.
Requested XYZ and qualitative acceptance are never position ground truth.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from fit_left_arm_general_dynamics import fit
from left_arm_board_kinematics import tcp_pose
from left_arm_rigid_body import RigidBodyModel


SPECS = [
    ('C4', 'outputs/board_drift_20261102/motion_control_logs/data/left_arm_v2_8_full_runs/arm_run_1793681494917_16715.txt',
     'outputs/board_drift_20261102/jetson_snapshot_v24_accepted/left_arm_board_grasp.json', [113.8,190.3,50.2]),
    ('C5', 'outputs/c5_test_20261106/accepted_jetson_raw/data/left_arm_v2_8_full_runs/arm_run_1794009150847_13024.txt',
     'outputs/c5_test_20261106/accepted_jetson_raw/data/left_arm_board_grasp.json', [137.5,247.5,30.2]),
    ('B3', 'outputs/board_workspace_20261106/height20_trial_evidence/arm_run_1794030889770_57622.txt',
     'scripts/data/left_arm_board_workspace_candidate.json', [82.5,137.5,20.2]),
]


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tagged(lines, tag):
    return [json.loads(line.split('=',1)[1]) for line in lines if line.startswith(tag+'=')][-1]


def nearby(a,b,threshold_deg=5.):
    # Entire seven-axis pose neighborhood, not an individual joint/frame split.
    return max(abs(np.degrees(float(a[n])-float(b[n]))) for n in b) <= threshold_deg


def extract(root):
    active=read(root/'scripts/data/left_arm_board_workspace_candidate.json')
    model=RigidBodyModel.load(root/'scripts/data/left_arm_general_dynamics.json')
    rows=[]
    for name,log_path,config_path,requested in SPECS:
        cfg=read(root/config_path)
        log=root/log_path;lines=log.read_text(encoding='utf-8').splitlines()
        if name=='C4':
            assert sha(log)==read(root/'outputs/board_drift_20261102/accepted_v24_sync_manifest.json')['source_log_sha256']
            acceptance='operator accepted ~2mm B drift and visible board gap; exact grasp height was not measured'
        elif name=='C5':
            accepted=read(root/'outputs/c5_test_20261106/accepted_summary.json')
            assert sha(log)==accepted['log_sha256']
            acceptance=accepted['operator_observation']['verbatim']
        else:
            accepted=read(root/'scripts/data/board_workspace_trials/validation_20261106_b3_v25_height20.json')
            assert sha(log)==accepted['evidence_manifest'][log.name]['sha256']
            acceptance=accepted['operator_validation']['original_response']
        hold=tagged(lines,'board same-session hold complete')
        endpoint=tagged(lines,'board endpoint report')
        assert hold['ok'] and hold['reason']=='duration_complete'
        assert hold['duration_s']>=4.5 and max(hold['max_velocity_rad_s'].values())<.05
        final=hold['final'];source='hold-result-final-feedback';count=1;mad=None;drift=None
        if name=='B3':
            trace=root/'outputs/board_workspace_20261106/height20_trial_evidence/trace_1794030891337280893.jsonl'
            records=[json.loads(s) for s in trace.read_text(encoding='utf-8').splitlines()]
            assert records[-1]['event']=='trace_closed' and records[-1]['queue_dropped_frames']==0
            frames=[r for r in records if r.get('event')=='motor_frame' and r['context']['stage']=='model_hold']
            cutoff=frames[0]['monotonic_s']+.75
            frames=[r for r in frames if r['monotonic_s']>=cutoff]
            assert frames[-1]['monotonic_s']-frames[0]['monotonic_s']>=3.
            count=len(frames);source='median of final stable same-serial hold; first .75s excluded'
            final={n:{k:float(np.median([r['feedback'][n][k] for r in frames])) for k in ('pos','vel','tau')} for n in model.names}
            drift={n:float(np.degrees(np.ptp([r['feedback'][n]['pos'] for r in frames]))) for n in model.names}
            assert max(drift.values())<.25
            assert max(abs(r['feedback'][n]['vel']) for r in frames for n in model.names)<.05
            mad={n:float(np.median(abs(np.array([r['feedback'][n]['tau'] for r in frames])-final[n]['tau']))) for n in model.names}
        pose={n:final[n]['pos'] for n in model.names}
        point=tcp_pose(model,[pose[n] for n in model.names],cfg['tcp_in_wrist_frame_m'])[:3,3]
        own=(point-cfg['board_to_cad_translation_m'])@np.array(cfg['board_to_cad_rotation'])*1000
        common=(point-active['board_to_cad_translation_m'])@np.array(active['board_to_cad_rotation'])*1000
        trace_sources=[json.loads(s.split('=',1)[1]) for s in lines if s.startswith('board planning input=')]
        actual_board_sha=trace_sources[0]['board_model_sha256'] if trace_sources else None
        if actual_board_sha is not None:assert actual_board_sha==sha(root/config_path)
        assert cfg['source_step_sha256']==active['source_step_sha256']
        assert cfg['rigid_model_sha256']==sha(root/'scripts/data/left_arm_general_dynamics.json')
        rows.append(dict(id=name,log=log_path,log_sha256=sha(log),board_model=config_path,
            board_model_sha256=sha(root/config_path),step_sha256=cfg['source_step_sha256'],
            rigid_model_sha256=cfg['rigid_model_sha256'],tcp_in_wrist_frame_m=cfg['tcp_in_wrist_frame_m'],
            board_to_cad_rotation=cfg['board_to_cad_rotation'],board_to_cad_translation_m=cfg['board_to_cad_translation_m'],
            tcp_equal=np.array_equal(cfg['tcp_in_wrist_frame_m'],active['tcp_in_wrist_frame_m']),
            board_transform_equal=(np.array_equal(cfg['board_to_cad_rotation'],active['board_to_cad_rotation']) and
                                   np.array_equal(cfg['board_to_cad_translation_m'],active['board_to_cad_translation_m'])),
            requested_board_mm=requested,encoder_pose_rad=pose,measured_torque_Nm={n:final[n]['tau'] for n in model.names},
            operator_acceptance=acceptance,independent_fitting_pose_count=1,
            own_frame_encoder_FK_mm=own.tolist(),common_frame_encoder_FK_mm=common.tolist(),
            model_FK_minus_request_mm=(common-requested).tolist(),external_TCP_measurement=False,
            nominal_joint_error_deg=endpoint['max_nominal_error_deg'],
            stable_hold=dict(duration_s=hold['duration_s'],max_velocity_rad_s=max(hold['max_velocity_rad_s'].values()),
                source=source,complete_frames=count,position_span_deg=drift,torque_mad_Nm=mad),
            fitting_qualification=dict(geometry_target_labels=False,quasi_static_torque_proxy=True,
                calibrated_gravity_measurement=False,new_inertia_measurement=False,
                limitations=['motor feedback torque is not external force measurement','stiction/history not independently removed',
                    'C4/C5 have summary endpoint only; no dense final-window variability',
                    'matching stored board transforms does not establish absence of physical board movement']),
            body_policy=cfg.get('body_collision'),joint_limits=cfg['joint_limits_encoder_rad']))
    return model,rows


def add_proxies(history, rows):
    result=copy.deepcopy(history)
    for row in rows:
        for n in row['encoder_pose_rad']:
            result['joints'][n]['gravity_samples'].append(dict(pose_rad=row['encoder_pose_rad'],
                tau=row['measured_torque_Nm'][n],direction='stationary',source='quasi-static-hold-proxy',
                experiment_id=row['id'],uncalibrated_torque_proxy=True))
    return result


def without_pose_neighbors(history, pose):
    result=copy.deepcopy(history);removed={}
    for n,body in result['joints'].items():
        removed[n]={}
        for field in ('gravity_samples','inertia_samples'):
            old=body[field];body[field]=[s for s in old if not nearby(s['pose_rad'],pose)]
            removed[n][field]=len(old)-len(body[field])
    return result,removed


def evaluate(model_data, train, heldout):
    m=RigidBodyModel(model_data);bias={}
    # Nuisance offsets are learned only from training observations, never
    # from the held-out torque. They are NOT deployed feedforward offsets.
    for j,n in enumerate(m.names):
        a=[];b=[]
        for s in train['joints'][n]['gravity_samples']:
            q=m.encoder_to_model([s['pose_rad'][k] for k in m.names])
            predicted=m.signs[j]*m.gravity_torques(q)[j]
            a.append([1.,{'positive':1.,'negative':-1.}.get(s.get('direction'),0.)])
            b.append(s['tau']-predicted)
        bias[n]=float(np.linalg.lstsq(a,b,rcond=1e-9)[0][0])
    q=m.encoder_to_model([heldout['encoder_pose_rad'][n] for n in m.names])
    g=m.signs*m.gravity_torques(q)
    residual={n:float(g[j]+bias[n]-heldout['measured_torque_Nm'][n]) for j,n in enumerate(m.names)}
    return dict(residual_Nm=residual,rmse_Nm=float(np.sqrt(np.mean(np.square(list(residual.values()))))),
                train_only_nuisance_offset_Nm=bias)


def run(root,out):
    model,rows=extract(root)
    geometry_path=root/'outputs/left_arm_general_model/left_arm_cad_exact.json'
    history_path=root/'outputs/board_workspace_20261106/baseline/left_arm_v3_1_dynamics.json'
    inspection_path=root/'outputs/left_arm_step_analysis/revised_2/step_inspection.json'
    geometry,history,inspection=map(read,(geometry_path,history_path,inspection_path))
    source_shas=set(model.data['identification_sources'].values())
    assert all(sha(p) in source_shas for p in (geometry_path,history_path,inspection_path))
    protected={p:sha(root/p) for p in ['scripts/data/left_arm_general_dynamics.json',
        'scripts/data/left_arm_v3_1_dynamics.json','scripts/data/left_arm_board_workspace_candidate.json']}
    baseline,base_report=fit(geometry,history,inspection)
    reproduction_error=max(float(np.max(abs(np.array(baseline['links'][n]['inertial']['inertia_com_kg_m2'])-
                        np.array(model.data['links'][n]['inertial']['inertia_com_kg_m2'])))) for n in baseline['links'])
    assert reproduction_error<1e-10,'baseline fit not reproducible; do not compare candidates'
    augmented=add_proxies(history,rows)
    candidate,candidate_report=fit(geometry,augmented,inspection)
    cv=[]
    for test in rows:
        cleaned,removed=without_pose_neighbors(history,test['encoder_pose_rad'])
        train_rows=[r for r in rows if r['id']!=test['id'] and not nearby(r['encoder_pose_rad'],test['encoder_pose_rad'])]
        assert all(not nearby(s['pose_rad'],test['encoder_pose_rad']) for body in cleaned['joints'].values()
                   for field in ('gravity_samples','inertia_samples') for s in body[field])
        train=add_proxies(cleaned,train_rows)
        old,_=fit(geometry,cleaned,inspection);new,_=fit(geometry,train,inspection)
        cv.append(dict(heldout_experiment=test['id'],included_experiments=[r['id'] for r in train_rows],
            exclusion_rule='exclude full experiment plus all historical seven-axis poses within 5deg max-axis distance',
            removed_history=removed,baseline=evaluate(old,cleaned,test),augmented=evaluate(new,train,test)))
    candidate['identification_sources']={str(p.relative_to(root)).replace('\\','/'):sha(p) for p in (geometry_path,history_path,inspection_path)}
    candidate['identification']['added_hold_proxy_records']=len(rows)*model.n
    candidate['identification']['added_independent_pose_groups']=len(rows)
    candidate['runtime_apply_requires_explicit_opt_in']=True
    candidate['offline_candidate_only']=True
    candidate['identification']['hold_proxy_experiments']=[r['id'] for r in rows]
    candidate['identification']['do_not_treat_as_measured_mass_properties']=True
    summary=dict(schema=1,motion_issued=False,runtime_files_replaced=False,shared_model_only=True,
        per_square_fitting=False,trials=rows,baseline_fit_reproduction_tensor_max_error=reproduction_error,
        baseline_training=base_report,augmented_training=candidate_report,grouped_cross_validation=cv,
        baseline_heldout_rmse_Nm=float(np.sqrt(np.mean([c['baseline']['rmse_Nm']**2 for c in cv]))),
        augmented_heldout_rmse_Nm=float(np.sqrt(np.mean([c['augmented']['rmse_Nm']**2 for c in cv]))),
        decision=dict(runtime_model_promoted=False,
            reason='Augmented fit worsens every held-out experiment. Only 3 grouped pose summaries; uncalibrated torque proxies and no independent XYZ labels.',
            geometry_refitted=False,geometry_reason='No new externally measured TCP correspondences',
            next_motion_requires_operator_notification=True),protected_runtime_sha256=protected)
    summary['heldout_rmse_change_percent']=100*(summary['augmented_heldout_rmse_Nm']/summary['baseline_heldout_rmse_Nm']-1)
    summary['every_heldout_fold_worse']=all(c['augmented']['rmse_Nm']>c['baseline']['rmse_Nm'] for c in cv)
    out.mkdir(parents=True,exist_ok=True)
    (out/'common_calibration_audit.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (out/'hold_proxy_augmented_history.json').write_text(json.dumps(augmented,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (out/'left_arm_general_dynamics_hold_proxy_candidate.json').write_text(json.dumps(candidate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    assert all(sha(root/p)==v for p,v in protected.items())
    print(json.dumps({k:summary[k] for k in ['baseline_fit_reproduction_tensor_max_error','baseline_heldout_rmse_Nm',
          'augmented_heldout_rmse_Nm','decision']}))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.root.resolve(),args.output.resolve())
