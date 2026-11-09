"""Offline pure-position cross-check; free orientation is diagnostic only."""
import os
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[name]='1'
import hashlib
import json
from pathlib import Path
import sys
import time

sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
EVIDENCE=ROOT/'outputs/a8_lower25_execution_20261108/control_evidence'
sys.path.insert(0,str(ROOT/'tmp/gripper_geometry_deps'))
sys.path.insert(0,str(EVIDENCE/'source'))
import numpy as np
from scipy.optimize import least_squares
import left_arm_board_grasp as board
from left_arm_board_task_pose import batch_tool_frames
from left_arm_board_kinematics import tcp_pose,rotation_error


def main():
    began=time.monotonic()
    inputs=json.loads((OUT/'inputs.json').read_text())
    model=board.RigidBodyModel.load(EVIDENCE/'models/left_arm_general_dynamics.json')
    cfg=json.loads((EVIDENCE/'models/left_arm_board_workspace_candidate.json').read_text())
    bounds=np.array(cfg['joint_limits_encoder_rad'])
    tcp=np.array(cfg['tcp_in_wrist_frame_m'])
    rb,tb=np.array(cfg['board_to_cad_rotation']),np.array(cfg['board_to_cad_translation_m'])
    target=rb@(np.array(inputs['requested_board_mm'])/1000)+tb
    q0=np.array([inputs['held_encoder_rad'][n] for n in model.names])
    original=np.array(inputs['held_wrist_rotation_in_cad'])
    scene=board.collision_scene(model,cfg)
    def objective(qs):
        return np.sum((batch_tool_frames(model,qs,tcp)[:,:3,3]-target)**2,axis=1)
    results=[]
    seeds=[]
    for seed in (20261108,20261109):
        rng=np.random.default_rng(seed);size=98
        population=rng.uniform(bounds[:,0],bounds[:,1],(size,model.n));population[0]=q0
        values=objective(population);own=np.arange(size)
        for generation in range(700):
            idx=rng.integers(0,size,(size,3))
            while True:
                invalid=np.any(idx==own[:,None],axis=1)|(idx[:,0]==idx[:,1])|(idx[:,0]==idx[:,2])|(idx[:,1]==idx[:,2])
                if not np.any(invalid):break
                idx[invalid]=rng.integers(0,size,(int(invalid.sum()),3))
            mutant=population[idx[:,0]]+rng.uniform(.5,.95,(size,1))*(population[idx[:,1]]-population[idx[:,2]])
            invalid=(mutant<bounds[:,0])|(mutant>bounds[:,1])
            mutant=np.where(invalid,rng.uniform(bounds[:,0],bounds[:,1],mutant.shape),mutant)
            mask=rng.random((size,model.n))<.85;mask[own,rng.integers(model.n,size=size)]=True
            trial=np.where(mask,mutant,population);new=objective(trial);accept=new<values
            population[accept],values[accept]=trial[accept],new[accept]
        seeds.extend(population[np.argsort(values)[:2]])
        results.append({'seed':seed,'generations':700,'population':size,
            'best_position_error_mm':float(np.sqrt(values.min())*1000)})
    fits=[]
    for seed in [q0]+seeds:
        result=least_squares(lambda q:(tcp_pose(model,q,tcp)[:3,3]-target)*1000.,seed,
            jac=lambda q:model.point_jacobian(model.encoder_to_model(q),model.n-1,tcp)[0]*model.signs[None,:]*1000.,
            bounds=(bounds[:,0],bounds[:,1]),max_nfev=1500,ftol=1e-13,xtol=1e-13,gtol=1e-13)
        frame=tcp_pose(model,result.x,tcp)
        j=model.point_jacobian(model.encoder_to_model(result.x),model.n-1,tcp)[0]*model.signs[None,:]
        fits.append({'encoder_rad':dict(zip(model.names,result.x.tolist())),
            'predicted_board_mm':((frame[:3,3]-tb)@rb*1000).tolist(),
            'position_error_mm':float(np.linalg.norm(result.fun)),
            'full_frame_change_deg':float(np.degrees(np.linalg.norm(rotation_error(original,frame[:3,:3])))),
            'scene_minimum_clearance_mm':float(scene(result.x).min()*1000),
            'smallest_position_jacobian_singular_value_m_per_rad':float(np.linalg.svd(j,compute_uv=False)[-1]),
            'active_bounds':[n for k,n in enumerate(model.names)
                if min(result.x[k]-bounds[k,0],bounds[k,1]-result.x[k])<1e-5],
            'nfev':result.nfev,'optimality':float(result.optimality),'message':result.message})
    best=min(fits,key=lambda row:row['position_error_mm'])
    report={'motion_issued':False,'network_accessed':False,'configuration_modified':False,
        'orientation_is_unconstrained_diagnostic_not_an_accepted_task':True,
        'target_board_mm':inputs['requested_board_mm'],'differential_evolution_runs':results,
        'least_squares_trials':fits,'best':best,
        'position_within_original_point5mm_tolerance':best['position_error_mm']<=.5,
        'global_unreachable_proven':False,'elapsed_s':time.monotonic()-began,
        'helper_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (OUT/'free_orientation_position_crosscheck.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'output':str(OUT/'free_orientation_position_crosscheck.json'),'best':best,
        'DE':results,'elapsed_s':report['elapsed_s']}))


if __name__=='__main__':main()
