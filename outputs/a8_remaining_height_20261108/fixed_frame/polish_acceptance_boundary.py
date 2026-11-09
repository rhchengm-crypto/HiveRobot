"""Independent local maximum within UNCHANGED fixed-R acceptance tolerances.

Uses the existing .5mm Cartesian ball and .005rad full-R error, never a new
desired frame or relaxed limit. No hardware/runtime/model/config writes.
"""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import hashlib,json,math
from pathlib import Path
import sys,time
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2];TRIAL=ROOT/'outputs/a8_lower25_execution_20261108';SRC=TRIAL/'source_snapshot/scripts'
sys.path.insert(0,str(ROOT/'tmp/gripper_geometry_deps'));sys.path.insert(0,str(SRC))
import numpy as np
from scipy.optimize import minimize
import left_arm_board_grasp as b
from left_arm_board_kinematics import tcp_pose,rotation_error


def read(p):return json.loads(Path(p).read_text(encoding='utf8'))
def write(p,x):Path(p).write_text(json.dumps(x,indent=2,ensure_ascii=False),encoding='utf8')


def main():
    began=time.monotonic();cfg=read(TRIAL/'common_trial_config.json');m=b.RigidBodyModel.load(SRC/'data/left_arm_general_dynamics.json')
    tcp=np.array(cfg['tcp_in_wrist_frame_m']);rb,tb=np.array(cfg['board_to_cad_rotation']),np.array(cfg['board_to_cad_translation_m'])
    bounds=np.array(cfg['joint_limits_encoder_rad']);feedback=read(TRIAL/'execution_audit.json')['inline_hold']['final'];q0=np.array([feedback[n]['pos'] for n in m.names])
    pose0=tcp_pose(m,q0,tcp);rref=pose0[:3,:3];xyz0=(pose0[:3,3]-tb)@rb*1000;scene=b.collision_scene(m,cfg)
    frame_limit=2*(1-math.cos(.005))
    def fk(q):
        frames=m.fk(m.encoder_to_model(q));rr=frames[-1,:3,:3];p=frames[-1,:3,3]+rr@tcp;axes=frames[:,:3,2]
        jp=rb.T@np.cross(axes,p-frames[:,:3,3]).T*m.signs[None,:]*1000
        jr=[]
        for axis,sign in zip(axes,m.signs):
            x,y,z=axis;skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            jr.append((sign*skew@rr).ravel())
        return (p-tb)@rb*1000,rr,jp,np.array(jr).T
    def pos_c(x):
        xyz,rr,jp,jr=fk(x[:7]);e=xyz-xyz0+[0,0,x[7]]
        return .25-e@e
    def pos_j(x):
        xyz,rr,jp,jr=fk(x[:7]);e=xyz-xyz0+[0,0,x[7]]
        return -2*np.r_[e@jp,e[2]]
    def rot_c(x):
        xyz,rr,jp,jr=fk(x[:7]);dr=(rr-rref).ravel()
        return 1-.5*(dr@dr)/frame_limit
    def rot_j(x):
        xyz,rr,jp,jr=fk(x[:7]);dr=(rr-rref).ravel()
        return np.r_[-jr.T@dr/frame_limit,0.]
    constraints=[dict(type='ineq',fun=pos_c,jac=pos_j),dict(type='ineq',fun=rot_c,jac=rot_j),
        dict(type='ineq',fun=lambda x:1000*scene(x[:7]),jac=lambda x:np.column_stack((1000*scene.jacobian(x[:7]),np.zeros(len(scene(x[:7]))))))]
    previous=read(OUT/'report.json');last=read(OUT/'short_vertical_DIAGNOSTIC_NOT_EXECUTABLE.json')['waypoints'][-1]
    seeds=[np.r_[[last['encoder_rad'][n] for n in m.names],1.5],np.r_[[previous['independent_exact_XY_R_minimum_height_local_results'][0]['encoder_rad'][n] for n in m.names],.4828593247]]
    results=[]
    for seed in seeds:
        sol=minimize(lambda x:-x[7],seed,jac=lambda x:np.r_[np.zeros(7),-1.],method='SLSQP',
            bounds=list(map(tuple,bounds))+[(0.,10.)],constraints=constraints,options=dict(maxiter=350,ftol=1e-11))
        q,d=sol.x[:7],float(sol.x[7]);xyz,rr,jp,jr=fk(q);e=xyz-xyz0+[0,0,d]
        re=float(np.linalg.norm(rotation_error(rref,rr)));clear=float(np.min(scene(q)))
        results.append(dict(optimizer_success=bool(sol.success),message=str(sol.message),nfev=sol.nfev,
            maximum_demonstrated_requested_descent_mm=d,achieved_model_descent_mm=float(xyz0[2]-xyz[2]),
            encoder_rad=dict(zip(m.names,q.tolist())),position_error_mm=float(np.linalg.norm(e)),
            full_frame_error_deg=math.degrees(re),lateral_error_mm=float(np.linalg.norm(e[:2])),
            minimum_scene_margin_mm=clear*1000,endpoint_passed=bool(np.linalg.norm(e)<=.500001 and re<=.00500001 and clear>=-1e-9)))
    valid=[r for r in results if r['endpoint_passed']];maximum=max(valid,key=lambda r:r['maximum_demonstrated_requested_descent_mm']) if valid else None
    # Sequential local constrained correction from ACTUAL q; never assume the
    # extremal endpoint has a connected admissible descent.
    grid=math.floor((maximum['maximum_demonstrated_requested_descent_mm']+1e-8)*2)/2 if maximum else 0.
    branch=None;failures=[];dense=None
    def objective(x,d):
        xyz,rr,jp,jr=fk(x);dr=(rr-rref).ravel();e=xyz-xyz0+[0,0,d]
        # Position closeness first; small rotation regularization chooses a
        # near-fixed frame while hard .005rad constraint remains unchanged.
        return .5*(e@e)+.02*(dr@dr)/frame_limit,jp.T@e+.04*jr.T@dr/frame_limit
    while grid>=.5:
        q=q0.copy();rows=[];failed=None
        for d in np.r_[np.arange(.25,grid,.25),grid]:
            fixed_constraints=[dict(type='ineq',fun=lambda qi:rot_c(np.r_[qi,d]),jac=lambda qi:rot_j(np.r_[qi,d])[:7]),
                dict(type='ineq',fun=lambda qi:1000*scene(qi),jac=lambda qi:1000*scene.jacobian(qi))]
            sol=minimize(lambda qi:objective(qi,d)[0],q,jac=lambda qi:objective(qi,d)[1],method='SLSQP',
                bounds=list(map(tuple,bounds)),constraints=fixed_constraints,options=dict(maxiter=180,ftol=1e-12))
            nq=sol.x;xyz,rr,jp,jr=fk(nq);e=xyz-xyz0+[0,0,d];re=float(np.linalg.norm(rotation_error(rref,rr)))
            step=math.degrees(float(np.max(np.abs(nq-q))))
            if np.linalg.norm(e)>.5 or re>.005+1e-9 or step>15:
                failed=dict(requested_descent_mm=float(d),position_error_mm=float(np.linalg.norm(e)),frame_error_deg=math.degrees(re),joint_step_deg=step);break
            try:b.check_transit_edge(m,q,nq,cfg)
            except b.BoardPlanningError as exc:failed=dict(requested_descent_mm=float(d),edge_failure=exc.diagnostics);break
            rows.append((float(d),nq));q=nq
        if failed:failures.append(dict(grid_mm=grid,failure=failed));grid-=.5;continue
        path=[(0.,q0)]+rows;xy=[];angles=[];height=[];margins=[];steps=[]
        for (da,a),(db,bb) in zip(path,path[1:]):
            steps.append(math.degrees(float(np.max(np.abs(bb-a)))))
            count=max(2,int(np.ceil(np.max(np.abs(bb-a))/math.radians(.05))))
            for alpha in np.linspace(0,1,count+1):
                qi=a+alpha*(bb-a);xyz,rr,jp,jr=fk(qi)
                xy.append(float(np.linalg.norm(xyz[:2]-xyz0[:2])));angles.append(math.degrees(float(np.linalg.norm(rotation_error(rref,rr)))))
                height.append(float(xyz[2]));margins.append(float(np.min(scene(qi))*1000))
        dense=dict(samples=len(xy),maximum_lateral_error_mm=max(xy),maximum_full_frame_error_deg=max(angles),
            achieved_model_descent_mm=float(xyz0[2]-height[-1]),minimum_scene_margin_mm=min(margins),maximum_joint_edge_step_deg=max(steps),
            descent_monotonic=bool(all(y<=x+1e-7 for x,y in zip(height,height[1:]))))
        if max(xy)>.5 or max(angles)>math.degrees(.005)+1e-7 or min(margins)<0 or not dense['descent_monotonic']:
            failures.append(dict(grid_mm=grid,dense_failure=dense));grid-=.5;continue
        trial=dict(start_rad=dict(zip(m.names,q0.tolist())),selected_wrist_rotation_in_board=(rb.T@rref).tolist(),
            waypoints=[dict(stage='contact',target_board_mm=(xyz0-[0,0,d]).tolist(),encoder_rad=dict(zip(m.names,qi.tolist()))) for d,qi in rows])
        orientation=b.check_planned_tool_orientation(m,cfg,trial);gripper=b.check_planned_gripper_clearance(m,cfg,trial)
        branch=dict(executable=False,requested_descent_mm=grid,trial_geometry=trial,orientation=orientation,gripper=gripper,dense=dense)
        write(OUT/'polished_short_vertical_DIAGNOSTIC_NOT_EXECUTABLE.json',branch);break
    output=dict(motion_issued=False,network_accessed=False,hardware_accessed=False,executable=False,
        desired_frame_or_bounds_changed=False,position_tolerance_mm=.5,strict_full_frame_tolerance_rad=.005,
        independent_SLSQP_endpoint_results=results,conservative0p5mm_grid_continuous_descent_mm=grid if branch else 0.,
        continuous_branch_passed=branch is not None,dense_audit=dense,branch_failures=failures,
        desired_R_fixed_from_actual_held_feedback=True,physical_displacement_or_contact_verified=False,
        scope='Finite independent constrained optimization of the original acceptance tolerances; not a globally proven or physical maximum.',
        source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),elapsed_s=time.monotonic()-began)
    write(OUT/'independent_acceptance_boundary_polish.json',output)
    print(json.dumps(output),flush=True)


if __name__=='__main__':main()
