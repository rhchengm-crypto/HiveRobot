"""Offline maximum demonstrated fixed-frame vertical continuation.

Actual held encoders/FK define XY and full R. No runtime/source/model changes,
motor/network APIs, stamp, or physical-height claims. Finite local evidence.
"""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[k]='1'
import hashlib,json,math
from pathlib import Path
import sys,time
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
TRIAL=ROOT/'outputs/a8_lower25_execution_20261108'
SCRIPTS=TRIAL/'source_snapshot/scripts'
sys.path.insert(0,str(ROOT/'tmp/gripper_geometry_deps'))
sys.path.insert(0,str(SCRIPTS))
import numpy as np
from scipy.optimize import minimize
import left_arm_board_grasp as b
from left_arm_board_kinematics import tcp_pose,solve_ik,rotation_error


def read(p): return json.loads(Path(p).read_text(encoding='utf8'))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x): Path(p).write_text(json.dumps(x,indent=2,ensure_ascii=False),encoding='utf8')


def main():
    began=time.monotonic()
    cp,mp,ap=TRIAL/'common_trial_config.json',SCRIPTS/'data/left_arm_general_dynamics.json',TRIAL/'execution_audit.json'
    m=b.RigidBodyModel.load(mp);cfg=b.load_board_config(cp,m)
    bounds=np.array(cfg['joint_limits_encoder_rad']);tcp=np.array(cfg['tcp_in_wrist_frame_m'])
    rb,tb=np.array(cfg['board_to_cad_rotation']),np.array(cfg['board_to_cad_translation_m'])
    feedback=read(ap)['inline_hold']['final'];q0=np.array([feedback[n]['pos'] for n in m.names])
    p0=tcp_pose(m,q0,tcp);R=p0[:3,:3];xyz0=(p0[:3,3]-tb)@rb*1000
    np.testing.assert_allclose(xyz0,[9.639697000830376,406.65055148807653,62.17518852976162],atol=1e-8,rtol=0)
    scene=b.collision_scene(m,cfg);b.check_board_intersection(m,q0,cfg)
    def board(q): return (tcp_pose(m,q,tcp)[:3,3]-tb)@rb*1000
    def assess(q,down):
        pose=tcp_pose(m,q,tcp);xyz=(pose[:3,3]-tb)@rb*1000;target=xyz0-[0,0,down]
        re=float(np.linalg.norm(rotation_error(R,pose[:3,:3])));pe=float(np.linalg.norm(xyz-target))
        try:
            b.check_board_intersection(m,q,cfg);clear=True;failure=None
        except b.BoardPlanningError as exc:
            clear=False;failure=exc.diagnostics
        return dict(requested_descent_mm=float(down),achieved_model_descent_mm=float(xyz0[2]-xyz[2]),
            predicted_board_mm=xyz.tolist(),residual_board_mm=(xyz-target).tolist(),position_error_mm=pe,
            lateral_error_mm=float(np.linalg.norm(xyz[:2]-xyz0[:2])),full_frame_error_deg=math.degrees(re),
            passed=bool(pe<=.5 and re<=.005 and clear and np.all(q>=bounds[:,0]-1e-9) and np.all(q<=bounds[:,1]+1e-9)),
            encoder_rad=dict(zip(m.names,q.tolist())),scene_passed=clear,scene_failure=failure,
            minimum_scene_margin_mm=float(np.min(scene(q))*1000),
            limit_margins_deg={n:dict(lower=math.degrees(float(q[i]-bounds[i,0])),upper=math.degrees(float(bounds[i,1]-q[i]))) for i,n in enumerate(m.names)})
    calls=[]
    def solve(down,seed,phase):
        point=xyz0-[0,0,down]
        answer=solve_ik(m,rb@(point/1000)+tb,seed,tcp,bounds,R,max_iterations=180,
            position_tolerance_m=.00002,rotation_tolerance_rad=.00005,clearance_function=scene)
        q=np.array([answer['encoder_rad'][n] for n in m.names]);row=assess(q,down)
        row.update(phase=phase,iterations=answer['iterations'],internal_tight_solver_success=answer['success'])
        calls.append(row)
        return q,row
    # Coarse endpoint sweep doubles as an initial branch continuation. Failure
    # is bracketed only; never converted into a global reachability assertion.
    q=q0.copy();sweep=[(0.,q0.copy())];failed=None
    for down in np.arange(.5,10.0001,.5):
        nq,row=solve(float(down),q,'coarse0.5mm')
        if not row['passed']:
            failed=(float(down),nq,row);break
        try: b.check_transit_edge(m,q,nq,cfg)
        except b.BoardPlanningError as exc:
            failed=(float(down),nq,dict(row,passed=False,swept_edge_failure=exc.diagnostics));break
        sweep.append((float(down),nq));q=nq
    low=sweep[-1][0];high=failed[0] if failed else 10.;lowq=sweep[-1][1].copy()
    first_fail=failed[2] if failed else None
    if failed:
        for _ in range(6):
            mid=(low+high)/2;nq,row=solve(mid,lowq,'endpoint-bisection')
            if row['passed']:
                low,lowq=mid,nq
            else: high=mid
    # Independent constrained extremum: min height with EXACT original XY/R,
    # so acceptance tolerance cannot masquerade as a perfect vertical point.
    def jpos(q):
        jv,_=m.point_jacobian(m.encoder_to_model(q),m.n-1,tcp)
        return rb.T@(jv*m.signs)*1000
    def exact_eq(q): return np.r_[(board(q)-xyz0)[:2]/1000,rotation_error(R,tcp_pose(m,q,tcp)[:3,:3])]
    exact=[]
    for seed in (q0,lowq):
        sol=minimize(lambda q:board(q)[2]/1000,seed,jac=lambda q:jpos(q)[2]/1000,
            method='SLSQP',bounds=list(map(tuple,bounds)),
            constraints=[dict(type='eq',fun=exact_eq),dict(type='ineq',fun=scene,jac=scene.jacobian)],
            options=dict(maxiter=250,ftol=1e-13))
        row=assess(sol.x,float(xyz0[2]-board(sol.x)[2]))
        row.update(optimizer_success=bool(sol.success),optimizer_message=str(sol.message),nfev=sol.nfev,
            exact_constraint_max_residual=float(np.max(np.abs(exact_eq(sol.x)))))
        exact.append(row)
    # Conservative0.5mm-grid proposal, then recompute its full short vertical
    # path from ACTUAL q using0.25mm requested spacing; dense whole-edge audit.
    grid=max(0.,math.floor((low+1e-9)*2)/2)
    branch=None;branch_failure=None;dense=None
    while grid>=.5:
        q=q0.copy();path=[(0.,q.copy())];branch_failure=None
        for down in np.r_[np.arange(.25,grid,.25),grid]:
            nq,row=solve(float(down),q,'conservative-continuation')
            if not row['passed'] or np.max(np.abs(nq-q))>math.radians(15):
                branch_failure=dict(code='IK-or-joint-continuity',answer=row);break
            try: b.check_transit_edge(m,q,nq,cfg)
            except b.BoardPlanningError as exc:
                branch_failure=dict(code='swept-edge',diagnostics=exc.diagnostics);break
            path.append((float(down),nq));q=nq
        if branch_failure is not None:
            grid-=.5;continue
        xy=[];angles=[];height=[];gaps=[];steps=[]
        for (da,a),(db,bb) in zip(path,path[1:]):
            steps.append(math.degrees(float(np.max(np.abs(bb-a)))))
            count=max(2,int(np.ceil(np.max(np.abs(bb-a))/math.radians(.05))))
            for alpha in np.linspace(0,1,count+1):
                qi=a+alpha*(bb-a);pose=tcp_pose(m,qi,tcp);xyz=(pose[:3,3]-tb)@rb*1000
                xy.append(float(np.linalg.norm(xyz[:2]-xyz0[:2])));angles.append(math.degrees(float(np.linalg.norm(rotation_error(R,pose[:3,:3])))))
                height.append(float(xyz[2]));gaps.append(float(np.min(scene(qi))*1000))
        dense=dict(samples=len(xy),max_lateral_error_mm=max(xy),max_full_frame_error_deg=max(angles),
            minimum_scene_margin_mm=min(gaps),maximum_joint_edge_step_deg=max(steps),
            initial_model_height_mm=float(xyz0[2]),minimum_model_height_mm=min(height),
            achieved_model_descent_mm=float(xyz0[2]-height[-1]),
            descent_monotonic=bool(all(y<=x+1e-7 for x,y in zip(height,height[1:]))),
            complete_swept_edges_checked=True)
        if max(xy)>.5 or max(angles)>math.degrees(.005) or min(gaps)<0 or not dense['descent_monotonic']:
            branch_failure=dict(code='dense-task-audit',audit=dense);grid-=.5;continue
        branch=path;break
    if branch:
        # Run the same full gripper/frame audits as the common task planner.
        trial=dict(start_rad=dict(zip(m.names,q0.tolist())),selected_wrist_rotation_in_board=(rb.T@R).tolist(),
            waypoints=[dict(stage='contact',target_board_mm=(xyz0-[0,0,d]).tolist(),encoder_rad=dict(zip(m.names,q.tolist()))) for d,q in branch[1:]])
        orientation=b.check_planned_tool_orientation(m,cfg,trial)
        gripper=b.check_planned_gripper_clearance(m,cfg,trial)
        write(OUT/'short_vertical_DIAGNOSTIC_NOT_EXECUTABLE.json',dict(executable=False,motion_issued=False,
            actual_feedback_start_encoder_rad=dict(zip(m.names,q0.tolist())),requested_descent_mm=grid,
            waypoints=trial['waypoints'],dense_audit=dense,orientation=orientation,gripper_clearance=gripper))
    else: orientation=gripper=None
    report=dict(schema=1,motion_issued=False,hardware_accessed=False,network_accessed=False,
        runtime_sources_models_or_active_config_modified=False,executable=False,physical_acceptance=None,
        actual_held_encoder_rad=dict(zip(m.names,q0.tolist())),actual_held_model_board_mm=xyz0.tolist(),
        actual_complete_R_frozen=R.tolist(),XY_preserved_from_actual_model_FK=True,
        same_scoped_trial_bounds=True,position_tolerance_mm=.5,strict_frame_tolerance_rad=.005,
        target_is_model_prediction_not_physical_board_or_contact_datum=True,
        endpoint_acceptance_bracket_requested_descent_mm=[low,high],
        bracket_scope='Finite local endpoint sweep/bisection under strict acceptance, not an exact reach/global bound.',
        first_failed0p5mm_grid_endpoint=first_fail,endpoint_calls=calls,
        independent_exact_XY_R_minimum_height_local_results=exact,
        conservative_continuous0p5mm_grid_descent_mm=grid if branch else 0.,
        conservative_branch_passed=branch is not None,dense_branch_audit=dense,
        common_orientation_audit=orientation,common_gripper_audit=gripper,last_branch_failure=branch_failure,
        no_direct_motion_or_execution_plan_prepared=True,
        source_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in (cp,mp,ap,SCRIPTS/'left_arm_board_grasp.py',SCRIPTS/'left_arm_board_kinematics.py',SCRIPTS/'left_arm_board_collision.py',Path(__file__))},
        limitations=['Finite local continuation, bisection and constrained optimization do not prove a global maximum or physical reach.',
            'Approximate operator remaining10mm is not an instrumented board/contact datum.',
            'Any reported descent is model-space feasibility under the trial software envelope, not execution authorization or guaranteed physical displacement.',
            'No orientation, bounds, geometry, XY offset or gain changes; current body/gripper models remain physically unvalidated.'],
        elapsed_s=time.monotonic()-began)
    write(OUT/'report.json',report)
    print(json.dumps(dict(endpoint_bracket_mm=[low,high],conservative_grid_mm=grid if branch else 0.,
        dense=dense,exact_minima=exact,elapsed_s=report['elapsed_s'])),flush=True)


if __name__=='__main__': main()
