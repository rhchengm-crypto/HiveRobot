"""Finite offline minimum-Z search with unchanged trial bounds and <=5deg R.

Endpoint feasibility alone is not a usable path. Any reported connection is
checked as an in-place new-frame transition followed by a vertical branch.
"""
import os
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[name]='1'
import hashlib,json,math
from pathlib import Path
import sys,time
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
EVIDENCE=ROOT/'outputs/a8_lower25_execution_20261108/control_evidence'
sys.path.insert(0,str(ROOT/'tmp/gripper_geometry_deps'))
sys.path.insert(0,str(EVIDENCE/'source'))
import numpy as np
from scipy.optimize import minimize
import left_arm_board_grasp as board
from left_arm_board_kinematics import tcp_pose,rotation_error,solve_ik
from left_arm_rigid_body import axis_rotation


def read(p):return json.loads(Path(p).read_text(encoding='utf8'))


def main():
    began=time.monotonic()
    audit=read(ROOT/'outputs/a8_lower25_execution_20261108/execution_audit.json')
    m=board.RigidBodyModel.load(EVIDENCE/'models/left_arm_general_dynamics.json')
    cfg=read(EVIDENCE/'models/left_arm_board_workspace_candidate.json')
    tcp=np.array(cfg['tcp_in_wrist_frame_m']);bounds=np.array(cfg['joint_limits_encoder_rad'])
    rb,tb=np.array(cfg['board_to_cad_rotation']),np.array(cfg['board_to_cad_translation_m'])
    q0=np.array([audit['inline_hold']['final'][n]['pos'] for n in m.names])
    initial=tcp_pose(m,q0,tcp);reference=initial[:3,:3]
    start=(initial[:3,3]-tb)@rb*1000
    scene=board.collision_scene(m,cfg)
    def fk(q):
        frames=m.fk(m.encoder_to_model(q));r=frames[-1,:3,:3]
        p=frames[-1,:3,3]+r@tcp;axes=frames[:,:3,2]
        jp=np.cross(axes,p-frames[:,:3,3]).T*m.signs[None,:]
        derivative=[]
        for axis,sign in zip(axes,m.signs):
            x,y,z=axis;skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
            derivative.append((sign*skew@r).ravel())
        return p,r,jp,np.array(derivative).T
    def full(q):
        _,r,_,jr=fk(q);delta=(r-reference).ravel()
        return .5*float(delta@delta),jr.T@delta
    def height(q):return float(((fk(q)[0]-tb)@rb)[2]*1000)
    def height_jac(q):return rb[:,2]@fk(q)[2]*1000
    def xy(q):return ((fk(q)[0]-tb)@rb*1000)[:2]-start[:2]
    def xy_jac(q):return rb[:,:2].T@fk(q)[2]*1000
    constraints=[{'type':'eq','fun':xy,'jac':xy_jac},
        {'type':'ineq','fun':lambda q:scene(q)*1000,'jac':lambda q:scene.jacobian(q)*1000},
        {'type':'ineq','fun':lambda q:2*(1-math.cos(math.radians(5.)))-full(q)[0],
         'jac':lambda q:-full(q)[1]}]
    seeds=[q0,np.array([audit['endpoint']['nominal_target_rad'][n] for n in m.names]),
           np.array([read(OUT/'free_orientation_position_crosscheck.json')['best']['encoder_rad'][n]
                     for n in m.names])]
    trials=[]
    for index,seed in enumerate(seeds):
        result=minimize(height,seed,jac=height_jac,method='SLSQP',bounds=list(map(tuple,bounds)),
            constraints=constraints,options={'maxiter':300,'ftol':1e-11})
        p,r,_,_=fk(result.x);xyz=(p-tb)@rb*1000
        angle=float(np.degrees(np.linalg.norm(rotation_error(reference,r))))
        try:board.check_board_intersection(m,result.x,cfg);scene_passed=True;failure=None
        except board.BoardPlanningError as exc:scene_passed=False;failure=exc.diagnostics
        passed=bool(np.linalg.norm(xyz[:2]-start[:2])<=.5 and angle<=5.+1e-6
                    and np.min(scene(result.x))>=-1e-9 and scene_passed
                    and np.all(result.x>=bounds[:,0]-1e-9) and np.all(result.x<=bounds[:,1]+1e-9))
        trials.append({'seed_index':index,'optimizer_success':bool(result.success),
            'message':str(result.message),'iterations':result.nit,
            'encoder_rad':dict(zip(m.names,result.x.tolist())),'model_board_mm':xyz.tolist(),
            'endpoint_descent_mm':float(start[2]-xyz[2]),'xy_error_mm':float(np.linalg.norm(xyz[:2]-start[:2])),
            'full_frame_change_from_actual_deg':angle,'endpoint_passed':passed,
            'new_requested_wrist_rotation_in_cad':r.tolist(),
            'scene_minimum_clearance_mm':float(np.min(scene(result.x))*1000),
            'scene_failure':failure,'active_bounds':[n for k,n in enumerate(m.names)
                 if min(result.x[k]-bounds[k,0],bounds[k,1]-result.x[k])<1e-5]})
    best=min((t for t in trials if t['endpoint_passed']),key=lambda t:t['model_board_mm'][2],default=None)
    rows=[];failure=None;previous=q0.copy();rotation_rows=0
    if best is not None:
        desired=np.array(best['new_requested_wrist_rotation_in_cad'])
        rotvec=rotation_error(desired,reference);angle=np.linalg.norm(rotvec)
        stages=[]
        for i in range(1,max(1,math.ceil(math.degrees(angle)/.5))+1):
            count=max(1,math.ceil(math.degrees(angle)/.5));fraction=i/count
            r=axis_rotation(rotvec/angle,angle*fraction)@reference if angle>1e-12 else reference
            stages.append(('orientation_at_held_XYZ',start.copy(),r))
        rotation_rows=len(stages)
        total=best['endpoint_descent_mm']
        for i in range(1,max(1,math.ceil(total/.25))+1):
            count=max(1,math.ceil(total/.25));xyz=start.copy();xyz[2]-=total*i/count
            stages.append(('strict_new_frame_vertical_descent',xyz,desired))
        for stage,xyz,r in stages:
            answer=solve_ik(m,rb@(xyz/1000)+tb,previous,tcp,bounds,r,
                max_iterations=200,position_tolerance_m=.0005,rotation_tolerance_rad=.005,
                clearance_function=scene)
            q=np.array([answer['encoder_rad'][n] for n in m.names])
            step=float(np.degrees(np.max(np.abs(q-previous))))
            row={'stage':stage,'target_board_mm':xyz.tolist(),'encoder_rad':answer['encoder_rad'],
                 'position_error_mm':answer['position_error_mm'],'rotation_error_deg':answer['rotation_error_deg'],
                 'maximum_joint_step_deg':step,'IK_success':answer['success']}
            try:
                if not answer['success']:raise ValueError('strict requested pose IK failed')
                if step>12:raise ValueError('joint continuity >12deg')
                board.check_transit_edge(m,previous,q,cfg);row['complete_edge_passed']=True
            except (board.BoardPlanningError,ValueError) as exc:
                failure={'stage':stage,'error':str(exc),'details':getattr(exc,'diagnostics',None)}
                row['complete_edge_passed']=False
            rows.append(row)
            if failure:break
            previous=q
    actual_end=(tcp_pose(m,previous,tcp)[:3,3]-tb)@rb*1000
    complete=bool(best and not failure and rows and rows[-1]['stage']=='strict_new_frame_vertical_descent')
    report={'motion_issued':False,'network_accessed':False,'runtime_configuration_modified':False,
        'front_domain':'Same completed trial12deg lower extension; no new domain expansion.',
        'held_model_board_mm':start.tolist(),'maximum_requested_frame_change_deg':5.,
        'strict_position_tolerance_mm':.5,'strict_rotation_tolerance_rad':.005,
        'endpoint_trials':trials,'best_endpoint_only':best,'connection_rows':rows,
        'orientation_transition_rows':rotation_rows,'connection_failure':failure,
        'complete_connection_plus_descent':complete,
        'demonstrated_requested_descent_mm':best['endpoint_descent_mm'] if complete else None,
        'model_predicted_end_board_mm':actual_end.tolist() if complete else None,
        'model_predicted_achieved_descent_mm':float(start[2]-actual_end[2]) if complete else None,
        'executable':False,'physical_qualification':False,'global_maximum_proven':False,
        'limitations':['All positions are encoder/model FK predictions, not physical measurements.',
            'Failed full10 searches do not prove physical unreachability.',
            'A locally optimized endpoint is not usable unless the complete new-frame transition and vertical path pass.'],
        'elapsed_s':time.monotonic()-began,
        'helper_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (OUT/'max_descent_new_frame_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'output':str(OUT/'max_descent_new_frame_report.json'),
        'best_endpoint_only':best,'complete_connection':complete,'failure':failure,
        'requested_descent_mm':report['demonstrated_requested_descent_mm'],
        'model_achieved_descent_mm':report['model_predicted_achieved_descent_mm'],
        'elapsed_s':report['elapsed_s']}))


if __name__=='__main__':main()
