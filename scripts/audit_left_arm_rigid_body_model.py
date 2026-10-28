#!/usr/bin/env python3
"""Offline CAD/FK consistency and historical-data identifiability audit.

No fitted gravity coefficients are exported for robot use. Gravity directions
are explicitly hypothetical sensitivity cases; static holding torque includes
friction, contact, controller and sensor effects. Synthetic inertias only
validate algorithms and never populate the physical model JSON files.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from left_arm_rigid_body import RigidBodyModel


ROOT=Path(__file__).resolve().parents[1]


def read_urdf_fk(path,q):
    result=[];current=np.eye(4)
    for joint,angle in zip(ET.parse(path).findall('joint'),q):
        origin=joint.find('origin');xyz=np.fromstring(origin.get('xyz'),sep=' ')
        r,p,y=np.fromstring(origin.get('rpy'),sep=' ')
        cr,sr=np.cos(r),np.sin(r);cp,sp=np.cos(p),np.sin(p);cy,sy=np.cos(y),np.sin(y)
        rotation=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                           [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
        t=np.eye(4);t[:3,:3]=rotation;t[:3,3]=xyz
        c,s=np.cos(angle),np.sin(angle);motion=np.eye(4);motion[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
        current=current@t@motion;result.append(current.copy())
    return np.array(result)


def pose_groups(poses,radius_deg=2.):
    parents=list(range(len(poses)))
    def find(i):
        while parents[i]!=i:parents[i]=parents[parents[i]];i=parents[i]
        return i
    for i in range(len(poses)):
        delta=np.arctan2(np.sin(poses[i+1:]-poses[i]),np.cos(poses[i+1:]-poses[i]))
        for k in np.flatnonzero(np.degrees(np.sqrt(np.mean(delta**2,axis=1)))<=radius_deg):
            a,b=find(i),find(i+1+int(k));parents[b]=a
    groups={}
    return np.array([groups.setdefault(find(i),len(groups)) for i in range(len(poses))])


def normalized_design(design):
    scales=np.sqrt(np.mean(design**2,axis=0));scales[scales<1e-12]=1
    x=design/scales
    singular=np.linalg.svd(x,compute_uv=False)
    rank=int(np.sum(singular>(singular[0]*1e-8 if len(singular) else 0)))
    return x,scales,rank,singular


def ridge(design,response):
    x,scales,_,_=normalized_design(design)
    return np.linalg.solve(x.T@x+1e-3*np.eye(x.shape[1]),x.T@response)/scales


def gravity_data_audit(model,historical,output):
    samples=[]
    for joint_index,name in enumerate(model.names):
        for source in historical['joints'][name]['gravity_samples']:
            pose=np.array([source['pose_rad'][n] for n in model.names])
            samples.append({'joint_index':joint_index,'name':name,'pose':pose,
                            'tau_model':model.signs[joint_index]*float(source['tau']),
                            'direction':source.get('direction'),'captured_at':source.get('captured_at')})
    poses=np.array([s['pose'] for s in samples]);groups=pose_groups(poses)
    response=np.array([s['tau_model'] for s in samples]);n=model.n
    # This vector would be down if both shoulder axes were horizontal. It is
    # a prior, not a base/world registration established by the photograph.
    up=np.cross(model.axes[1],model.axes[0]);up/=np.linalg.norm(up)
    if up[2]<0:up=-up
    cases={'unverified_CAD_minus_Z':np.array([0.,0.,-9.81]),
           'unverified_both_shoulder_axes_horizontal':-9.81*up}
    report={'source_gravity_sample_count':len(samples),
            'source_inertia_sample_count':sum(len(v['inertia_samples']) for v in historical['joints'].values()),
            'pose_cluster_radius_rms_deg':2.,'connected_pose_clusters':int(groups.max()+1),
            'cluster_sizes':{str(g):int(np.sum(groups==g)) for g in np.unique(groups)},
            'method':'geometry-based gravity regressor plus per-joint constant and approach-direction nuisance terms; normalized ridge for diagnostic predictions only',
            'assumptions':['gravity direction is not measured in the CAD frame',
                           'approach labels on static data are history proxies, not measured velocities',
                           'contact/payload/temperature and pure-gravity status are not certified',
                           'unconstrained diagnostic coefficients are not physical inertial parameters'],
            'cases':{},'fitted_physical_model_exported':False}
    for label,g in cases.items():
        design=[]
        for sample in samples:
            q=model.encoder_to_model(sample['pose']);i=sample['joint_index']
            row=np.zeros(4*n+2*n);row[:4*n]=model.gravity_regressor(q,g)[i]
            row[4*n+i]=1
            row[5*n+i]=model.signs[i]*{'positive':1.,'negative':-1.}.get(sample['direction'],0.)
            design.append(row)
        design=np.array(design);_,_,rank,singular=normalized_design(design)
        _,_,rigid_rank,rigid_singular=normalized_design(design[:,:4*n])
        rng=np.random.default_rng(521)
        generic=np.vstack([model.gravity_regressor(rng.uniform(-1.5,1.5,n),g) for _ in range(80)])
        _,_,structural_rank,structural_singular=normalized_design(generic)
        beta=ridge(design,response);training=design@beta-response;holdout=np.zeros(len(samples));folds=[]
        for group in np.unique(groups):
            test=groups==group;train=~test
            prediction=design[test]@ridge(design[train],response[train]);holdout[test]=prediction-response[test]
            folds.append({'held_out_pose_cluster':int(group),'test_rows':int(test.sum()),
                          'train_design_rank':normalized_design(design[train])[2],
                          'rmse_Nm':float(np.sqrt(np.mean(holdout[test]**2)))})
        report['cases'][label]={'gravity_m_s2':g.tolist(),'physical_gravity_verified':False,
            'rigid_gravity_columns':4*n,'rigid_gravity_rank':rigid_rank,
            'generic_pose_structural_gravity_rank':structural_rank,
            'observed_rank_reaches_structural_rank':rigid_rank==structural_rank,
            'rigid_gravity_observable_condition_number':float(rigid_singular[0]/rigid_singular[rigid_rank-1]),
            'generic_pose_observable_condition_number':float(structural_singular[0]/structural_singular[structural_rank-1]),
            'columns_with_nuisance':design.shape[1],'rank_with_nuisance':rank,
            'singular_values_normalized':singular.tolist(),
            'training_rmse_Nm':float(np.sqrt(np.mean(training**2))),
            'held_out_entire_pose_cluster_rmse_Nm':float(np.sqrt(np.mean(holdout**2))),
            'held_out_per_joint_rmse_Nm':{name:float(np.sqrt(np.mean(holdout[[s['name']==name for s in samples]]**2))) for name in model.names},
            'folds':folds}
    conflicts=[]
    for i,left in enumerate(samples):
        for right in samples[i+1:]:
            if left['name']!=right['name']:continue
            delta=np.arctan2(np.sin(left['pose']-right['pose']),np.cos(left['pose']-right['pose']))
            distance=float(np.degrees(np.sqrt(np.mean(delta**2))))
            difference=abs(left['tau_model']-right['tau_model'])
            if distance<=.5 and difference>=.1:
                conflicts.append({'joint':left['name'],'pose_rms_distance_deg':distance,'holding_torque_difference_Nm':difference,
                                  'left_time':left['captured_at'],'right_time':right['captured_at'],
                                  'left_direction':left['direction'],'right_direction':right['direction']})
    report['near_pose_holding_torque_conflicts']=sorted(conflicts,key=lambda r:r['holding_torque_difference_Nm'],reverse=True)
    report['interpretation']='Compare observed rank with generic-pose structural rank: mass/first-moment columns have intrinsic serial-chain null spaces, so 14/28 alone is NOT evidence of insufficient pose coverage. Gravity cannot identify the full physical inertias. Inspect conditioning and grouped holdout; establish gravity/frame, contact/payload and synchronized dynamics separately.'
    (output/'historical_data_identifiability.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    return report,samples


def render_reference_and_fk(model,cad_dir,output):
    from PIL import Image,ImageDraw,ImageFont
    image=Image.new('RGB',(1800,1320),'#f5f7fa');draw=ImageDraw.Draw(image)
    font=lambda size:ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',size)
    draw.text((35,22),'七轴连杆与新版 STEP 的参考姿态核对',font=font(33),fill='#1c344d')
    draw.text((35,78),'M6 → M5 → M4 → M7 → M3 → M2 → M1；图中连线连接坐标原点，不是外壳长度或质心。',font=font(23),fill='#344b61')
    draw.text((35,115),'末端止于 M1 腕轴坐标系，未把夹爪尖端冒充已标定 TCP。单位：mm。',font=font(23),fill='#344b61')
    mesh=np.load(cad_dir/'original_cad_review_mesh.npz')['triangles'];frames=model.fk(np.zeros(model.n));points=frames[:,:3,3]*1000
    labels=[model.rows[n]['cad_label'] for n in model.names]
    for panel,eye in enumerate(([1.,0,0],[1.,.65,.12])):
        eye=np.array(eye);eye/=np.linalg.norm(eye);right=np.cross([0,0,1],eye);right/=np.linalg.norm(right);up=np.cross(eye,right)
        x0=30+panel*890;y0=175
        draw.rounded_rectangle((x0,y0,x0+850,y0+1060),14,fill='white',outline='#c4cfda',width=2)
        draw.text((x0+20,y0+15),'侧视' if panel==0 else '斜视',font=font(25),fill='#263e56')
        lo=np.array([(mesh@right).min(),(mesh@up).min()]);hi=np.array([(mesh@right).max(),(mesh@up).max()])
        scale=min(490/(hi[0]-lo[0]),900/(hi[1]-lo[1]))
        def project(points):
            points=np.asarray(points);return np.stack([x0+70+(points@right-lo[0])*scale,y0+995-(points@up-lo[1])*scale],axis=-1)
        pixels=project(mesh);centers=mesh.mean(axis=1)
        normals=np.cross(mesh[:,1]-mesh[:,0],mesh[:,2]-mesh[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)
        shades=.74+.22*abs(normals@eye)
        for i in np.argsort(centers@eye):
            gray=int(231*shades[i]);draw.polygon([tuple(p) for p in pixels[i]],fill=(gray,gray+min(3,255-gray),gray+min(5,255-gray)))
        joint_pixels=project(points)
        draw.line([tuple(p) for p in joint_pixels],fill='#167db5',width=5)
        for i,(label,name) in enumerate(zip(labels,model.names)):
            x,y=joint_pixels[i];draw.ellipse((x-6,y-6,x+6,y+6),fill='#db542e',outline='white',width=1)
            end=project(points[i]+model.axes[i]*35)
            draw.line((x,y,*end),fill='#b34383',width=3)
            offset={'M6':-35,'M5':5,'M4':35}.get(label,0)
            tx=x0+455;ty=float(y)+offset
            draw.line((x,y,tx-10,ty+10),fill='#3a566f',width=1)
            draw.text((tx,ty),label+' '+name,font=font(21),fill='#1d3c55')
    draw.text((35,1260),'灰色：新版 STEP 原始几何；蓝线：运动学链；紫线：模型正轴。仅离线显示，不发送电机命令。',font=font(22),fill='#344b61')
    image.save(output/'cad_fk_reference_overlay.png')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'outputs/left_arm_general_model')
    parser.add_argument('--cad-dir',type=Path,default=ROOT/'outputs/left_arm_step_analysis/revised_2')
    parser.add_argument('--historical-model',type=Path,default=ROOT/'scripts/left_arm_v3_1_dynamics.json')
    args=parser.parse_args();out=args.output_dir
    model=RigidBodyModel.load(out/'left_arm_cad_exact.json');nominal=RigidBodyModel.load(out/'left_arm_nominal90.json')
    rng=np.random.default_rng(20261021);errors=[];urdf_errors=[];nominal_errors=[];angle_errors=[]
    historical=json.loads(args.historical_model.read_text(encoding='utf8'))
    audit,samples=gravity_data_audit(model,historical,out)
    poses=[rng.uniform(-.6,.6,model.n) for _ in range(100)]
    poses.extend(model.encoder_to_model(sample['pose']) for sample in samples)
    for q in poses:
        a=model.fk(q);b=model.fk_poe(q);u=read_urdf_fk(out/'left_arm_cad_exact.urdf',q);n=nominal.fk(q)
        errors.append(float(np.max(abs(a-b))));urdf_errors.append(float(np.max(abs(a-u))))
        nominal_errors.append(float(np.linalg.norm(a[-1,:3,3]-n[-1,:3,3])))
        cosine=np.clip((np.trace(a[-1,:3,:3].T@n[-1,:3,:3])-1)/2,-1,1)
        angle_errors.append(float(np.degrees(np.arccos(cosine))))
    source=json.loads((args.cad_dir/'cad_check_dimensions.json').read_text(encoding='utf8'))
    source_datums={row['controller_joint_name']:np.array(row['point_mm'])*.001 for row in source['motor_datums']}
    zero=model.fk(np.zeros(model.n));cad_error=max(np.linalg.norm(zero[i,:3,3]-source_datums[name]) for i,name in enumerate(model.names))
    assert max(errors)<1e-11 and max(urdf_errors)<1e-11 and cad_error<1e-12
    report={'source_step_sha256':source['source_sha256'],'joint_order':list(model.names),'moving_links':model.n,
            'cad_reference_max_joint_origin_error_m':float(cad_error),
            'configurations_checked':len(poses),'test_domain_note':'random mathematical angles plus historical poses, NOT hardware-approved motion or collision validation',
            'poe_vs_local_chain_max_matrix_element_error':max(errors),'urdf_vs_chain_max_matrix_element_error':max(urdf_errors),
            'nominal90_vs_exact_at_reference_joint_origin_error_m':float(np.max(abs(nominal.fk(np.zeros(model.n))[:,:3,3]-zero[:,:3,3]))),
            'nominal90_vs_exact_max_wrist_frame_origin_difference_m':max(nominal_errors),
            'nominal90_vs_exact_max_wrist_orientation_difference_deg':max(angle_errors),
            'end_frame_is_gripper_tcp':False,'physical_inertias_filled':False,'gravity_registration_verified':False,
            'gravity_in_cad_frame_m_s2':model.configured_gravity().tolist(),
            'gravity_configuration':model.data.get('gravity_configuration'),
            'hardware_execution_enabled':False,'source_CAD_and_existing_controller_files_modified':False,
            'validation_scope':'CAD consistency and numerical algorithms; not physical endpoint accuracy or full dynamics identification'}
    (out/'offline_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    render_reference_and_fk(model,args.cad_dir,out)
    required={'kinematics_complete':['seven-axis parent-child chain','reference transforms','encoder signs and offsets','CAD-exact and nominal-90 variants','independent POE/local-chain/URDF arithmetic checks'],
              'physical_inputs_still_needed':[
                  {'item':'moving-body inertial parameters or sufficiently exciting synchronized joint data','why':'STEP geometry has no densities; motor proxy volumes are not real masses. Existing closed-loop scalar inertias cannot determine the 7x7 coupled mass matrix.'},
                  {'item':'claw state and payload inertial contribution','why':'For fixed fingers, combine claw/payload mass properties into the final wrist body. Opening/closing finger dynamics are outside the seven-axis model.'},
                  {'item':'motor torque scale/bias and any reflected drive inertia','why':'MIT packet range validation alone is not a physical torque calibration'}],
              'existing_data_result':{'gravity_rows':audit['source_gravity_sample_count'],'effective_inertia_rows':audit['source_inertia_sample_count'],'pose_clusters':audit['connected_pose_clusters'],'fit_promoted_to_controller':False},
              'cartesian_endpoint_extension':{'status':'TCP not yet defined; FK currently ends at M1 wrist frame','blocks_joint_space_dynamics':False,'reason':'joint-space dynamics can include tool inertia in wrist_link without defining a grasp TCP'},
              'exact_jetson_sources_for_next_data_review':['/home/nvidia/hive_robot/DM_Control_Python/data/left_arm_v3_1_inertia_runs.jsonl','/home/nvidia/hive_robot/DM_Control_Python/data/left_arm_v3_1_model_runs.jsonl'],
              'gravity_configuration':model.data.get('gravity_configuration'),
              'next_stage':'use user-authorized nominal gravity; establish tool conventions and physical priors, then validate on robot rather than request physical angle/distance measurements',
              'preserved':['existing Model Hold takeover','adaptive learning and saved data','ordinary Clearance shoulder_rotate active-target skip'],
              'algorithm_references':['https://modernrobotics.northwestern.edu/','https://github.com/ros/urdfdom']}
    (out/'remaining_physical_inputs.json').write_text(json.dumps(required,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'fk':report,'data':{key:audit[key] for key in ('source_gravity_sample_count','source_inertia_sample_count','connected_pose_clusters')},
                      'diagnostic_fits':{key:{k:value[k] for k in ('rigid_gravity_rank','rank_with_nuisance','training_rmse_Nm','held_out_entire_pose_cluster_rmse_Nm')} for key,value in audit['cases'].items()}},indent=2))


if __name__=='__main__':main()
