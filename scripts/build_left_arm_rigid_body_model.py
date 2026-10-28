#!/usr/bin/env python3
"""Build offline serial-link models from the approved revised STEP registration.

Does not import or modify motion control, learning or the identified-data JSON.
No material density, physical inertia, TCP or hardware joint limits are
invented. Gravity uses an explicit nominal installation assumption. The URDF
is a kinematics-only preview.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from left_arm_rigid_body import RigidBodyModel, invert_transform


ROOT=Path(__file__).resolve().parents[1]


def axis_frame(axis,point):
    z=np.asarray(axis,dtype=float);z=z/np.linalg.norm(z)
    prefer=np.array([1.,0,0]) if abs(z[0])<0.95 else np.array([0.,1,0])
    x=prefer-z*(prefer@z);x/=np.linalg.norm(x)
    result=np.eye(4);result[:3,:3]=np.column_stack((x,np.cross(z,x),z));result[:3,3]=point
    return result


def set_nominal_gravity(data, nominal_axes):
    """User-authorized nominal installation; not a measured registration.

    Both nominal shoulder hinge axes are horizontal. Select the down branch
    toward CAD -Z, consistent with the approved hanging-arm reference pose.
    Use the SAME installation vector for exact and nominal link geometries.
    """
    order=data['joint_order']
    front=np.asarray(nominal_axes[order.index('shoulder_front')],dtype=float)
    side=np.asarray(nominal_axes[order.index('shoulder_side')],dtype=float)
    up=np.cross(side,front)
    if np.linalg.norm(up)<1e-10:raise ValueError('parallel shoulder axes cannot define nominal vertical')
    up/=np.linalg.norm(up)
    if up[2]<0:up=-up
    data['gravity_in_cad_frame_m_s2']=(-9.80665*up).tolist()
    data['gravity_configuration']={
        'source':'nominal-horizontal-shoulder-axes',
        'status':'assumed_for_on_robot_validation',
        'physically_measured':False,
        'magnitude_m_s2':9.80665,
        'assumptions':['approved nominal 90-degree shoulder geometry',
                       'nominal shoulder_front and shoulder_side axes horizontal',
                       'down points toward negative CAD Z in approved reference pose'],
        'user_instruction':'Use the CAD-based nominal direction directly; no additional physical angle or distance measurements.'}
    data['base_frame']='original revised STEP reference axes and origin; nominal gravity installation configured'


def make_chain(order,axes,points,signs,reference,friction=None,variant='test'):
    """Pure geometry constructor, also used by analytic mechanical tests."""
    previous=np.eye(4);rows={};links={}
    for i,name in enumerate(order):
        current=axis_frame(axes[i],points[i])
        rows[name]={'parent_link':'cad_base' if i==0 else order[i-1]+'_link',
                    'child_link':name+'_link','axis_in_joint_frame':[0.,0.,1.],
                    'axis_in_cad_reference':list(axes[i]),'axis_point_in_cad_reference_m':list(points[i]),
                    'parent_to_joint_at_reference':(invert_transform(previous)@current).tolist(),
                    'cad_reference_link_transform':current.tolist(),'encoder_sign':int(signs[i]),
                    'reference_encoder_rad':float(reference[i]),
                    'friction_prior':(friction or {}).get(name,{'coulomb':None,'viscous':None})}
        links[name+'_link']={'parent_joint':name,'inertial':{'mass_kg':None,'com_m':None,'inertia_com_kg_m2':None},
                              'physical_ownership':'rigid body downstream of this joint, including attached motor/rotor/brackets; CAD part ownership not yet mass-verified'}
        previous=current
    return {'schema':1,'type':'offline_serial_rigid_body_model','geometry_variant':variant,
            'units':{'length':'m','angle':'rad','mass':'kg','torque':'N*m'},
            'joint_order':list(order),'joints':rows,'links':links,
            'base_frame':'original revised STEP reference axes and origin; not yet registered to torso/world gravity',
            'end_frame':order[-1]+'_link','end_frame_is_tcp':False,
            'tool_tcp_transform':None,'gravity_in_cad_frame_m_s2':None,
            'claw_and_payload_policy':'For a fixed claw state, combine tool/payload mass properties into wrist_link. TCP is not needed for joint-space inverse dynamics. Moving-finger dynamics are not included in this seven-axis model.',
            'reflected_rotor_inertias_kg_m2':None,
            'physical_joint_limits':None,'hardware_execution_enabled':False}


def rotation_rpy(rotation):
    pitch=np.arctan2(-rotation[2,0],np.hypot(rotation[0,0],rotation[1,0]))
    if abs(np.cos(pitch))>1e-9:
        roll=np.arctan2(rotation[2,1],rotation[2,2]);yaw=np.arctan2(rotation[1,0],rotation[0,0])
    else:
        roll=0.;yaw=np.arctan2(-rotation[0,1],rotation[1,1])
    return [float(roll),float(pitch),float(yaw)]


def write_urdf(model,path):
    robot=ET.Element('robot',name='hive_left_arm_offline_kinematic_preview')
    robot.append(ET.Comment('KINEMATICS ONLY. No physical inertias, TCP, collision geometry, verified limits, transmissions or controller plugins. continuous describes a mathematical rotation only, not an unlimited physical joint.'))
    ET.SubElement(robot,'link',name='cad_base')
    fmt=lambda values:' '.join(f'{float(v):.16g}' for v in values)
    for name in model.names:
        row=model.rows[name]
        ET.SubElement(robot,'link',name=row['child_link'])
        joint=ET.SubElement(robot,'joint',name=name,type='continuous')
        ET.SubElement(joint,'parent',link=row['parent_link']);ET.SubElement(joint,'child',link=row['child_link'])
        transform=np.array(row['parent_to_joint_at_reference'])
        ET.SubElement(joint,'origin',xyz=fmt(transform[:3,3]),rpy=fmt(rotation_rpy(transform[:3,:3])))
        ET.SubElement(joint,'axis',xyz='0 0 1')
    ET.indent(robot,space='  ')
    ET.ElementTree(robot).write(path,encoding='utf-8',xml_declaration=True)


def build(registration_path,dynamics_path,output):
    registration=json.loads(registration_path.read_text(encoding='utf8'))
    step=Path(registration['source_step_path'])
    if hashlib.sha256(step.read_bytes()).hexdigest()!=registration['source_step_sha256']:
        raise ValueError('approved STEP source hash changed')
    if not registration['cad_pose_user_confirmed']:raise ValueError('unconfirmed CAD pose')
    dynamics=json.loads(dynamics_path.read_text(encoding='utf8'))
    order=registration['joint_order'];rows=registration['joints']
    if any(not rows[name]['direction_user_confirmed'] for name in order):raise ValueError('unconfirmed encoder direction')
    axes=np.array([rows[name]['axis_unit_direction_in_cad_reference'] for name in order])
    points=np.array([rows[name]['axis_point_in_cad_reference_mm'] for name in order])*0.001
    signs=[rows[name]['sign'] for name in order];reference=[rows[name]['reference_encoder_rad'] for name in order]
    friction={name:{**dynamics['joints'][name]['friction'],'status':'historical prior, not newly identified during direction registration'} for name in order}
    exact=make_chain(order,axes,points,signs,reference,friction,'cad_exact')
    nominal_axes=axes.copy();front=order.index('shoulder_front');side=order.index('shoulder_side');rotate=order.index('shoulder_rotate')
    nominal=np.cross(axes[front],axes[rotate]);nominal/=np.linalg.norm(nominal)
    if nominal@axes[side]<0:nominal=-nominal
    nominal_axes[side]=nominal
    simplified=make_chain(order,nominal_axes,points,signs,reference,friction,'nominal_shoulder_90deg')
    sources={'approved_step_path':str(step),'approved_step_sha256':registration['source_step_sha256'],
             'registration_path':str(registration_path.resolve()),'registration_sha256':hashlib.sha256(registration_path.read_bytes()).hexdigest(),
             'historical_dynamics_path':str(dynamics_path.resolve()),'historical_dynamics_updated_at':dynamics.get('updated_at')}
    output.mkdir(parents=True,exist_ok=True)
    for data,label in ((exact,'cad_exact'),(simplified,'nominal90')):
        set_nominal_gravity(data,nominal_axes)
        data['sources']=sources
        for name in order:data['joints'][name]['cad_label']=rows[name]['cad_label']
        data['nominal_geometry_note']='Only the M5 axis direction is orthogonalized to M6 and M4 in nominal90. All reference origins and encoder signs stay fixed. Exact geometry is retained for CAD comparison.'
        data['missing_for_physical_prediction']=['per-moving-link physically consistent mass/first moment/inertia or sufficient identifiable dynamic data',
            'claw/payload mass properties and fixed-state convention','motor-side torque scale/bias and optional reflected drive inertia']
        data['preserved_existing_behaviors']=['Model Hold takeover','adaptive learning and saved state','ordinary Clearance shoulder_rotate active-target skip']
        data['dynamic_inertial_parameter_order']=['mass','hx','hy','hz','Ixx','Iyy','Izz','Ixy','Ixz','Iyz']
        (output/f'left_arm_{label}.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
        write_urdf(RigidBodyModel(data),output/f'left_arm_{label}.urdf')
    return exact,simplified


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registration',type=Path,default=ROOT/'outputs/left_arm_step_analysis/revised_2/kinematic_calibration.json')
    parser.add_argument('--dynamics',type=Path,default=ROOT/'scripts/left_arm_v3_1_dynamics.json')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'outputs/left_arm_general_model')
    args=parser.parse_args();a,b=build(args.registration,args.dynamics,args.output_dir)
    print(json.dumps({'links':len(a['links']),'variants':[a['geometry_variant'],b['geometry_variant']],
                      'output_dir':str(args.output_dir),'hardware_execution_enabled':False}))


if __name__=='__main__':main()
