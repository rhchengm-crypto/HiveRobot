#!/usr/bin/env python3
"""Offline seven-axis geometry and rigid-body dynamics, SI units only.

No CAN imports, motion entry points, existing controller hooks or learning
writes. Joint positions here are CAD/model angles; encoder conversion is
explicit. Physical inertial parameters are required for dynamics. Gravity may
be supplied explicitly or read from the model's recorded installation setting.

Dynamics uses a world-frame Newton-Euler forward/backward recursion. The
linear parameter order for each body is m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,
where h=m*c and I is about the body-frame origin, not its centre of mass.
The mass-matrix Jacobian sum is an independent implementation for validation.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


PARAMETERS = ('mass','hx','hy','hz','Ixx','Iyy','Izz','Ixy','Ixz','Iyz')


def vector(value, length, name='vector'):
    result=np.asarray(value,dtype=float)
    if result.shape!=(length,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain {length} finite numbers')
    return result


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def axis_rotation(axis, angle):
    a=vector(axis,3,'axis');norm=np.linalg.norm(a)
    if norm<1e-12:raise ValueError('zero axis')
    k=skew(a/norm)
    return np.eye(3)+np.sin(angle)*k+(1-np.cos(angle))*(k@k)


def rigid_about(point,axis,angle):
    p=vector(point,3);r=axis_rotation(axis,angle)
    t=np.eye(4);t[:3,:3]=r;t[:3,3]=p-r@p
    return t


def invert_transform(t):
    result=np.eye(4);result[:3,:3]=t[:3,:3].T
    result[:3,3]=-t[:3,:3].T@t[:3,3]
    return result


def inertial_parameters(inertials):
    """Validate physical COM inertias, then convert to linear parameters."""
    rows=[]
    for body in inertials:
        if any(body.get(k) is None for k in ('mass_kg','com_m','inertia_com_kg_m2')):
            raise ValueError('physical mass/COM/inertia parameters incomplete; no dynamics prediction permitted')
        mass=float(body['mass_kg']);com=vector(body['com_m'],3,'COM')
        tensor=np.asarray(body['inertia_com_kg_m2'],dtype=float)
        if not np.isfinite(mass) or mass<=0:raise ValueError('body mass must be positive')
        if tensor.shape!=(3,3) or not np.isfinite(tensor).all() or not np.allclose(tensor,tensor.T,atol=1e-12):
            raise ValueError('COM inertia must be finite symmetric 3x3')
        eig=np.linalg.eigvalsh(tensor)
        if eig[0]<=0 or eig[-1]>eig[:-1].sum()+1e-12:
            raise ValueError('COM inertia must be positive and satisfy principal-inertia triangle inequalities')
        origin=tensor+mass*((com@com)*np.eye(3)-np.outer(com,com))
        rows.append([mass,*(mass*com),origin[0,0],origin[1,1],origin[2,2],origin[0,1],origin[0,2],origin[1,2]])
    return np.asarray(rows)


def quintic_reference(start,target,time_s,duration_s,active_mask=None):
    """Offline C2 joint reference with analytic position/velocity/acceleration.

    A skipped coordinate remains at its start value with zero derivatives.
    This function does not choose a robot path, verify collision/limits, or
    replace any existing trajectory/hold/skip policy.
    """
    start=np.asarray(start,dtype=float)
    if start.ndim!=1 or len(start)==0 or not np.isfinite(start).all():raise ValueError('invalid start vector')
    target=vector(target,len(start),'target')
    if not np.isfinite(time_s) or not np.isfinite(duration_s) or duration_s<=0:raise ValueError('finite time and positive duration required')
    mask=np.ones(len(start),dtype=bool) if active_mask is None else np.asarray(active_mask,dtype=bool)
    if mask.shape!=start.shape:raise ValueError('active mask shape mismatch')
    delta=np.where(mask,target-start,0.)
    s=float(np.clip(time_s/duration_s,0.,1.))
    blend=10*s**3-15*s**4+6*s**5
    velocity=(30*s**2-60*s**3+30*s**4)/duration_s
    acceleration=(60*s-180*s**2+120*s**3)/duration_s**2
    return {'q_ref':start+blend*delta,'dq_ref':velocity*delta,
            'ddq_ref':acceleration*delta,'active_mask':mask.copy()}


class RigidBodyModel:
    def __init__(self,data):
        if data.get('schema')!=1:raise ValueError('unsupported geometry schema')
        self.data=data;self.names=tuple(data['joint_order']);self.n=len(self.names)
        if self.n==0 or len(set(self.names))!=self.n:raise ValueError('invalid joint order')
        self.rows=data['joints']
        self.signs=np.array([self.rows[n]['encoder_sign'] for n in self.names])
        if not np.isin(self.signs,(-1,1)).all():raise ValueError('unconfirmed encoder signs')
        self.reference=vector([self.rows[n]['reference_encoder_rad'] for n in self.names],self.n)
        self.parents=[];self.home=[];self.axes=[];self.points=[]
        for i,name in enumerate(self.names):
            row=self.rows[name]
            if row['parent_link']!=('cad_base' if i==0 else self.names[i-1]+'_link'):
                raise ValueError('model is not the declared serial parent-child chain')
            t=np.asarray(row['parent_to_joint_at_reference'],dtype=float)
            if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1]):
                raise ValueError('invalid homogeneous joint transform')
            r=t[:3,:3]
            if not np.allclose(r.T@r,np.eye(3),atol=1e-10) or not np.isclose(np.linalg.det(r),1):
                raise ValueError('joint transform must have a proper rotation')
            self.parents.append(t)
            self.home.append(np.asarray(row['cad_reference_link_transform'],dtype=float))
            self.axes.append(vector(row['axis_in_cad_reference'],3))
            self.points.append(vector(row['axis_point_in_cad_reference_m'],3))
        self.home=np.array(self.home);self.axes=np.array(self.axes);self.points=np.array(self.points)
        if not np.allclose(np.linalg.norm(self.axes,axis=1),1,atol=1e-10):raise ValueError('nonunit CAD axis')
        self.inertials=[data['links'][name+'_link']['inertial'] for name in self.names]
        reference_frames=self.fk(np.zeros(self.n))
        if self.home.shape!=(self.n,4,4) or not np.isfinite(self.home).all() or not np.allclose(reference_frames,self.home,atol=1e-10):
            raise ValueError('reference link frames contradict the parent-child transforms')
        if not np.allclose(self.home[:,:3,3],self.points,atol=1e-10) or not np.allclose(self.home[:,:3,2],self.axes,atol=1e-10):
            raise ValueError('reference axis locations/directions contradict the link frames')

    @classmethod
    def load(cls,path):
        return cls(json.loads(Path(path).read_text(encoding='utf8')))

    def encoder_to_model(self,q):
        # Preserve absolute encoder branch; do not wrap states independently.
        return self.signs*(vector(q,self.n,'encoder angles')-self.reference)

    def model_to_encoder(self,q):
        return self.reference+self.signs*vector(q,self.n,'model angles')

    def fk(self,q):
        q=vector(q,self.n,'model angles');current=np.eye(4);result=[]
        for t,angle in zip(self.parents,q):
            motion=np.eye(4);motion[:3,:3]=axis_rotation([0,0,1],angle)
            current=current@t@motion;result.append(current.copy())
        return np.array(result)

    def fk_poe(self,q):
        """Independent space-axis product of exponentials for all link frames."""
        q=vector(q,self.n);product=np.eye(4);result=[]
        for p,a,angle,home in zip(self.points,self.axes,q,self.home):
            product=product@rigid_about(p,a,angle)
            result.append(product@home)
        return np.array(result)

    def point_jacobian(self,q,link_index,point_local=None):
        if not 0<=link_index<self.n:raise ValueError('invalid link index')
        frames=self.fk(q);point=np.zeros(3) if point_local is None else vector(point_local,3)
        world=frames[link_index,:3,3]+frames[link_index,:3,:3]@point
        linear=np.zeros((3,self.n));angular=np.zeros((3,self.n))
        for i in range(link_index+1):
            axis=frames[i,:3,2];origin=frames[i,:3,3]
            angular[:,i]=axis;linear[:,i]=np.cross(axis,world-origin)
        return linear,angular

    def _parameters(self,inertials):
        result=inertial_parameters(self.inertials if inertials is None else inertials)
        if result.shape!=(self.n,10):raise ValueError('one inertial record per moving link is required')
        return result

    def _inverse_linear(self,q,dq,ddq,gravity,parameters):
        q=vector(q,self.n);dq=vector(dq,self.n);ddq=vector(ddq,self.n)
        gravity=vector(gravity,3,'gravity in CAD base frame');frames=self.fk(q)
        p_prev=np.zeros(3);omega=np.zeros(3);alpha=np.zeros(3);acc=-gravity
        forces=[];moments=[]
        for i,frame in enumerate(frames):
            p=frame[:3,3];r=p-p_prev;axis=frame[:3,2];rotation=frame[:3,:3]
            acc=acc+np.cross(alpha,r)+np.cross(omega,np.cross(omega,r))
            next_alpha=alpha+axis*ddq[i]+np.cross(omega,axis*dq[i])
            omega=omega+axis*dq[i];alpha=next_alpha
            par=parameters[i];mass=par[0];h=rotation@par[1:4]
            xx,yy,zz,xy,xz,yz=par[4:]
            tensor=rotation@np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])@rotation.T
            forces.append(mass*acc+np.cross(alpha,h)+np.cross(omega,np.cross(omega,h)))
            moments.append(tensor@alpha+np.cross(omega,tensor@omega)+np.cross(h,acc))
            p_prev=p
        tau=np.zeros(self.n)
        for i in range(self.n-1,-1,-1):
            if i<self.n-1:
                moments[i]=moments[i]+moments[i+1]+np.cross(frames[i+1,:3,3]-frames[i,:3,3],forces[i+1])
                forces[i]=forces[i]+forces[i+1]
            tau[i]=frames[i,:3,2]@moments[i]
        return tau

    def configured_gravity(self,gravity=None):
        """Explicit override wins; otherwise use recorded CAD-frame gravity."""
        value=self.data.get('gravity_in_cad_frame_m_s2') if gravity is None else gravity
        if value is None:raise ValueError('gravity is not configured; provide an explicit CAD-frame vector')
        return vector(value,3,'gravity in CAD base frame').copy()

    def inverse_dynamics(self,q,dq,ddq,gravity=None,*,inertials=None):
        return (self._inverse_linear(q,dq,ddq,self.configured_gravity(gravity),self._parameters(inertials))
                + self.rotor_inertias()*vector(ddq,self.n))

    def rotor_inertias(self):
        value=self.data.get('reflected_rotor_inertias_kg_m2')
        result=np.zeros(self.n) if value is None else vector(value,self.n,'reflected rotor inertias')
        if np.any(result<0):raise ValueError('negative reflected rotor inertia')
        return result

    def mass_matrix(self,q,*,inertials=None):
        parameters=self._parameters(inertials);zero=np.zeros(self.n)
        return np.column_stack([self._inverse_linear(q,zero,np.eye(self.n)[i],np.zeros(3),parameters) for i in range(self.n)])+np.diag(self.rotor_inertias())

    def mass_matrix_jacobians(self,q,*,inertials=None):
        bodies=self.inertials if inertials is None else inertials
        self._parameters(bodies);frames=self.fk(q);result=np.zeros((self.n,self.n))
        for i,body in enumerate(bodies):
            jv,jw=self.point_jacobian(q,i,body['com_m']);r=frames[i,:3,:3]
            tensor=r@np.asarray(body['inertia_com_kg_m2'])@r.T
            result+=body['mass_kg']*(jv.T@jv)+jw.T@tensor@jw
        return result+np.diag(self.rotor_inertias())

    def gravity_torques(self,q,gravity=None,*,inertials=None):
        zero=np.zeros(self.n)
        return self.inverse_dynamics(q,zero,zero,gravity,inertials=inertials)

    def coriolis_bias(self,q,dq,*,inertials=None):
        return self.inverse_dynamics(q,dq,np.zeros(self.n),np.zeros(3),inertials=inertials)

    def potential_energy(self,q,gravity=None,*,inertials=None):
        bodies=self.inertials if inertials is None else inertials;self._parameters(bodies)
        gravity=self.configured_gravity(gravity);frames=self.fk(q)
        return -sum(body['mass_kg']*(gravity@(frame[:3,3]+frame[:3,:3]@np.asarray(body['com_m']))) for frame,body in zip(frames,bodies))

    def gravity_regressor(self,q,gravity=None):
        """Columns per link: mass, first moments hx,hy,hz (kg, kg*m)."""
        gravity=self.configured_gravity(gravity);frames=self.fk(q);result=np.zeros((self.n,4*self.n))
        for i,frame in enumerate(frames):
            origin=frame[:3,3];r=frame[:3,:3]
            for k in range(i+1):
                axis=frames[k,:3,2]
                result[k,4*i]=-gravity@np.cross(axis,origin-frames[k,:3,3])
                for c in range(3):result[k,4*i+1+c]=-gravity@np.cross(axis,r[:,c])
        return result

    def dynamics_regressor(self,q,dq,ddq,gravity=None):
        gravity=self.configured_gravity(gravity)
        result=np.zeros((self.n,10*self.n))
        for i in range(10*self.n):
            parameters=np.zeros((self.n,10));parameters.flat[i]=1
            result[:,i]=self._inverse_linear(q,dq,ddq,gravity,parameters)
        return result

    def friction(self,dq,velocity_scale=0.05):
        if not np.isfinite(velocity_scale) or velocity_scale<=0:raise ValueError('positive friction velocity scale required')
        dq=vector(dq,self.n);result=[]
        for name,v in zip(self.names,dq):
            f=self.rows[name]['friction_prior']
            if f.get('coulomb') is None or f.get('viscous') is None:raise ValueError('friction parameters unavailable')
            if any(not np.isfinite(f[k]) or f[k]<0 for k in ('coulomb','viscous')):raise ValueError('friction priors must be finite and nonnegative')
            result.append(f['coulomb']*np.tanh(v/velocity_scale)+f['viscous']*v)
        return np.array(result)
