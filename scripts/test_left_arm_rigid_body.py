import json
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from build_left_arm_rigid_body_model import make_chain
from left_arm_rigid_body import RigidBodyModel, axis_rotation, inertial_parameters, quintic_reference


ROOT=Path(__file__).resolve().parents[1]


def synthetic_bodies(n):
    return [{'mass_kg':0.4+0.1*i,'com_m':[0.02,-0.015,-0.07],
             'inertia_com_kg_m2':np.diag([0.002,0.003,0.004]).tolist()} for i in range(n)]


def planar_two_link():
    return RigidBodyModel(make_chain(['a','b'],[[0,0,1]]*2,[[0,0,0],[0.7,0,0]],[1,-1],[0,0]))


def urdf_fk(path,angles):
    tree=ET.parse(path);result=[];current=np.eye(4)
    for joint,q in zip(tree.findall('joint'),angles):
        origin=joint.find('origin');xyz=np.fromstring(origin.get('xyz'),sep=' ');roll,pitch,yaw=np.fromstring(origin.get('rpy'),sep=' ')
        cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch);cy,sy=np.cos(yaw),np.sin(yaw)
        rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]);ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]);rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        fixed=np.eye(4);fixed[:3,:3]=rz@ry@rx;fixed[:3,3]=xyz
        c,s=np.cos(q),np.sin(q);motion=np.eye(4);motion[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
        current=current@fixed@motion;result.append(current.copy())
    return np.array(result)


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models=[RigidBodyModel.load(ROOT/f'outputs/left_arm_general_model/left_arm_{variant}.json') for variant in ('cad_exact','nominal90')]

    def test_reference_encoder_maps_to_revised_cad(self):
        for model in self.models:
            np.testing.assert_allclose(model.encoder_to_model(model.reference),0,atol=0)
            np.testing.assert_allclose(model.fk(np.zeros(model.n)),model.home,atol=1e-13)
            np.testing.assert_allclose(model.fk(np.zeros(model.n))[:,:3,3],model.points,atol=1e-13)

    def test_poe_and_local_chain_across_random_configurations(self):
        rng=np.random.default_rng(173)
        for model in self.models:
            for _ in range(30):
                q=rng.uniform(-1.2,1.2,model.n)
                np.testing.assert_allclose(model.fk(q),model.fk_poe(q),atol=2e-12)
                np.testing.assert_allclose(model.encoder_to_model(model.model_to_encoder(q)),q,atol=1e-14)

    def test_exported_urdf_matches_independent_parser(self):
        rng=np.random.default_rng(651)
        for model,variant in zip(self.models,('cad_exact','nominal90')):
            for _ in range(8):
                q=rng.normal(0,.4,model.n)
                np.testing.assert_allclose(urdf_fk(ROOT/f'outputs/left_arm_general_model/left_arm_{variant}.urdf',q),model.fk(q),atol=2e-12)

    def test_jacobian_against_position_and_rotation_differences(self):
        model=self.models[0];q=np.linspace(-.3,.4,model.n);local=np.array([.03,.02,-.04]);eps=1e-6
        jv,jw=model.point_jacobian(q,model.n-1,local);rotation=model.fk(q)[-1,:3,:3]
        for i in range(model.n):
            plus=q.copy();minus=q.copy();plus[i]+=eps;minus[i]-=eps
            a=model.fk(plus)[-1];b=model.fk(minus)[-1]
            velocity=((a[:3,:3]@local+a[:3,3])-(b[:3,:3]@local+b[:3,3]))/(2*eps)
            spin=((a[:3,:3]-b[:3,:3])/(2*eps))@rotation.T
            np.testing.assert_allclose(velocity,jv[:,i],atol=2e-10)
            np.testing.assert_allclose([spin[2,1],spin[0,2],spin[1,0]],jw[:,i],atol=2e-10)

    def test_child_motion_does_not_move_upstream_frames(self):
        model=self.models[0];zero=model.fk(np.zeros(model.n))
        for i in range(model.n):
            q=np.zeros(model.n);q[i]=.2
            np.testing.assert_allclose(model.fk(q)[:i],zero[:i],atol=1e-14)

    def test_nominal_shoulder_axis_is_orthogonal(self):
        model=self.models[1]
        self.assertAlmostEqual(model.axes[0]@model.axes[1],0,places=13)
        self.assertAlmostEqual(model.axes[1]@model.axes[2],0,places=13)

    def test_invalid_input_and_missing_physical_parameters_rejected(self):
        model=self.models[0]
        with self.assertRaises(ValueError):model.fk([float('nan')]*7)
        with self.assertRaises(ValueError):model.fk([0]*6)
        with self.assertRaisesRegex(ValueError,'incomplete'):
            model.gravity_torques(np.zeros(7),[0,0,-9.81])
        bad=synthetic_bodies(1);bad[0]['inertia_com_kg_m2']=np.diag([.001,.001,.1]).tolist()
        with self.assertRaisesRegex(ValueError,'triangle'):inertial_parameters(bad)


class DynamicsTests(unittest.TestCase):
    def test_model_gravity_default_and_explicit_override(self):
        model=RigidBodyModel.load(ROOT/'outputs/left_arm_general_model/left_arm_cad_exact.json')
        bodies=synthetic_bodies(model.n);q=np.linspace(-.1,.2,model.n);zero=np.zeros(model.n)
        g=model.configured_gravity()
        self.assertAlmostEqual(float(np.linalg.norm(g)),9.80665,places=12)
        self.assertLess(g[2],0)
        self.assertFalse(model.data['gravity_configuration']['physically_measured'])
        for method,args in ((model.gravity_torques,(q,)),(model.potential_energy,(q,)),
                            (model.inverse_dynamics,(q,zero,zero))):
            np.testing.assert_allclose(method(*args,inertials=bodies),method(*args,g,inertials=bodies))
        np.testing.assert_allclose(model.gravity_regressor(q),model.gravity_regressor(q,g))
        np.testing.assert_allclose(model.dynamics_regressor(q,zero,zero),model.dynamics_regressor(q,zero,zero,g))
        np.testing.assert_allclose(model.gravity_torques(q,[0,0,0],inertials=bodies),0,atol=1e-12)
        g[:]=0
        self.assertGreater(np.linalg.norm(model.configured_gravity()),9)

    def test_nominal_installation_is_shared_by_geometry_variants(self):
        exact=RigidBodyModel.load(ROOT/'outputs/left_arm_general_model/left_arm_cad_exact.json')
        nominal=RigidBodyModel.load(ROOT/'outputs/left_arm_general_model/left_arm_nominal90.json')
        g=nominal.configured_gravity()
        np.testing.assert_allclose(exact.configured_gravity(),g,atol=1e-13)
        self.assertAlmostEqual(g@nominal.axes[0],0,places=12)
        self.assertAlmostEqual(g@nominal.axes[1],0,places=12)

    def test_missing_or_invalid_gravity_is_not_silently_assumed(self):
        model=planar_two_link()
        with self.assertRaisesRegex(ValueError,'not configured'):model.gravity_regressor([0,0])
        with self.assertRaises(ValueError):model.configured_gravity([0,float('nan'),0])
        np.testing.assert_array_equal(model.configured_gravity([0,0,0]),[0,0,0])

    def test_single_pendulum_known_gravity_and_inertia(self):
        model=RigidBodyModel(make_chain(['a'],[[0,0,1]],[[0,0,0]],[1],[0]))
        body=[{'mass_kg':2.,'com_m':[.4,0,0],'inertia_com_kg_m2':np.diag([.001,.1076666666667,.1076666666667]).tolist()}]
        q=[.35]
        np.testing.assert_allclose(model.gravity_torques(q,[0,-9.81,0],inertials=body),[2*.4*9.81*np.cos(.35)],atol=1e-12)
        np.testing.assert_allclose(model.mass_matrix(q,inertials=body),[[.1076666666667+2*.4**2]],atol=1e-12)

    def test_two_link_analytic_mass_and_coriolis(self):
        model=planar_two_link();l=.7;c1=.35;c2=.25;m1=1.3;m2=.9;i1=.04;i2=.02
        bodies=[{'mass_kg':m1,'com_m':[c1,0,0],'inertia_com_kg_m2':np.diag([.01,i1,i1]).tolist()},
                {'mass_kg':m2,'com_m':[c2,0,0],'inertia_com_kg_m2':np.diag([.005,i2,i2]).tolist()}]
        q=np.array([.4,-.7]);dq=np.array([.3,-.2]);co=np.cos(q[1]);h=m2*l*c2*np.sin(q[1])
        expected=np.array([[i1+i2+m1*c1*c1+m2*(l*l+c2*c2+2*l*c2*co),i2+m2*(c2*c2+l*c2*co)],
                           [i2+m2*(c2*c2+l*c2*co),i2+m2*c2*c2]])
        np.testing.assert_allclose(model.mass_matrix(q,inertials=bodies),expected,atol=1e-12)
        np.testing.assert_allclose(model.coriolis_bias(q,dq,inertials=bodies),[-h*(2*dq[0]*dq[1]+dq[1]**2),h*dq[0]**2],atol=1e-12)
        gravity=[(m1*c1+m2*l)*9.81*np.cos(q[0])+m2*c2*9.81*np.cos(q.sum()),m2*c2*9.81*np.cos(q.sum())]
        np.testing.assert_allclose(model.gravity_torques(q,[0,-9.81,0],inertials=bodies),gravity,atol=1e-12)

    def test_seven_link_mass_matches_jacobians_and_is_positive(self):
        model=RigidBodyModel.load(ROOT/'outputs/left_arm_general_model/left_arm_cad_exact.json');bodies=synthetic_bodies(7);rng=np.random.default_rng(95)
        for _ in range(5):
            q=rng.normal(0,.4,7);mass=model.mass_matrix(q,inertials=bodies)
            np.testing.assert_allclose(mass,mass.T,atol=2e-14)
            np.testing.assert_allclose(mass,model.mass_matrix_jacobians(q,inertials=bodies),atol=2e-14)
            self.assertGreater(np.linalg.eigvalsh(mass)[0],0)

    def test_gravity_is_potential_gradient_and_regressor_matches(self):
        model=RigidBodyModel.load(ROOT/'outputs/left_arm_general_model/left_arm_cad_exact.json');bodies=synthetic_bodies(7);q=np.linspace(-.2,.3,7);g=[.3,-1.,-9.7];eps=1e-6
        expected=model.gravity_torques(q,g,inertials=bodies);numeric=[]
        for i in range(7):
            a=q.copy();b=q.copy();a[i]+=eps;b[i]-=eps
            numeric.append((model.potential_energy(a,g,inertials=bodies)-model.potential_energy(b,g,inertials=bodies))/(2*eps))
        np.testing.assert_allclose(numeric,expected,atol=2e-9)
        parameters=inertial_parameters(bodies)
        np.testing.assert_allclose(model.gravity_regressor(q,g)@parameters[:,:4].reshape(-1),expected,atol=1e-12)

    def test_dynamic_regressor_and_decomposition(self):
        model=RigidBodyModel.load(ROOT/'outputs/left_arm_general_model/left_arm_cad_exact.json');bodies=synthetic_bodies(7)
        q=np.linspace(.2,-.4,7);dq=np.linspace(-.1,.3,7);ddq=np.linspace(.4,-.2,7);g=[0,0,-9.81]
        torque=model.inverse_dynamics(q,dq,ddq,g,inertials=bodies)
        np.testing.assert_allclose(model.dynamics_regressor(q,dq,ddq,g)@inertial_parameters(bodies).reshape(-1),torque,atol=2e-12)
        np.testing.assert_allclose(model.mass_matrix(q,inertials=bodies)@ddq+model.coriolis_bias(q,dq,inertials=bodies)+model.gravity_torques(q,g,inertials=bodies),torque,atol=2e-12)

    def test_coriolis_obeys_kinetic_energy_identity(self):
        model=RigidBodyModel.load(ROOT/'outputs/left_arm_general_model/left_arm_cad_exact.json');bodies=synthetic_bodies(7)
        q=np.linspace(.1,.3,7);dq=np.linspace(.3,-.2,7);eps=1e-6
        mass_dot=(model.mass_matrix(q+eps*dq,inertials=bodies)-model.mass_matrix(q-eps*dq,inertials=bodies))/(2*eps)
        self.assertAlmostEqual(float(dq@model.coriolis_bias(q,dq,inertials=bodies)),float(.5*dq@mass_dot@dq),places=10)

    def test_signed_torque_mapping_preserves_virtual_power(self):
        model=RigidBodyModel.load(ROOT/'outputs/left_arm_general_model/left_arm_cad_exact.json')
        dq_encoder=np.linspace(-.2,.3,7);tau_model=np.linspace(-1.,2.,7)
        self.assertAlmostEqual(float(tau_model@(model.signs*dq_encoder)),float((model.signs*tau_model)@dq_encoder),places=14)


class ReferenceTests(unittest.TestCase):
    def test_endpoints_have_zero_velocity_and_acceleration(self):
        start=np.array([.1,-.2]);target=np.array([.4,.3])
        for time_s,expected in ((-1,start),(0,start),(3,target),(4,target)):
            row=quintic_reference(start,target,time_s,3.)
            np.testing.assert_allclose(row['q_ref'],expected,atol=1e-14)
            np.testing.assert_allclose(row['dq_ref'],0,atol=1e-14)
            np.testing.assert_allclose(row['ddq_ref'],0,atol=1e-14)

    def test_derivatives_and_skip_mask(self):
        start=np.zeros(7);target=np.linspace(.1,.7,7);mask=np.ones(7,dtype=bool);mask[2]=False
        t=.7;eps=1e-5;duration=2.
        a=quintic_reference(start,target,t+eps,duration,mask)
        b=quintic_reference(start,target,t-eps,duration,mask)
        center=quintic_reference(start,target,t,duration,mask)
        np.testing.assert_allclose((a['q_ref']-b['q_ref'])/(2*eps),center['dq_ref'],atol=2e-10)
        np.testing.assert_allclose((a['dq_ref']-b['dq_ref'])/(2*eps),center['ddq_ref'],atol=2e-10)
        for row in (a,b,center):
            self.assertEqual(row['q_ref'][2],0);self.assertEqual(row['dq_ref'][2],0);self.assertEqual(row['ddq_ref'][2],0)


if __name__=='__main__':unittest.main()
