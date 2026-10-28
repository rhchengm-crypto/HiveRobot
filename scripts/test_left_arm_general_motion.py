import copy
import io
import json
import tempfile
import time
import types
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import numpy as np

import left_arm_general_motion as general
import left_arm_v3_1_motion_dynamics as motion
from fit_left_arm_general_dynamics import nonnegative_fit
from left_arm_rigid_body import RigidBodyModel
from test_left_arm_v31_motion_dynamics import load_legacy_without_hardware

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT/'outputs/left_arm_general_model/left_arm_general_dynamics.json'


class DeterministicWorker:
    """Mathematical oracle for trajectory tests, NOT the runtime backend."""
    def __init__(self, data):
        self.model=RigidBodyModel(data)
        self.session=0;self.sequence=0;self.result=None;self.frozen=False;self.alive=True

    def new_session(self, include_gravity=False):
        self.session+=1
        self.include_gravity=include_gravity
        return self.session

    def submit(self,session,now,entry,q,dq,ddq):
        if self.frozen:return True
        self.sequence+=1
        self.result={'session':session,'sequence':self.sequence,'submitted':now,'ok':True,'compute_ms':1.,
                     'prediction':self.model.signs*(self.model.inverse_dynamics(q,dq,ddq)+self.model.friction(dq)
                                                    -self.model.gravity_torques(entry))}
        self.result['gravity_delta'] = (self.model.signs*(self.model.gravity_torques(q)-self.model.gravity_torques(entry))
                                       if self.include_gravity else np.zeros(self.model.n))
        return True

    def poll(self):return self.result
    def is_alive(self):return self.alive
    def close(self,wait=True):self.alive=False


class GeneralMotionTests(unittest.TestCase):
    def config(self, mode, path):
        with mock.patch.dict('os.environ', {general.LOG_ENV: str(path)}), redirect_stdout(io.StringIO()):
            config=general.load_configuration(MODEL, mode)
            config['worker']=DeterministicWorker(config['model'].data)
            return config

    def test_fit_uses_all_historical_records_and_physical_mass_matrix(self):
        model = RigidBodyModel.load(MODEL)
        self.assertEqual(model.data['identification']['unique_inertia_records_used'], 29)
        self.assertEqual(model.data['identification']['gravity_records_used'], 73)
        self.assertFalse(model.data['identification']['physical_parameters_independently_verified'])
        for q in np.random.default_rng(21).uniform(-.5,.5,(8,7)):
            a = model.mass_matrix(q)
            np.testing.assert_allclose(a, a.T, atol=1e-12)
            self.assertGreater(np.linalg.eigvalsh(a)[0], 0)
            np.testing.assert_allclose(a,model.mass_matrix_jacobians(q),atol=1e-12)
            dq = np.linspace(-.1,.2,7)
            ddq = np.linspace(.2,-.2,7)
            np.testing.assert_allclose(model.inverse_dynamics(q,dq,ddq),
                a@ddq+model.coriolis_bias(q,dq)+model.gravity_torques(q),atol=1e-12)

    def test_nonnegative_solver_and_lower_bound(self):
        answer,_ = nonnegative_fit(np.eye(3),[1,-2,3],[0,0,.5],ridge=0)
        np.testing.assert_allclose(answer,[1,0,3],atol=1e-10)

    def test_missing_parameters_refuse_apply_before_hardware(self):
        path=ROOT/'outputs/left_arm_general_model/left_arm_cad_exact.json'
        with redirect_stdout(io.StringIO()):
            self.assertTrue(general.load_configuration(path,'shadow')['missing_inertials'])
        with self.assertRaisesRegex(ValueError,'mass/COM/inertia'):
            general.load_configuration(path,'apply')

    def test_apply_uses_encoder_torque_sign_and_entry_anchor(self):
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            config=self.config('apply',Path(folder)/'log.jsonl')
            model=config['model'];pose=dict(zip(model.names,model.reference))
            session=general.GeneralMotionSession(config,pose,['arm_roll'])
            q=dict(pose,arm_roll=pose['arm_roll']+.04)
            v=dict.fromkeys(q,0.);a=dict(v);v['arm_roll']=.04;a['arm_roll']=.1
            session.prepare(q,.5,0,v,a)
            qm=model.encoder_to_model([q[n] for n in model.names])
            expected=model.signs*(model.inverse_dynamics(qm,model.signs*np.array(list(v.values())),
                     model.signs*np.array(list(a.values())))+model.friction(model.signs*np.array(list(v.values())))
                     -model.gravity_torques(session.entry_q))
            np.testing.assert_allclose(session.prediction,expected,atol=1e-12)
            self.assertEqual(session.transform('arm_roll',18,1,.2),(18,1,.2))

    def run_trajectory(self, kind, mode, log_path, sparse=False):
        legacy=load_legacy_without_hardware()
        arm=legacy.LeftArmV2.__new__(legacy.LeftArmV2)
        model=RigidBodyModel.load(MODEL)
        pose=dict(zip(model.names,model.reference))
        if mode is None:
            arm.motion_dynamics_model=None
        else:
            config=self.config(mode,log_path)
            if sparse:
                from test_left_arm_v31_motion_dynamics import fixture
                config['legacy']=fixture()[0]
            arm.motion_dynamics_model={'rigid_body_configuration':config}
        arm.specs=dict.fromkeys(list(model.names)+['claw'])
        arm.motors={n:n for n in arm.specs}
        arm.positions=lambda names:{n:pose.get(n,1.) for n in names}
        output=[]
        arm.ctrl=types.SimpleNamespace(controlMIT=lambda *args:output.append(args))
        clock=types.SimpleNamespace(now=0.)
        def sleep(s):clock.now+=s
        with (mock.patch.object(legacy.time,'time',lambda:clock.now),
              mock.patch.object(legacy.time,'monotonic',lambda:clock.now),
              mock.patch.object(legacy.time,'sleep',sleep),redirect_stdout(io.StringIO())):
            target=pose['elbow']+.15
            holds={'wrist':pose['wrist'],'claw':1.}
            if kind=='single':
                arm.move_target_with_holds('elbow',target,2.,40,1.5,holds,{},3,.3,
                    active_tau=1.,control_dt=.01,trajectory='blend_smootherstep',linear_blend=.35)
            elif kind=='group':
                arm.move_targets_with_holds({'elbow':target},2.,{'elbow':{'kp':40,'kd':1.5}},
                    holds,{},3,.3,control_dt=.01,trajectory='blend_smootherstep',linear_blend=.35,
                    progress_windows={'elbow':(.2,.8)})
            elif kind=='auto':
                arm.move_targets_auto_gains({'elbow':target},2.,{'elbow':{'kp':40,'kd':1.5}},3,.3)
            else:
                arm.move_targets({'elbow':target},2.,40,1.5)
        return output

    def test_all_four_real_loops_shadow_is_command_identical(self):
        with tempfile.TemporaryDirectory() as folder:
            for kind in ('single','group','auto','basic'):
                with self.subTest(kind=kind):
                    path=Path(folder)/(kind+'.jsonl')
                    baseline=self.run_trajectory(kind,None,path)
                    shadow=self.run_trajectory(kind,'shadow',path)
                    self.assertEqual(baseline,shadow)
                    report=json.loads(path.read_text(encoding='utf8'))
                    self.assertEqual(report['prediction_status'],'available')
                    self.assertNotIn('shoulder_rotate',report['commanded_joints'])
                    self.assertTrue(report['terminal_correction_zero'])
                    self.assertGreater(max(abs(x['ddq_model_ref'][3]) for x in report['reference_samples']),0)
                    self.assertEqual(report['reference_samples'][-1]['ddq_model_ref'],[0]*7)

    def test_all_four_real_loops_apply_preserves_baseline_except_bounded_tau(self):
        with tempfile.TemporaryDirectory() as folder:
            for kind in ('single','group','auto','basic'):
                with self.subTest(kind=kind):
                    path=Path(folder)/(kind+'.jsonl')
                    baseline=self.run_trajectory(kind,None,path)
                    applied=self.run_trajectory(kind,'apply',path)
                    self.assertEqual(len(baseline),len(applied))
                    for old,new in zip(baseline,applied):
                        self.assertEqual(old[:5],new[:5])
                        self.assertLessEqual(abs(old[5]-new[5]),general.MAX_TAU_CORRECTION+1e-12)
                        self.assertNotEqual(new[0],'shoulder_rotate')
                        if new[0]=='claw':self.assertEqual(old,new)
                    self.assertTrue(any(a!=b for a,b in zip(baseline,applied)))
                    self.assertEqual(baseline[-10:],applied[-10:])
                    self.assertTrue(json.loads(path.read_text(encoding='utf8'))['terminal_correction_zero'])

    def test_reference_derivatives_match_actual_trajectory_including_window(self):
        for kind in ('smoothstep','smootherstep','blend_smootherstep','linear'):
            def shape(t):
                if kind=='linear':return t
                if kind=='smoothstep':return t*t*(3-2*t)
                s=t**3*(10-15*t+6*t*t)
                return .35*t+.65*s if kind=='blend_smootherstep' else s
            for window in (None,(.2,.8)):
                def shaped(t):
                    s=shape(t)
                    if window is None:return s
                    u=np.clip((s-.2)/.6,0,1)
                    return u*u*(3-2*u)
                t=.41;h=1e-4;duration=2.
                v,a=motion.reference_derivatives(kind,t,duration,.35,window)
                self.assertAlmostEqual(v,(shaped(t+h)-shaped(t-h))/(2*h*duration),places=6)
                self.assertAlmostEqual(a,(shaped(t+h)-2*shaped(t)+shaped(t-h))/(h*h*duration**2),places=5)

    def test_board_gravity_is_not_capped_to_point_five_or_faded_at_endpoint(self):
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            config=self.config('apply',Path(folder)/'log.jsonl')
            config['board_gravity_baseline']=True
            model=config['model']
            start=dict(zip(model.names,[.8291370868682861,-2.853627920150757,2.3409247398376465,
                        -1.2796597480773926,-2.6613640785217285,.1661325991153717,-2.0708401203155518]))
            end=dict(zip(model.names,[1.942282795906067,-2.72163724899292,2.3409247398376465,
                      .8169298768043518,-2.6537346839904785,.3065156042575836,-1.0263599157333374]))
            session=general.GeneralMotionSession(config,start,list(model.names))
            zeros=dict.fromkeys(start,0.)
            baseline=-1.1145299673080444
            previous=baseline
            for frame in range(201):
                u=min(frame/160,1.)
                target={n:start[n]+(end[n]-start[n])*u for n in start}
                session.prepare(target,u,frame*.05,zeros,zeros)
                output=session.transform('elbow',70,3,baseline)[2]
                self.assertLessEqual(abs(output-previous),general.TAU_SLEW_PER_SECOND*.05+1e-10)
                previous=output
            expected=model.signs[3]*(model.gravity_torques(model.encoder_to_model(list(end.values())))[3]
                                     -model.gravity_torques(session.entry_q)[3])
            self.assertGreater(abs(session.gravity_applied['elbow']),.5)
            self.assertAlmostEqual(output,baseline+expected,places=10)
            self.assertEqual(session.corrections['elbow'],0.)  # no double counted gravity
            self.assertGreater(output,0.)  # observed bad v5 command was negative
            session.finish()
            report=json.loads((Path(folder)/'log.jsonl').read_text())
            self.assertTrue(report['terminal_gravity_retained_for_support'])

    def test_board_stale_prediction_keeps_gravity_then_aborts_instead_of_fading_it(self):
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            config=self.config('apply',Path(folder)/'log.jsonl');config['board_gravity_baseline']=True
            model=config['model'];pose=dict(zip(model.names,model.reference));zeros=dict.fromkeys(pose,0.)
            session=general.GeneralMotionSession(config,pose,['elbow'])
            target=dict(pose,elbow=pose['elbow']+.3)
            for frame in range(20):
                session.prepare(target,.5,frame*.01,zeros,zeros);session.transform('elbow',70,3,1.)
            previous=session.gravity_applied['elbow'];config['worker'].frozen=True
            session.prepare(target,.5,.35,zeros,zeros);session.transform('elbow',70,3,1.)
            self.assertGreaterEqual(abs(session.gravity_applied['elbow']),abs(previous))
            with self.assertRaisesRegex(RuntimeError,'250ms'):
                session.prepare(target,.5,.6,zeros,zeros)

    def test_stale_dead_and_previous_session_results_do_not_drive_motors(self):
        with tempfile.TemporaryDirectory() as folder,redirect_stdout(io.StringIO()):
            config=self.config('apply',Path(folder)/'log.jsonl')
            model=config['model'];pose=dict(zip(model.names,model.reference))
            session=general.GeneralMotionSession(config,pose,['arm_roll'])
            target=dict(pose,arm_roll=pose['arm_roll']+.04)
            zeros=dict.fromkeys(pose,0.)
            for frame in range(10):
                session.prepare(target,.5,frame*.01,zeros,zeros)
                session.transform('arm_roll',18,1,.2)
            self.assertGreater(session.accepted_results,0)
            worker=config['worker'];worker.frozen=True
            session.prepare(target,.5,.3,zeros,zeros)
            self.assertIsNone(session.prediction)
            self.assertEqual(session.goal['arm_roll'],0)
            self.assertGreater(session.stale_frames,0)
            worker.alive=False
            session.prepare(target,.5,.31,zeros,zeros)
            self.assertIsNone(session.prediction)
            self.assertGreater(session.worker_error_frames,0)
            worker.alive=True
            other=general.GeneralMotionSession(config,pose,['arm_roll'])
            other.prepare(target,.5,.32,zeros,zeros)
            self.assertIsNone(other.prediction)
            self.assertEqual(other.accepted_results,0)

    def test_runtime_requires_worker_ready_before_begin_and_contains_no_sync_id(self):
        with tempfile.TemporaryDirectory() as folder,redirect_stdout(io.StringIO()):
            config=self.config('shadow',Path(folder)/'log.jsonl')
            model=config['model'];pose=dict(zip(model.names,model.reference));zeros=dict.fromkeys(pose,0.)
            config.pop('worker')
            with self.assertRaisesRegex(RuntimeError,'before serial'):
                general.GeneralMotionSession(config,pose,['elbow'])
            config['worker']=DeterministicWorker(model.data)
            with (mock.patch.object(model,'inverse_dynamics',side_effect=AssertionError('parent performed inverse dynamics')),
                  mock.patch.object(model,'gravity_torques',side_effect=AssertionError('parent performed gravity dynamics'))):
                session=general.GeneralMotionSession(config,pose,['elbow'])
                session.prepare(pose,.5,0.,zeros,zeros)
                self.assertIsNotNone(session.prediction)

    def test_summary_omits_large_reference_arrays_and_needs_no_model(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'trace.jsonl'
            path.write_text(json.dumps({'build':general.BUILD,'mode':'shadow','frames':100,
                              'prediction_status':'available','reference_samples':['x'*50000]})+'\n{partial',encoding='utf8')
            output=io.StringIO()
            with (mock.patch('sys.argv',['tool','--summary','--last','2','--log',str(path)]),
                  mock.patch.object(general,'load_configuration',side_effect=AssertionError('summary loaded motion model')),
                  redirect_stdout(output)):
                general.main()
            lines=[json.loads(line) for line in output.getvalue().splitlines()]
            self.assertEqual(lines[0]['prediction_status'],'available')
            self.assertNotIn('reference_samples',lines[0])
            self.assertEqual(lines[1]['error'],'incomplete log line')
            self.assertLess(len(output.getvalue()),2000)

    def test_action_summary_only_uses_latest_matching_workflow(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'trace.jsonl'
            rows=[{'workflow_action':'white-bishop-placement','workflow_run_id':'old','frames':11},
                  {'workflow_action':'home','workflow_run_id':'home-run','frames':22},
                  {'workflow_action':'white-bishop-placement','workflow_run_id':'new','frames':33},
                  {'workflow_action':'white-bishop-placement','workflow_run_id':'new','frames':44}]
            path.write_text('\n'.join(json.dumps(r) for r in rows),encoding='utf8')
            out=io.StringIO()
            with (mock.patch('sys.argv',['tool','--summary','--action','white-bishop-placement','--last','4','--log',str(path)]),
                  redirect_stdout(out)):
                general.main()
            result=[json.loads(line) for line in out.getvalue().splitlines()]
            self.assertEqual([r['frames'] for r in result],[33,44])
            self.assertTrue(all(r['workflow_run_id']=='new' for r in result))
            out=io.StringIO()
            with (mock.patch('sys.argv',['tool','--summary','--action','not-recorded','--log',str(path)]),
                  redirect_stdout(out)):
                general.main()
            self.assertEqual(json.loads(out.getvalue())['summary_status'],'no-matching-motion-records')


if __name__=='__main__':unittest.main()
