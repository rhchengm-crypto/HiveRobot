import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

import left_arm_v3_1_model_control as control
import chessboard_vision_v3_0_web_control as web


class BoardHoldGainTests(unittest.TestCase):
    def test_terminal_command_survives_biased_feedback_and_entire_hold(self):
        command={'kp':90.,'kd':4.,'q':-.003,'dq':0.,'tau_ff':-3.2}
        sent=[];clock=[0.]
        arm=types.SimpleNamespace(
            ctrl=types.SimpleNamespace(controlMIT=lambda motor,*values:sent.append(values)),
            motors={'shoulder_front':'motor'},close=mock.Mock(),enable=mock.Mock(),
            read_status=lambda names:{n:{'pos':0.,'vel':-.0019536,'tau':-4.1 if n=='shoulder_front' else 0.}
                                      for n in names})
        def sleep(dt):clock[0]+=dt
        with tempfile.TemporaryDirectory() as directory:
            model=control.empty_model();model['gain_units_verified']=True
            pose=dict.fromkeys(control.JOINTS,0.)
            model['joints']['shoulder_front']['gravity_samples']=[{'pose_rad':pose,'tau':-4.}]*3
            model['joints']['shoulder_front']['inertia_samples']=[{'pose_rad':pose,'effective_inertia':.167}]
            path=Path(directory)/'model.json';control.save_model(path,model);before=path.read_bytes()
            args=control.build_parser().parse_args(['--model',str(path),'model-hold','--joints','shoulder_front',
                '--seconds','.1','--motors-already-enabled','--execute','--log',str(Path(directory)/'run.jsonl'),
                '--motion-gain-floor-json',json.dumps({'shoulder_front':command})])
            with (mock.patch.object(control,'open_arm') as opened,
                  mock.patch.object(control.time,'time',side_effect=lambda:clock[0]),
                  mock.patch.object(control.time,'sleep',side_effect=sleep),contextlib.redirect_stdout(io.StringIO())):
                result=control.model_hold(args,arm_context=(None,arm),prepared_model=model)
            opened.assert_not_called()
            self.assertTrue(result['ok']);self.assertEqual(path.read_bytes(),before)
        self.assertGreater(len(sent),2)
        for values in sent:self.assertEqual(values,tuple(command[k] for k in ('kp','kd','q','dq','tau_ff')))
        arm.enable.assert_not_called()
        arm.close.assert_not_called()

    def test_board_same_session_passes_existing_arm_and_refreshes_before_hold(self):
        import left_arm_board_grasp as board
        commands={n:{'kp':6.,'kd':.8,'q':0.,'dq':0.,'tau_ff':.1} for n in control.JOINTS}
        events=[]
        arm=types.SimpleNamespace(motors={n:n for n in control.JOINTS},
            ctrl=types.SimpleNamespace(controlMIT=lambda *values:events.append('refresh')),
            close=mock.Mock(),enable=mock.Mock())
        model={};rigid=object();legacy=object()
        def hold(args,**kwargs):
            self.assertEqual(len(events),7)
            self.assertEqual(kwargs['arm_context'],(legacy,arm))
            self.assertIs(kwargs['prepared_model'],model)
            self.assertIs(kwargs['fallback_model'],rigid)
            return {'ok':True}
        with mock.patch.object(control,'model_hold',side_effect=hold),contextlib.redirect_stdout(io.StringIO()):
            result=board.same_session_model_hold(arm,legacy,{'endpoint':{'model_hold_motion_gains':commands}},
                                                model,rigid,'unused.json')
        self.assertTrue(result['ok']);arm.close.assert_not_called();arm.enable.assert_not_called()

    def test_terminal_command_rejects_nonstationary_or_partial_payload(self):
        for extra in ({'q':0.},{'q':0.,'dq':.01,'tau_ff':0.},
                      {'q':0.,'dq':0.,'tau_ff':float('nan')}):
            with self.assertRaises(ValueError):
                control.motion_hold_gain_floor(json.dumps({'shoulder_front':{'kp':90.,'kd':4.,**extra}}),['shoulder_front'])

    def test_gain_validation_precedes_hardware(self):
        for row in ({'shoulder_front':{'kp':float('nan'),'kd':4.}},
                    {'shoulder_front':{'kp':90.,'kd':6.}},{}):
            with self.assertRaises(ValueError):
                control.motion_hold_gain_floor(json.dumps(row),['shoulder_front'])

    def test_reanchored_torque_law_matches_preceding_motion_when_gains_retained(self):
        kp,kd=90.,4.;q0=.619707;v0=-.0019536;old_target=.572183;old_ff=-3.024639
        measured=kp*(old_target-q0)-kd*v0+old_ff
        bridge=measured+kd*v0
        for q,v in ((q0,v0),(q0-.003,-.1),(q0+.003,.1)):
            self.assertAlmostEqual(kp*(q0-q)-kd*v+bridge,
                                   kp*(old_target-q)-kd*v+old_ff)

    def test_only_board_completion_supplies_the_recorded_gains(self):
        gains={n:{'kp':90. if n=='shoulder_front' else 22.,'kd':4. if n=='shoulder_front' else 1.4}
               for n in control.JOINTS}
        run={'action':'dynamics:board-grasp','stdout':'board endpoint report= '+json.dumps({'model_hold_motion_gains':gains})}
        self.assertEqual(web.board_hold_motion_gains(run),gains)
        self.assertIsNone(web.board_hold_motion_gains({**run,'action':'home'}))
        self.assertIsNone(web.board_hold_motion_gains({'action':'dynamics:board-grasp'}))

    def test_loaded_hold_retains_stiffness_after_a_load_change_without_moving_its_target(self):
        class Plant:
            now=0.;q=0.;v=0.;command=None
            motors={'shoulder_front':'shoulder_front'}
            def __init__(self):
                self.ctrl=types.SimpleNamespace(controlMIT=self.send)
                self.enable=mock.Mock();self.close=mock.Mock();self.history=[]
            def send(self,name,kp,kd,q,dq,tau):
                self.command=(kp,kd,q,dq,tau);self.history.append(self.command)
            def read_status(self,names):
                tau=-7.35 if self.command is None else self.output()
                return {n:{'pos':self.q if n=='shoulder_front' else 0.,
                           'vel':self.v if n=='shoulder_front' else 0.,
                           'tau':tau if n=='shoulder_front' else 0.} for n in names}
            def output(self):
                kp,kd,q,dq,tau=self.command
                return kp*(q-self.q)+kd*(dq-self.v)+tau
            def sleep(self,seconds):
                dt=seconds/max(1,math.ceil(seconds/.001))
                for _ in range(max(1,math.ceil(seconds/.001))):
                    if self.command:
                        load=-7.35 if self.now<.4 else -6.9
                        self.v+=(self.output()-load)/.167*dt
                        self.q+=self.v*dt
                    self.now+=dt
        plant=Plant()
        with tempfile.TemporaryDirectory() as directory:
            model=control.empty_model();model['gain_units_verified']=True
            pose=dict.fromkeys(control.JOINTS,0.)
            model['joints']['shoulder_front']['gravity_samples']=[{'pose_rad':pose,'tau':-4.}]*3
            model['joints']['shoulder_front']['inertia_samples']=[{'pose_rad':pose,'effective_inertia':.167}]
            path=Path(directory)/'model.json';control.save_model(path,model);before=path.read_bytes()
            args=control.build_parser().parse_args(['--model',str(path),'model-hold','--joints','shoulder_front',
                '--seconds','3','--motors-already-enabled','--execute','--log',str(Path(directory)/'run.jsonl'),
                '--motion-gain-floor-json',json.dumps({'shoulder_front':{'kp':90.,'kd':4.}})])
            with (mock.patch.object(control,'open_arm',return_value=(None,plant)),
                  mock.patch.object(control.time,'time',side_effect=lambda:plant.now),
                  mock.patch.object(control.time,'sleep',side_effect=plant.sleep),
                  contextlib.redirect_stdout(io.StringIO())):
                result=control.model_hold(args)
            self.assertEqual(path.read_bytes(),before)
        self.assertTrue(result['ok'])
        self.assertLess(result['max_abs_error_deg']['shoulder_front'],.5)
        self.assertLess(result['max_velocity_rad_s']['shoulder_front'],.1)
        for kp,kd,q,dq,tau in plant.history:
            self.assertGreaterEqual(kp,90.);self.assertGreaterEqual(kd,4.)
            self.assertEqual(q,0.);self.assertEqual(dq,0.)
        plant.enable.assert_not_called()


if __name__=='__main__':unittest.main()
