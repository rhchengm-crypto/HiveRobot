"""Transient load-error rejection, bounds, and no duplicate terminal trim."""
import unittest
import contextlib
import io
import itertools
import types
from unittest import mock
import numpy as np
import left_arm_board_grasp as board
from left_arm_board_grasp import TouchTrackingTrim
from left_arm_v3_1_model_control import JOINTS


class TouchTrackingTests(unittest.TestCase):
    def test_segment_restart_and_hold_do_not_add_the_trim_twice(self):
        names=list(JOINTS);commands={n:(90.,4.,0.,0.,.2) for n in names}
        preload_errors=[]
        class Arm:
            motors={n:n for n in names}
            def __init__(self):self.ctrl=types.SimpleNamespace(controlMIT=self.send)
            def send(self,n,*values):commands[n]=values
            def read_status(self,requested):
                return {n:{'pos':0.,'vel':0.,'tau':commands[n][0]*commands[n][2]+commands[n][4]} for n in requested}
            def enable(self,requested):pass
            def move_targets_with_holds(self,target,**kwargs):
                before=self.read_status(names)
                for n in names:
                    self.ctrl.controlMIT(n,90.,4.,0.,0.,kwargs['move_tau_ff'][n]['start_tau'])
                after=self.read_status(names)
                preload_errors.append(max(abs(after[n]['tau']-before[n]['tau']) for n in names))
                for _ in range(20):
                    for n in names:
                        self.ctrl.controlMIT(n,90.,4.,target[n],0.,kwargs['move_tau_ff'][n]['end_tau'])
                    kwargs['watchdog'](target)
        arm=Arm();legacy=types.SimpleNamespace(HOME_GAINS={n:{'kp':90.,'kd':4.} for n in names})
        rows=[{'stage':'contact','encoder_rad':{n:(q if n==names[0] else 0.) for n in names},
               'target_board_mm':[100.,100.,50.]} for q in (.001,.002)]
        clock=itertools.count(0.,.04)
        with (mock.patch('left_arm_board_home_handoff.install_home_handoff'),
              mock.patch.object(board.time,'monotonic',side_effect=lambda:next(clock)),
              contextlib.redirect_stdout(io.StringIO())):
            result=board.execute(arm,legacy,{'stage':'touch','waypoints':rows})
        self.assertLess(max(preload_errors),1e-10)
        self.assertGreater(result['endpoint']['touch_tracking_trim']['peak_tau_Nm'][names[0]],0.)
        for n,row in result['endpoint']['model_hold_motion_gains'].items():
            arm.ctrl.controlMIT(n,row['kp'],row['kd'],row['q'],row['dq'],row['tau_ff'])
            self.assertEqual(commands[n],tuple(row[k] for k in ('kp','kd','q','dq','tau_ff')))

    def test_unknown_load_bias_no_longer_leaves_different_axis_errors(self):
        gains={'front':{'kp':90.,'kd':4.},'side':{'kp':70.,'kd':3.}}
        def simulate(trim_enabled):
            trim=TouchTrackingTrim(gains);q=np.zeros(2);v=np.zeros(2)
            kp=np.array([90.,70.]);kd=np.array([4.,3.]);mass=np.array([.167,.417])
            load=np.array([.3,-.25]);ref=np.zeros(2)
            for i in range(6000):
                # Feedback arrives at 25Hz; integrate at that rate, not at the
                # frequency of serial sends. The motor's local PD runs faster.
                if i%40==0:
                    trim.update(dict(zip(gains,ref)),{n:{'pos':q[j]} for j,n in enumerate(gains)},i*.001)
                correction=np.array(list(trim.correction.values())) if trim_enabled else 0.
                acceleration=(kp*(ref-q)-kd*v+correction-load)/mass
                v+=acceleration*.001;q+=v*.001
            return np.linalg.norm(q),trim
        original,_=simulate(False);corrected,trim=simulate(True)
        self.assertLess(corrected,original*.1)
        self.assertLessEqual(max(trim.peak.values()),.5)

    def test_long_feedback_gap_cannot_cause_a_large_integral_jump(self):
        trim=TouchTrackingTrim({'joint':{'kp':90.,'kd':4.}})
        trim.update({'joint':1.},{'joint':{'pos':0.}},0.)
        trim.update({'joint':1.},{'joint':{'pos':0.}},10.)
        self.assertLessEqual(trim.correction['joint'],.1)
        for i in range(20):trim.update({'joint':1.},{'joint':{'pos':0.}},11+i)
        self.assertEqual(trim.correction['joint'],.5)
        trim.update({'joint':-1.},{'joint':{'pos':0.}},32.)
        self.assertLess(trim.correction['joint'],.5)

    def test_fresh_feedback_required_and_nonfinite_feedback_rejected(self):
        trim=TouchTrackingTrim({'joint':{'kp':90.,'kd':4.}})
        trim.update({'joint':1.},{'joint':{'pos':0.}},0.)
        trim.update({'joint':1.},{'joint':{'pos':0.}},0.)
        self.assertEqual(trim.correction['joint'],0.)
        with self.assertRaises(ValueError):
            trim.update({'joint':1.},{'joint':{'pos':float('nan')}},.1)


if __name__=='__main__':unittest.main()
