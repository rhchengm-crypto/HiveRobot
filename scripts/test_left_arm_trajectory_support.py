import copy
import json
from pathlib import Path
import types
import unittest

import numpy as np

import left_arm_board_grasp as board
import left_arm_trajectory_support as support
import left_arm_general_motion as general


class PlannedSupportTests(unittest.TestCase):
    def setUp(self):
        self.model=types.SimpleNamespace(names=['shoulder_front'],n=1,signs=np.ones(1),
            encoder_to_model=lambda q:np.asarray(q),gravity_torques=lambda q:np.array([float(q[0])]))
        self.path={'start_rad':{'shoulder_front':0.},'waypoints':[
            {'encoder_rad':{'shoulder_front':-10.582434822969931}}]}
        self.envelope=support.gravity_envelope(self.model,self.path)

    def anchor(self):
        tau=2.1401710510253906
        status={'shoulder_front':{'pos':0.,'vel':0.,'tau':tau}}
        gains={'shoulder_front':{'kp':90.,'kd':4.}}
        anchor=board.TrajectorySupportBaseline(self.model,{'shoulder_front':0.},
            {'shoulder_front':tau},status,gains,{'shoulder_front':[-4.8,3.8]},
            {'shoulder_front':28.},self.envelope,self.path)
        return anchor,status,gains

    def test_recorded_a7_gravity_is_not_discarded_and_later_pd_is_not_learned(self):
        anchor,status,gains=self.anchor()
        position=self.path['waypoints'][-1]['encoder_rad']
        for measured in (-10.56,-15.,-20.):
            status['shoulder_front']['tau']=measured
            command,audit=anchor.command(position,status,gains)
            self.assertAlmostEqual(command['shoulder_front']['end_tau'],-8.44226377194454)
            self.assertFalse(audit['shoulder_front']['clamped'])
            self.assertEqual(anchor.model_limits['shoulder_front'],(-4.8,3.8))
            self.assertAlmostEqual(anchor.limits['shoulder_front'][0],-8.94226377194454)

    def test_budget_does_not_expand_from_unplanned_pose(self):
        anchor,status,gains=self.anchor()
        command,audit=anchor.command({'shoulder_front':-25.},status,gains)
        self.assertAlmostEqual(command['shoulder_front']['end_tau'],-8.94226377194454)
        self.assertTrue(audit['shoulder_front']['clamped'])

    def test_changed_path_rejects_cached_envelope(self):
        path=copy.deepcopy(self.path);path['waypoints'][-1]['encoder_rad']['shoulder_front']=-20.
        with self.assertRaisesRegex(ValueError,'differs'):
            support.support_limits(self.envelope,path,{'shoulder_front':2.},
                {'shoulder_front':0.},{'shoulder_front':[-4.8,3.8]},{'shoulder_front':28.})

    def test_protocol_range_remains_checked_not_silently_expanded(self):
        with self.assertRaisesRegex(ValueError,'exceeds MIT range'):
            support.support_limits(self.envelope,self.path,{'shoulder_front':-25.},
                {'shoulder_front':0.},{'shoulder_front':[-4.8,3.8]},{'shoulder_front':28.})

    def test_adapter_keeps_the_planned_budget_and_existing_slew(self):
        anchor,_,_=self.anchor()
        session=general.GeneralMotionSession.__new__(general.GeneralMotionSession)
        session.config={'mode':'apply','board_tau_limits':anchor.limits}
        session.board_gravity=True;session.dt=.01
        for field in ('corrections','peak','goal','gravity_applied','gravity_peak'):
            setattr(session,field,{'shoulder_front':0.})
        session.gravity_goal={'shoulder_front':-3.64226377194454}
        values=[]
        for _ in range(300):values.append(session.transform('shoulder_front',90.,4.,-4.8)[2])
        self.assertAlmostEqual(values[-1],-8.44226377194454)
        self.assertLessEqual(np.max(np.abs(np.diff([-4.8,*values]))),.015+1e-12)
        session.gravity_goal['shoulder_front']=-25.
        for _ in range(1000):last=session.transform('shoulder_front',90.,4.,-4.8)
        self.assertEqual(last[:2],(90.,4.))
        self.assertAlmostEqual(last[2],anchor.limits['shoulder_front'][0])

    def test_recorded_low_feedback_detected_during_motion_without_full_scene_search(self):
        here=Path(__file__).parent
        model=board.RigidBodyModel.load(here/'data/left_arm_general_dynamics.json')
        cfg=json.loads((here/'data/left_arm_board_workspace_candidate.json').read_text())
        q=[-.04596780240535736,-2.736133337020874,1.876668930053711,-.6872282028198242,
           -2.268444299697876,-.4980163276195526,-2.3409247398376465]
        feedback={n:{'pos':v} for n,v in zip(model.names,q)}
        result=support.gripper_feedback_clearance(model,cfg,feedback)
        self.assertAlmostEqual(result['lowest_vertex_height_mm'],2.133088032207578,places=6)
        self.assertLess(result['clearance_mm'],0.)
        self.assertEqual(result['minimum_height_mm'],5.)


if __name__=='__main__':unittest.main()
