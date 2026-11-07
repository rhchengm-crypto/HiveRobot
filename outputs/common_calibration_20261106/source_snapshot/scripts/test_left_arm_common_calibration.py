import copy
import unittest
from audit_left_arm_common_calibration import add_proxies,without_pose_neighbors


class GroupedCalibrationTests(unittest.TestCase):
    def test_heldout_pose_removes_both_label_types_and_preserves_evidence(self):
        pose={'a':0.,'b':0.}
        near={'pose_rad':{'a':.01,'b':.02}}
        far={'pose_rad':{'a':.5,'b':0.}}
        history={'joints':{'a':{'gravity_samples':[near,far],
                               'inertia_samples':[near,far]}}}
        frozen=copy.deepcopy(history)
        split,removed=without_pose_neighbors(history,pose)
        self.assertEqual(history,frozen)
        self.assertEqual(split['joints']['a']['gravity_samples'],[far])
        self.assertEqual(split['joints']['a']['inertia_samples'],[far])
        self.assertEqual(removed['a'],{'gravity_samples':1,'inertia_samples':1})

    def test_dense_trace_is_one_pose_vote_and_never_a_new_inertia(self):
        old={'pose_rad':{'a':0.},'effective_inertia':.1}
        history={'joints':{'a':{'gravity_samples':[], 'inertia_samples':[old]}}}
        row={'id':'B3','encoder_pose_rad':{'a':.1},'measured_torque_Nm':{'a':2.},
             'stable_hold':{'complete_frames':1000}}
        out=add_proxies(history,[row])
        self.assertEqual(len(out['joints']['a']['gravity_samples']),1)
        self.assertEqual(out['joints']['a']['inertia_samples'],[old])
        self.assertEqual(history['joints']['a']['gravity_samples'],[])
        self.assertTrue(out['joints']['a']['gravity_samples'][0]['uncalibrated_torque_proxy'])


if __name__=='__main__':unittest.main()
