import sys,json,math,types,unittest
from pathlib import Path
sys.path.insert(0,'scripts')
import numpy as np
import left_arm_board_grasp as b
import chessboard_vision_v3_0_web_control as w
class ProbeTests(unittest.TestCase):
 def setUp(self):
  self.m=b.RigidBodyModel.load('scripts/data/left_arm_general_dynamics.json');self.c=b.load_board_config('scripts/data/left_arm_board_grasp.json',self.m)
  self.q=json.loads(Path('outputs/left_arm_general_model/v17_full_test_report.json').read_text())['endpoint']['measured_pose_rad']
 def test_only_requested_axis_changes_and_no_saved_pose(self):
  p=b.plan_direction_check(self.m,self.c,self.q,'shoulder_rotate',1.)
  for n in self.m.names:self.assertAlmostEqual(p['waypoints'][0]['encoder_rad'][n]-self.q[n],math.radians(1) if n=='shoulder_rotate' else 0)
  self.assertGreater(p['direction_check']['predicted_tcp_delta_board_mm'][0],6)
  self.assertEqual(p['target_source'],'isolated-axis-calibration');self.assertFalse(p['motion_issued'])
 def test_invalid_step_rejected_before_motion(self):
  for n in [0,3.01,float('nan')]:
   with self.assertRaises(ValueError):b.plan_direction_check(self.m,self.c,self.q,'shoulder_rotate',n)
 def test_wider_step_only_at_raised_pose(self):
  from left_arm_board_kinematics import tcp_pose
  with self.assertRaisesRegex(ValueError, '100 mm'):
   b.plan_direction_check(self.m,self.c,self.q,'shoulder_rotate',3.)
  entry=json.loads(Path('outputs/left_arm_general_model/v17_full_test_report.json').read_text())['entry']['start_measured_rad']
  p=b.plan_direction_check(self.m,self.c,entry,'shoulder_rotate',3.)
  self.assertEqual(p['direction_check']['encoder_step_deg'],3.)
 def test_api_only_accepts_small_diagnostic_step(self):
  a=types.SimpleNamespace(python_bin='python3',rigid_body_model='a',dynamics_model='b',rigid_body_mode='apply')
  p=dict(x_mm=113.8,y_mm=190.3,stage='cartesian-check',direction_check_joint='shoulder_rotate',direction_check_step_deg=1.)
  self.assertIn('--direction-check-joint',w.build_dynamics_command(a,'board-grasp',p))
  p['direction_check_step_deg']=3.01
  with self.assertRaises(ValueError):w.build_dynamics_command(a,'board-grasp',p)
unittest.main()
