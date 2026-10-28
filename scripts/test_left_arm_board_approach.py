import unittest
import numpy as np
import left_arm_board_grasp as board
import chessboard_vision_v3_0_web_control as web
from pathlib import Path


class ModelApproachTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        here = Path(__file__).resolve().parent
        cls.model = board.RigidBodyModel.load(here/'data/left_arm_general_dynamics.json')
        cls.cfg = board.load_board_config(here/'data/left_arm_board_grasp.json', cls.model)
        cls.start = dict(zip(cls.model.names, [.7696269154548645, -2.853627920150757,
            2.342832088470459, -1.2834744453430176, -2.6575493812561035,
            .16308079659938812, -2.0708401203155518]))

    def test_home_to_xyz_uses_ik_and_checked_entry_then_vertical_descent(self):
        r = board.plan(self.model, self.cfg, self.start, 113.8, 190.3,
                       'white_knight', 'approach', 50.2)
        self.assertEqual(r['target_source'], 'requested-board-xyz')
        self.assertEqual(r['entry_route'], ['entry_clearance', 'source_above'])
        previous = np.array([self.start[n] for n in self.model.names])
        for row in r['waypoints']:
            q = np.array([row['encoder_rad'][n] for n in self.model.names])
            board.check_transit_edge(self.model, previous, q, self.cfg)
            if row['stage'] == 'entry_clearance':
                self.assertEqual(q[2], self.start['shoulder_rotate'])
            if row['stage'] == 'contact':
                pose = board.tcp_pose(self.model, q, self.cfg['tcp_in_wrist_frame_m'])
                xyz = (pose[:3, 3]-np.array(self.cfg['board_to_cad_translation_m'])) @ np.array(self.cfg['board_to_cad_rotation'])*1000
                np.testing.assert_allclose(xyz, row['target_board_mm'], atol=.5)
                np.testing.assert_allclose(row['target_board_mm'][:2], [113.8,190.3], atol=.5)
            previous=q
        self.assertEqual(r['waypoints'][-1]['stage'], 'contact')
        np.testing.assert_allclose(r['waypoints'][-1]['target_board_mm'], [113.8,190.3,50.2])
        other = board.plan(self.model,self.cfg,self.start,115.8,190.3,'white_knight','approach',50.2)
        self.assertNotEqual(r['waypoints'][-1]['encoder_rad'],other['waypoints'][-1]['encoder_rad'])
        self.assertFalse(r['motion_issued'])

    def test_feedback_entry_separates_xy_acquisition_from_vertical_descent(self):
        # Actual hover from the run with observed 15mm drift, not its nominal IK pose.
        measured = dict(zip(self.model.names, [.6589990258216858,-2.831883668899536,
            2.481689214706421,.7104982137680054,-2.7296483516693115,
            .17643244564533234,-3.2389180660247803]))
        rows, audit=board.feedback_contact_path(self.model,self.cfg,measured,[113.8,190.3,50.2])
        self.assertEqual(audit['board_offset_applied_mm'],[0.,0.,0.])
        z=audit['start_tcp_board_mm'][2]
        previous=np.array([measured[n] for n in self.model.names])
        for row in rows:
            q=np.array([row['encoder_rad'][n] for n in self.model.names])
            pose=board.tcp_pose(self.model,q,self.cfg['tcp_in_wrist_frame_m'])
            self.assertLess(np.linalg.norm(board.rotation_error(
                np.array(audit['wrist_rotation_in_cad']),pose[:3,:3])),.005)
            board.check_transit_edge(self.model,previous,q,self.cfg)
            if row['stage']=='entry_xy_alignment':
                self.assertAlmostEqual(row['target_board_mm'][2],z)
            else:
                np.testing.assert_allclose(row['target_board_mm'][:2],[113.8,190.3],atol=1e-8)
                self.assertLess(row['target_board_mm'][2],z)
            previous=q
        np.testing.assert_allclose(rows[-1]['target_board_mm'],[113.8,190.3,50.2])
        self.assertIn('entry_xy_alignment',{r['stage'] for r in rows})

    def test_executor_rebuilds_contact_from_feedback_and_keeps_same_controller(self):
        import types, io
        from contextlib import redirect_stdout
        from unittest import mock
        measured=dict(zip(self.model.names,[.6589990258216858,-2.831883668899536,
            2.481689214706421,.7104982137680054,-2.7296483516693115,
            .17643244564533234,-3.2389180660247803]))
        route=board.plan(self.model,self.cfg,self.start,113.8,190.3,'white_knight','approach',50.2)
        status={n:dict(pos=self.start[n],vel=0.,tau=0.) for n in self.model.names}
        arm=mock.Mock();arm.motors={n:object() for n in self.model.names}
        arm.read_status.side_effect=lambda names:{n:dict(status[n]) for n in names}
        calls=[]
        def move(target,**kwargs):
            calls.append(dict(target))
            for n in target: status[n]['pos']=target[n]
            if len(calls)==2:
                for n in measured: status[n]['pos']=measured[n]
        arm.move_targets_with_holds.side_effect=move
        legacy=types.SimpleNamespace(HOME_GAINS={n:dict(kp=22.,kd=1.4) for n in self.model.names})
        with redirect_stdout(io.StringIO()) as output:
            result=board.execute(arm,legacy,route,geometry=(self.model,self.cfg))
        log=output.getvalue()
        self.assertIn('board feedback descent entry=',log)
        self.assertIn('entry_xy_alignment',log)
        self.assertTrue(result['ok'])
        self.assertGreater(len(calls),14)
        self.assertIsNotNone(arm._board_fault_support)
        arm.close.assert_not_called();arm.disable.assert_not_called()
        for call in arm.move_targets_with_holds.call_args_list:

            for n in self.model.names:
                if n!='shoulder_rotate':self.assertEqual(call.kwargs['move_gains'][n],legacy.HOME_GAINS[n])
            self.assertGreaterEqual(call.kwargs['move_gains']['shoulder_rotate']['kp'],22.)
            self.assertLessEqual(call.kwargs['move_gains']['shoulder_rotate']['kp'],36.)
        q=np.array([calls[-1][n] for n in self.model.names])
        pose=board.tcp_pose(self.model,q,self.cfg['tcp_in_wrist_frame_m'])
        xyz=(pose[:3,3]-np.array(self.cfg['board_to_cad_translation_m']))@np.array(self.cfg['board_to_cad_rotation'])*1000
        np.testing.assert_allclose(xyz,[113.8,190.3,50.2],atol=.5)

    def test_direction_probe_solves_xyz_without_transit_or_saved_target(self):
        q=np.array([.693331778049469,-2.8284504413604736,2.487029790878296,
            .4530022144317627,-2.7449073791503906,.11692225188016891,-2.951667070388794])
        start=dict(zip(self.model.names,q.tolist()))
        f=board.tcp_pose(self.model,q,self.cfg['tcp_in_wrist_frame_m'])
        xyz=(f[:3,3]-np.array(self.cfg['board_to_cad_translation_m']))@np.array(self.cfg['board_to_cad_rotation'])*1000
        for delta in ([0,0,3],[3,0,0]):
            goal=xyz+delta
            r=board.plan(self.model,self.cfg,start,*goal[:2],'white_knight','cartesian-check',goal[2])
            self.assertEqual(r['entry_route'],[])
            np.testing.assert_allclose(r['requested_delta_board_mm'],delta,atol=1e-9)
            self.assertFalse(r['physical_direction_verified'])
            self.assertEqual(len(r['waypoints']),int(np.ceil(np.linalg.norm(delta))))
            end=np.array([r['waypoints'][-1]['encoder_rad'][n] for n in self.model.names])
            f2=board.tcp_pose(self.model,end,self.cfg['tcp_in_wrist_frame_m'])
            actual=(f2[:3,3]-np.array(self.cfg['board_to_cad_translation_m']))@np.array(self.cfg['board_to_cad_rotation'])*1000
            np.testing.assert_allclose(actual,goal,atol=.1)
        with self.assertRaisesRegex(ValueError,'0.5..5mm'):
            board.plan(self.model,self.cfg,start,xyz[0]+6,xyz[1],'white_knight','cartesian-check',xyz[2])

    def test_swivel_stiffness_uses_geometry_and_retains_damping_and_other_gains(self):
        import types,copy
        legacy=types.SimpleNamespace(HOME_GAINS={n:dict(kp=24.,kd=4.5) for n in self.model.names},
            NUDGE_GAINS={'shoulder_rotate':dict(kp=60.,kd=3.)})
        before=copy.deepcopy(legacy.HOME_GAINS)
        extended=dict(zip(self.model.names,[.693331778049469,-2.8284504413604736,
            2.487029790878296,.4530022144317627,-2.7449073791503906,.11692225188016891,-2.951667070388794]))
        gain,a=board.board_motion_gains(legacy,(self.model,self.cfg),extended)
        self.assertGreater(gain['shoulder_rotate']['kp'],24.)
        self.assertLessEqual(gain['shoulder_rotate']['kp'],36.)
        self.assertEqual(gain['shoulder_rotate']['kd'],4.5)
        self.assertEqual(a['target_offset_rad'],0.)
        self.assertEqual(legacy.HOME_GAINS,before)
        for n in self.model.names:
            if n!='shoulder_rotate':self.assertEqual(gain[n],before[n])
        base,_=board.board_motion_gains(legacy,None,extended)
        self.assertEqual(base,before)


    def test_web_passes_xyz_and_model_mode_without_angle_targets(self):
        args=web.build_parser().parse_args([])
        args.rigid_body_model='data/left_arm_general_dynamics.json'
        args.rigid_body_mode='apply'
        command=web.build_dynamics_command(args,'board-grasp',dict(x_mm=113.8,y_mm=190.3,
            grip_height_mm=50.2,piece='white_knight',stage='approach',execute=True))
        for flag,value in [('--x-mm','113.8'),('--y-mm','190.3'),('--grip-height-mm','50.2'),('--mode','apply'),('--stage','approach')]:
            self.assertEqual(command[command.index(flag)+1],value)
        self.assertIn('--execute',command)
        self.assertNotIn('--targets-json',command)
        self.assertIn('approach',board.SUPPORTED_EXECUTION_STAGES)

    def test_touch_still_requires_hover_and_grasp_remains_unreleased(self):
        with self.assertRaises(board.BoardPlanningError):
            board.plan(self.model,self.cfg,self.start,113.8,190.3,'white_knight','touch',50.2)
        self.assertNotIn('grasp',board.SUPPORTED_EXECUTION_STAGES)


if __name__ == '__main__':
    unittest.main()
