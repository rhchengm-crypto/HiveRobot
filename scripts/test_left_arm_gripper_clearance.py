import copy
import json
from pathlib import Path
import unittest
from unittest.mock import Mock

import numpy as np

import left_arm_board_grasp as board
from left_arm_board_collision import BodyCollisionScene, collision_scene


class GripperClearanceTests(unittest.TestCase):
    def setUp(self):
        here=Path(__file__).parent
        self.model=board.RigidBodyModel.load(here/'data/left_arm_general_dynamics.json')
        self.cfg=json.loads((here/'data/left_arm_board_workspace_candidate.json').read_text())
        self.cfg['gripper_vertex_envelope']=json.loads((here/'data/left_arm_gripper_vertex_envelope.json').read_text())

    def test_envelope_gradient_matches_independent_perturbations(self):
        scene=BodyCollisionScene(self.model,self.cfg)
        rng=np.random.default_rng(84)
        for _ in range(8):
            q=self.model.reference+rng.uniform(-.5,.5,self.model.n)
            measured=np.array([(scene(q+np.eye(self.model.n)[j]*1e-6)[-1]
                              -scene(q-np.eye(self.model.n)[j]*1e-6)[-1])/2e-6
                              for j in range(self.model.n)])
            np.testing.assert_allclose(scene.jacobian(q)[-1],measured,atol=1e-7,rtol=1e-5)

    def test_travel_floor_changes_scene_without_mutating_contact_config(self):
        q=self.model.reference
        baseline=copy.deepcopy(self.cfg)
        contact=collision_scene(self.model,self.cfg)
        travel=collision_scene(self.model,board.transit_clearance_config(self.cfg))
        self.assertAlmostEqual(contact(q)[-1]-travel(q)[-1],.015)
        self.assertIsNot(contact,travel)
        self.assertEqual(self.cfg,baseline)
        self.assertAlmostEqual(collision_scene(self.model,self.cfg)(q)[-1],contact(q)[-1])

    def test_bad_mesh_and_lower_travel_floor_are_rejected(self):
        cfg=copy.deepcopy(self.cfg)
        cfg['gripper_vertex_envelope']['vertices_in_wrist_frame_m'][0][0]=float('nan')
        with self.assertRaises(ValueError):BodyCollisionScene(self.model,cfg)
        cfg=copy.deepcopy(self.cfg)
        cfg['gripper_vertex_envelope']['travel_minimum_height_mm']=1.
        with self.assertRaises(ValueError):board.transit_clearance_config(cfg)

    def test_envelope_cannot_be_silently_disabled_with_body_scene(self):
        cfg=copy.deepcopy(self.cfg)
        cfg['body_collision']['enabled']=False
        with self.assertRaises(ValueError):collision_scene(self.model,cfg)

    def test_prepared_low_route_is_rejected_before_any_motor_access(self):
        q=np.array([.8297784648205575,-2.551795269433715,2.1665751384840135,
                    .16974764060480024,-2.3263475346379723,-.19315373627836913,-1.8898811027599114])
        start=np.array([1.0489183057024332,-2.7990654057995497,2.046153336261129,
                    .40875356762647336,-2.622131083332203,-.1971617543596269,-1.9451147102689876])
        trajectory=dict(stage='approach',start_rad=dict(zip(self.model.names,start)),
            validation_trial={'approach_contact':True},
            waypoints=[dict(stage='source_above',encoder_rad=dict(zip(self.model.names,q)))])
        arm,legacy=Mock(),Mock()
        with self.assertRaises(board.BoardPlanningError) as failure:
            board.execute(arm,legacy,trajectory,geometry=(self.model,self.cfg),allow_body_validation_trial=True)
        self.assertEqual(failure.exception.diagnostics['code'],'planned_gripper_clearance_failed')
        self.assertEqual(arm.mock_calls,[])
        self.assertEqual(legacy.mock_calls,[])

    def test_recorded_a7_centerline_pass_does_not_hide_fingers_below_plane(self):
        # Lowest planned A7 approach sample from the rejected 2026-11-07 run.
        q=np.array([.8297784648205575,-2.551795269433715,2.1665751384840135,
                    .16974764060480024,-2.3263475346379723,-.19315373627836913,-1.8898811027599114])
        old=copy.deepcopy(self.cfg);old.pop('gripper_vertex_envelope')
        board.check_board_intersection(self.model,q,old)
        with self.assertRaises(board.BoardPlanningError) as failure:
            board.check_board_intersection(self.model,q,self.cfg)
        self.assertEqual(failure.exception.diagnostics['code'],'gripper_surface_clearance')
        self.assertAlmostEqual(failure.exception.diagnostics['worst_pair']['lowest_vertex_height_mm'],-10.975,places=2)

    def test_accepted_a6_lowest_approach_pose_meets_common_travel_floor(self):
        q=np.array([1.0489183057024332,-2.7990654057995497,2.046153336261129,
                    .40875356762647336,-2.622131083332203,-.1971617543596269,-1.9451147102689876])
        cfg=board.transit_clearance_config(self.cfg)
        board.check_board_intersection(self.model,q,cfg)
        self.assertGreater(BodyCollisionScene(self.model,cfg)(q)[-1],0.)


if __name__=='__main__':unittest.main()
