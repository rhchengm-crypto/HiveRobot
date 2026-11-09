import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

import build_left_arm_path_reference_library as library
import left_arm_board_task_pose as task


class ReferencePieceScopeTests(unittest.TestCase):
    def test_generic_piece_types_cover_both_colors_and_exclusions_win(self):
        reference = dict(applicable_pieces=['pawn', 'knight'], excluded_pieces=['pawn'],
                         grasp_verified=False)
        for piece in ('pawn', 'white_pawn', 'black_pawn'):
            self.assertFalse(task.reference_applies_to_piece(reference, piece))
        for piece in ('knight', 'white_knight', 'black_knight'):
            self.assertTrue(task.reference_applies_to_piece(reference, piece))
        self.assertFalse(task.reference_applies_to_piece(reference, 'white_rook'))
        self.assertTrue(task.reference_applies_to_piece({}, 'white_pawn'))
        self.assertTrue(task.reference_applies_to_piece({}, 'legacy_unknown_profile'))
        self.assertFalse(task.reference_applies_to_piece({'applicable_pieces': []}, 'white_knight'))
        colored = {'applicable_pieces': ['white_knight']}
        self.assertTrue(task.reference_applies_to_piece(colored, 'white_knight'))
        self.assertFalse(task.reference_applies_to_piece(colored, 'black_knight'))

    def test_malformed_scope_is_rejected_instead_of_used_without_restrictions(self):
        for reference in ({'excluded_pieces': 'pawn'}, {'applicable_pieces': [None]},
                          {'grasp_verified': 'false'}):
            with self.assertRaises(ValueError):
                task.reference_applies_to_piece(reference, 'white_knight')

    def test_pawn_ranking_skips_nearest_restricted_reference_and_audit_cannot_use_it(self):
        restricted = dict(name='non-pawn-a8', reference_kind='validated_path',
                          reference_board_xy_mm=[1., 2.], excluded_pieces=['pawn'],
                          grasp_verified=False)
        # No seed on the excluded reference: filtering must precede FK/ranking.
        allowed = dict(name='legacy', reference_kind='validated_path',
                       reference_board_xy_mm=[5., 6.], seed_encoder_rad={'q': 0.},
                       wrist_rotation_in_board=np.eye(3).tolist())
        cfg = dict(approved_task_pose_references=[restricted, allowed],
                   validated_path_reference_policy='primary-then-nearest-validated-path',
                   profiles={'white_pawn': {}}, tcp_in_wrist_frame_m=[0., 0., 0.],
                   board_to_cad_rotation=np.eye(3).tolist(), board_to_cad_translation_m=[0., 0., 0.])
        original = copy.deepcopy(cfg)
        planner = SimpleNamespace(
            validate_request=lambda *args: (np.array([1., 2., 3.]), {}),
            check_planning_budget=lambda cfg: None,
            check_planned_tool_orientation=lambda *args: {'passed': True})
        with mock.patch.object(task, 'tcp_pose', return_value=np.eye(4)), \
             mock.patch.object(task, 'plan_continuous_approach', return_value={}) as solve:
            result = task.plan_from_approved_references(
                SimpleNamespace(names=['q']), cfg, {}, 1., 2., 'white_pawn', 3., planner)
        self.assertEqual(result['approved_task_pose_reference'], 'legacy')
        self.assertEqual(result['reference_selection']['excluded_by_piece_scope'], 1)
        self.assertEqual(solve.call_args.args[1]['profiles']['white_pawn']['seed_encoder_rad'], {'q': 0.})
        self.assertEqual(cfg, original)
        with self.assertRaisesRegex(ValueError, 'not applicable'):
            task.approved_profile(cfg, 'white_pawn', 'non-pawn-a8')

    def test_all_excluded_references_leave_primary_common_planning_available(self):
        cfg = dict(profiles={'black_pawn': {}}, approved_task_pose_references=[
            dict(name='non-pawn', excluded_pieces=['pawn'], grasp_verified=False)])
        planner = SimpleNamespace(check_planned_tool_orientation=lambda *args: {'passed': True})
        with mock.patch.object(task, 'plan_continuous_approach', return_value={}) as solve:
            result = task.plan_from_approved_references(
                None, cfg, {}, 1., 2., 'black_pawn', 3., planner)
        self.assertEqual(solve.call_args.args[1], cfg)
        self.assertNotIn('approved_task_pose_reference', result)

    def test_builder_preserves_scope_in_evidence_and_runtime_seed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)

            def write(relative, value):
                path = root/relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(value), encoding='utf-8')
                return path

            model = write('scripts/data/left_arm_general_dynamics.json', {})
            active = dict(source_step_sha256='step', rigid_model_sha256=library.sha(model),
                          tcp_in_wrist_frame_m=[0., 0., 0.],
                          board_to_cad_rotation=np.eye(3).tolist(),
                          board_to_cad_translation_m=[0., 0., 0.])
            active_path = write('active.json', active)
            frozen = write('outputs/run/left_arm_board_workspace_candidate.json', active)
            log = root/'outputs/run/arm_run_1.txt'
            log.write_text('\n'.join([
                'board same-session hold complete={}', 'board endpoint report={}',
                'board plan={"target_board_mm":[1,2,3]}',
                'board waypoint={"entry_feedback":{"q":{"pos":0.1}}}',
            ]), encoding='utf-8')
            scope = dict(applicable_pieces=['king', 'queen', 'bishop', 'knight', 'rook'],
                         excluded_pieces=['pawn'], grasp_verified=False)
            write('scripts/data/board_workspace_trials/validation_test.json', dict(
                experiment_id='scope-test', request=dict(x_mm=1., y_mm=2., grip_height_mm=3.),
                evidence=dict(directory='outputs/run', control_log=log.name),
                encoder_endpoint_rad={'q': .1},
                model_snapshots={'left_arm_board_workspace_candidate.json': library.sha(frozen)},
                operator_validation=dict(accepted=True, **scope)))
            write('outputs/common_calibration_20261106/common_calibration_audit.json', {'trials': []})
            with mock.patch.object(library.RigidBodyModel, 'load', return_value=SimpleNamespace(names=['q'])), \
                 mock.patch.object(library, 'tcp_pose', return_value=np.eye(4)):
                built, candidate = library.build(root, active_path)
            self.assertEqual(built['eligible_reference_count'], 1)
            for record in (built['records'][0], candidate['approved_task_pose_references'][0]):
                self.assertEqual(task.reference_piece_scope(record), scope)
            self.assertFalse(candidate['approved_task_pose_references'][0]['grasp_verified'])

    def test_builder_accepts_top_level_scope_and_operator_qualification_overrides_it(self):
        record = dict(excluded_pieces=['pawn'], grasp_verified=False)
        self.assertEqual(library.registry_piece_scope(record), record)
        record['operator_validation'] = dict(excluded_pieces=['white_pawn'])
        self.assertEqual(library.registry_piece_scope(record),
                         dict(excluded_pieces=['white_pawn'], grasp_verified=False))


if __name__ == '__main__':
    unittest.main()
