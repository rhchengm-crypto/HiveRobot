import unittest
import tempfile
import json
from pathlib import Path
import numpy as np
from chess_crown_geometry_v2_8 import (GeometryStore,fit_transform,coordinate_plan,
    validate_profile,validate_placement_anchor,validate_grasp_anchor,crown_point,PAGE)

class GeometryTests(unittest.TestCase):
    def samples(self):
        return [{'split':split,'board_mm':p,'arm_mm':(np.array(p)+[100,200,300]).tolist()}
            for split,p in [('train',[0,0,0]),('train',[440,0,0]),('train',[0,440,0]),
                            ('check',[220,220,0]),('check',[440,440,0])]]
    def test_transform_and_crown_plan(self):
        cal=fit_transform(self.samples())
        profile=validate_profile(dict(piece_class='white_bishop',piece_height_mm=60,
            grip_height_mm=50,open_width_mm=30,closed_width_mm=15))
        plan=coordinate_plan(cal,profile,[192.5,192.5],'c1')
        np.testing.assert_allclose(plan['waypoints'][1]['tcp_arm_mm'],[292.5,392.5,350])
        np.testing.assert_allclose(plan['waypoints'][4]['tcp_arm_mm'],[237.5,227.5,350])
        self.assertFalse(plan['executable'])
    def test_bad_calibration_rejected(self):
        s=self.samples();s[-1]['arm_mm'][0]+=20
        with self.assertRaises(ValueError):fit_transform(s)
        s=self.samples();s[-1]=dict(s[0],split='check')
        with self.assertRaises(ValueError):fit_transform(s)
    def test_crown_ray_uses_height_plane(self):
        p=crown_point([0,0,600],[.1,.2,-1],np.eye(4),60)
        np.testing.assert_allclose(p,[54,108,60])
        with self.assertRaises(ValueError):crown_point([0,0,600],[1,0,0],np.eye(4),60)

    def test_validated_placement_anchor_is_stored_without_removing_existing_data(self):
        joints=('shoulder_front','shoulder_side','shoulder_rotate','elbow',
                'arm_roll','wrist_side','wrist')
        record=dict(piece_class='white_bishop',target_square='C1',
            saved_move_name='white_bishop_place',pose_rad={name:.1 for name in joints},
            board_tcp_mm=[137.5,27.5,55],tolerance_deg=.5,
            validation_errors_deg={name:.4 for name in joints})
        checked=validate_placement_anchor(record)
        self.assertTrue(checked['validated'])
        self.assertEqual(checked['target_square'],'c1')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'geometry.json'
            path.write_text(json.dumps({'schema':1,'profiles':{'keep':{}},
                'calibration':None}),encoding='utf-8')
            result=GeometryStore(path).action('placement-anchor',record)
            self.assertIn('keep',result['data']['profiles'])
            self.assertEqual(result['data']['placement_anchors']['white_bishop:c1'],checked)

    def test_d4_grasp_anchor_keeps_pose_and_metric_validation_separate(self):
        joints=('shoulder_front','shoulder_side','shoulder_rotate','elbow',
                'arm_roll','wrist_side','wrist')
        record=dict(piece_class='white_bishop',source_square='D4',
            saved_move_name='bishop01',pose_rad={name:.1 for name in joints},
            board_tcp_mm=[192.5,192.5,55],grip_section_width_mm=14,
            tolerance_deg=.5,validation_errors_deg={name:.4 for name in joints},
            board_correspondence_status='user_confirmed_nominal_square_center')
        with tempfile.TemporaryDirectory() as directory:
            store=GeometryStore(Path(directory)/'geometry.json')
            data=store.action('grasp-anchor',record)['data']
            anchor=data['grasp_anchors']['white_bishop:d4']
            self.assertTrue(anchor['pose_validated'])
            self.assertFalse(anchor['metric_transform_validated'])
            self.assertFalse(anchor['contact_confirmed'])
            self.assertEqual(anchor['saved_move_name'],'bishop01')

    def test_partial_c4_grasp_anchor_preserves_unknown_height_and_width(self):
        joints=('shoulder_front','shoulder_side','shoulder_rotate','elbow',
                'arm_roll','wrist_side','wrist')
        record=dict(piece_class='white_knight',source_square='c4',
            saved_move_name='white_knight_c4',pose_rad={name:.1 for name in joints},
            board_xy_mm=[137.5,192.5],board_tcp_mm=None,
            tolerance_deg=.5,validation_errors_deg={name:.4 for name in joints},
            pose_source='reconstructed_from_final_encoder_and_errors')
        with tempfile.TemporaryDirectory() as directory:
            store=GeometryStore(Path(directory)/'geometry.json')
            anchor=store.action('grasp-anchor',record)['data']['grasp_anchors']['white_knight:c4']
            self.assertTrue(anchor['pose_validated'])
            self.assertIsNone(anchor['board_tcp_mm'])
            self.assertIsNone(anchor['grip_section_width_mm'])
            self.assertEqual(anchor['board_xy_mm'],[137.5,192.5])

    def test_import_preview_merge_and_backup_preserve_existing_data(self):
        source=Path(__file__).resolve().parent/'data'/'chess_crown_geometry_v2_8.json'
        bundle=json.loads(source.read_text(encoding='utf-8'))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'geometry.json'
            existing={'schema':1,'profiles':{'trained':{'keep':True}},
                'calibration':{'keep':True},'placement_anchors':{},'grasp_anchors':{}}
            path.write_text(json.dumps(existing),encoding='utf-8')
            original=path.read_bytes()
            store=GeometryStore(path)
            preview=store.action('import-anchors',{'geometry':bundle,'dry_run':True})
            self.assertEqual(preview['preview']['counts']['new'],4)
            self.assertEqual(preview['preview']['counts']['conflicts'],0)
            self.assertEqual(path.read_bytes(),original)
            result=store.action('import-anchors',{'geometry':bundle,
                'expected_revision':preview['preview']['current_revision']})
            self.assertTrue(Path(result['backup_path']).exists())
            self.assertEqual(Path(result['backup_path']).read_bytes(),original)
            saved=store.read()
            self.assertEqual(saved['profiles'],existing['profiles'])
            self.assertEqual(saved['calibration'],existing['calibration'])
            self.assertTrue(saved['grasp_anchors']['white_bishop:d4']['contact_confirmed'])
            self.assertTrue(saved['grasp_anchors']['white_knight:c4']['contact_confirmed'])
            self.assertIn('grasp_confirmation',saved['grasp_anchors']['white_bishop:d4'])
            self.assertIn('workflow_validation',saved['placement_anchors']['white_bishop:c1'])
            self.assertIn('white_knight:b1',saved['placement_anchors'])
            self.assertFalse(result['motion_enabled'])

    def test_import_conflict_and_stale_preview_do_not_write(self):
        source=Path(__file__).resolve().parent/'data'/'chess_crown_geometry_v2_8.json'
        bundle=json.loads(source.read_text(encoding='utf-8'))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'geometry.json'
            store=GeometryStore(path)
            preview=store.action('import-anchors',{'geometry':bundle,'dry_run':True})
            store.action('profile',dict(piece_class='white_bishop',piece_height_mm=60,
                grip_height_mm=50,open_width_mm=30,closed_width_mm=15))
            changed=path.read_bytes()
            with self.assertRaisesRegex(ValueError,'重新预览'):
                store.action('import-anchors',{'geometry':bundle,
                    'expected_revision':preview['preview']['current_revision']})
            self.assertEqual(path.read_bytes(),changed)
            existing=store.read()
            existing['grasp_anchors']['white_bishop:d4']=bundle['grasp_anchors']['white_bishop:d4'].copy()
            existing['grasp_anchors']['white_bishop:d4']['pose_rad']=dict(
                existing['grasp_anchors']['white_bishop:d4']['pose_rad'],wrist=0)
            path.write_text(json.dumps(existing),encoding='utf-8')
            preview=store.action('import-anchors',{'geometry':bundle,'dry_run':True})
            self.assertEqual(preview['preview']['counts']['conflicts'],1)
            with self.assertRaisesRegex(ValueError,'姿态冲突'):
                store.action('import-anchors',{'geometry':bundle,
                    'expected_revision':preview['preview']['current_revision']})
            self.assertEqual(store.read()['grasp_anchors']['white_bishop:d4']['pose_rad']['wrist'],0)

    def test_import_merges_contact_evidence_into_matching_pose(self):
        source=Path(__file__).resolve().parent/'data'/'chess_crown_geometry_v2_8.json'
        bundle=json.loads(source.read_text(encoding='utf-8'))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'geometry.json'
            old=bundle['grasp_anchors']['white_bishop:d4'].copy()
            old['contact_confirmed']=False
            old.pop('grasp_confirmation')
            old['runtime_note']='keep this robot note'
            initial={'schema':1,'profiles':{},'calibration':None,
                'placement_anchors':{},'grasp_anchors':{'white_bishop:d4':old}}
            path.write_text(json.dumps(initial),encoding='utf-8')
            store=GeometryStore(path)
            preview=store.action('import-anchors',{'geometry':bundle,'dry_run':True})
            self.assertIn('grasp_anchors:white_bishop:d4',preview['preview']['changes']['updated'])
            store.action('import-anchors',{'geometry':bundle,
                'expected_revision':preview['preview']['current_revision']})
            saved=store.read()['grasp_anchors']['white_bishop:d4']
            self.assertTrue(saved['contact_confirmed'])
            self.assertEqual(saved['runtime_note'],'keep this robot note')
            self.assertEqual(saved['grasp_confirmation'],bundle['grasp_anchors']['white_bishop:d4']['grasp_confirmation'])

    def test_import_page_has_preview_and_write_controls(self):
        self.assertIn('预览锚点',PAGE)
        self.assertIn('/api/crown/import-anchors',PAGE)
        self.assertIn('写入已预览锚点',PAGE)

if __name__=='__main__':unittest.main()
