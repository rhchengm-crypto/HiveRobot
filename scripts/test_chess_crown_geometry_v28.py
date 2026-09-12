import unittest
import tempfile
import json
from pathlib import Path
import numpy as np
from chess_crown_geometry_v2_8 import (GeometryStore,fit_transform,coordinate_plan,
    validate_profile,validate_placement_anchor,crown_point)

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

if __name__=='__main__':unittest.main()
