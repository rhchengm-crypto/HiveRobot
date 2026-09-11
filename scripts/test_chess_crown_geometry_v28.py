import unittest
import numpy as np
from chess_crown_geometry_v2_8 import fit_transform,coordinate_plan,validate_profile,crown_point

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

if __name__=='__main__':unittest.main()
