import tempfile
import unittest
from pathlib import Path

import numpy as np

from inspect_left_arm_step import StepInspection, decode_name, references


SAMPLE = """
ISO-10303-21;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#2=CARTESIAN_POINT('',(3.,4.,5.));
#3=DIRECTION('',(0.,0.,-1.));
#4=DIRECTION('',(1.,0.,0.));
#5=AXIS2_PLACEMENT_3D('',#2,#3,#4);
#6=CYLINDRICAL_SURFACE('',#5,28.5);
#7=VERTEX_POINT('',#2);
#8=TEST_GEOMETRIC_REFERENCES(#6,#7);
#9=ADVANCED_BREP_SHAPE_REPRESENTATION('part',(#8),#1);
#10=SHAPE_REPRESENTATION('Wrist motor.step',(),#1);
#11=SHAPE_REPRESENTATION_RELATIONSHIP('','',#10,#9);
#12=PRODUCT('quoted; name with #9999','', '',());
#15=CARTESIAN_POINT('',(0.,0.,0.));
#16=AXIS2_PLACEMENT_3D('',#15,#3,#4);
#20=CARTESIAN_POINT('',(10.,20.,30.));
#21=AXIS2_PLACEMENT_3D('',#20,#3,#4);
#22=ITEM_DEFINED_TRANSFORMATION('','',#16,#21);
#23=(REPRESENTATION_RELATIONSHIP('','',#10,#24)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#22)SHAPE_REPRESENTATION_RELATIONSHIP());
#24=SHAPE_REPRESENTATION('root',(),#1);
ENDSEC;
END-ISO-10303-21;
"""


class StepInspectionTests(unittest.TestCase):
    def test_quoted_step_strings_do_not_create_references_or_break_entities(self):
        self.assertEqual(references("PRODUCT('not #88',#3)"), [3])
        self.assertEqual(decode_name('RB\\X2\\624B81C2\\X0\\'), 'RB手臂')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.step'
            path.write_text(SAMPLE, encoding='utf-8')
            inspection = StepInspection(path)
            self.assertIn("quoted; name", inspection.entities[12])
            self.assertEqual(inspection.graph[12], [])

    def test_assembly_transform_is_applied_to_candidate_axis_and_vertices(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.step'
            path.write_text(SAMPLE, encoding='utf-8')
            inspection = StepInspection(path)
            report, clouds = inspection.inspect()
            axis = report['motor_axis_candidates'][0]['motor_axis_candidate']
            np.testing.assert_allclose(axis['direction'], [0, 0, 1])
            np.testing.assert_allclose(axis['point_nearest_cad_origin_mm'], [13, 24, 0])
            np.testing.assert_allclose(clouds[0]['vertices_mm'][0], [13, 24, 35])
            self.assertFalse(report['deployment_ready'])
            self.assertIn('unverified', axis['status'])

    def test_different_length_units_are_rejected_instead_of_silently_mis_scaled(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.step'
            path.write_text(SAMPLE.replace('.MILLI.,.METRE.', '$,.METRE.'), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'millimetre'):
                StepInspection(path)


if __name__ == '__main__':
    unittest.main()
