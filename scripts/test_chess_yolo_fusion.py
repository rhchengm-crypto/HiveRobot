import unittest
from chessboard_vision_v2_7_web_control import merge_yolo_inspection, height_render_detections


class FusionTests(unittest.TestCase):
    def test_render_uses_final_identity_not_overlapping_alternative(self):
        pawn={'piece_class':'white_pawn','confidence':.43657,'bbox_xyxy':[297.95,155.15,336.81,208.22]}
        bishop={'piece_class':'white_bishop','confidence':.39515,'bbox_xyxy':[298.94,153.79,336.87,208.14]}
        raw=[pawn,bishop]
        final=dict(pawn,square='d4',height={'ok':True,'estimated_height_mm':25.82})
        self.assertEqual(height_render_detections(raw,{'d4':final}),[final])
        corrected=dict(final,piece_class='white_bishop',identity_method='height_assisted_yolo')
        self.assertEqual(height_render_detections(raw,{'d4':corrected}),[corrected])
        self.assertEqual(height_render_detections(raw,{}),[])
        self.assertEqual(len(raw),2)

    def test_yolo_recovers_geometry_miss(self):
        result={'piece_results':{'d5':{'detected':False,'reason':'low_temporal_votes_full_board','temporal_votes':2,'temporal_samples':7}},'identified_pieces':{},'detected_squares':[],'detected_count':0}
        mapped={'d5':{'piece_class':'white_bishop','confidence':.841635,'center_px':[316.177,160.859],'center_mm':[193.562,231.538],'bbox_xyxy':[298.049,112.663,334.305,166.214]}}
        identities=merge_yolo_inspection(result,mapped)
        self.assertEqual(result['detected_squares'],['d5'])
        self.assertEqual(result['detected_count'],1)
        self.assertEqual(identities['d5']['piece_id'],'white_bishop')
        self.assertEqual(result['piece_results']['d5']['geometry_detection']['temporal_votes'],2)
        self.assertTrue(result['piece_results']['d5']['detected'])

    def test_no_yolo_preserves_unknown_geometry(self):
        result={'piece_results':{'a1':{'detected':True,'method':'depth'}},'identified_pieces':{'a1':{'piece_id':'unknown_piece'}}}
        merge_yolo_inspection(result,{})
        self.assertEqual(result['detected_squares'],['a1'])
        self.assertEqual(result['identified_pieces']['a1']['piece_id'],'unknown_piece')

    def test_both_empty(self):
        result={'piece_results':{'d5':{'detected':False}}}
        merge_yolo_inspection(result,{})
        self.assertEqual(result['detected_count'],0)
        self.assertEqual(result['identified_pieces'],{})


if __name__=='__main__': unittest.main()
