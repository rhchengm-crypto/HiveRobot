import copy
import unittest

from chessboard_vision_v2_7 import reconcile_yolo_projection_result, suppress_yolo_projection_artifacts, adjacent_square


class ProjectionTests(unittest.TestCase):
    def scene(self, owner='d5', ghost='d6'):
        pieces={owner:dict(detected=True,method='depth',temporal_samples=7,temporal_votes=7,median_raise_m=.0307,center_px=[317.65,141.96]),
                ghost:dict(detected=True,method='depth',confidence=1,temporal_samples=7,temporal_votes=1,median_raise_m=.0929,center_px=[314.76,118.09])}
        yolo={owner:dict(confidence=.858,bbox_xyxy=[295.63,102.82,336.08,165.58])}
        return pieces,yolo

    def test_recorded_d5_d6_and_consistent_counts(self):
        p,y=self.scene(); original=copy.deepcopy(p)
        result=dict(piece_results=p,detected_squares=['d5','d6'],detected_count=2,identified_pieces={'d5':{},'d6':{}})
        reconcile_yolo_projection_result(result,y)
        self.assertEqual(result['detected_squares'],['d5'])
        self.assertEqual(result['detected_count'],1)
        self.assertNotIn('d6',result['identified_pieces'])
        self.assertEqual(result['piece_results']['d6']['projection_owner_square'],'d5')
        self.assertEqual(p,original)

    def test_all_board_positions_and_neighbor_directions(self):
        squares=[f+r for f in 'abcdefgh' for r in '12345678']
        for owner in squares:
            for ghost in squares:
                if not adjacent_square(owner,ghost) or owner==ghost: continue
                p,y=self.scene(owner,ghost)
                with self.subTest(owner=owner,ghost=ghost):
                    self.assertFalse(suppress_yolo_projection_artifacts(p,y)[ghost]['detected'])

    def test_real_neighbors_and_insufficient_evidence_preserved(self):
        changes=[dict(temporal_votes=7),dict(center_px=[350,118]),dict(center_px=[315,160]),dict(median_raise_m=.035),dict(temporal_samples=1),dict(method='rgb')]
        for change in changes:
            p,y=self.scene();p['d6'].update(change)
            self.assertTrue(suppress_yolo_projection_artifacts(p,y)['d6']['detected'])
        p,y=self.scene();y['d6']=dict(confidence=.8,bbox_xyxy=[300,90,335,125])
        self.assertTrue(suppress_yolo_projection_artifacts(p,y)['d6']['detected'])
        p,y=self.scene();y['d5']['confidence']=.3
        self.assertTrue(suppress_yolo_projection_artifacts(p,y)['d6']['detected'])
        self.assertTrue(suppress_yolo_projection_artifacts(p,{})['d6']['detected'])

    def test_e5_king_three_of_seven_projection(self):
        p,y=self.scene('e5','e6')
        p['e5'].update(median_raise_m=.025649,center_px=[365.440,140.174])
        p['e6'].update(temporal_votes=3,median_raise_m=.087245,center_px=[368.958,117.106])
        y['e5'].update(confidence=.724127,bbox_xyxy=[342.926,98.418,383.656,164.579])
        self.assertFalse(suppress_yolo_projection_artifacts(p,y)['e6']['detected'])

    def test_vote_ratio_boundaries(self):
        for samples,votes,suppressed in [(5,2,True),(5,3,False),(7,3,True),(7,4,False),(9,4,True),(9,5,False),(6,3,False)]:
            p,y=self.scene()
            p['d6'].update(temporal_samples=samples,temporal_votes=votes)
            with self.subTest(samples=samples,votes=votes):
                self.assertEqual(not suppress_yolo_projection_artifacts(p,y)['d6']['detected'],suppressed)
        p,y=self.scene();p['d5']['temporal_votes']=3
        self.assertTrue(suppress_yolo_projection_artifacts(p,y)['d6']['detected'])


if __name__=='__main__': unittest.main()
