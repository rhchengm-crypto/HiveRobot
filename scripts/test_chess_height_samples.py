import copy
import json
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from chessboard_vision_v2_7 import CHESS_PIECE_YOLO_CLASSES
from chessboard_vision_v2_7_web_control import LiveCameraState
from chess_piece_height_samples import save_height_sample, height_sample_stats, validate_pair


class HeightSampleTests(unittest.TestCase):
    def capture(self):
        camera=LiveCameraState()
        now=time.time()
        camera.set_rgb(np.zeros((48,64,3),np.uint8),{'source_stamp_s':now,'encoding':'bgr8'})
        camera.set_depth(np.full((24,32),650,np.uint16),{'source_stamp_s':now+.02,'encoding':'16UC1'})
        return camera.height_snapshot()

    def payload(self,c='white_bishop'):
        return dict(piece_class=c,square='d5',measured_height_mm=65,orientation='back',split='train')

    def test_all_classes_and_raw_depth_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            for cls in CHESS_PIECE_YOLO_CLASSES:
                capture=self.capture()
                result=save_height_sample(directory,capture,self.payload(cls),{'homography_board_to_image':np.eye(3).tolist()})
                folder=Path(result['sample_dir'])
                self.assertTrue((folder/'rgb.png').exists())
                np.testing.assert_array_equal(np.load(folder/'depth.npy',allow_pickle=False),capture['depth'])
                meta=json.loads((folder/'sample.json').read_text())
                self.assertEqual(meta['height_label_source'],'ruler_manual')
                self.assertIsNone(meta['estimated_height_mm'])
                self.assertEqual(meta['quality']['depth_scale_to_m'],.001)
                with self.assertRaises(ValueError):save_height_sample(directory,capture,self.payload(cls),{'homography_board_to_image':np.eye(3).tolist()})
            stats=height_sample_stats(directory)
            self.assertEqual(stats['total'],12)
            self.assertTrue(all(item['train']==1 for item in stats['classes'].values()))

    def test_bad_pairs_rejected(self):
        for mode in ['missing','stale','skew','timestamp','invalid_depth','encoding']:
            c=self.capture()
            if mode=='missing':c['depth']=None
            if mode=='stale':c['rgb_received_at']-=5
            if mode=='skew':c['depth_metadata']['source_stamp_s']+=1
            if mode=='timestamp':c['rgb_metadata']={}
            if mode=='invalid_depth':c['depth'][:]=0
            if mode=='encoding':c['depth_metadata']['encoding']='mono16'
            with self.subTest(mode=mode),self.assertRaises(ValueError):validate_pair(c)

    def test_float_depth_and_input_validation(self):
        c=self.capture();c['depth']=np.full((24,32),.65,np.float32);c['depth_metadata']['encoding']='32FC1'
        self.assertEqual(validate_pair(c)['depth_scale_to_m'],1)
        with tempfile.TemporaryDirectory() as directory:
            for changes in [dict(measured_height_mm=0),dict(measured_height_mm=float('nan')),dict(piece_class='../bad'),dict(square='z9'),dict(split='oops')]:
                p=self.payload();p.update(changes)
                with self.assertRaises((ValueError,KeyError)):save_height_sample(directory,c,p,{'homography_board_to_image':np.eye(3).tolist()})
            self.assertEqual(height_sample_stats(directory)['total'],0)


if __name__=='__main__':unittest.main()
