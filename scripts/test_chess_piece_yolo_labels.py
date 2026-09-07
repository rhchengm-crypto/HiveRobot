"""Regression checks for full-piece annotation and base-to-square mapping."""
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from chess_piece_yolo_dataset import add_labeled_image, square_yolo_box
from chess_piece_yolo_infer import map_detections_to_squares
from chess_piece_yolo_labels import load_sample, save_sample, samples, require_reviewed_labels, sample_paths, validate_rows


class FullPieceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.cal = self.root / "cal.json"
        self.cal.write_text(json.dumps({"homography_board_to_image": [[1,0,0],[0,-1,440],[0,0,1]]}))
        self.image = self.root / "source.jpg"
        cv2.imwrite(str(self.image), np.zeros((480,640,3), dtype=np.uint8))
        self.dataset = self.root / "dataset"
        add_labeled_image(self.image,self.cal,self.dataset,"train",{"d4":"white_queen"},"sample",.72)

    def test_drafts_extend_upward_and_require_review(self):
        sample = load_sample(self.dataset,"train/sample.jpg")
        old = square_yolo_box("d4", np.array([[1.,0,0],[0,-1,440],[0,0,1]]), (480,640,3), .72)
        _,cx,cy,w,h = sample["rows"][0]
        self.assertLess(cy-h/2,old[1]-old[3]/2)
        self.assertAlmostEqual(cy+h/2,old[1]+old[3]/2,places=5)
        with self.assertRaises(ValueError): require_reviewed_labels(self.dataset)

    def test_save_backups_cache_invalidation_and_stale_review(self):
        key = "train/sample.jpg"
        before = load_sample(self.dataset,key)
        label = self.dataset / "labels/train/sample.txt"
        original = label.read_bytes()
        cache = self.dataset / "labels/train.cache"
        cache.write_bytes(b"cached")
        saved = save_sample(self.dataset,key,[[4,.3,.4,.08,.13]],before["revision"])
        self.assertFalse(cache.exists())
        self.assertEqual(next((self.dataset/"label_backups/train").glob("*.txt")).read_bytes(),original)
        require_reviewed_labels(self.dataset)
        self.assertTrue(samples(self.dataset)[0]["reviewed"])
        with self.assertRaises(ValueError): save_sample(self.dataset,key,before["rows"],before["revision"])
        label.write_text("4 0.3 0.4 0.1 0.1\n")
        with self.assertRaises(ValueError): require_reviewed_labels(self.dataset)
        self.assertNotEqual(saved["revision"],load_sample(self.dataset,key)["revision"])

    def test_replaced_image_requires_review(self):
        s=load_sample(self.dataset,"train/sample.jpg")
        save_sample(self.dataset,s["key"],s["rows"],s["revision"])
        (self.dataset/"images/train/sample.jpg").write_bytes(self.image.read_bytes()+b"changed")
        with self.assertRaises(ValueError): require_reviewed_labels(self.dataset)

    def test_bad_paths_and_coordinates(self):
        for key in ("../source.jpg","train/../../source.jpg","train/..\\source.jpg"):
            with self.assertRaises(ValueError): sample_paths(self.dataset,key)
        for row in ([4,.5,.5,-.1,.1],[4,float('nan'),.5,.1,.1],[12,.5,.5,.1,.1],[4,.01,.5,.2,.1]):
            with self.assertRaises(ValueError): validate_rows([row])

    def test_full_box_base_maps_d5_not_d6(self):
        detection={"piece_class":"white_queen","confidence":.95,"bbox_xyxy":[178,135,208,207],"center_px":[193,171]}
        mapped=map_detections_to_squares([detection],str(self.cal),["d5","d6"])
        self.assertEqual(list(mapped),["d5"])
        self.assertEqual(mapped["d5"]["bbox_center_px"],[193,171])
        self.assertEqual(detection["center_px"],[193,171])
        self.assertEqual(map_detections_to_squares([detection],str(self.cal),["d4"]),{})

    def test_dedup_keeps_higher_confidence(self):
        first={"piece_class":"white_rook","confidence":.3,"bbox_xyxy":[178,135,208,207],"center_px":[193,171]}
        second=dict(first,piece_class="white_queen",confidence=.9)
        result=map_detections_to_squares([first,second],str(self.cal),["d5"])
        self.assertEqual(result["d5"]["piece_class"],"white_queen")


if __name__ == "__main__": unittest.main()
