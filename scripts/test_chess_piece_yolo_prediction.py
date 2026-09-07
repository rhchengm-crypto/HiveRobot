"""Prediction rendering uses actual model coordinates without changing input."""
import tempfile
import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib.request import urlopen
from urllib.error import HTTPError

import cv2
import numpy as np

from chess_piece_yolo_infer import save_prediction_image
from chessboard_vision_v2_7_web_control import make_handler


class PredictionTests(unittest.TestCase):
    def test_grid_and_prediction_on_same_image(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'combined.jpg'
            cv2.imwrite(str(source),np.zeros((480,640,3),dtype=np.uint8))
            original=source.read_bytes()
            calibration=Path(directory)/'cal.json'
            calibration.write_text(json.dumps({'homography_board_to_image':[[1,0,0],[0,1,0],[0,0,1]]}))
            target=save_prediction_image(str(source),[{'piece_class':'white_king','confidence':.8579,'bbox_xyxy':[100,120,145,185]}],str(calibration))
            frame=cv2.imread(target)
            # Yellow grid at x=55; green model box at x=100, away from grid.
            self.assertGreater(int(frame[30,55,1]),150)
            self.assertGreater(int(frame[30,55,2]),150)
            self.assertGreater(int(frame[150,100,1]),180)
            self.assertLess(int(frame[150,100,2]),100)
            self.assertEqual(source.read_bytes(),original)

    def test_render_input_preserved_and_serve(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)/"yolo_detect"
            root.mkdir()
            source=root/"detect_example.jpg"
            cv2.imwrite(str(source),np.zeros((200,300,3),dtype=np.uint8))
            original=source.read_bytes()
            target=Path(save_prediction_image(str(source),[{"piece_class":"white_queen","confidence":.607,"bbox_xyxy":[100,70,150,130]}]))
            self.assertEqual(source.read_bytes(),original)
            frame=cv2.imread(str(target))
            self.assertGreater(int(frame[100,100,1]),180)
            self.assertLess(int(frame[100,90,1]),40)
            server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(SimpleNamespace(output_dir=directory,yolo_dataset_dir=directory)))
            thread=threading.Thread(target=server.serve_forever,daemon=True)
            thread.start()
            try:
                base=f'http://127.0.0.1:{server.server_port}/yolo-prediction.jpg?name='
                with urlopen(base+target.name) as response:
                    self.assertEqual(response.read(),target.read_bytes())
                    self.assertEqual(response.headers['Content-Type'],'image/jpeg')
                with self.assertRaises(HTTPError) as error: urlopen(base+'../outside_pred.jpg')
                self.assertEqual(error.exception.code,404)
                error.exception.close()
            finally:
                server.shutdown();server.server_close();thread.join()

    def test_empty_detections_get_fresh_image(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'empty.jpg'
            cv2.imwrite(str(source),np.zeros((100,400,3),dtype=np.uint8))
            target=save_prediction_image(str(source),[])
            self.assertGreater(cv2.imread(target).sum(),0)


if __name__=='__main__': unittest.main()
