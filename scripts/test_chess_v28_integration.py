import json
import tempfile
import threading
import unittest
import urllib.request
import urllib.error
from pathlib import Path
from http.server import ThreadingHTTPServer
from unittest.mock import patch
import numpy as np
import chessboard_vision_v2_8_web_control as v28


class IntegrationTests(unittest.TestCase):
    def test_defaults_preserve_data_and_execution_gate(self):
        args=v28.build_parser().parse_args([])
        self.assertEqual(args.yolo_dataset_dir,v28.vision.DEFAULT_YOLO_DATASET_DIR)
        self.assertEqual(args.moves_file,v28.arm.DEFAULT_MOVES_FILE)
        self.assertEqual(args.calibration,v28.vision.DEFAULT_CALIBRATION_PATH)
        self.assertEqual(args.overlay_config,v28.vision.DEFAULT_WEB_OVERLAY_CONFIG_PATH)
        self.assertFalse(args.enable_execute)
        self.assertEqual(args.port,8098)
        self.assertTrue(args.move_script.endswith('left_arm_v2_8_move_library.py'))
        self.assertTrue(args.arm_script.endswith('left_arm_v2_8.py'))

    def test_shared_frames_preserve_metadata(self):
        stream=v28.arm.StreamState(); camera=v28.SharedCamera(stream)
        rgb=np.zeros((8,8,3),np.uint8); depth=np.full((8,8),600,np.uint16)
        camera.set_rgb(rgb,{'source_stamp_s':123})
        camera.set_depth(depth,{'source_stamp_s':123})
        self.assertEqual(camera.height_snapshot()['depth_metadata']['source_stamp_s'],123)
        np.testing.assert_array_equal(stream.snapshot()[1],depth)

    def test_http_pages_routes_and_disabled_arm(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);args=v28.build_parser().parse_args([])
            stream=v28.arm.StreamState();camera=v28.SharedCamera(stream)
            state=v28.vision.VisionState(str(p/'cal.json'),str(p/'output'),camera,
                v28.vision.CameraConfig(True,args.rgb_topic,args.depth_topic,85),
                str(p/'dataset'),args.yolo_docker_image,str(p/'overlay.json'))
            cfg=v28.arm.ControlConfig(args.arm_script,args.move_script,args.python_bin,
                False,False,str(p/'runs.jsonl'),str(p/'moves.json'))
            handler=v28.make_handler(state,stream,v28.arm.RunState(cfg.run_log),cfg,args)
            server=ThreadingHTTPServer(('127.0.0.1',0),handler)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            base='http://127.0.0.1:'+str(server.server_port)
            def get(path):
                with urllib.request.urlopen(base+path,timeout=5) as r:return r.read().decode()
            try:
                for path in ['/','/vision','/arm','/height-setup','/yolo-labels']:
                    self.assertIn('<html',get(path).lower())
                self.assertIn('run',json.loads(get('/api/state')))
                self.assertIn('rgb_seq',get('/api/status'))
                self.assertEqual(json.loads(get('/api/height/status'))['total'],0)
                with patch.object(v28.arm.subprocess,'Popen') as popen:
                    req=urllib.request.Request(base+'/api/action/claw-close',data=b'{}',method='POST')
                    with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(req)
                    self.assertEqual(error.exception.code,403)
                    error.exception.close()
                    popen.assert_not_called()
            finally:
                server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
