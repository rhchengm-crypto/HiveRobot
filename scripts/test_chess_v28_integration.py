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
        self.assertIn('replay-move:bishop01',v28.CLAW_HOME_INTERRUPT_ACTIONS)
        self.assertIn(v28.PLACEMENT1_ACTION,v28.CLAW_HOME_INTERRUPT_ACTIONS)
        self.assertIn(v28.PLACEMENT_C4_ACTION,v28.CLAW_HOME_INTERRUPT_ACTIONS)
        self.assertIn(v28.WHITE_BISHOP_PLACEMENT_ACTION,v28.CLAW_HOME_INTERRUPT_ACTIONS)
        self.assertEqual(v28.PLACEMENT1_ACTION,'placement1:bishop01')
        self.assertEqual(v28.WHITE_BISHOP_PLACEMENT_ACTION,'white-bishop-placement')

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
                arm_page=get('/arm')
                self.assertIn('id="closeClawAfterReplay"',arm_page)
                self.assertIn('id="placement1AfterReplay"',arm_page)
                self.assertIn('id="placementC4AfterReplay"',arm_page)
                self.assertIn('Placement1：夹取后到 Clearance',arm_page)
                self.assertIn('id="whiteBishopPlacementAfterReplay"',arm_page)
                self.assertIn('White Bishop Placement',arm_page)
                self.assertIn("await runAction('claw-close', true)",arm_page)
                self.assertIn("Restore Pre-Placement1 Clearance Data", arm_page)
                self.assertIn("waitForRunIdle('replay-move:' + name",arm_page)
                self.assertNotIn("finished.returncode",arm_page)
                self.assertIn("onclick=\"runAction('home')\">Home Move</button>",arm_page)
                self.assertIn('run',json.loads(get('/api/state')))
                self.assertIn('rgb_seq',get('/api/status'))
                self.assertEqual(json.loads(get('/api/height/status'))['total'],0)
                with patch.object(v28.arm.subprocess,'Popen') as popen:
                    req=urllib.request.Request(base+'/api/action/claw-close',data=b'{}',method='POST')
                    with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(req)
                    self.assertEqual(error.exception.code,403)
                    error.exception.close()
                    placement_body=json.dumps({'name':'bishop01','placement1':True}).encode()
                    req=urllib.request.Request(base+'/api/move/replay',data=placement_body,method='POST',headers={'Content-Type':'application/json'})
                    with self.assertRaises(urllib.error.HTTPError) as error:
                        urllib.request.urlopen(req)
                    self.assertEqual(error.exception.code,403)
                    payload=json.loads(error.exception.read().decode())
                    error.exception.close()
                    self.assertEqual(payload['command'][-1],'--placement1')
                    c4_body=json.dumps({'name':'white_knight_c4','placement_c4':True}).encode()
                    req=urllib.request.Request(base+'/api/move/replay',data=c4_body,method='POST',headers={'Content-Type':'application/json'})
                    with self.assertRaises(urllib.error.HTTPError) as error:
                        urllib.request.urlopen(req)
                    self.assertEqual(error.exception.code,403)
                    payload=json.loads(error.exception.read().decode())
                    error.exception.close()
                    self.assertEqual(payload['command'][-1],'--placement-c4')
                    wrong_c4_body=json.dumps({'name':'bishop01','placement_c4':True}).encode()
                    req=urllib.request.Request(base+'/api/move/replay',data=wrong_c4_body,method='POST',headers={'Content-Type':'application/json'})
                    with self.assertRaises(urllib.error.HTTPError) as error:
                        urllib.request.urlopen(req)
                    self.assertEqual(error.exception.code,400)
                    error.exception.close()
                    bishop_body=json.dumps({'name':'bishop01','white_bishop_placement':True}).encode()
                    req=urllib.request.Request(base+'/api/move/replay',data=bishop_body,method='POST',headers={'Content-Type':'application/json'})
                    with self.assertRaises(urllib.error.HTTPError) as error:
                        urllib.request.urlopen(req)
                    self.assertEqual(error.exception.code,403)
                    payload=json.loads(error.exception.read().decode())
                    error.exception.close()
                    self.assertEqual(payload['command'][-1],'--white-bishop-placement')
                    popen.assert_not_called()
            finally:
                server.shutdown();server.server_close();thread.join()

    def test_claw_home_interrupts_placement1_before_starting(self):
        class RunState:
            def __init__(self):
                self.cancel_args=None
                self.started=None
            def cancel_current(self,**kwargs):
                self.cancel_args=kwargs
                return {'action':v28.PLACEMENT1_ACTION,'running':False}
            def start(self,action,cmd,popen_kwargs):
                self.started=(action,cmd)
                return {'ok':True,'running':True,'action':action,'command':cmd}

        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);args=v28.build_parser().parse_args([])
            stream=v28.arm.StreamState();camera=v28.SharedCamera(stream)
            state=v28.vision.VisionState(str(p/'cal.json'),str(p/'output'),camera,
                v28.vision.CameraConfig(True,args.rgb_topic,args.depth_topic,85),
                str(p/'dataset'),args.yolo_docker_image,str(p/'overlay.json'))
            cfg=v28.arm.ControlConfig(args.arm_script,args.move_script,args.python_bin,
                False,True,str(p/'runs.jsonl'),str(p/'moves.json'))
            run_state=RunState()
            handler=v28.make_handler(state,stream,run_state,cfg,args)
            server=ThreadingHTTPServer(('127.0.0.1',0),handler)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                req=urllib.request.Request(
                    'http://127.0.0.1:'+str(server.server_port)+'/api/action/claw-home',
                    data=b'{}',method='POST')
                with urllib.request.urlopen(req,timeout=5) as response:
                    self.assertTrue(json.loads(response.read())['ok'])
                self.assertEqual(run_state.cancel_args['reason'],'claw_home')
                self.assertEqual(run_state.cancel_args['expected_actions'],v28.CLAW_HOME_INTERRUPT_ACTIONS)
                self.assertEqual(run_state.started[0],'claw-home')
            finally:
                server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
