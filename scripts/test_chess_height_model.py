import json
import tempfile
import time
import unittest
from pathlib import Path
import numpy as np
from chess_piece_height_model import fit_board, estimate_heights, build_model, assist, rays, calibration_path, analyze_live


def scene(tilt=0.):
    info={'width':640,'height':480,'K':[500,0,320,0,500,240,0,0,1],'D':[],'distortion_model':'plumb_bob'}
    ray=rays((480,640),info)
    n=np.array([0,np.sin(tilt),-np.cos(tilt)]);d=.7*np.cos(tilt)
    depth=(-d/(ray@n)).astype(np.float32)
    origin=np.array([-.22,.22*np.cos(tilt),.7+.22*np.sin(tilt)])
    x=np.array([.44,0,0]);y=np.array([0,-.44*np.cos(tilt),-.44*np.sin(tilt)])
    xyz=np.array([origin,origin+x,origin+x+y,origin+y])
    corners=xyz[:,:2]/xyz[:,2,None]*500+[320,240]
    cal=fit_board(depth,'32FC1',info,corners)
    def piece(height):
        # Synthetic flat-top object at d5, with correct metric camera geometry.
        center=origin+x*(3.5/8)+y*(4.5/8)+n*height
        u,v=np.rint(center[:2]/center[2]*500+[320,240]).astype(int)
        image=depth.copy();top=((height-d)/(ray@n)).astype(np.float32)
        image[v-8:v+9,u-8:u+9]=top[v-8:v+9,u-8:u+9]
        return image
    return info,depth,cal,piece


class MetricHeightTests(unittest.TestCase):
    def test_metric_height_with_tilt_and_invalid_pixels(self):
        for tilt in (0.,.25):
            info,empty,cal,piece=scene(tilt)
            depth=piece(.065);depth[10:15,10:15]=np.nan
            obs=estimate_heights(depth,'32FC1',cal,['d5','e5'])
            self.assertTrue(obs['d5']['ok'])
            self.assertAlmostEqual(obs['d5']['estimated_height_mm'],65,delta=.2)
            self.assertFalse(obs['e5']['ok'])
            self.assertFalse(estimate_heights(empty,'32FC1',cal,['d5'])['d5']['ok'])

    def test_bad_calibration_and_drift_rejected(self):
        info,empty,cal,piece=scene()
        with self.assertRaises(ValueError):estimate_heights(empty+.05,'32FC1',cal,['d5'])
        with self.assertRaises(ValueError):fit_board(empty,'32FC1',dict(info,K=[0]*9),cal['depth_corners'])
        with self.assertRaises(ValueError):fit_board(empty,'32FC1',info,[[0,0]]*4)

    def test_model_build_val_gate_test_is_holdout_and_class_correction(self):
        info,empty,cal,piece=scene()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);calibration_path(root).write_text(json.dumps(cal))
            for cls,height in [('white_pawn',.04),('white_bishop',.065)]:
                for i,split in enumerate(['train','train','val','test']):
                    folder=root/'samples'/f'height_{cls}_{i}';folder.mkdir(parents=True)
                    np.save(folder/'depth.npy',piece(.09 if split=='test' else height))
                    meta={'sample_id':folder.name,'piece_class':cls,'square':'d5','split':split,'metric_calibration_id':cal['id'],
                          'depth_metadata':{'encoding':'32FC1'},'measured_height_mm':height*1000}
                    (folder/'sample.json').write_text(json.dumps(meta))
            model=build_model(root)
            self.assertTrue(model['profiles']['white_bishop']['validated'])
            self.assertAlmostEqual(model['profiles']['white_bishop']['median_mm'],65,delta=.2)
            self.assertEqual(len(model['test_results']),2)
            mapped={'d5':{'piece_class':'white_pawn','confidence':.84,'bbox_xyxy':[1,2,3,4]}}
            obs=estimate_heights(piece(.065),'32FC1',cal,['d5'])
            corrected=assist(mapped,obs,model)['d5']
            self.assertEqual(corrected['piece_class'],'white_bishop')
            self.assertEqual(corrected['rgb_piece_class'],'white_pawn')
            self.assertEqual(mapped['d5']['piece_class'],'white_pawn')
            now=time.time()
            capture={'depth':piece(.065),'captured_at':now,'rgb_received_at':now,'depth_received_at':now,
                     'rgb_metadata':{'source_stamp_s':now},'depth_metadata':{'encoding':'32FC1','source_stamp_s':now},'camera_info':{'depth':info}}
            self.assertEqual(analyze_live(root,capture,mapped)['d5']['piece_class'],'white_bishop')
            capture['depth_metadata']['source_stamp_s']-=1
            self.assertEqual(analyze_live(root,capture,mapped)['d5']['piece_class'],'white_pawn')
            model['profiles']['white_bishop']['validated']=False
            self.assertEqual(assist(mapped,obs,model)['d5']['piece_class'],'white_pawn')

    def test_ambiguity_missing_model_and_bad_depth_leave_rgb_unchanged(self):
        mapped={'d5':{'piece_class':'white_pawn','confidence':.9}}
        obs={'d5':{'ok':True,'estimated_height_mm':65,'uncertainty_mm':2}}
        p={'median_mm':65,'tolerance_mm':5,'validated':True}
        model={'profiles':{'white_pawn':dict(p,median_mm=40),'white_bishop':p,'white_queen':p}}
        self.assertEqual(assist(mapped,obs,model)['d5']['piece_class'],'white_pawn')
        with tempfile.TemporaryDirectory() as directory:
            self.assertFalse(analyze_live(directory,{},mapped)['d5']['height']['ok'])

    def test_rgb_overlay_inverts_scale_and_offset(self):
        from chess_piece_height_web import depth_corners_from_overlay
        info,empty,cal,piece=scene()
        original=np.asarray(cal['depth_corners'])
        transform=np.array([.9,.8,12.,9.])
        data={'corner_space':'rgb','alignment_confirmed':True,
              'depth_to_rgb':transform.tolist(),'corners':(original*transform[:2]+transform[2:]).tolist()}
        capture={'rgb':np.zeros((480,640,3),np.uint8)}
        recovered=depth_corners_from_overlay(data,capture)
        np.testing.assert_allclose(recovered,original)
        result=fit_board(empty,'32FC1',info,recovered)
        np.testing.assert_allclose(result['edge_lengths_mm'],[440]*4,atol=.1)
        data['alignment_confirmed']=False
        with self.assertRaises(ValueError):depth_corners_from_overlay(data,capture)
        data['alignment_confirmed']=True;data['depth_to_rgb'][0]=0
        with self.assertRaises(ValueError):depth_corners_from_overlay(data,capture)

    def test_report_reads_utf8(self):
        from types import SimpleNamespace
        from chess_piece_height_web import handle_height_action
        from chess_piece_height_samples import height_dataset_dir
        with tempfile.TemporaryDirectory() as directory:
            state=SimpleNamespace(yolo_dataset_dir=directory+'/yolo')
            root=height_dataset_dir(state.yolo_dataset_dir);root.mkdir()
            (root/'height_model.json').write_text(json.dumps({'note':'已验证'},ensure_ascii=False),encoding='utf-8')
            self.assertEqual(handle_height_action(state,'report',{})['model']['note'],'已验证')


if __name__=='__main__':unittest.main()
