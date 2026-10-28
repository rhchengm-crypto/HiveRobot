"""Web workflow for empty-board depth calibration and height model building."""
import base64
import json
import time
import uuid
from pathlib import Path
import cv2
import numpy as np
from chess_piece_height_samples import height_dataset_dir, validate_pair
from chess_piece_height_model import fit_board, build_model, calibration_path, model_path
from chess_piece_yolo_labels import atomic_write


def depth_corners_from_overlay(data, capture):
    """Invert the explicitly confirmed, empty-board display alignment."""
    points=np.asarray(data['corners'],float)
    if data.get('corner_space','depth')=='depth':return points
    if data.get('corner_space')!='rgb' or data.get('alignment_confirmed') is not True:
        raise ValueError('Confirm that all four board edges align in the overlay first.')
    transform=np.asarray(data.get('depth_to_rgb'),float)
    if transform.shape!=(4,) or not np.isfinite(transform).all() or np.any(transform[:2]<=0):
        raise ValueError('Invalid overlay scale or translation.')
    h,w=capture['rgb'].shape[:2]
    if points.shape!=(4,2) or not np.isfinite(points).all() or np.any(points<0) or np.any(points[:,0]>=w) or np.any(points[:,1]>=h):
        raise ValueError('Select all four corners in the RGB image.')
    return (points-transform[2:])/transform[:2]


def handle_height_action(state, action, data):
    root=height_dataset_dir(state.yolo_dataset_dir)
    if action=='preview':
        capture=state.camera.height_snapshot();validate_pair(capture)
        z=capture['depth'].astype(float);valid=np.isfinite(z)&(z>0)
        low,high=np.percentile(z[valid],[2,98])
        gray=np.zeros(z.shape,np.uint8)
        gray[valid]=np.clip((z[valid]-low)/max(high-low,1e-6)*255,0,255).astype(np.uint8)
        depth_image=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR);depth_image[~valid]=(0,0,255)
        token=uuid.uuid4().hex
        with state.height_lock:
            state.height_previews={k:v for k,v in state.height_previews.items() if time.time()-v[0]<180}
            if len(state.height_previews)>=4:state.height_previews.pop(next(iter(state.height_previews)))
            state.height_previews[token]=(time.time(),capture)
        encode=lambda a:'data:image/png;base64,'+base64.b64encode(cv2.imencode('.png',a)[1]).decode()
        info=capture.get('camera_info',{}).get('depth')
        if not info:info={'width':z.shape[1],'height':z.shape[0],'K':[0,0,0,0,0,0,0,0,1],'D':[],'distortion_model':'plumb_bob','frame_id':capture['depth_metadata'].get('frame_id','')}
        return {'ok':True,'token':token,'depth_image':encode(depth_image),'rgb_image':encode(capture['rgb']),'intrinsics':info,
                'note':'Red indicates invalid depth. Click the board corners on the depth image. If intrinsics are zero, enter the actual camera parameters.'}
    if action=='calibrate':
        with state.height_lock:
            pending=state.height_previews.get(data['token'])
        if not pending or time.time()-pending[0]>180:raise ValueError('The frame has expired. Freeze the empty board again.')
        capture=pending[1]
        corners=depth_corners_from_overlay(data,capture)
        cal=fit_board(capture['depth'],capture['depth_metadata']['encoding'],data['intrinsics'],corners)
        cal['corner_selection']={'space':data.get('corner_space','depth'),'points':data['corners'],
                                 'depth_to_rgb':data.get('depth_to_rgb')}
        cal['depth_frame_id']=capture['depth_metadata'].get('frame_id','')
        atomic_write(calibration_path(root),json.dumps(cal,ensure_ascii=False,indent=2))
        # Preserve the empty-board frame as calibration evidence.
        np.save(str(root/'calibration_depth.npy'),capture['depth'],allow_pickle=False)
        if data.get('reuse_legacy') is True:
            atomic_write(root/'legacy_calibration_ack.json',json.dumps({'calibration_id':cal['id'],'confirmed_at':time.time()}))
        else:
            (root/'legacy_calibration_ack.json').unlink(missing_ok=True)
        return {'ok':True,'calibration':cal,'message':'Calibration saved. The previous height model no longer matches; rebuild it.'}
    if action=='build':return {'ok':True,'model':build_model(root)}
    if action=='report':
        return {'ok':True,'calibration':json.loads(calibration_path(root).read_text(encoding='utf-8')) if calibration_path(root).exists() else None,
                'model':json.loads(model_path(root).read_text(encoding='utf-8')) if model_path(root).exists() else None}
    raise ValueError('Unknown height operation')


HEIGHT_PAGE=r'''<!doctype html><html lang="en"><meta charset="utf-8"><title>Height Calibration and Model</title>
<style>body{font:16px system-ui;background:#17212b;color:#eee;margin:24px}p{max-width:1000px;line-height:1.6}button,input,textarea{font:inherit;margin:6px;padding:9px}canvas,img{max-width:100%;height:auto}textarea{width:90%;height:130px}pre{white-space:pre-wrap;word-break:break-all;background:#0d141a;padding:15px}label{display:block}</style>
<h1>Automatic Height Estimation and Height-Assisted Classification</h1>
<p>1. Clear the board and freeze the frame. 2. Adjust the depth overlay so all board edges align with RGB. 3. On RGB, click the outer corners of the 8x8 area in this order: a1, h1, h8, a8. Exclude the wooden frame. 4. Check depth intrinsics, save calibration, and build the height model.</p>
<button id="freeze">Freeze Empty Board</button><button id="reset">Clear Corners</button><button id="calibrate">Save 3D Calibration</button><button id="build">Build and Validate Height Model</button><button id="report">View Height Report</button>
<p>Actual depth-camera intrinsics (enter real camera values if they are not detected automatically; do not infer focal length from image size):</p><textarea id="intrinsics"></textarea>
<label><input id="legacy" type="checkbox">I confirm that old samples used the same camera pose, board pose, and depth resolution as this calibration; allow reuse.</label>
<p id="hint">Calibration and height-model building do not require YOLO retraining. Recalibrate after moving the camera.</p>
<h3>RGB and Depth Overlay: Click Corners on RGB</h3>
<p>Default scaling only fits the display and does not prove registration. Adjust opacity and verify all four edges overlap. If they cannot align simultaneously, do not confirm; camera registration parameters are required. This manual alignment applies only to the current empty-board plane.</p>
<label>Depth Opacity <input id="opacity" type="range" min="0" max="1" step="0.05" value="0.4"></label>
<label>Depth Horizontal Scale <input id="sx" type="number" min="0.001" step="0.001" value="1">Vertical Scale <input id="sy" type="number" min="0.001" step="0.001" value="1"></label>
<label>Depth Horizontal Offset (px) <input id="tx" type="number" step="1" value="0">Vertical Offset (px) <input id="ty" type="number" step="1" value="0"></label>
<label><input id="aligned" type="checkbox">I verified that all four empty-board edges overlap in RGB and depth.</label>
<button id="undo">Undo Last Corner</button><canvas id="depth"></canvas><p id="corners"></p>
<h3>RGB Reference Image</h3><img id="rgb"><pre id="result">No operation yet.</pre>
<script>
const $=id=>document.getElementById(id),canvas=$('depth'),ctx=canvas.getContext('2d');let points=[],img=null,rgbImg=null,token=null,busy=false;
const transform=()=>['sx','sy','tx','ty'].map(id=>Number($(id).value));
function draw(){if(!img||!rgbImg)return;ctx.clearRect(0,0,canvas.width,canvas.height);ctx.drawImage(rgbImg,0,0);const [sx,sy,tx,ty]=transform();ctx.save();ctx.globalAlpha=Number($('opacity').value);if(sx>0&&sy>0)ctx.drawImage(img,tx,ty,img.naturalWidth*sx,img.naturalHeight*sy);ctx.restore();ctx.strokeStyle='#00ff80';ctx.fillStyle='#00ff80';ctx.lineWidth=2;ctx.font='16px sans-serif';points.forEach((p,i)=>{ctx.beginPath();ctx.arc(...p,4,0,7);ctx.fill();ctx.fillText(['a1','h1','h8','a8'][i],p[0]+5,p[1]);});if(points.length>1){ctx.beginPath();ctx.moveTo(...points[0]);points.slice(1).forEach(p=>ctx.lineTo(...p));if(points.length===4)ctx.closePath();ctx.stroke();}$('corners').textContent='RGB corners: '+JSON.stringify(points);}
['sx','sy','tx','ty'].forEach(id=>$(id).oninput=()=>{$('aligned').checked=false;draw();});
$('opacity').oninput=draw;
$('undo').onclick=()=>{if(!busy){points.pop();draw();}};
canvas.onclick=e=>{if(busy||!img||points.length===4)return;let r=canvas.getBoundingClientRect();points.push([(e.clientX-r.left)*canvas.width/r.width,(e.clientY-r.top)*canvas.height/r.height]);draw();};
async function api(action,data={}){let r=await fetch('/api/height/'+action,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});let d=await r.json();if(!r.ok||!d.ok)throw Error(d.error||'Operation failed');return d;}
function work(fn){return async()=>{if(busy)return;busy=true;$('result').textContent='Processing...';try{await fn();}catch(e){$('result').textContent=e.message;}finally{busy=false;}};}
$('freeze').onclick=work(async()=>{const d=await api('preview');token=d.token;points=[];img=new Image();rgbImg=new Image();img.src=d.depth_image;rgbImg.src=d.rgb_image;await Promise.all([img.decode(),rgbImg.decode()]);canvas.width=rgbImg.naturalWidth;canvas.height=rgbImg.naturalHeight;$('sx').value=canvas.width/img.naturalWidth;$('sy').value=canvas.height/img.naturalHeight;$('tx').value=0;$('ty').value=0;$('aligned').checked=false;$('rgb').src=d.rgb_image;$('intrinsics').value=JSON.stringify(d.intrinsics,null,2);draw();$('result').textContent='Red indicates invalid depth. Align the overlay first, then click the four corners on RGB. Intrinsics must use the actual camera parameters.';});
$('reset').onclick=()=>{if(!busy){points=[];draw();}};
$('calibrate').onclick=work(async()=>{if(points.length!==4)throw Error('Select all four corners first.');if(!$('aligned').checked)throw Error('Verify and confirm that all four board edges align first.');const d=await api('calibrate',{token,corners:points,corner_space:'rgb',depth_to_rgb:transform(),alignment_confirmed:$('aligned').checked,intrinsics:JSON.parse($('intrinsics').value),reuse_legacy:$('legacy').checked});$('result').textContent=JSON.stringify(d,null,2);});
$('build').onclick=work(async()=>{$('result').textContent=JSON.stringify(await api('build'),null,2);});
$('report').onclick=work(async()=>{$('result').textContent=JSON.stringify(await api('report'),null,2);});
</script></html>'''
