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
        raise ValueError('请先确认叠加图中棋盘四边已对齐')
    transform=np.asarray(data.get('depth_to_rgb'),float)
    if transform.shape!=(4,) or not np.isfinite(transform).all() or np.any(transform[:2]<=0):
        raise ValueError('叠加缩放或平移参数无效')
    h,w=capture['rgb'].shape[:2]
    if points.shape!=(4,2) or not np.isfinite(points).all() or np.any(points<0) or np.any(points[:,0]>=w) or np.any(points[:,1]>=h):
        raise ValueError('请在 RGB 图内选择四角')
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
                'note':'红色为无效深度。请在深度图点棋盘角；内参为零时必须填写相机真实参数。'}
    if action=='calibrate':
        with state.height_lock:
            pending=state.height_previews.get(data['token'])
        if not pending or time.time()-pending[0]>180:raise ValueError('画面已过期，请重新冻结空棋盘')
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
        return {'ok':True,'calibration':cal,'message':'标定已保存；旧高度模型不再匹配，请重新建立高度模型'}
    if action=='build':return {'ok':True,'model':build_model(root)}
    if action=='report':
        return {'ok':True,'calibration':json.loads(calibration_path(root).read_text(encoding='utf-8')) if calibration_path(root).exists() else None,
                'model':json.loads(model_path(root).read_text(encoding='utf-8')) if model_path(root).exists() else None}
    raise ValueError('未知高度操作')


HEIGHT_PAGE=r'''<!doctype html><html lang="zh"><meta charset="utf-8"><title>高度标定与模型</title>
<style>body{font:16px system-ui;background:#17212b;color:#eee;margin:24px}p{max-width:1000px;line-height:1.6}button,input,textarea{font:inherit;margin:6px;padding:9px}canvas,img{max-width:100%;height:auto}textarea{width:90%;height:130px}pre{white-space:pre-wrap;word-break:break-all;background:#0d141a;padding:15px}label{display:block}</style>
<h1>自动估高与高度辅助分类</h1>
<p>① 清空棋盘并冻结画面。② 调整深度叠加层，让棋盘四边与 RGB 对齐。③ 在 RGB 上依次点击 a1、h1、h8、a8 所在的 8×8 区域外边角，不含木质外框。④ 检查深度内参并保存标定，再建立高度模型。</p>
<button id="freeze">冻结空棋盘</button><button id="reset">清空四角</button><button id="calibrate">保存三维标定</button><button id="build">建立高度模型并验证</button><button id="report">查看高度报告</button>
<p>深度相机真实内参（若未自动读取，需填写相机内参；不能凭图像大小猜测焦距）：</p><textarea id="intrinsics"></textarea>
<label><input id="legacy" type="checkbox">确认已有旧样本采集时，相机位置、棋盘位置、深度分辨率与当前标定一致，允许复用旧样本</label>
<p id="hint">标定和建立高度模型不需要重新训练 YOLO。更换相机位置后需重新标定。</p>
<h3>RGB 与深度叠加：在 RGB 上点击四角</h3>
<p>默认缩放只是适配显示，不代表相机已配准。调整透明度检查四边是否重合；若四边不能同时对齐，请勿确认，需要相机配准参数。本次手动对齐仅适用于当前空棋盘平面。</p>
<label>深度透明度 <input id="opacity" type="range" min="0" max="1" step="0.05" value="0.4"></label>
<label>深度横向缩放 <input id="sx" type="number" min="0.001" step="0.001" value="1">纵向缩放 <input id="sy" type="number" min="0.001" step="0.001" value="1"></label>
<label>深度水平平移(px) <input id="tx" type="number" step="1" value="0">垂直平移(px) <input id="ty" type="number" step="1" value="0"></label>
<label><input id="aligned" type="checkbox">已检查叠加图，空棋盘四边在 RGB 与深度中重合</label>
<button id="undo">撤销上一角</button><canvas id="depth"></canvas><p id="corners"></p>
<h3>RGB 参考图</h3><img id="rgb"><pre id="result">尚未操作。</pre>
<script>
const $=id=>document.getElementById(id),canvas=$('depth'),ctx=canvas.getContext('2d');let points=[],img=null,rgbImg=null,token=null,busy=false;
const transform=()=>['sx','sy','tx','ty'].map(id=>Number($(id).value));
function draw(){if(!img||!rgbImg)return;ctx.clearRect(0,0,canvas.width,canvas.height);ctx.drawImage(rgbImg,0,0);const [sx,sy,tx,ty]=transform();ctx.save();ctx.globalAlpha=Number($('opacity').value);if(sx>0&&sy>0)ctx.drawImage(img,tx,ty,img.naturalWidth*sx,img.naturalHeight*sy);ctx.restore();ctx.strokeStyle='#00ff80';ctx.fillStyle='#00ff80';ctx.lineWidth=2;ctx.font='16px sans-serif';points.forEach((p,i)=>{ctx.beginPath();ctx.arc(...p,4,0,7);ctx.fill();ctx.fillText(['a1','h1','h8','a8'][i],p[0]+5,p[1]);});if(points.length>1){ctx.beginPath();ctx.moveTo(...points[0]);points.slice(1).forEach(p=>ctx.lineTo(...p));if(points.length===4)ctx.closePath();ctx.stroke();}$('corners').textContent='RGB 四角：'+JSON.stringify(points);}
['sx','sy','tx','ty'].forEach(id=>$(id).oninput=()=>{$('aligned').checked=false;draw();});
$('opacity').oninput=draw;
$('undo').onclick=()=>{if(!busy){points.pop();draw();}};
canvas.onclick=e=>{if(busy||!img||points.length===4)return;let r=canvas.getBoundingClientRect();points.push([(e.clientX-r.left)*canvas.width/r.width,(e.clientY-r.top)*canvas.height/r.height]);draw();};
async function api(action,data={}){let r=await fetch('/api/height/'+action,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});let d=await r.json();if(!r.ok||!d.ok)throw Error(d.error||'操作失败');return d;}
function work(fn){return async()=>{if(busy)return;busy=true;$('result').textContent='处理中…';try{await fn();}catch(e){$('result').textContent=e.message;}finally{busy=false;}};}
$('freeze').onclick=work(async()=>{const d=await api('preview');token=d.token;points=[];img=new Image();rgbImg=new Image();img.src=d.depth_image;rgbImg.src=d.rgb_image;await Promise.all([img.decode(),rgbImg.decode()]);canvas.width=rgbImg.naturalWidth;canvas.height=rgbImg.naturalHeight;$('sx').value=canvas.width/img.naturalWidth;$('sy').value=canvas.height/img.naturalHeight;$('tx').value=0;$('ty').value=0;$('aligned').checked=false;$('rgb').src=d.rgb_image;$('intrinsics').value=JSON.stringify(d.intrinsics,null,2);draw();$('result').textContent='红色为无效深度。先对齐叠加层，再在 RGB 上点四角；内参必须使用深度相机真实参数。';});
$('reset').onclick=()=>{if(!busy){points=[];draw();}};
$('calibrate').onclick=work(async()=>{if(points.length!==4)throw Error('请先选取四角');if(!$('aligned').checked)throw Error('请先检查并确认棋盘四边已对齐');const d=await api('calibrate',{token,corners:points,corner_space:'rgb',depth_to_rgb:transform(),alignment_confirmed:$('aligned').checked,intrinsics:JSON.parse($('intrinsics').value),reuse_legacy:$('legacy').checked});$('result').textContent=JSON.stringify(d,null,2);});
$('build').onclick=work(async()=>{$('result').textContent=JSON.stringify(await api('build'),null,2);});
$('report').onclick=work(async()=>{$('result').textContent=JSON.stringify(await api('report'),null,2);});
</script></html>'''
