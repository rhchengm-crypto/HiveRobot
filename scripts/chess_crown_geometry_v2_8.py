"""Metric teaching and crown-coordinate planning. No motor commands here."""
import hashlib
import json
import time
import numpy as np
from chess_piece_yolo_labels import atomic_write
from chessboard_vision_v2_7 import CHESS_PIECE_YOLO_CLASSES, normalize_square


def array(value, shape):
    result=np.asarray(value,dtype=float)
    if result.shape!=shape or not np.isfinite(result).all():
        raise ValueError('坐标维度错误或包含无效数值')
    return result


def fit_transform(samples, max_error_mm=3.):
    """Fit a rigid transform, with separate held-out verification points."""
    groups={split:[s for s in samples if s.get('split')==split] for split in ('train','check')}
    if len(groups['train'])<3 or len(groups['check'])<2:
        raise ValueError('至少需要3个不共线标定点，以及2个独立检查点')
    points={}
    for split,rows in groups.items():
        a=np.array([array(s['board_mm'],(3,)) for s in rows])
        b=np.array([array(s['arm_mm'],(3,)) for s in rows])
        points[split]=(a,b)
    a,b=points['train'];ac=a-a.mean(0);bc=b-b.mean(0)
    if np.linalg.svd(ac,compute_uv=False)[1]<20 or np.linalg.svd(bc,compute_uv=False)[1]<20:
        raise ValueError('标定点分布过窄或共线，请分散到棋盘不同区域')
    u,_,vt=np.linalg.svd(ac.T@bc)
    d=np.eye(3);d[2,2]=np.linalg.det(vt.T@u.T)
    r=vt.T@d@u.T;t=b.mean(0)-r@a.mean(0)
    errors={split:np.linalg.norm(x@r.T+t-y,axis=1).tolist() for split,(x,y) in points.items()}
    if max(errors['train']+errors['check'])>max_error_mm:
        raise ValueError('坐标标定误差超过3mm：'+json.dumps(errors))
    # Repeating fitted points is not independent verification.
    for q in points['check'][0]:
        if np.min(np.linalg.norm(a-q,axis=1))<10:
            raise ValueError('检查点须距离标定点至少10mm')
    result={'rotation':r.tolist(),'translation_mm':t.tolist(),'errors_mm':errors,
            'created_at':time.time(),'samples':samples,'units':'mm'}
    result['id']=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()[:20]
    return result


def crown_point(ray_origin, ray_direction, board_from_camera, crown_height_mm):
    """Intersect the crown pixel ray with its taught height plane, not z=0."""
    transform=array(board_from_camera,(4,4))
    origin=transform[:3,:3]@array(ray_origin,(3,))+transform[:3,3]
    direction=transform[:3,:3]@array(ray_direction,(3,))
    if abs(direction[2])<1e-8:raise ValueError('视线与棋冠平面平行')
    distance=(float(crown_height_mm)-origin[2])/direction[2]
    if not np.isfinite(distance) or distance<=0:raise ValueError('棋冠平面不在相机前方')
    return origin+distance*direction


def validate_profile(data):
    cls=data['piece_class']
    if cls not in CHESS_PIECE_YOLO_CLASSES:raise ValueError('棋子类别无效')
    height=float(data['grip_height_mm']);total=float(data['piece_height_mm'])
    width=float(data['open_width_mm']);closed=float(data['closed_width_mm'])
    if not all(np.isfinite([height,total,width,closed])) or not 0<height<=total<=200 or not 0<=closed<width<=150:
        raise ValueError('请检查棋子高度、棋冠夹持高度和夹爪开合宽度')
    orientation=str(data.get('orientation','front'))
    if orientation not in ('front','back','left','right'):raise ValueError('朝向无效')
    offset=array(data.get('tcp_offset_mm',[0,0,0]),(3,))
    if np.linalg.norm(offset)>300:raise ValueError('工具偏移超过300mm，请检查单位')
    return dict(piece_class=cls,orientation=orientation,grip_height_mm=height,
                piece_height_mm=total,open_width_mm=width,closed_width_mm=closed,
                tcp_offset_mm=offset.tolist())


def coordinate_plan(calibration,profile,source_xy,target_square,clearance_mm=120):
    """Coordinates only: cannot establish arm/link collision safety or IK."""
    xy=array(source_xy,(2,));s=normalize_square(target_square)
    if np.any(xy<0) or np.any(xy>=440):raise ValueError('取棋位置不在棋盘内')
    clearance=float(clearance_mm)
    if not np.isfinite(clearance) or clearance<profile['piece_height_mm']+30 or clearance>400:
        raise ValueError('上方预览高度至少高于棋子30mm，且不得超过400mm')
    target=np.array([(ord(s[0])-97+.5)*55,(int(s[1])-.5)*55])
    grip=profile['grip_height_mm'];offset=np.array(profile['tcp_offset_mm'])
    r=array(calibration['rotation'],(3,3));t=array(calibration['translation_mm'],(3,))
    stages=[('source_above',xy,clearance),('crown_grip',xy,grip),('lift',xy,clearance),
            ('target_above',target,clearance),('release',target,grip),('retreat',target,clearance)]
    result=[]
    for name,pos,z in stages:
        board=np.array([pos[0],pos[1],z])+offset
        result.append({'stage':name,'tcp_board_mm':board.tolist(),'tcp_arm_mm':(r@board+t).tolist()})
    return {'waypoints':result,'executable':False,
            'reason':'仅坐标预览：尚需确认实际运动学、工具姿态、关节限位和整条路径碰撞检查',
            'calibration_id':calibration['id'],'profile':profile,'target_square':s}


class GeometryStore:
    def __init__(self,path):self.path=path
    def read(self):
        if self.path.exists():return json.loads(self.path.read_text(encoding='utf-8'))
        return {'schema':1,'profiles':{},'calibration':None}
    def action(self,action,data):
        current=self.read()
        if action=='status':return {'ok':True,'data':current,'motion_enabled':False}
        if action=='calibrate':current['calibration']=fit_transform(data['samples'])
        elif action=='profile':
            p=validate_profile(data);current['profiles'][p['piece_class']+':'+p['orientation']]=p
        elif action=='plan':
            if not current['calibration']:raise ValueError('请先完成坐标标定及独立检查')
            p=current['profiles'].get(data['profile'])
            if not p:raise ValueError('请先保存该棋子朝向的棋冠参数')
            return {'ok':True,'plan':coordinate_plan(current['calibration'],p,data['source_xy_mm'],data['target_square'],data.get('clearance_mm',120))}
        else:raise ValueError('未知几何操作')
        atomic_write(self.path,json.dumps(current,ensure_ascii=False,indent=2))
        return {'ok':True,'data':current,'motion_enabled':False}


PAGE='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>v2.8 棋冠几何示教</title>
<style>body{background:#17212b;color:#eee;font:16px system-ui;padding:24px}textarea{width:95%;height:180px}button,input,select{font:inherit;margin:5px;padding:8px}pre{white-space:pre-wrap}p{max-width:1000px}</style>
<h1>棋冠抓取：标定与坐标预览</h1>
<p>此阶段不运动机械臂。棋盘坐标：a1外角为原点，X向h1，Y向a8，Z离开棋盘，单位mm。机械臂坐标必须来自已确认的工具中心测量；不要填写相机坐标或把关节角当坐标。</p>
<h2>1. 坐标对应点</h2><p>输入至少3个分散、不共线的train点，另加2个未参与拟合的check点。每个点的棋盘坐标和机械臂坐标必须表示同一工具中心位置。</p>
<textarea id="samples" placeholder='[{"split":"train","board_mm":[0,0,0],"arm_mm":[实测X,实测Y,实测Z]}, ...]'></textarea><br><button onclick="send('calibrate',{samples:JSON.parse(el('samples').value)})">拟合并验证坐标</button>
<h2>2. 棋冠夹持参数</h2><p>分别记录类别和朝向。夹持高度是夹爪夹住棋冠的位置距棋盘的高度。TCP偏移以固定抓取姿态下的棋盘轴表示，当前不支持任意旋转姿态。</p>
<select id="cls">__CLASSES__</select><select id="orientation"><option>front</option><option>back</option><option>left</option><option>right</option></select><br>
<label>棋子全高<input id="total" type="number"></label><label>夹持高度<input id="grip" type="number"></label><br>
<label>张开宽度<input id="open" type="number"></label><label>夹持宽度<input id="closed" type="number"></label><br>
<label>TCP偏移 XYZ(mm)<input id="offset" value="0,0,0"></label><button onclick="profile()">保存棋冠参数</button>
<h2>3. 抓放坐标预览</h2><p>当前为手动输入实际取棋XY的坐标检查；尚未接入视觉棋冠定位和自动目标选择。D4中心为192.5,192.5，实际偏心时应使用实测位置。</p>
<input id="source" value="192.5,192.5"><input id="target" value="c1"><label>上方高度<input id="clearance" type="number" value="120"></label>
<button onclick="send('plan',{profile:el('cls').value+':'+el('orientation').value,source_xy_mm:vec('source'),target_square:el('target').value,clearance_mm:num('clearance')})">生成坐标预览（不执行）</button>
<button onclick="send('status',{})">读取已保存数据</button><pre id="result"></pre>
<script>const el=id=>document.getElementById(id),num=id=>Number(el(id).value),vec=id=>el(id).value.split(',').map(Number);
async function send(action,data){try{let r=await fetch('/api/crown/'+action,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});let d=await r.json();el('result').textContent=JSON.stringify(d,null,2);}catch(e){el('result').textContent=e.message;}}
function profile(){send('profile',{piece_class:el('cls').value,orientation:el('orientation').value,piece_height_mm:num('total'),grip_height_mm:num('grip'),open_width_mm:num('open'),closed_width_mm:num('closed'),tcp_offset_mm:vec('offset')});}</script></html>'''.replace('__CLASSES__',''.join('<option>'+c+'</option>' for c in CHESS_PIECE_YOLO_CLASSES))
