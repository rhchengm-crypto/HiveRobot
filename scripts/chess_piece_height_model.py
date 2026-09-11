"""Metric depth-to-board heights and validated, abstaining class assistance."""
from __future__ import annotations
import hashlib
import json
import time
from pathlib import Path
import cv2
import numpy as np
from chessboard_vision_v2_7 import BOARD_SIZE_MM, CHESS_PIECE_YOLO_CLASSES, normalize_square
from chess_piece_yolo_labels import atomic_write


def depth_m(depth, encoding):
    if str(encoding).lower() == '16uc1' and depth.dtype == np.uint16:
        return depth.astype(np.float64)*.001
    if str(encoding).lower() == '32fc1' and depth.dtype == np.float32:
        return depth.astype(np.float64)
    raise ValueError('深度必须为 16UC1(mm) 或 32FC1(m)')


def rays(shape, info):
    h,w=shape
    if (info.get('height'),info.get('width')) != (h,w):
        raise ValueError('深度内参分辨率不匹配')
    k=np.asarray(info['K'],dtype=float).reshape(3,3)
    d=np.asarray(info.get('D',[]),dtype=float)
    if d.size not in (0,4,5,8,12,14):raise ValueError('畸变系数长度无效')
    if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0 or not np.isfinite(d).all():
        raise ValueError('相机内参无效')
    if info.get('distortion_model','plumb_bob') not in ('plumb_bob','rational_polynomial',''):
        raise ValueError('暂不支持此深度镜头畸变模型')
    y,x=np.mgrid[:h,:w]
    xy=cv2.undistortPoints(np.stack([x,y],axis=-1).astype(np.float64).reshape(-1,1,2),k,d if d.size else None)
    return np.column_stack([xy.reshape(-1,2),np.ones(h*w)]).reshape(h,w,3)


def fit_board(depth, encoding, info, corners):
    """Fit an empty-board plane in depth optical coordinates, metric units."""
    z=depth_m(depth,encoding); ray=rays(z.shape,info)
    corners=np.asarray(corners,dtype=float)
    h,w=z.shape
    if corners.shape!=(4,2) or not np.isfinite(corners).all() or np.any(corners<0) or np.any(corners[:,0]>=w) or np.any(corners[:,1]>=h):
        raise ValueError('请在深度图内依次选取 a1,h1,h8,a8 四个棋盘外边角')
    poly=np.rint(corners).astype(np.int32)
    if not cv2.isContourConvex(poly) or cv2.contourArea(poly)<400:
        raise ValueError('四角顺序或面积不正确')
    mask=np.zeros(z.shape,np.uint8);cv2.fillConvexPoly(mask,poly,1)
    mask=cv2.erode(mask,np.ones((5,5),np.uint8))>0
    valid=mask & np.isfinite(z) & (z>.1) & (z<3)
    if valid.sum()<300 or valid.sum()<mask.sum()*.7:
        raise ValueError('空棋盘有效深度不足')
    points=(ray*z[...,None])[valid]
    points=points[np.linspace(0,len(points)-1,min(4000,len(points))).astype(int)]
    rng=np.random.default_rng(41);best=np.zeros(len(points),bool)
    for _ in range(100):
        a,b,c=points[rng.choice(len(points),3,replace=False)]
        n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-8:continue
        n/=length;inliers=np.abs((points-a)@n)<.003
        if inliers.sum()>best.sum():best=inliers
    if best.mean()<.85:raise ValueError('棋盘平面不稳定，请清空棋盘并检查深度四角')
    center=points[best].mean(axis=0)
    _,_,vh=np.linalg.svd(points[best]-center,full_matrices=False)
    n=vh[-1]
    if n@center>0:n=-n
    offset=-float(n@center)
    rmse=float(np.sqrt(np.mean((points[best]@n+offset)**2)))
    cr=cv2.undistortPoints(corners.reshape(-1,1,2),np.asarray(info['K'],float).reshape(3,3),np.asarray(info.get('D',[]),float) if info.get('D') else None).reshape(-1,2)
    cr=np.column_stack([cr,np.ones(4)])
    denom=cr@n
    if np.any(np.abs(denom)<.05):raise ValueError('视角太倾斜，无法建立棋盘坐标')
    xyz=cr*(-offset/denom)[:,None]
    lengths=np.linalg.norm(np.roll(xyz,-1,axis=0)-xyz,axis=1)
    if np.any(np.abs(lengths-BOARD_SIZE_MM/1000)>.045):
        edges='，'.join(f'{name}={value*1000:.1f}mm' for name,value in zip(('a1→h1','h1→h8','h8→a8','a8→a1'),lengths))
        raise ValueError(f'三维棋盘边长与 {BOARD_SIZE_MM:g}mm 不符：{edges}。允许偏差 ±45mm，无需点选到完全精确。请检查是否选了 8×8 区域外边角（不含木框）、RGB/深度对齐、深度内参及单位。')
    # Least-squares frame from all four corners; vertical projection is
    # implicit because its two axes lie in the fitted board plane.
    design=np.array([[0,0,1],[1,0,1],[1,1,1],[0,1,1]],float)
    frame=np.linalg.lstsq(design,xyz,rcond=None)[0]
    basis=frame[:2].T
    if np.linalg.cond(basis)>1.3:raise ValueError('棋盘四角不是近似正方形')
    result={'schema':1,'created_at':time.time(),'depth_shape':list(z.shape),'intrinsics':info,
            'depth_corners':corners.tolist(),'normal':n.tolist(),'offset_m':offset,
            'origin':frame[2].tolist(),'basis':basis.tolist(),'plane_rmse_mm':rmse*1000,
            'inlier_fraction':float(best.mean()),'edge_lengths_mm':(lengths*1000).tolist()}
    result['id']=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()[:24]
    return result


def estimate_heights(depth, encoding, calibration, squares):
    z=depth_m(depth,encoding)
    if list(z.shape)!=calibration['depth_shape']:raise ValueError('深度分辨率改变，请重新标定高度')
    points=rays(z.shape,calibration['intrinsics'])*z[...,None]
    finite=np.isfinite(points).all(axis=2)&(z>.1)&(z<3)
    xyz=np.nan_to_num(points,nan=0,posinf=0,neginf=0)
    heights=xyz@np.asarray(calibration['normal'])+calibration['offset_m']
    uv=(xyz-np.asarray(calibration['origin']))@np.linalg.pinv(np.asarray(calibration['basis'])).T
    board=finite&(uv[...,0]>.02)&(uv[...,0]<.98)&(uv[...,1]>.02)&(uv[...,1]<.98)
    near=board&(np.abs(heights)<.015)
    if near.sum()<200 or near.sum()<max(1,board.sum())*.4 or abs(float(np.median(heights[near])))>.003:
        raise ValueError('当前棋盘与高度标定不一致或遮挡过多，请清空检查并重新标定')
    out={}
    for s in squares:
        s=normalize_square(s);col=ord(s[0])-97;row=int(s[1])-1
        region=board&(uv[...,0]>=col/8)&(uv[...,0]<(col+1)/8)&(uv[...,1]>=row/8)&(uv[...,1]<(row+1)/8)
        mask=(region&(heights>.005)&(heights<.20)).astype(np.uint8)
        count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
        components=sorted(range(1,count),key=lambda i:stats[i,cv2.CC_STAT_AREA],reverse=True)
        if not components or stats[components[0],cv2.CC_STAT_AREA]<30:
            out[s]={'ok':False,'reason':'insufficient_piece_depth'};continue
        if len(components)>1 and stats[components[1],cv2.CC_STAT_AREA]>max(20,.3*stats[components[0],cv2.CC_STAT_AREA]):
            out[s]={'ok':False,'reason':'multiple_depth_components'};continue
        vals=heights[labels==components[0]]*1000
        value=float(np.percentile(vals,98))
        support=int(np.count_nonzero(np.abs(vals-value)<=5))
        if support<5:
            out[s]={'ok':False,'reason':'insufficient_top_support'};continue
        out[s]={'ok':True,'estimated_height_mm':value,'top_support':support,'point_count':len(vals),
                'uncertainty_mm':max(2.,calibration['plane_rmse_mm']*3),
                'method':'depth_3d_perpendicular_to_board_p98','calibration_id':calibration['id']}
    return out


def model_path(root):return Path(root)/'height_model.json'
def calibration_path(root):return Path(root)/'metric_calibration.json'


def build_model(root):
    root=Path(root)
    calibration=json.loads(calibration_path(root).read_text(encoding='utf-8'))
    records=[];errors=[]
    for path in sorted((root/'samples').glob('height_*/sample.json')):
        try:
            meta=json.loads(path.read_text(encoding='utf-8'))
            if meta.get('metric_calibration_id') not in (None,calibration['id']):raise ValueError('sample_uses_other_calibration')
            # Older samples may be reused only after explicit capture-time
            # setup compatibility confirmation; no inference from filenames.
            if meta.get('metric_calibration_id') is None and not (root/'legacy_calibration_ack.json').exists():
                raise ValueError('legacy_sample_requires_same_setup_confirmation')
            if meta.get('metric_calibration_id') is None:
                ack=json.loads((root/'legacy_calibration_ack.json').read_text())
                if ack.get('calibration_id')!=calibration['id']:raise ValueError('legacy_confirmation_expired')
            info=meta.get('camera_intrinsics') or {}
            depth_info=info.get('depth')
            if depth_info and (depth_info.get('width')!=calibration['intrinsics']['width'] or depth_info.get('height')!=calibration['intrinsics']['height'] or not np.allclose(depth_info['K'],calibration['intrinsics']['K'],rtol=.01,atol=.01)):
                raise ValueError('sample_intrinsics_changed')
            depth=np.load(path.parent/'depth.npy',allow_pickle=False)
            obs=estimate_heights(depth,meta['depth_metadata']['encoding'],calibration,[meta['square']])[meta['square']]
            if not obs['ok']:raise ValueError(obs['reason'])
            cls=meta['piece_class'];split=meta['split']
            if cls not in CHESS_PIECE_YOLO_CLASSES or split not in ('train','val','test'):raise ValueError('invalid_label')
            measured=meta.get('measured_height_mm')
            records.append({'sample_id':meta['sample_id'],'class':cls,'split':split,
                'height':obs['estimated_height_mm'],'error_mm':None if measured is None else obs['estimated_height_mm']-float(measured)})
        except (ValueError,KeyError,OSError,TypeError) as exc:errors.append({'sample':path.parent.name,'reason':str(exc)})
    profiles={}
    for cls in CHESS_PIECE_YOLO_CLASSES:
        train=[r for r in records if r['class']==cls and r['split']=='train']
        val=[r for r in records if r['class']==cls and r['split']=='val']
        if len(train)<2:continue
        values=np.array([r['height'] for r in train]);median=float(np.median(values))
        tolerance=max(5.,float(np.max(np.abs(values-median)))+2.)
        checked=[r for r in train+val if r['error_mm'] is not None]
        metric_ok=bool(checked) and all(abs(r['error_mm'])<=10 for r in checked)
        validated=bool(val) and all(abs(r['height']-median)<=tolerance for r in val) and metric_ok
        profiles[cls]={'median_mm':median,'tolerance_mm':tolerance,'train_count':len(train),'val_count':len(val),
                       'validated':validated,'metric_error_checked':metric_ok}
    test=[r for r in records if r['split']=='test']
    for r in test:
        r['height_candidates']=[cls for cls,p in profiles.items() if cls.split('_')[0]==r['class'].split('_')[0] and p['validated'] and abs(r['height']-p['median_mm'])<=p['tolerance_mm']]
    result={'schema':1,'calibration_id':calibration['id'],'built_at':time.time(),'profiles':profiles,
            'measurements':records,'test_results':test,'skipped_samples':errors,
            'active_classes':[c for c,p in profiles.items() if p['validated']],
            'class_status':{c:('已验证' if profiles.get(c,{}).get('validated') else '需要至少2份有效train、1份val及尺量误差校验') for c in CHESS_PIECE_YOLO_CLASSES},
            'note':'Test samples are reported only; never used to fit ranges or enable a class.'}
    atomic_write(model_path(root),json.dumps(result,ensure_ascii=False,indent=2))
    return result


def assist(mapped, observations, model):
    result={}
    profiles=model.get('profiles',{})
    for square,original in mapped.items():
        item=dict(original);obs=observations.get(square,{'ok':False,'reason':'no_height'})
        item['height']=dict(obs);item['height']['decision']='unchanged'
        if obs.get('ok'):
            cls=item['piece_class'];h=obs['estimated_height_mm'];color=cls.split('_')[0]
            candidates=[c for c,p in profiles.items() if c.split('_')[0]==color and p['validated'] and abs(h-p['median_mm'])+obs['uncertainty_mm']<=p['tolerance_mm']]
            item['height']['candidates']=candidates
            p=profiles.get(cls)
            # Require both current and alternative classes validated; height
            # never invents colors, geometry or an untrained category.
            if p and p['validated'] and len(candidates)==1 and candidates[0]!=cls and abs(h-p['median_mm'])>p['tolerance_mm']+obs['uncertainty_mm']:
                item['rgb_piece_class']=cls;item['rgb_confidence']=item['confidence']
                item['piece_class']=candidates[0];item['class_id']=CHESS_PIECE_YOLO_CLASSES.index(candidates[0])
                item['identity_method']='height_assisted_yolo'
                item['height']['decision']='corrected';item['confidence_semantics']='original_rgb_score_not_fused_probability'
        result[square]=item
    return result


def analyze_live(root, capture, mapped):
    """Optional assistance never breaks RGB detection when depth is unusable."""
    try:
        root=Path(root)
        cal=json.loads(calibration_path(root).read_text(encoding='utf-8'))
        captured=capture.get('captured_at',time.time())
        if any(not np.isfinite(captured-capture[k+'_received_at']) or not -.1<=captured-capture[k+'_received_at']<=2 for k in ('rgb','depth')):
            raise ValueError('stale_rgb_depth_at_capture')
        current_info=capture.get('camera_info',{}).get('depth')
        if current_info and not np.allclose(current_info['K'],cal['intrinsics']['K'],rtol=.01,atol=.01):
            raise ValueError('depth_intrinsics_changed')
        expected=cal.get('depth_frame_id','');actual=capture['depth_metadata'].get('frame_id','')
        if expected and expected!=actual:raise ValueError('depth_frame_changed')
        a=capture['rgb_metadata']['source_stamp_s'];b=capture['depth_metadata']['source_stamp_s']
        if not np.isfinite([a,b]).all() or min(a,b)<=0 or abs(a-b)>.1:raise ValueError('rgb_depth_timestamp_mismatch')
        if capture['depth'] is None:raise ValueError('missing_depth')
        observations=estimate_heights(capture['depth'],capture['depth_metadata']['encoding'],cal,list(mapped))
        model=json.loads(model_path(root).read_text(encoding='utf-8')) if model_path(root).exists() else {'profiles':{}}
        if model.get('calibration_id',cal['id'])!=cal['id']:model={'profiles':{}}
        return assist(mapped,observations,model)
    except (OSError,ValueError,KeyError,TypeError,np.linalg.LinAlgError) as exc:
        return {s:dict(p,height={'ok':False,'reason':str(exc),'decision':'unchanged'}) for s,p in mapped.items()}
