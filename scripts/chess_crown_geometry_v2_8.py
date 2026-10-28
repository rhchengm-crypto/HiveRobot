"""Metric teaching and crown-coordinate planning. No motor commands here."""
import hashlib
import copy
import json
import shutil
import time
import numpy as np
from chess_piece_yolo_labels import atomic_write
from chessboard_vision_v2_7 import CHESS_PIECE_YOLO_CLASSES, normalize_square


def array(value, shape):
    result=np.asarray(value,dtype=float)
    if result.shape!=shape or not np.isfinite(result).all():
        raise ValueError('Coordinate dimensions are invalid or contain non-finite values.')
    return result


def fit_transform(samples, max_error_mm=3.):
    """Fit a rigid transform, with separate held-out verification points."""
    groups={split:[s for s in samples if s.get('split')==split] for split in ('train','check')}
    if len(groups['train'])<3 or len(groups['check'])<2:
        raise ValueError('At least 3 non-collinear calibration points and 2 independent check points are required.')
    points={}
    for split,rows in groups.items():
        a=np.array([array(s['board_mm'],(3,)) for s in rows])
        b=np.array([array(s['arm_mm'],(3,)) for s in rows])
        points[split]=(a,b)
    a,b=points['train'];ac=a-a.mean(0);bc=b-b.mean(0)
    if np.linalg.svd(ac,compute_uv=False)[1]<20 or np.linalg.svd(bc,compute_uv=False)[1]<20:
        raise ValueError('Calibration points are too close together or collinear. Spread them across different board areas.')
    u,_,vt=np.linalg.svd(ac.T@bc)
    d=np.eye(3);d[2,2]=np.linalg.det(vt.T@u.T)
    r=vt.T@d@u.T;t=b.mean(0)-r@a.mean(0)
    errors={split:np.linalg.norm(x@r.T+t-y,axis=1).tolist() for split,(x,y) in points.items()}
    if max(errors['train']+errors['check'])>max_error_mm:
        raise ValueError('Coordinate calibration error exceeds 3 mm: '+json.dumps(errors))
    # Repeating fitted points is not independent verification.
    for q in points['check'][0]:
        if np.min(np.linalg.norm(a-q,axis=1))<10:
            raise ValueError('Each check point must be at least 10 mm from every calibration point.')
    result={'rotation':r.tolist(),'translation_mm':t.tolist(),'errors_mm':errors,
            'created_at':time.time(),'samples':samples,'units':'mm'}
    result['id']=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()[:20]
    return result


def crown_point(ray_origin, ray_direction, board_from_camera, crown_height_mm):
    """Intersect the crown pixel ray with its taught height plane, not z=0."""
    transform=array(board_from_camera,(4,4))
    origin=transform[:3,:3]@array(ray_origin,(3,))+transform[:3,3]
    direction=transform[:3,:3]@array(ray_direction,(3,))
    if abs(direction[2])<1e-8:raise ValueError('The viewing ray is parallel to the crown plane.')
    distance=(float(crown_height_mm)-origin[2])/direction[2]
    if not np.isfinite(distance) or distance<=0:raise ValueError('The crown plane is not in front of the camera.')
    return origin+distance*direction


def validate_profile(data):
    cls=data['piece_class']
    if cls not in CHESS_PIECE_YOLO_CLASSES:raise ValueError('Invalid piece class.')
    height=float(data['grip_height_mm']);total=float(data['piece_height_mm'])
    width=float(data['open_width_mm']);closed=float(data['closed_width_mm'])
    if not all(np.isfinite([height,total,width,closed])) or not 0<height<=total<=200 or not 0<=closed<width<=150:
        raise ValueError('Check piece height, crown grip height, and claw opening/closing widths.')
    orientation=str(data.get('orientation','front'))
    if orientation not in ('front','back','left','right'):raise ValueError('Invalid orientation.')
    offset=array(data.get('tcp_offset_mm',[0,0,0]),(3,))
    if np.linalg.norm(offset)>300:raise ValueError('Tool offset exceeds 300 mm. Check the units.')
    return dict(piece_class=cls,orientation=orientation,grip_height_mm=height,
                piece_height_mm=total,open_width_mm=width,closed_width_mm=closed,
                tcp_offset_mm=offset.tolist())


def validate_placement_anchor(data):
    """Validate a taught joint pose tied to one chessboard destination."""
    cls=str(data['piece_class'])
    if cls not in CHESS_PIECE_YOLO_CLASSES:raise ValueError('Invalid piece class.')
    square=normalize_square(data['target_square'])
    move_name=str(data['saved_move_name']).strip()
    if not move_name:raise ValueError('Saved Move name cannot be empty.')
    joint_names=('shoulder_front','shoulder_side','shoulder_rotate','elbow',
                 'arm_roll','wrist_side','wrist')
    pose=data.get('pose_rad',{})
    if set(pose)!=set(joint_names):raise ValueError('The placement pose must contain all seven positioning joints.')
    pose={name:float(pose[name]) for name in joint_names}
    if not np.isfinite(list(pose.values())).all():raise ValueError('The joint pose contains invalid values.')
    tolerance=float(data.get('tolerance_deg',.5))
    errors={name:float(value) for name,value in data.get('validation_errors_deg',{}).items()}
    if errors and set(errors)!=set(joint_names):raise ValueError('Validation errors must contain all seven positioning joints.')
    if not np.isfinite([tolerance,*errors.values()]).all() or tolerance<=0:
        raise ValueError('Invalid validation tolerance or errors.')
    validated=bool(errors) and max(abs(value) for value in errors.values())<=tolerance
    board_tcp=data.get('board_tcp_mm')
    if board_tcp is not None:board_tcp=array(board_tcp,(3,)).tolist()
    anchor = {'piece_class':cls,'target_square':square,'saved_move_name':move_name,
            'pose_rad':pose,'joint_units':'radians','board_tcp_mm':board_tcp,
            'tolerance_deg':tolerance,'validation_errors_deg':errors,
            'validated':validated,'validation_source':str(data.get('validation_source','')),
            'purpose':str(data.get('purpose','geometry_training_anchor')),
            'includes_claw_release':bool(data.get('includes_claw_release',False)),
            'updated_at':str(data.get('updated_at','')) or time.strftime('%Y-%m-%d %H:%M:%S')}
    if 'workflow_validation' in data:
        if not isinstance(data['workflow_validation'], dict):
            raise ValueError('workflow_validation must be an object.')
        anchor['workflow_validation'] = data['workflow_validation']
    return anchor


def validate_grasp_anchor(data):
    """Record an empirical grasp pose without claiming a calibrated 3-D transform."""
    anchor=validate_placement_anchor({
        'piece_class':data['piece_class'],
        'target_square':data['source_square'],
        'saved_move_name':data['saved_move_name'],
        'pose_rad':data['pose_rad'],
        'board_tcp_mm':data.get('board_tcp_mm'),
        'tolerance_deg':data.get('tolerance_deg',.5),
        'validation_errors_deg':data.get('validation_errors_deg',{}),
        'validation_source':data.get('validation_source',''),
        'purpose':'geometry_training_anchor',
        'updated_at':data.get('updated_at',''),
    })
    anchor['source_square']=anchor.pop('target_square')
    anchor['pose_validated']=anchor.pop('validated')
    anchor['board_correspondence_status']=str(data.get('board_correspondence_status','unverified'))
    anchor['contact_confirmed']=data.get('contact_confirmed') is True
    anchor['metric_transform_validated']=False
    xy=data.get('board_xy_mm')
    anchor['board_xy_mm']=array(xy,(2,)).tolist() if xy is not None else None
    width=data.get('grip_section_width_mm')
    anchor['grip_section_width_mm']=float(width) if width is not None else None
    if width is not None and (not np.isfinite(anchor['grip_section_width_mm']) or anchor['grip_section_width_mm']<=0):
        raise ValueError('Invalid crown grip width.')
    anchor['grip_dimensions_source']=str(data.get('grip_dimensions_source',''))
    anchor['pose_source']=str(data.get('pose_source',''))
    for field in ('contact_confirmation_method',):
        if field in data: anchor[field]=str(data[field])
    for field in ('workflow_validation','grasp_confirmation'):
        if field in data:
            if not isinstance(data[field],dict):raise ValueError(field+' must be an object.')
            anchor[field]=data[field]
    return anchor


def validate_anchor_bundle(data):
    if not isinstance(data,dict) or data.get('schema')!=1:
        raise ValueError('Geometry file schema must be 1.')
    result={}
    for section,validator,square_field in (
        ('placement_anchors',validate_placement_anchor,'target_square'),
        ('grasp_anchors',validate_grasp_anchor,'source_square'),
    ):
        records=data.get(section,{})
        if not isinstance(records,dict) or len(records)>128:
            raise ValueError(section+' must be an object containing no more than 128 records.')
        checked={}
        for key,raw in records.items():
            if not isinstance(raw,dict):raise ValueError(str(key)+' must be an object.')
            anchor=validator(raw)
            expected=anchor['piece_class']+':'+anchor[square_field]
            if key!=expected:raise ValueError('Anchor key does not match its content: '+str(key))
            checked[key]=anchor
        result[section]=checked
    if not any(result.values()):raise ValueError('The file contains no importable anchors.')
    return result


def geometry_revision(current):
    raw=json.dumps(current,ensure_ascii=False,sort_keys=True,separators=(',',':'))
    return hashlib.sha256(raw.encode('utf-8')).hexdigest()[:20]


def merge_anchor_bundle(current,bundle):
    """Merge teaching evidence while refusing to replace a different taught pose."""
    result=copy.deepcopy(current)
    changes={'new':[],'updated':[],'unchanged':[],'conflicts':[]}
    for section in ('placement_anchors','grasp_anchors'):
        saved=result.setdefault(section,{})
        if not isinstance(saved,dict):raise ValueError('In the robot geometry file, '+section+' has an invalid format.')
        for key,incoming in bundle[section].items():
            label=section+':'+key
            old=saved.get(key)
            if old is None:
                saved[key]=incoming
                changes['new'].append(label)
                continue
            if not isinstance(old,dict) or old.get('saved_move_name')!=incoming['saved_move_name']:
                changes['conflicts'].append(label+' (different Saved Move)')
                continue
            old_pose=old.get('pose_rad')
            if not isinstance(old_pose,dict) or set(old_pose)!=set(incoming['pose_rad']):
                changes['conflicts'].append(label+' (incomplete joint pose)')
                continue
            try:
                pose_difference=max(abs(float(old_pose[j])-v) for j,v in incoming['pose_rad'].items())
            except (TypeError,ValueError):
                pose_difference=float('inf')
            if not np.isfinite(pose_difference) or pose_difference>1e-4:
                changes['conflicts'].append(label+' (different joint pose)')
                continue
            merged=copy.deepcopy(old)
            if section=='grasp_anchors':
                if incoming.get('contact_confirmed'):
                    merged['contact_confirmed']=True
                fields=('contact_confirmation_method','grasp_confirmation','workflow_validation')
            else:
                fields=('workflow_validation',)
            for field in fields:
                if field in incoming:merged[field]=incoming[field]
            if merged!=old:
                merged['updated_at']=incoming['updated_at']
                saved[key]=merged
                changes['updated'].append(label)
            else:
                changes['unchanged'].append(label)
    return result,changes


def coordinate_plan(calibration,profile,source_xy,target_square,clearance_mm=120):
    """Coordinates only: cannot establish arm/link collision safety or IK."""
    xy=array(source_xy,(2,));s=normalize_square(target_square)
    if np.any(xy<0) or np.any(xy>=440):raise ValueError('The pickup location is outside the board.')
    clearance=float(clearance_mm)
    if not np.isfinite(clearance) or clearance<profile['piece_height_mm']+30 or clearance>400:
        raise ValueError('Preview clearance must be at least 30 mm above the piece and no more than 400 mm.')
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
            'reason':'Coordinate preview only: actual kinematics, tool orientation, joint limits, and full-path collision checks still require validation.',
            'calibration_id':calibration['id'],'profile':profile,'target_square':s}


class GeometryStore:
    def __init__(self,path):self.path=path
    def read(self):
        if self.path.exists():return json.loads(self.path.read_text(encoding='utf-8'))
        return {'schema':1,'profiles':{},'placement_anchors':{},'grasp_anchors':{},'calibration':None}
    def action(self,action,data):
        current=self.read()
        if action=='status':return {'ok':True,'data':current,'motion_enabled':False}
        if action=='import-anchors':
            bundle=validate_anchor_bundle(data.get('geometry'))
            proposed,changes=merge_anchor_bundle(current,bundle)
            revision=geometry_revision(current)
            preview={'changes':changes,'counts':{key:len(value) for key,value in changes.items()},
                     'current_revision':revision,'motion_enabled':False}
            if data.get('dry_run') is True:
                return {'ok':True,'preview':preview}
            if changes['conflicts']:
                raise ValueError('Pose conflicts found; nothing was written: '+', '.join(changes['conflicts']))
            if data.get('expected_revision')!=revision:
                raise ValueError('Robot-side data changed. Preview again before writing.')
            backup=None
            if self.path.exists():
                backup=self.path.with_name(self.path.name+'.bak.'+str(time.time_ns()))
                shutil.copy2(self.path,backup)
            atomic_write(self.path,json.dumps(proposed,ensure_ascii=False,indent=2))
            return {'ok':True,'preview':preview,'backup_path':str(backup) if backup else None,
                    'written_revision':geometry_revision(proposed),'motion_enabled':False}
        if action=='calibrate':current['calibration']=fit_transform(data['samples'])
        elif action=='profile':
            p=validate_profile(data);current['profiles'][p['piece_class']+':'+p['orientation']]=p
        elif action=='placement-anchor':
            anchor=validate_placement_anchor(data)
            current.setdefault('placement_anchors',{})[
                anchor['piece_class']+':'+anchor['target_square']
            ]=anchor
        elif action=='grasp-anchor':
            anchor=validate_grasp_anchor(data)
            current.setdefault('grasp_anchors',{})[
                anchor['piece_class']+':'+anchor['source_square']
            ]=anchor
        elif action=='plan':
            if not current['calibration']:raise ValueError('Complete coordinate calibration and independent checks first.')
            p=current['profiles'].get(data['profile'])
            if not p:raise ValueError('Save crown parameters for this piece orientation first.')
            return {'ok':True,'plan':coordinate_plan(current['calibration'],p,data['source_xy_mm'],data['target_square'],data.get('clearance_mm',120))}
        else:raise ValueError('Unknown geometry operation')
        atomic_write(self.path,json.dumps(current,ensure_ascii=False,indent=2))
        return {'ok':True,'data':current,'motion_enabled':False}


PAGE='''<!doctype html><html lang="en"><meta charset="utf-8"><title>v2.8 Crown Geometry Teaching</title>
<style>body{background:#17212b;color:#eee;font:16px system-ui;padding:24px}textarea{width:95%;height:180px}button,input,select{font:inherit;margin:5px;padding:8px}pre{white-space:pre-wrap}p{max-width:1000px}</style>
<h1>Crown Grasping: Calibration and Coordinate Preview</h1>
<p>This page does not move the robot arm. Board coordinates use the outer a1 corner as origin, X toward h1, Y toward a8, and Z away from the board, in millimeters. Robot coordinates must come from confirmed tool-center measurements; do not enter camera coordinates or joint angles.</p>
<h2>1. Coordinate Correspondences</h2><p>Enter at least 3 spread-out, non-collinear train points and 2 check points excluded from fitting. Each board/robot coordinate pair must represent the same tool-center position.</p>
<textarea id="samples" placeholder='[{"split":"train","board_mm":[0,0,0],"arm_mm":[measuredX,measuredY,measuredZ]}, ...]'></textarea><br><button onclick="send('calibrate',{samples:JSON.parse(el('samples').value)})">Fit and Validate Coordinates</button>
<h2>2. Crown Grip Parameters</h2><p>Record each class and orientation separately. Grip height is the height above the board where the claw holds the crown. TCP offset uses board axes at a fixed grasp orientation; arbitrary rotated poses are not currently supported.</p>
<select id="cls">__CLASSES__</select><select id="orientation"><option>front</option><option>back</option><option>left</option><option>right</option></select><br>
<label>Piece Height<input id="total" type="number"></label><label>Grip Height<input id="grip" type="number"></label><br>
<label>Open Width<input id="open" type="number"></label><label>Grip Width<input id="closed" type="number"></label><br>
<label>TCP Offset XYZ (mm)<input id="offset" value="0,0,0"></label><button onclick="profile()">Save Crown Parameters</button>
<h2>3. Pick-and-Place Coordinate Preview</h2><p>This preview checks manually entered pickup XY coordinates. Visual crown localization and automatic target selection are not connected yet. The D4 center is 192.5,192.5; use the measured position when the piece is offset.</p>
<input id="source" value="192.5,192.5"><input id="target" value="c1"><label>Clearance Height<input id="clearance" type="number" value="120"></label>
<button onclick="send('plan',{profile:el('cls').value+':'+el('orientation').value,source_xy_mm:vec('source'),target_square:el('target').value,clearance_mm:num('clearance')})">Generate Coordinate Preview (No Execution)</button>
<h2>4. Import Validated Anchors from This Computer</h2>
<p>Select the local chess_crown_geometry_v2_8.json file. Preview new, updated, and conflicting records first. Writing only merges grasp and placement anchors; it does not change existing robot calibration, parameters, or other training data. A same-name anchor with a different pose is rejected.</p>
<input id="anchorFile" type="file" accept=".json,application/json" onchange="resetImport()">
<button onclick="previewImport()">Preview Anchors</button>
<button id="writeAnchors" onclick="writeImport()" disabled>Write Previewed Anchors</button>
<pre id="importResult"></pre>
<button onclick="send('status',{})">Read Saved Data</button><pre id="result"></pre>
<script>const el=id=>document.getElementById(id),num=id=>Number(el(id).value),vec=id=>el(id).value.split(',').map(Number);
async function send(action,data){try{let r=await fetch('/api/crown/'+action,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});let d=await r.json();el('result').textContent=JSON.stringify(d,null,2);}catch(e){el('result').textContent=e.message;}}
function profile(){send('profile',{piece_class:el('cls').value,orientation:el('orientation').value,piece_height_mm:num('total'),grip_height_mm:num('grip'),open_width_mm:num('open'),closed_width_mm:num('closed'),tcp_offset_mm:vec('offset')});}
let pendingImport=null;
function resetImport(){pendingImport=null;el('writeAnchors').disabled=true;el('importResult').textContent='';}
async function anchorRequest(body){let response=await fetch('/api/crown/import-anchors',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});let result=await response.json();if(!response.ok||!result.ok)throw Error(result.error||result.message||JSON.stringify(result));return result;}
async function previewImport(){resetImport();try{let file=el('anchorFile').files[0];if(!file)throw Error('Select a geometry JSON file first.');let geometry=JSON.parse(await file.text());let result=await anchorRequest({geometry,dry_run:true});pendingImport={geometry,expected_revision:result.preview.current_revision};el('importResult').textContent=JSON.stringify(result.preview,null,2);el('writeAnchors').disabled=result.preview.counts.conflicts>0;}catch(e){el('importResult').textContent=e.message;}}
async function writeImport(){if(!pendingImport)return;el('writeAnchors').disabled=true;try{let result=await anchorRequest(pendingImport);el('importResult').textContent=JSON.stringify(result,null,2);pendingImport=null;}catch(e){el('importResult').textContent=e.message+'. Preview again before writing.';pendingImport=null;}}
</script></html>'''.replace('__CLASSES__',''.join('<option>'+c+'</option>' for c in CHESS_PIECE_YOLO_CLASSES))
