"""Paired RGB/depth samples with ruler-measured height labels (all 12 classes).

No depth-to-height shortcut: camera intrinsics, registration and a metric
board plane are needed before estimating perpendicular height from depth.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import threading
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

from chessboard_vision_v2_7 import CHESS_PIECE_YOLO_CLASSES, normalize_square

LOCK = threading.Lock()


def height_dataset_dir(yolo_dataset_dir):
    return Path(yolo_dataset_dir).parent / "chess_piece_heights"


def validate_pair(capture, now=None):
    now = time.time() if now is None else now
    if capture['rgb'] is None or capture['depth'] is None:
        raise ValueError('RGB/depth is not ready. Check the camera.')
    for kind in ('rgb', 'depth'):
        age = now - capture[kind+'_received_at']
        if not math.isfinite(age) or age < -.1 or age > 2:
            raise ValueError('The frame has not updated for more than 2 seconds. Try again.')
    rgb_stamp = capture['rgb_metadata'].get('source_stamp_s')
    depth_stamp = capture['depth_metadata'].get('source_stamp_s')
    if not all(isinstance(s, (int,float)) and math.isfinite(s) and s > 0 for s in (rgb_stamp, depth_stamp)):
        raise ValueError('ROS image timestamps are missing, so RGB/depth synchronization cannot be verified.')
    delta = abs(rgb_stamp-depth_stamp)
    if delta > .1:
        raise ValueError(f'RGB/depth time difference: {delta:.3f} seconds, exceeding 0.1 seconds. Keep the piece still and try again.')
    depth = capture['depth']
    encoding = capture['depth_metadata'].get('encoding','').lower()
    if depth.ndim != 2 or not ((encoding=='16uc1' and depth.dtype==np.uint16) or (encoding=='32fc1' and depth.dtype==np.float32)):
        raise ValueError('Height sampling only accepts ROS 16UC1 (mm) or 32FC1 (m) depth images with known units.')
    valid = np.isfinite(depth) & (depth > 0)
    if not valid.any():
        raise ValueError('The depth image contains no valid depth. Nothing was saved.')
    return {'source_timestamp_delta_s':delta,'pairing':'latest_frames_with_source_timestamp_check',
            'max_allowed_delta_s':.1,'depth_valid_fraction':float(valid.mean()),
            'depth_unit':'mm' if encoding=='16uc1' else 'm',
            'depth_scale_to_m':.001 if encoding=='16uc1' else 1.0}


def save_height_sample(root, capture, payload, calibration):
    piece_class = payload.get('piece_class')
    if piece_class not in CHESS_PIECE_YOLO_CLASSES:
        raise ValueError('Select a valid white or black piece class.')
    square = normalize_square(str(payload.get('square','')))
    raw_height = payload.get('measured_height_mm')
    height = None if raw_height in (None,'') else float(raw_height)
    if height is not None and (not math.isfinite(height) or not 1 <= height <= 250):
        raise ValueError('Enter the measured physical height (1-250 mm, from the base bottom to the highest point).')
    split = payload.get('split','train')
    if split not in ('train','val','test'):
        raise ValueError('Split must be train, val, or test.')
    orientation = str(payload.get('orientation',''))
    if orientation not in ('front','back','left','right','other'):
        raise ValueError('Select the piece orientation.')
    piece_id = str(payload.get('piece_id','')).strip()
    notes = str(payload.get('notes','')).strip()
    if len(piece_id)>80 or len(notes)>1000:
        raise ValueError('The piece ID or notes are too long.')
    if np.asarray(calibration.get('homography_board_to_image')).shape != (3,3):
        raise ValueError('Board calibration is missing. Calibrate the grid with Inspect Square on the main page first.')
    quality = validate_pair(capture)
    from chess_piece_height_model import calibration_path, estimate_heights
    observation={'ok':False,'reason':'height_calibration_missing'}
    metric_id=None
    try:
        metric=json.loads(calibration_path(root).read_text(encoding='utf-8'))
        metric_id=metric['id']
        observation=estimate_heights(capture['depth'],capture['depth_metadata']['encoding'],metric,[square])[square]
    except (OSError,ValueError,KeyError,TypeError,np.linalg.LinAlgError) as exc:
        observation={'ok':False,'reason':str(exc)}
    # The same received pair cannot silently inflate the dataset, even when
    # the caller changes its class/split. New acquisitions get new source stamps.
    stamp_key = {k:capture[k+'_metadata'] for k in ('rgb','depth')}
    fingerprint = hashlib.sha256(json.dumps(stamp_key,sort_keys=True).encode()+capture['rgb'].tobytes()+capture['depth'].tobytes()).hexdigest()[:24]
    root = Path(root)
    sample_id = 'height_'+fingerprint
    target = root/'samples'/sample_id
    with LOCK:
        if target.exists():
            raise ValueError('This frame pair has already been saved. Change the position or orientation before capturing again.')
        target.parent.mkdir(parents=True,exist_ok=True)
        draft = target.parent/('.pending_'+uuid.uuid4().hex)
        draft.mkdir()
        try:
            if not cv2.imwrite(str(draft/'rgb.png'),capture['rgb']):
                raise ValueError('Failed to save the RGB image.')
            np.save(str(draft/'depth.npy'),capture['depth'],allow_pickle=False)
            metadata = {'schema_version':1,'sample_id':sample_id,'created_at_unix_s':time.time(),
                        'piece_class':piece_class,'class_id':CHESS_PIECE_YOLO_CLASSES.index(piece_class),
                        'piece_id':piece_id,'square':square,'orientation':orientation,'split':split,'notes':notes,
                        'measured_height_mm':height,'height_label_source':'ruler_manual' if height is not None else 'class_only',
                        'estimated_height_mm':observation.get('estimated_height_mm'),'metric_height_ready':observation['ok'],
                        'height_observation':observation,'metric_calibration_id':metric_id,
                        'rgb_depth_registration':'unverified','camera_intrinsics':capture.get('camera_info') or None,
                        'board_plane_3d':None,'calibration':calibration,
                        'quality':quality,'rgb_shape':list(capture['rgb'].shape),
                        'depth_shape':list(capture['depth'].shape),'depth_dtype':str(capture['depth'].dtype),
                        'rgb_metadata':capture['rgb_metadata'],'depth_metadata':capture['depth_metadata'],
                        'rgb_received_at':capture['rgb_received_at'],'depth_received_at':capture['depth_received_at'],
                        'rgb_seq':capture['rgb_seq'],'depth_seq':capture['depth_seq'],
                        'rgb_file':'rgb.png','depth_file':'depth.npy'}
            (draft/'sample.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
            os.rename(draft,target)
        finally:
            if draft.exists():
                if draft.resolve().parent != (root/'samples').resolve() or not draft.name.startswith('.pending_'):
                    raise RuntimeError('Invalid temporary sample cleanup path')
                shutil.rmtree(draft)
    return {'ok':True,'sample_id':sample_id,'sample_dir':str(target),
            'measured_height_mm':height,'quality':quality,'estimated_height_mm':observation.get('estimated_height_mm'),'height_observation':observation}


def height_sample_stats(root):
    classes = {c:{'train':0,'val':0,'test':0,'total':0,'min_height_mm':None,'max_height_mm':None} for c in CHESS_PIECE_YOLO_CLASSES}
    errors=[]
    for path in sorted((Path(root)/'samples').glob('height_*/sample.json')):
        try:
            data=json.loads(path.read_text(encoding='utf-8'))
            item=classes[data['piece_class']]
            height=None if data.get('measured_height_mm') is None else float(data['measured_height_mm'])
            if (height is not None and not math.isfinite(height)) or data['split'] not in ('train','val','test'):
                raise ValueError('invalid sample metadata')
            item[data['split']]+=1;item['total']+=1
            if height is not None:
                item['min_height_mm']=height if item['min_height_mm'] is None else min(height,item['min_height_mm'])
                item['max_height_mm']=height if item['max_height_mm'] is None else max(height,item['max_height_mm'])
        except (KeyError,ValueError,OSError,TypeError) as exc:
            errors.append({'file':str(path),'error':str(exc)})
    return {'ok':True,'dataset_dir':str(root),'classes':classes,'total':sum(c['total'] for c in classes.values()),'errors':errors}
