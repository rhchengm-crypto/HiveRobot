"""Compose original camera frames; no generated content or geometric correction."""
import cv2,json,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;rows=[];sources=[]
for version,folder,label in [('V22','rgbd_v22','~10 mm B; no improvement'),('V23','rgbd_v23','~5 mm B; fingertip contact'),('V24','rgbd_v24','~2 mm B; visible gap; accepted')]:
 d=OUT/folder;s=json.loads((d/'selection.json').read_text());indices=[s['contact_min'],s['contact_max']];images=[]
 for index in indices:
  im=cv2.imdecode(np.fromfile(d/('%06d.jpg'%index),np.uint8),1);assert im is not None;images.append(im)
 band=np.full((62,1280,3),248,np.uint8)
 cv2.putText(band,version+' | '+label,(16,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(30,40,50),1,cv2.LINE_AA)
 cv2.putText(band,'Window start: frame '+str(indices[0]),(16,51),cv2.FONT_HERSHEY_SIMPLEX,.5,(60,60,60),1,cv2.LINE_AA)
 cv2.putText(band,'Window end: frame '+str(indices[1]),(656,51),cv2.FONT_HERSHEY_SIMPLEX,.5,(60,60,60),1,cv2.LINE_AA)
 rows.append(np.vstack([band,np.hstack(images)]));sources.append({'version':version,'source_folder':s['source_folder'],'frames':indices,'window':'recorded contact and endpoint-settle window, selected using recorder tracking metadata'})
figure=np.vstack(rows);dest=OUT/'figures';dest.mkdir(exist_ok=True);cv2.imencode('.png',figure)[1].tofile(dest/'raw_frame_comparison.png')
(dest/'raw_frame_comparison_sources.json').write_text(json.dumps({'sources':sources,'operations':['decode original JPEG','add separate label bands','concatenate unchanged camera pixels'],'limitations':['Images have perspective and moving-camera-object projection effects; do not infer metric lateral drift directly.','Window may include endpoint settle; physical drift labels are operator reports.','Configurations differ in absolute height; this is not a matched-height comparison.']},indent=2));print('Saved original-frame comparison and provenance')
