"""Create viewing copies from archived RGB frames; raw JPEG/depth remain authoritative."""
import json,zipfile,cv2,numpy as np,hashlib
from pathlib import Path
OUT=Path(__file__).resolve().parent
m=json.loads((OUT/'raw_evidence/archive_manifest.json').read_text());members={v['path']:v for v in m['members']};archives={};manifest=[]
def read(name):
 part=members[name]['archive_part']
 if part not in archives:archives[part]=zipfile.ZipFile(OUT/'raw_evidence'/part)
 raw=archives[part].read(name);assert hashlib.sha256(raw).hexdigest()==members[name]['sha256'];return raw
for case in json.loads((OUT/'experiment_trials.json').read_text())['trials']:
 folder='data/rgbd_audits/'+Path(case['raw_RGBD_source_directory']).name
 rows=[json.loads(l) for l in read(folder+'/frames.jsonl').decode().splitlines()]
 selected=[v for v in rows if str(v.get('full_log_path','')).endswith('/'+Path(case['source_log']).name)]
 assert selected,'No exact run-associated frames'
 start=selected[0]['rgb_stamp'];path=OUT/'figures'/(case['id']+'_review.mp4');writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),2.,(640,520));assert writer.isOpened()
 for row in selected:
  name=folder+('/%06d.jpg'%row['index']);image=cv2.imdecode(np.frombuffer(read(name),np.uint8),1)
  band=np.full((40,640,3),248,np.uint8);stage=(row.get('tracking')or{}).get('stage','no tracking label')
  caption='%s | frame %d | RGB t=%.2fs | %s'%(case['id'],row['index'],row['rgb_stamp']-start,stage)
  cv2.putText(band,caption,(8,25),cv2.FONT_HERSHEY_SIMPLEX,.43,(25,35,45),1,cv2.LINE_AA);writer.write(np.vstack([image,band]))
 writer.release();check=cv2.VideoCapture(str(path));assert check.isOpened() and int(check.get(cv2.CAP_PROP_FRAME_COUNT))==len(selected);check.release()
 manifest.append({'trial':case['id'],'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'size':path.stat().st_size,'source_folder':folder,'source_frame_indices':[v['index'] for v in selected],'source_elapsed_rgb_seconds':selected[-1]['rgb_stamp']-start,'playback_fps':2.,'operations':['decode original JPEG','append external text band','lossy MP4V viewing-copy encoding'],'warning':'Nominal2fps viewing copy, not exact time resampling. No audio. Use archived JPEG/depth/ROS timestamps for measurements.'})
for z in archives.values():z.close()
(OUT/'figures/review_video_manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps([{k:v for k,v in x.items() if k in ['trial','file','size','source_elapsed_rgb_seconds']} for x in manifest]))
