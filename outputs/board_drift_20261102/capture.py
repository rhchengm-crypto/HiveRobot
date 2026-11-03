"""Bounded read-only RGB-D collection for this session; no motor commands."""
import subprocess,shlex,json,sys,base64,io,zipfile
from pathlib import Path
OUT=Path(__file__).resolve().parent
phase=sys.argv[1];assert phase in ('start','first','collect')
meta=json.loads((OUT/'capture.json').read_text()) if phase!='start' else {}
code=r'''
import subprocess,shlex,time,shutil,json,os,signal,io,zipfile,base64
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
if PHASE=='start':
 assert shutil.disk_usage(str(root)).free>2_000_000_000
 for proc in Path('/proc').iterdir():
  if not proc.name.isdigit():continue
  try:a=[v.decode() for v in (proc/'cmdline').read_bytes().split(bytes([0])) if v]
  except (OSError,UnicodeError):continue
  assert 'record_left_arm_rgbd_audit.py' not in a,'Existing recorder'
 folder=root/'data/rgbd_audits'/('drift_20261102_'+str(time.time_ns()));folder.parent.mkdir(exist_ok=True)
 log=folder.parent/(folder.name+'.log')
 cmd='source /opt/ros/noetic/setup.bash && source /home/nvidia/ascam_ws/devel/setup.bash && exec python3 -u record_left_arm_rgbd_audit.py --seconds 600 --fps 2 --output '+shlex.quote(str(folder))
 with log.open('wb') as f:p=subprocess.Popen(['/bin/bash','-c',cmd],cwd=str(root),stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 time.sleep(3);assert p.poll() is None,log.read_text()
 print(json.dumps({'pid':p.pid,'output':str(folder),'log':str(log),'motion_issued':False}))
else:
 folder=Path(META['output'])
 if PHASE=='first':
  images=sorted(folder.glob('*.jpg'));assert images,'Camera has produced no paired frames'
  print(base64.b64encode(images[-1].read_bytes()).decode())
 else:
  proc=Path('/proc')/str(META['pid'])
  if proc.exists():
   a=[v.decode() for v in (proc/'cmdline').read_bytes().split(bytes([0])) if v]
   assert 'record_left_arm_rgbd_audit.py' in a and str(folder) in a
   os.kill(META['pid'],signal.SIGTERM);time.sleep(1)
  rows=[json.loads(x) for x in (folder/'frames.jsonl').read_text().splitlines()]
  contact=[x['index'] for x in rows if x.get('running') is True and (x.get('tracking') or {}).get('stage') in ('contact','endpoint_settle')]
  b=io.BytesIO()
  with zipfile.ZipFile(b,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as z:
   for name in ('frames.jsonl','camera.json'):z.write(str(folder/name),name)
   if contact:
    lo=max(0,min(contact)-3);hi=min(len(rows)-1,max(contact)+10)
    for i in sorted(set([lo,min(contact),max(contact),hi])):
     for ext in ('.jpg',):
      p=folder/('%06d'%i+ext)
      if p.exists():z.write(str(p),p.name)
    z.writestr('selection.json',json.dumps({'contact_min':min(contact),'contact_max':max(contact),'first_included':lo,'last_included':hi,'source_folder':str(folder)}))
  print(base64.b64encode(b.getvalue()).decode())
'''.replace('PHASE',repr(phase)).replace('META',repr(meta))
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=1','nvidia@192.168.201.199','python3 -c '+shlex.quote(code)],capture_output=True,timeout=55)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
if phase=='start':
 (OUT/'capture.json').write_bytes(r.stdout);print(r.stdout.decode())
elif phase=='first':
 (OUT/'camera_before_motion.jpg').write_bytes(base64.b64decode(r.stdout));print('Saved camera_before_motion.jpg')
else:
 data=base64.b64decode(r.stdout);folder=OUT/'rgbd';folder.mkdir(exist_ok=True)
 with zipfile.ZipFile(io.BytesIO(data)) as z:
  for name in z.namelist():
   assert Path(name).name==name
   (folder/name).write_bytes(z.read(name))
 print(json.dumps({'bytes_transferred':len(data),'images':len(list(folder.glob('*.jpg'))),'depth_frames':len(list(folder.glob('*.npy')))}))
