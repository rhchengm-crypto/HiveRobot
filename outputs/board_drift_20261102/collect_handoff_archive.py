"""Inventory/pack/download experiment evidence only; no motor-control imports.

Archives are ordinary ZIP parts under 45 MiB, independently readable. Source
RGB/depth frames remain untouched on Jetson. No private SSH key is archived.
"""
import argparse,base64,json,shlex,subprocess,zlib,hashlib
from pathlib import Path
OUT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('phase',choices=['inventory','pack','fetch']);args=parser.parse_args()
sessions={name:json.loads((OUT/name).read_text())['output'] for name in ['capture.json','capture_v22.json','capture_v23.json','capture_v24.json']}
key=str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519')
remote=r'''
import json,hashlib,zipfile,time,urllib.request
from pathlib import Path
from datetime import datetime,timezone
root=Path('/home/nvidia/hive_robot/DM_Control_Python');archive=root/'data/handoffs/20261102';archive.mkdir(parents=True,exist_ok=True)
# November2 2026 is PST (UTC-8); explicit UTC bounds also support Jetson Python3.8.
lo=datetime(2026,11,2,8,tzinfo=timezone.utc).timestamp();hi=datetime(2026,11,3,8,tzinfo=timezone.utc).timestamp()
files=[]
for directory in SESSIONS.values():files.extend(p for p in Path(directory).iterdir() if p.is_file())
files.extend(p for p in (root/'data/left_arm_v2_8_full_runs').glob('*.txt') if lo<=p.stat().st_mtime<hi)
files.extend(p for p in (root/'data').glob('*.json') if p.is_file())
files.extend(p for p in (root/'data').glob('left_arm*.jsonl') if p.is_file())
files.extend(p for p in (root/'data/board_validation').glob('*.json') if p.is_file())
files.extend(p for p in (root/'data/code_backups').rglob('*') if p.is_file() and lo<=p.stat().st_mtime<hi)
files.extend(p for p in root.glob('left_arm*.py') if p.is_file())
files.extend(p for p in root.glob('*web_control.py') if p.is_file())
files.extend(p for p in root.glob('DM_CAN.py') if p.is_file())
files=sorted(set(files));basic=[{'path':p.relative_to(root).as_posix(),'size':p.stat().st_size,'mtime_utc':datetime.fromtimestamp(p.stat().st_mtime,timezone.utc).isoformat()} for p in files]
if PHASE=='inventory':
 c=urllib.request.build_opener(urllib.request.ProxyHandler({}));run=json.load(c.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5)).get('run')or{}
 processes=[]
 for proc in Path('/proc').iterdir():
  if not proc.name.isdigit():continue
  try:a=[v.decode() for v in (proc/'cmdline').read_bytes().split(bytes([0])) if v]
  except (OSError,UnicodeError):continue
  if any(Path(v).name in {'left_arm_board_grasp.py','left_arm_v2_8.py','left_arm_v3_1_model_control.py','left_arm_general_move.py','left_arm_v3_0_move_library.py','record_left_arm_rgbd_audit.py'} for v in a[:3]):processes.append({'pid':proc.name,'argv':a})
 result={'motion_issued':False,'captured_at_utc':datetime.now(timezone.utc).isoformat(),'local_experiment_date':'2026-11-02','local_timezone':'America/Los_Angeles','files':basic,'total_bytes':sum(v['size'] for v in basic),'run':{k:run.get(k) for k in ['action','ok','running','returncode','full_log_path']},'motor_or_recorder_processes':processes,'root':str(root),'sessions':SESSIONS}
elif PHASE=='pack':
 assert not (archive/'archive_manifest.json').exists(),'Archive already exists; do not overwrite evidence'
 limit=45*1024*1024;parts=[];members=[];index=0;z=None;current=None
 for p in files:
  relative=p.relative_to(root).as_posix();raw=p.read_bytes();digest=hashlib.sha256(raw).hexdigest()
  if z is None or (z.fp.tell()>limit-p.stat().st_size and z.fp.tell()>0):
   if z:z.close();parts.append({'file':current.name,'size':current.stat().st_size,'sha256':hashlib.sha256(current.read_bytes()).hexdigest()})
   index+=1;current=archive/('experiment_evidence_%03d.zip'%index);z=zipfile.ZipFile(current,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6)
  z.writestr(relative,raw);members.append({'path':relative,'size':len(raw),'sha256':digest,'archive_part':current.name})
 if z:z.close();parts.append({'file':current.name,'size':current.stat().st_size,'sha256':hashlib.sha256(current.read_bytes()).hexdigest()})
 result={'motion_issued':False,'created_utc':datetime.now(timezone.utc).isoformat(),'local_experiment_date':'2026-11-02','source_root':str(root),'source_sessions':SESSIONS,'format':'independent ZIP archives; combine extracted trees to restore dataset','parts':parts,'members':members,'uncompressed_bytes':sum(v['size'] for v in members),'compressed_bytes':sum(v['size'] for v in parts)}
 (archive/'archive_manifest.json').write_text(json.dumps(result,indent=2))
else:result=json.loads((archive/'archive_manifest.json').read_text())
import zlib,base64
print(base64.b64encode(zlib.compress(json.dumps(result).encode(),9)).decode())
'''.replace('SESSIONS',repr(sessions)).replace('PHASE',repr(args.phase))
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-C','-i',key,'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(remote.encode(),9)),capture_output=True,timeout=600 if args.phase=='pack' else 40)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(zlib.decompress(base64.b64decode(r.stdout)))
if args.phase=='inventory':
 (OUT/'final_readonly_inventory.json').write_text(json.dumps(d,indent=2));print(json.dumps({k:v for k,v in d.items() if k not in ['files','sessions']}));print(json.dumps({'file_count':len(d['files']),'largest':sorted(d['files'],key=lambda x:x['size'],reverse=True)[:8]}));raise SystemExit
dest=OUT/'raw_evidence';dest.mkdir(exist_ok=True)
existing_manifest=dest/'archive_manifest.json'
if existing_manifest.exists():
 prior=json.loads(existing_manifest.read_text());by_name={v['file']:v for v in prior['parts']}
 for part in d['parts']:
  old=by_name.get(part['file'])
  if old and old.get('storage_chunks'):
   assert old['sha256']==part['sha256'],'Refuse to replace existing archived evidence with different bytes'
   part['storage_chunks']=old['storage_chunks']
 if prior.get('git_storage'):d['git_storage']=prior['git_storage']
existing_manifest.write_text(json.dumps(d,indent=2))
if args.phase=='fetch':
 for part in d['parts']:
  p=dest/part['file']
  if p.exists() and hashlib.sha256(p.read_bytes()).hexdigest()==part['sha256']:continue
  print(json.dumps({'downloading':part['file'],'bytes':part['size']}),flush=True)
  scp=subprocess.run(['scp','-q','-i',key,'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199:/home/nvidia/hive_robot/DM_Control_Python/data/handoffs/20261102/'+part['file'],part['file']],cwd=dest,capture_output=True,timeout=600)
  if scp.returncode:raise RuntimeError(scp.stderr.decode(errors='replace'))
  assert hashlib.sha256(p.read_bytes()).hexdigest()==part['sha256'],'Download hash mismatch'
  print(json.dumps({'verified':part['file']}),flush=True)
print(json.dumps({'phase':args.phase,'parts':len(d['parts']),'members':len(d['members']),'uncompressed_bytes':d['uncompressed_bytes'],'compressed_bytes':d['compressed_bytes'],'motion_issued':False}))
