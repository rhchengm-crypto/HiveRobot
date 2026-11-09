"""Freeze the completed A8_take reference-bound trial; no motor operations."""
from pathlib import Path
import hashlib,json,os,re,signal,time,urllib.request,zipfile

ROOT=Path('/home/nvidia/hive_robot/DM_Control_Python').resolve()
CONTEXT=ROOT/'data/board_workspace_trials/a8_reference_execution_20261108'
LEARNING=['left_arm_v2_8_clearance_bias.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_6_adaptive_hold_target_bias.json']
SOURCES=['left_arm_board_grasp.py','left_arm_board_task_pose.py','left_arm_trajectory_support.py',
 'left_arm_board_collision.py','left_arm_board_kinematics.py','left_arm_board_transit.py',
 'left_arm_v2_6.py','left_arm_v2_8.py','left_arm_general_motion.py','left_arm_rigid_body.py',
 'left_arm_v3_1_motion_dynamics.py','left_arm_v3_1_model_control.py','left_arm_fault_support.py',
 'left_arm_board_home_handoff.py','left_arm_motion_trace.py','chessboard_vision_v3_0_web_control.py',
 'record_left_arm_rgbd_audit.py']
MODELS=['left_arm_general_dynamics.json','left_arm_board_workspace_candidate.json',
 'left_arm_v3_1_dynamics.json','left_arm_v2_home.json','left_arm_v2_table_clearance.json',
 'left_arm_validated_path_library.json','left_arm_v2_6_saved_moves.json']

def digest(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()

def project_path(value):
 p=Path(value);p=(p if p.is_absolute() else ROOT/p).resolve()
 assert p==ROOT or ROOT in p.parents,'Evidence path escapes project root'
 return p

def run_of(document):
 return document.get('run') if isinstance(document.get('run'),dict) else document

def chain(run):
 seen=set()
 while isinstance(run,dict) and id(run) not in seen:
  seen.add(id(run));yield run;run=run.get('preceding_motion')

def completed_phase(document,action):
 current=run_of(document)
 assert current.get('ok') and current.get('returncode')==0,action+' did not succeed'
 assert not current.get('running') and not current.get('support_running'),action+' still active'
 matches=[r for r in chain(current) if r.get('action')==action]
 assert len(matches)==1,'Phase identity absent or ambiguous: '+action
 primary=matches[0]
 assert primary.get('ok') and primary.get('returncode')==0
 assert not primary.get('running') and not primary.get('support_running')
 hold=current.get('post_move_model_hold') or {}
 assert hold.get('state')=='complete','Post-motion hold has not completed: '+action
 return current,primary

def matching_launch(primary,path):
 if not path.exists():return False
 launch=run_of(json.loads(path.read_text()))
 actual,submitted=primary.get('command'),launch.get('command')
 async_inline=(primary.get('action')=='dynamics:board-grasp' and isinstance(submitted,list)
     and '--inline-model-hold' not in submitted and actual==submitted+['--inline-model-hold'])
 assert actual==submitted or async_inline,'Command changed from '+path.name
 assert primary.get('started_at')==launch.get('started_at'),'Start identity differs from '+path.name
 if launch.get('full_log_path'):assert primary.get('full_log_path')==launch['full_log_path']
 return True

def stop_camera(recording):
 media=project_path(recording['recording_folder'])
 assert media.parent==(ROOT/'data/rgbd_audits').resolve() and media.name.startswith('a8_reference_20261108_')
 pid=int(recording['recorder_pid']);assert pid>1 and pid!=os.getpid()
 proc=Path('/proc')/str(pid)
 try:
  if (proc/'stat').read_text().rsplit(')',1)[1].split()[0]=='Z':return media,{'pid':pid,'signal_sent':False,'state':'already-exited'}
  argv=[x.decode(errors='strict') for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
 except FileNotFoundError:return media,{'pid':pid,'signal_sent':False,'state':'already-stopped'}
 assert any(Path(v).name=='record_left_arm_rgbd_audit.py' for v in argv[:4]),'PID differs; no signal sent'
 assert '--output' in argv and argv.index('--output')+1<len(argv)
 assert Path(argv[argv.index('--output')+1]).resolve()==media,'Recorder output differs; no signal sent'
 os.kill(pid,signal.SIGINT)
 for _ in range(60):
  try:
   if (proc/'stat').read_text().rsplit(')',1)[1].split()[0]=='Z':return media,{'pid':pid,'signal_sent':True,'state':'matching-recorder-exited'}
  except FileNotFoundError:return media,{'pid':pid,'signal_sent':True,'state':'matching-recorder-stopped'}
  time.sleep(.2)
 raise RuntimeError('Camera recorder did not exit; no other signal sent')

def workflow_identities(text):
 decoder=json.JSONDecoder();pairs=set()
 for line in text.splitlines():
  if 'workflow_run_id' not in line or 'workflow_action' not in line:continue
  start=line.find('{')
  if start<0:continue
  try:row,_=decoder.raw_decode(line[start:])
  except ValueError:continue
  if isinstance(row,dict) and row.get('workflow_action') and row.get('workflow_run_id'):
   pairs.add((row['workflow_action'],row['workflow_run_id']))
 return pairs

def main():
 read=lambda p:json.loads(Path(p).read_text())
 prep=read(CONTEXT/'preparation.json');verified=read(CONTEXT/'verified_plan.json')
 stages={}
 for filename,action in [('home_complete_status.json','home'),('clearance_complete_status.json','table-clearance'),('execution_complete_status.json','dynamics:board-grasp')]:
  stages[action]=completed_phase(read(CONTEXT/filename),action)
 assert matching_launch(stages['home'][1],CONTEXT/'home_start.json')
 matching_launch(stages['table-clearance'][1],CONTEXT/'clearance_start.json')
 for candidate in ('execution_start.json','execute_start.json'):
  if matching_launch(stages['dynamics:board-grasp'][1],CONTEXT/candidate):break
 board_current,board_primary=stages['dynamics:board-grasp']
 command=board_primary['command'];assert '--execute' in command and '--prepared-plan' in command
 prepared=project_path(command[command.index('--prepared-plan')+1])
 assert prepared==project_path(verified['prepared_plan_path']) and digest(prepared)==verified['plan_sha256']
 client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
 def status():
  with client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=5) as response:return json.load(response)
 def require_same_final(s):
  r,p=completed_phase(s,'dynamics:board-grasp')
  assert p.get('command')==board_primary.get('command') and p.get('started_at')==board_primary.get('started_at')
  assert p.get('full_log_path')==board_primary.get('full_log_path')
 require_same_final(status())
 media,recorder=stop_camera(read(CONTEXT/'recording.json'))
 final=status();require_same_final(final)
 dest=ROOT/'data/experiment_archives'/('a8_reference_execution_20261108_'+str(time.time_ns()));dest.mkdir(parents=True)
 files={};missing=[]
 def add(path,logical=None,required=False):
  p=project_path(path)
  if not p.is_file():
   if required:raise FileNotFoundError(str(p))
   missing.append(str(p));return
  name=logical or p.relative_to(ROOT).as_posix()
  assert not Path(name).is_absolute() and '..' not in Path(name).parts
  target=dest/'snapshot'/name;target.parent.mkdir(parents=True,exist_ok=True)
  target.write_bytes(p.read_bytes());files[name]=target
 # Complete context includes original preparation, model before activation,
 # runtime_snapshot, prepared plan and immediately pre-Home learning copies.
 for p in sorted(CONTEXT.rglob('*')):
  if p.is_file():add(p)
 texts=[];identities=set();phase_summary={}
 for action,(current,primary) in stages.items():
  phase_summary[action]={'primary_log':primary['full_log_path'],'post_hold_log':current['full_log_path'],
   'started_at':primary.get('started_at'),'finished_at':current.get('finished_at'),
   'returncode':current['returncode'],'post_move_model_hold':current.get('post_move_model_hold')}
  for run in chain(current):
   if run.get('full_log_path'):
    add(run['full_log_path'],required=True)
    text=project_path(run['full_log_path']).read_text(errors='replace');texts.append(text);identities.update(workflow_identities(text))
   cmd=run.get('command') or []
   for flag in ('--plan-output','--prepared-plan'):
    if flag in cmd:add(cmd[cmd.index(flag)+1],required=True)
 add(prepared,required=True)
 traces=sorted(set(re.findall(r'trace_\d+\.jsonl','\n'.join(texts))))
 for name in traces:add(ROOT/'data/board_motion_traces'/name,required=True)
 if not traces:missing.append('No continuous board motor trace referenced by completed logs')
 for name in sorted(set(re.findall(r'approach_[a-f0-9]{16}\.json','\n'.join(texts)))):
  add(ROOT/'data/board_workspace_trials'/name)
 for name in SOURCES:add(ROOT/name,'source/'+name,required=True)
 for name in MODELS:add(ROOT/'data'/name,'models/'+name,required=True)
 before_hashes=prep.get('source_hashes_before') or {};collection_hashes={}
 for name,expected in before_hashes.items():
  p=project_path(name);logical=('models/' if name.startswith('data/') else 'source/')+p.name
  add(p,logical,required=True);collection_hashes[name]={'before_activation_sha256':expected,'collection_sha256':digest(p),'same_as_before_activation':digest(p)==expected}
 learning={};preflight=read(CONTEXT/'preflight.json')
 for name in LEARNING:
  before=CONTEXT/('before_'+name);after=ROOT/'data'/name
  add(before,'learning_before/'+name);add(after,'learning_after/'+name)
  row={'before_available':before.is_file(),'after_available':after.is_file()}
  if before.is_file():
   row['before_sha256']=digest(before);expected=(preflight.get('learning_hashes') or {}).get(name)
   row['before_matches_preflight_sha256']=expected==row['before_sha256'] if expected else None
  if after.is_file():row['after_sha256']=digest(after)
  if before.is_file() and after.is_file():row['bytes_changed']=row['before_sha256']!=row['after_sha256']
  learning[name]=row
 # Copy only exact workflow/run-id rows, not the historical rolling log.
 filtered=dest/'general_motion_records.jsonl';matches=[]
 rolling=ROOT/'data/left_arm_general_motion_runs.jsonl'
 with filtered.open('wb') as output:
  if rolling.is_file():
   with rolling.open('rb') as stream:
    for index,line in enumerate(stream,1):
     try:row=json.loads(line)
     except ValueError:continue
     if (row.get('workflow_action'),row.get('workflow_run_id')) in identities:
      output.write(line);matches.append({'source_line':index,'workflow_action':row['workflow_action'],'workflow_run_id':row['workflow_run_id'],'frames':row.get('frames'),'sha256':hashlib.sha256(line).hexdigest()})
  else:missing.append(str(rolling))
 if not matches:missing.append('No scoped general-motion JSONL records found for log-bound workflow identities')
 add(filtered,'general_motion_records.jsonl',required=True)
 summary={'motion_issued_by_collector':False,'arm_process_not_interrupted':True,
  'reference_endpoint':'A8_take','target_kind':'FK XYZ of user-approved physical endpoint',
  'target_board_mm':[prep['request'][key] for key in ('x_mm','y_mm','grip_height_mm')],
  'target_board_mm_rounded':[11.798,405.601,84.623],
  'original_nominal_A8_XYZ_solved':False,'empty_load':True,'claw_closed':False,
  'phases':phase_summary,'continuous_traces':traces,'workflow_identities':[list(v) for v in sorted(identities)],
  'general_motion_records':matches,'learning_snapshots':learning,'source_hash_comparison':collection_hashes,
  'monitoring_limitations':'Normal Home/Clearance lack per-frame actual feedback and an automatic feedback-oscillation detector; general-motion reference samples are commands. Preserve separate operator observation. Missing camera frames are not synthesized.',
  'operator_feedback_context_included':(CONTEXT/'operator_feedback.json').exists()}
 for name,data in [('web_status.json',final),('result_summary.json',summary)]:
  p=dest/name;p.write_text(json.dumps(data,ensure_ascii=False,indent=2));add(p,name,required=True)
 report={'motion_issued_by_collector':False,'arm_process_not_interrupted':True,'run_identity_verified':True,
  'archive_directory':str(dest),'reference_endpoint':'A8_take','target_board_mm':summary['target_board_mm'],
  'original_nominal_A8_XYZ_solved':False,'phase_logs':phase_summary,'recorder':recorder,
  'continuous_traces':traces,'general_motion_records':len(matches),'missing_optional_items':missing}
 media_files=sorted(p for p in media.rglob('*') if p.is_file()) if media.is_dir() else []
 rgb={p.stem for p in media_files if p.suffix.lower()=='.jpg'};depth={p.stem for p in media_files if p.suffix.lower()=='.npy'}
 report.update(rgb_frames=len(rgb),depth_frames=len(depth),rgb_depth_pairs=len(rgb&depth),rgbd_pairing_complete=bool(rgb) and rgb==depth)
 if not media_files:missing.append('RGB-D media directory has no files; archive manifest is empty')
 frame_log=media/'frames.jsonl'
 report['frame_log_rows']=sum(bool(line.strip()) for line in frame_log.read_text().splitlines()) if frame_log.is_file() else None
 if rgb!=depth:missing.append('RGB and depth frame indices differ; incomplete pairs preserved without synthesis')
 if report['frame_log_rows']!=len(rgb&depth):missing.append('Frame journal row count does not equal paired frame count')
 for label,entries in [('control',files),('rgbd',{p.relative_to(media).as_posix():p for p in media_files})]:
  manifest={};archive=dest/(label+'_evidence.zip')
  with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=3) as z:
   for name,p in sorted(entries.items()):
    data=p.read_bytes();manifest[name]={'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()};z.writestr(name,data)
   z.writestr('manifest.json',json.dumps(manifest,indent=2))
  (dest/(label+'_manifest.json')).write_text(json.dumps(manifest,indent=2))
  with zipfile.ZipFile(archive) as z:assert z.testzip() is None,'ZIP CRC failure'
  report[label]={'path':str(archive),'bytes':archive.stat().st_size,'sha256':digest(archive),'zip_crc_verified':True}
 (dest/'collection_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))

if __name__=='__main__':main()
