import json,subprocess,shlex,base64,zlib,sys
from pathlib import Path
version=sys.argv[1];assert version in ('v21_restored',)
out=Path(__file__).parent;paths=[]
for phase in ('home_motion','home_complete','preview_complete','status'):
 d=json.loads((out/(version+'_full_test_'+phase+'.json')).read_text());paths.append(d['run']['full_log_path'])
code="import json,zlib,base64;from pathlib import Path;print(base64.b64encode(zlib.compress(json.dumps({p:Path(p).read_text(errors='replace') for p in "+repr(paths)+"}).encode())).decode())"
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(code)],capture_output=True,timeout=30)
if r.returncode:raise RuntimeError(r.stderr.decode('utf8',errors='replace'))
logs=json.loads(zlib.decompress(base64.b64decode(r.stdout)))
for name,s in logs.items():(out.parents[1]/'scripts/data/left_arm_v2_8_full_runs'/Path(name).name).write_text(s,encoding='utf8')
s=logs[paths[-1]];d=json.JSONDecoder()
def record(prefix):return d.raw_decode(s.rsplit(prefix,1)[1].lstrip())[0]
e=record('board endpoint report= ');h=record('board same-session hold complete= ');entry=record('board feedback descent entry= ')
a=[json.loads(l.split('= ',1)[1]) for l in s.splitlines() if l.startswith('board tracking audit= ')]
stages={}
for label in dict.fromkeys(v['stage'] for v in a):
 aa=[v for v in a if v['stage']==label];n='shoulder_rotate'
 stages[label]={'max_rotate_error_deg':max(abs(v['error_deg'][n]) for v in aa),'kp_min':min(v['last_sent_commands'][n]['kp'] for v in aa),'kp_max':max(v['last_sent_commands'][n]['kp'] for v in aa),'command_q_range_rad':[min(v['last_sent_commands'][n]['q'] for v in aa),max(v['last_sent_commands'][n]['q'] for v in aa)],'actual_q_range_rad':[min(v['feedback'][n]['pos'] for v in aa),max(v['feedback'][n]['pos'] for v in aa)]}
report={'ok':True,'physical_validation_pending':True,'logs_synced':list(logs),'entry':entry,'stages':stages,'endpoint':e,'hold':{k:h.get(k) for k in ('ok','reason','final_error_deg','max_abs_error_deg','serial_session_reused','motion_gain_floor')},'handoff':record('board same-session hold begin= ')}
(out/(version+'_full_test_report.json')).write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps({'logs_synced':len(logs),'stages':stages,'redundancy_policy':entry.get('redundancy_policy'),'fixed_encoder_rad':entry.get('fixed_encoder_rad'),'fallback_reason':entry.get('fixed_swivel_infeasible_reason'),'hold_reason':h.get('reason'),'hold_rotate_command':h.get('motion_gain_floor',{}).get('shoulder_rotate'),'handoff':report['handoff']}))
