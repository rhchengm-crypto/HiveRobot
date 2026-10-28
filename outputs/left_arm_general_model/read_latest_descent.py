import json, subprocess, shlex
from pathlib import Path
code = r'''
import json,sys
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
sys.path.insert(0,str(root))
files=sorted((root/'data/left_arm_v2_8_full_runs').glob('*.txt'),key=lambda p:p.stat().st_mtime,reverse=True)
found=[]
for p in files[:30]:
 s=p.read_text(errors='replace')
 if 'board endpoint report=' in s or ('motion_gain_floor' in s and 'duration_complete' in s):
  found.append((p,s))
 if len(found)>=2:break
out=[]
decoder=json.JSONDecoder()
for p,s in found:
 row={'path':str(p),'tail':s[-2000:]}
 if 'board same-session hold complete= ' in s:
  held=decoder.raw_decode(s.rsplit('board same-session hold complete= ',1)[1].lstrip())[0]
  endpoint=decoder.raw_decode(s.rsplit('board endpoint report=',1)[1].lstrip())[0]
  begin=decoder.raw_decode(s.rsplit('board same-session hold begin= ',1)[1].lstrip())[0]
  out.extend([{'path':str(p),'hold':held,'handoff':begin,'serial_open_messages':s.count('Serial port is open'),'tail':s[-2000:]},
              {'path':str(p),'endpoint':endpoint,'tail':s[-2000:]}])
  break
 if 'board endpoint report=' in s:
  row['endpoint']=decoder.raw_decode(s.rsplit('board endpoint report=',1)[1].lstrip())[0]
 else:
  for i,c in enumerate(s):
   if c=='{':
    try: obj=decoder.raw_decode(s[i:])[0]
    except ValueError: continue
    if obj.get('reason')=='duration_complete' and 'final' in obj: row['hold']=obj;break
 out.append(row)
print(json.dumps(out))
'''
cmd=['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(code)]
r=subprocess.run(cmd,capture_output=True)
if r.returncode: raise RuntimeError(r.stderr.decode('utf8',errors='replace'))
data=json.loads(r.stdout)
Path(__file__).with_name('latest_descent_logs.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(data,ensure_ascii=False))
