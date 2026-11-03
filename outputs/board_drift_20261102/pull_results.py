"""Download this completed run and current learned JSONs without overwriting source."""
import json,subprocess,hashlib
from pathlib import Path
OUT=Path(__file__).resolve().parent
r=json.loads((OUT/'status_latest.json').read_text())['run'];assert r.get('running') is False and r.get('returncode')==0
root='/home/nvidia/hive_robot/DM_Control_Python/'
log=r['full_log_path'];assert log.startswith(root+'data/left_arm_v2_8_full_runs/') and log.endswith('.txt')
dest=OUT/'jetson_snapshot';dest.mkdir(exist_ok=True)
files=[log]+[root+'data/'+n for n in ('left_arm_v3_1_dynamics.json','left_arm_general_dynamics.json','left_arm_board_grasp.json','left_arm_v2_8_adaptive.json')]
done=[]
for source in files:
 p=dest/Path(source).name
 cmd=['scp','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199:'+source,str(p)]
 result=subprocess.run(cmd,capture_output=True,timeout=40)
 if result.returncode:
  done.append({'source':source,'error':result.stderr.decode(errors='replace')});continue
 done.append({'source':source,'local':str(p),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(OUT/'sync_manifest.json').write_text(json.dumps(done,indent=2));print(json.dumps(done))
