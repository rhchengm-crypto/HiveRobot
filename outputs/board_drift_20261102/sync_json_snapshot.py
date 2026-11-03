"""Pull model/learning records as one compressed read-only bundle."""
import subprocess,shlex,json,base64,zlib,hashlib
from pathlib import Path
OUT=Path(__file__).resolve().parent
code=r'''
import json,zlib,base64,hashlib
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python/data')
names=['left_arm_general_dynamics.json','left_arm_board_grasp.json','left_arm_v3_1_dynamics.json','left_arm_v2_6_adaptive_hold_tau.json','left_arm_v2_6_adaptive_active_tau.json','left_arm_v2_6_adaptive_target_bias.json','left_arm_v2_6_adaptive_hold_target_bias.json']
files={n:(root/n).read_text() for n in names if (root/n).exists()}
log=root/'left_arm_v2_8_full_runs/arm_run_1793671770732_5069.txt'
report={'files':files,'full_log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'full_log_size':log.stat().st_size}
print(base64.b64encode(zlib.compress(json.dumps(report).encode(),9)).decode())
'''
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(code)],capture_output=True,timeout=40)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(zlib.decompress(base64.b64decode(r.stdout)));dest=OUT/'jetson_snapshot';dest.mkdir(exist_ok=True)
for n,txt in d.pop('files').items():
 assert Path(n).name==n
 (dest/n).write_text(txt,encoding='utf8')
local=dest/'arm_run_1793671770732_5069.txt'
d['full_log_verified']=hashlib.sha256(local.read_bytes()).hexdigest()==d['full_log_sha256']
d['synced_json_names']=sorted(p.name for p in dest.glob('*.json'))
(OUT/'sync_manifest.json').write_text(json.dumps(d,indent=2));print(json.dumps(d))
