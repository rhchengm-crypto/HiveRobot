import json,base64,zlib,shlex,subprocess,hashlib
from pathlib import Path
out=Path(__file__).parent;root=out.parents[1]
d=json.loads((out/'board_v16_deploy_result.json').read_text());backup=d['backup']
code="import json,base64;from pathlib import Path;root=Path("+repr(backup)+");print(json.dumps({n:base64.b64encode((root/n).read_bytes()).decode() for n in ['left_arm_board_grasp.py','left_arm_board_kinematics.py']}))"
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(code)],capture_output=True,timeout=25)
if r.returncode:raise RuntimeError(r.stderr.decode('utf8',errors='replace'))
payload=json.loads(r.stdout);expected={'left_arm_board_grasp.py':'1229c75eab61597144b3daac86f5faffc5aeee8464ed16cc65ed6e63ac3a0174','left_arm_board_kinematics.py':'1784df490b4b7a1db68ccd369350932ef9621cac03944f93974af32653cefebf'}
for n,data in payload.items():
 b=base64.b64decode(data);assert hashlib.sha256(b).hexdigest()==expected[n];compile(b,n,'exec')
 (out/('withdrawn_v16_'+n)).write_bytes((root/'scripts'/n).read_bytes())
 (root/'scripts'/n).write_bytes(b)
print(json.dumps({'restored_local':list(payload),'motion_issued':False}))
