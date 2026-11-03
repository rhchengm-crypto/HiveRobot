"""Read software/hardware environment; no driver setup or motor access."""
import json,platform,sys,subprocess,shlex
from pathlib import Path
import numpy,cv2
OUT=Path(__file__).resolve().parent
code=r'''
import json,platform,sys,numpy,cv2
from pathlib import Path
def read(p):
 try:return Path(p).read_bytes().decode(errors='replace').strip('\x00\n ')
 except OSError:return None
print(json.dumps({'python':sys.version,'platform':platform.platform(),'numpy':numpy.__version__,'opencv':cv2.__version__,'device_tree_model':read('/proc/device-tree/model'),'L4T_release':read('/etc/nv_tegra_release'),'OS_release':read('/etc/os-release'),'scope':'read only; does not verify motor torque units, camera metrology, fanless hardware configuration, or physical safety'}))
'''
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(code)],capture_output=True,timeout=25)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
record={'analysis_host':{'python':sys.version,'platform':platform.platform(),'numpy':numpy.__version__,'opencv':cv2.__version__},'jetson':json.loads(r.stdout),'camera':json.loads((OUT/'rgbd_v24/camera.json').read_text())}
(OUT/'environment_record.json').write_text(json.dumps(record,indent=2));print(json.dumps({'saved':True,'Jetson_model':record['jetson']['device_tree_model'],'Jetson_python':record['jetson']['python'].split()[0]}))
