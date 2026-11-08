import base64,hashlib,json,urllib.request
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
with client.open('http://127.0.0.1:8100/api/v3/dynamics/status',timeout=8) as response:status=json.load(response)
logs=[]
for p in sorted((root/'data/left_arm_v2_8_full_runs').glob('arm_run_*.txt')):
    try:epoch=int(p.name.split('_')[2])
    except (ValueError,IndexError):continue
    if epoch<1794113244000:continue
    raw=p.read_bytes()
    logs.append(dict(path=str(p),bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),base64=base64.b64encode(raw).decode()))
print(json.dumps(dict(motion_issued=False,status=status,logs=logs)))
