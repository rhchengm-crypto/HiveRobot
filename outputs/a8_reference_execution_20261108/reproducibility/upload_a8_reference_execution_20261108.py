import subprocess
from pathlib import Path
out=Path('outputs/a8_reference_execution_20261108')
remote='/home/nvidia/hive_robot/DM_Control_Python/data/board_workspace_trials/a8_reference_execution_20261108/'
args=['scp','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),
    '-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256',
    '-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5',
    '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2']
for name in ('board_workspace_candidate.json','local_plan.json','preparation.json'):
    subprocess.run(args+[str(out/name),'nvidia@192.168.201.199:'+remote+name],check=True,timeout=45)
print('uploaded 3 preparation artifacts; no runtime replacement or motion')
