from pathlib import Path
import json
r=Path('/home/nvidia/hive_robot/DM_Control_Python/data/board_workspace_trials/a8_lower25_execution_20261108')
o=json.loads((r/'recording.json').read_text())
p=Path(o['recording_folder']).with_suffix('.log')
print(json.dumps(dict(recording=o,log=p.read_text(errors='replace')[-5000:])))
