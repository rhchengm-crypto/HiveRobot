from pathlib import Path
import json,shutil,time,hashlib
root=Path('/home/nvidia/hive_robot/DM_Control_Python');out=root/'data/board_workspace_trials/a8_reference_execution_20261108'
rec=json.loads((out/'recording.json').read_text());folder=Path(rec['recording_folder'])
path=sorted(folder.glob('*.jpg'))[-1]
dest=out/'monitor.jpg';shutil.copyfile(path,dest)
meta=dict(source=str(path),path=str(dest),age_s=time.time()-path.stat().st_mtime,
    sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),motion_commands_sent=False)
assert meta['age_s']<3,meta
(out/'monitor.json').write_text(json.dumps(meta));print(json.dumps(meta))
