import subprocess,shlex,json,sys
from pathlib import Path
tag=sys.argv[1];assert tag in ('axis_audit','descent_audit');seconds=600
code=r'''
import json,subprocess,shlex,time,shutil
from pathlib import Path
root=Path('/home/nvidia/hive_robot/DM_Control_Python');assert shutil.disk_usage(str(root)).free>2_000_000_000
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:a=[v.decode() for v in (p/'cmdline').read_bytes().split(bytes([0])) if v]
 except (OSError,UnicodeError):continue
 assert 'record_left_arm_rgbd_audit.py' not in a,'A read-only recorder is already running'
folder=root/'data/rgbd_audits'/(TAG+'_'+str(time.time_ns()));log=folder.parent/(folder.name+'.log')
cmd='source /opt/ros/noetic/setup.bash && source /home/nvidia/ascam_ws/devel/setup.bash && exec python3 -u record_left_arm_rgbd_audit.py --seconds 600 --fps 2 --output '+shlex.quote(str(folder))
with log.open('wb') as f:p=subprocess.Popen(['/bin/bash','-c',cmd],cwd=str(root),stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(2);assert p.poll() is None,log.read_text();print(json.dumps({'pid':p.pid,'output':str(folder),'log':str(log),'motion_commands_sent':False}))
'''.replace('TAG',repr(tag))
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','jetson','python3 -c '+shlex.quote(code)],capture_output=True,timeout=20);r.check_returncode();print(r.stdout.decode());Path(__file__).with_name('rgbd_'+tag+'_capture.json').write_bytes(r.stdout)
