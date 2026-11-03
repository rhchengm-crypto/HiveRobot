"""Read live camera calibration only, never the motor serial port."""
import subprocess,shlex,json
from pathlib import Path
OUT=Path(__file__).resolve().parent
code=r'''
import rospy,json
from sensor_msgs.msg import CameraInfo
rospy.init_node('hive_readonly_camera_calibration',anonymous=True,disable_signals=True)
out={}
for name,topic in [('rgb','/ascamera_hp60c/rgb0/camera_info'),('depth','/ascamera_hp60c/depth0/camera_info')]:
 m=rospy.wait_for_message(topic,CameraInfo,timeout=5)
 out[name]={'frame':m.header.frame_id,'width':m.width,'height':m.height,'K':list(m.K),'D':list(m.D),'R':list(m.R),'P':list(m.P)}
print(json.dumps(out))
'''
shell='source /opt/ros/noetic/setup.bash && exec python3 -c '+shlex.quote(code)
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','bash -c '+shlex.quote(shell)],capture_output=True,timeout=20)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
d=json.loads(r.stdout);(OUT/'live_rgb_depth_calibration.json').write_text(json.dumps(d,indent=2));print(json.dumps(d))
