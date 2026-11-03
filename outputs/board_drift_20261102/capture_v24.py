from pathlib import Path
source=(Path(__file__).resolve().parent/'capture.py').read_text()
source=source.replace('capture.json','capture_v24.json').replace("OUT/'rgbd'", "OUT/'rgbd_v24'").replace('camera_before_motion.jpg','camera_v24_before_motion.jpg')
exec(compile(source,__file__,'exec'))
