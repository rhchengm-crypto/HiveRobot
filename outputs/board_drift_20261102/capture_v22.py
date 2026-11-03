from pathlib import Path
source=(Path(__file__).resolve().parent/'capture.py').read_text()
source=source.replace("capture.json", "capture_v22.json").replace("OUT/'rgbd'", "OUT/'rgbd_v22'").replace('camera_before_motion.jpg','camera_v22_before_motion.jpg')
exec(compile(source, __file__, 'exec'))
