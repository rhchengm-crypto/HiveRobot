from pathlib import Path
source=(Path(__file__).resolve().parent/'validate_plate_motion.py').read_text()
source=source.replace('plate_feature_tracks.json','independent_plate_tracks_v22.json').replace('arm_run_1793671770732_5069.txt','arm_run_1793679218917_10319.txt').replace('train=frames<430','train=frames<338').replace('plate_motion_validation.json','independent_plate_validation_v22.json')
exec(compile(source,__file__,'exec'))
