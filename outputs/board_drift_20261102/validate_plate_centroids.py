from pathlib import Path
source=(Path(__file__).resolve().parent/'validate_plate_motion.py').read_text()
source=source.replace('plate_feature_tracks.json','plate_centroid_tracks.json').replace('plate_motion_validation.json','plate_centroid_validation.json')
exec(compile(source,__file__,'exec'))
