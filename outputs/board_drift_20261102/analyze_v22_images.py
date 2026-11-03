from pathlib import Path
source=(Path(__file__).resolve().parent/'analyze_remote_images.py').read_text()
source=source.replace('capture.json','capture_v22.json').replace('visual_motion_audit.json','visual_motion_audit_v22.json').replace('descent_comparison.jpg','descent_comparison_v22.jpg')
source=source.replace("(OUT/'descent_comparison_v22.jpg').read_bytes()", "(OUT/'descent_comparison.jpg').read_bytes()")
exec(compile(source,__file__,'exec'))
