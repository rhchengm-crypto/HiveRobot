from pathlib import Path
source=(Path(__file__).resolve().parent/'pull_v22_log.py').read_text().replace('v22_run_latest.json','v23_run_latest.json')
exec(compile(source,__file__,'exec'))
