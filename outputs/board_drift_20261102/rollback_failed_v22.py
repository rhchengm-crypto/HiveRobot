"""Undo only our two-file V22 experiment after its failed physical validation."""
import hashlib,json,subprocess
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
manifest=json.loads((OUT/'v22_deployment.json').read_text())
for name,h in manifest['files'].items():
 p=ROOT/'scripts'/name
 assert hashlib.sha256(p.read_bytes()).hexdigest()==h['new_sha256'], 'Local file changed since experiment: '+name
 original=subprocess.check_output(['git','show','HEAD:scripts/'+name],cwd=ROOT)
 assert hashlib.sha256(original).hexdigest()==h['old_sha256'], 'HEAD is not the pretrial version'
 p.write_bytes(original)
trial=json.loads((OUT/'v22_trial_record.json').read_text())
trial.update(physical_observation={'B_drift_mm':10,'stage':'during descent','oscillation':False,'operator_conclusion':'no improvement'},validation='failed; swivel constraint is not a fix',next_action='Restore pretrial code and return Home; do not deploy rejected distal-axis fits.')
(OUT/'v22_trial_record.json').write_text(json.dumps(trial,indent=2))
source=(OUT/'deploy_v22.py').read_text().replace("'v22_'", "'rollback_v22_'").replace('v22_deployment.json','v22_rollback_deployment.json')
exec(compile(source,str(OUT/'deploy_v22.py'),'exec'))
