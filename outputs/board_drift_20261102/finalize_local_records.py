"""Index derived records and verify offline reproduction; never contacts motors."""
import ast, hashlib, json, re, subprocess, sys
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
results = []
for trial in ['original', 'V22', 'V23', 'V24']:
    p = subprocess.run([sys.executable, str(OUT/'reproduce_trial_metrics.py'), '--trial', trial], capture_output=True, text=True, check=True)
    results.append(json.loads(p.stdout))
(OUT/'offline_reproduction_results.json').write_text(json.dumps(results, indent=2))
credential = re.compile(rb'-----BEGIN (?:OPENSSH |RSA |EC )?PRIVATE KEY-----|sk-proj-[A-Za-z0-9_-]{25,}|gh[pousr]_[A-Za-z0-9]{25,}|github_pat_[A-Za-z0-9_]{30,}|AKIA[0-9A-Z]{16}|AIza[0-9A-Za-z_-]{30,}')
rows = []
for p in sorted(OUT.rglob('*')):
    if not p.is_file() or '__pycache__' in p.parts or p.name in ['records_index.json', 'github_sync_receipt.json']:
        continue
    if p.suffix=='.zip' and p.parent.name=='raw_evidence':continue
    raw = p.read_bytes()
    if p.suffix in ['.py', '.json', '.jsonl', '.txt', '.md', '.csv']:
        assert not credential.search(raw), 'Review credential pattern: '+str(p)
    if p.suffix == '.py': ast.parse(raw.decode('utf8'), filename=str(p))
    rows.append({'path':p.relative_to(ROOT).as_posix(), 'bytes':len(raw), 'sha256':hashlib.sha256(raw).hexdigest()})
(OUT/'records_index.json').write_text(json.dumps({'scope':'Nov2 derived artifacts and compressed original evidence; excludes self and subsequent Git receipt', 'files':rows}, indent=2))
print(json.dumps({'offline_trials_reproduced':len(results), 'indexed_files':len(rows), 'credential_pattern_findings':0, 'motion_issued':False}))
