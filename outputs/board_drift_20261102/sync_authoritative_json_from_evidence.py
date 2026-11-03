"""Bring local runtime JSONs up to the archived latest Jetson state; never uploads runtime files."""
import json,zipfile,hashlib,subprocess
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
validation=json.loads((OUT/'evidence_package_validation.json').read_text());assert not validation['credential_pattern_findings']
m=json.loads((OUT/'raw_evidence/archive_manifest.json').read_text());changed=[];matched=[]
for row in m['members']:
 relative=Path(row['path'])
 if len(relative.parts)!=2 or relative.parts[0]!='data' or relative.suffix!='.json':continue
 with zipfile.ZipFile(OUT/'raw_evidence'/row['archive_part']) as z:raw=z.read(row['path'])
 assert hashlib.sha256(raw).hexdigest()==row['sha256'];json.loads(raw)
 local=ROOT/'scripts'/relative
 if local.exists() and local.read_bytes()==raw:matched.append(relative.name);continue
 before=local.read_bytes() if local.exists() else None
 if before is not None:
  historical=subprocess.run(['git','show','HEAD:scripts/'+relative.as_posix()],cwd=ROOT,capture_output=True)
  assert historical.returncode==0 and before==historical.stdout,'Local JSON has independent edits; do not overwrite '+str(relative)
  backup=OUT/'local_json_before_final_sync'/relative.name;backup.parent.mkdir(exist_ok=True);backup.write_bytes(before)
 local.write_bytes(raw);changed.append({'file':relative.name,'old_sha256':hashlib.sha256(before).hexdigest() if before is not None else None,'new_sha256':row['sha256']})
report={'source':'SHA-verified final Jetson archive, no motion since snapshot','direction':'Jetson evidence -> local scripts/data only','changed':changed,'already_identical':matched,'runtime_files_uploaded_to_Jetson':False}
(OUT/'authoritative_json_sync.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
