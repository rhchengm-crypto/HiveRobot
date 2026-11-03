"""Verify all raw bytes and high-confidence credential patterns before Git publication."""
import json,hashlib,zipfile,re,ast
from archive_storage import materialize_archives
from pathlib import Path
OUT=Path(__file__).resolve().parent
m=materialize_archives(OUT);expected={v['path']:v for v in m['members']};seen=set();findings=[]
patterns=[rb'-----BEGIN (?:OPENSSH |RSA |EC )?PRIVATE KEY-----',rb'sk-proj-[A-Za-z0-9_-]{25,}',rb'gh[pousr]_[A-Za-z0-9]{25,}',rb'github_pat_[A-Za-z0-9_]{30,}',rb'AKIA[0-9A-Z]{16}',rb'AIza[0-9A-Za-z_-]{30,}']
credential=re.compile(b'|'.join(patterns))
for part in m['parts']:
 for row in part.get('storage_chunks',[]):
  chunk=(OUT/'raw_evidence'/row['file']).read_bytes()
  assert len(chunk)==row['size'] and hashlib.sha256(chunk).hexdigest()==row['sha256']
 p=OUT/'raw_evidence'/part['file'];assert p.stat().st_size<50*1024*1024,'Unexpectedly large Git object'
 assert hashlib.sha256(p.read_bytes()).hexdigest()==part['sha256'],'Archive checksum mismatch'
 with zipfile.ZipFile(p) as z:
  for info in z.infolist():
   assert info.filename in expected and info.filename not in seen
   raw=z.read(info.filename);row=expected[info.filename]
   assert len(raw)==row['size'] and hashlib.sha256(raw).hexdigest()==row['sha256']
   assert row['archive_part']==part['file'];seen.add(info.filename)
   if info.filename.endswith(('.json','.jsonl','.py','.txt','.md')) and credential.search(raw):findings.append({'file':info.filename,'type':'credential-pattern match; content deliberately not printed'})
assert len(seen)==len(expected)
for p in OUT.glob('*.py'):ast.parse(p.read_text(encoding='utf8'),filename=str(p))
for p in OUT.rglob('*.json'):
 if 'restored_dataset' not in p.parts:json.loads(p.read_text(encoding='utf8'))
report={'raw_archives_verified':len(m['parts']),'raw_members_verified':len(seen),'raw_bytes_verified':sum(v['size'] for v in expected.values()),'credential_pattern_findings':findings,'note':'Pattern check is an additional credential check, not proof of absence of all sensitive information. SSH keys were never selected.'}
(OUT/'evidence_package_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));assert not findings,'Review credential-pattern findings before staging archives'
