"""Reassemble byte-exact original ZIPs from small Git-friendly chunks; offline only."""
import hashlib,json
from pathlib import Path
def materialize_archives(out=None):
 out=Path(out) if out else Path(__file__).resolve().parent
 folder=out/'raw_evidence';manifest=json.loads((folder/'archive_manifest.json').read_text())
 for part in manifest['parts']:
  p=folder/part['file']
  if not p.exists():
   rows=part.get('storage_chunks');assert rows,'Missing ZIP and storage chunks: '+part['file']
   dest=p.with_suffix('.assembling')
   with dest.open('wb') as f:
    for row in rows:
     source=folder/row['file'];raw=source.read_bytes()
     assert len(raw)==row['size'] and hashlib.sha256(raw).hexdigest()==row['sha256'],'Chunk verification failed'
     f.write(raw)
   assert hashlib.sha256(dest.read_bytes()).hexdigest()==part['sha256'],'Reassembled archive mismatch'
   dest.replace(p)
  assert p.stat().st_size==part['size'] and hashlib.sha256(p.read_bytes()).hexdigest()==part['sha256']
 return manifest
if __name__=='__main__':
 m=materialize_archives();print(json.dumps({'original_archives_verified':len(m['parts']),'raw_members':len(m['members']),'motion_issued':False}))
