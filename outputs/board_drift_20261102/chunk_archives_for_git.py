"""Split already verified original ZIP bytes into small immutable storage chunks."""
import hashlib,json
from pathlib import Path
OUT=Path(__file__).resolve().parent;folder=OUT/'raw_evidence';dest=folder/'chunks';dest.mkdir(exist_ok=True)
path=folder/'archive_manifest.json';m=json.loads(path.read_text());count=0
for part in m['parts']:
 raw=(folder/part['file']).read_bytes();assert hashlib.sha256(raw).hexdigest()==part['sha256'];rows=[]
 for i,start in enumerate(range(0,len(raw),2*1024*1024),1):
  data=raw[start:start+2*1024*1024];p=dest/(part['file']+'.'+str(i).zfill(3)+'.bin')
  if p.exists():assert p.read_bytes()==data
  else:p.write_bytes(data)
  rows.append({'file':p.relative_to(folder).as_posix(),'size':len(data),'sha256':hashlib.sha256(data).hexdigest()});count+=1
 part['storage_chunks']=rows
m['git_storage']='Original ZIP bytes split into ordered <=2 MiB chunks; archive_storage.py reassembles and verifies original SHA-256. Original ZIPs remain locally available and are ignored in Git.'
path.write_text(json.dumps(m,indent=2));print(json.dumps({'original_archives':len(m['parts']),'storage_chunks':count,'bytes_unchanged':True}))
