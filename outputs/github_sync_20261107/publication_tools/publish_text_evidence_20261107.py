import hashlib,json,subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'outputs/github_sync_20261107'
def git(*args,input=None):
 r=subprocess.run(['git',*args],cwd=root,input=input,capture_output=True)
 if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace')[-4000:])
 return r.stdout
assert not git('diff','--cached','--name-only').strip()
paths=git('ls-files','--others','--exclude-standard','-z','--','outputs').decode().strip('\0').split('\0')
paths=sorted(p for p in paths if p and not p.startswith(('outputs/github_sync_20261107/','outputs/mac_handoff_20261107/')))
assert all('20261107' in p for p in paths), 'Unexpected unrelated output; inspect before publishing'
files=[dict(path=p,bytes=(root/p).stat().st_size,sha256=hashlib.sha256((root/p).read_bytes()).hexdigest()) for p in paths]
assert all(r['bytes']<50*1024*1024 for r in files)
groups=[];group=[];size=0
for row in files:
 if group and size+row['bytes']>50*1024*1024:groups.append(group);group=[];size=0
 group.append(row);size+=row['bytes']
if group:groups.append(group)
receipt=dict(status='publishing',files=files,batches=[],private_mac_export_excluded=True,motion_issued=False)
def checkpoint():(out/'text_evidence_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
checkpoint()
for i,g in enumerate(groups,1):
 for row in g:assert hashlib.sha256((root/row['path']).read_bytes()).hexdigest()==row['sha256']
 git('add','--pathspec-from-file=-','--pathspec-file-nul',input=b'\0'.join(x['path'].encode() for x in g)+b'\0')
 git('commit','-m',f'Archive November 7 experimental control evidence {i}/{len(groups)}')
 head=git('rev-parse','HEAD').decode().strip()
 git('push','origin','main')
 actual=git('ls-remote','origin','refs/heads/main').decode().split()[0];assert actual==head
 receipt['batches'].append(dict(number=i,commit=head,files=len(g),bytes=sum(x['bytes'] for x in g)))
 checkpoint();print('PUBLISHED '+str(i)+'/'+str(len(groups))+' '+head+' files='+str(len(g)),flush=True)
receipt.update(status='all-selected-text-evidence-published',verified_remote_commit=head)
checkpoint();print(json.dumps(dict(files=len(files),raw_bytes=sum(r['bytes'] for r in files),batches=len(groups),verified_remote_commit=head)),flush=True)
