import hashlib,json,subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[1]
out=root/'outputs/github_sync_20261107';out.mkdir(exist_ok=True)
def git(*args,input=None):
    r=subprocess.run(['git',*args],cwd=root,input=input,capture_output=True)
    if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace')[-4000:])
    return r.stdout
assert git('branch','--show-current').decode().strip()=='main'
staged=git('diff','--cached','--name-only').decode().splitlines()
assert not staged,'Existing staged changes must be preserved'
paths=set(git('diff','--name-only','--','scripts').decode().splitlines())
paths.update(git('ls-files','--others','--exclude-standard','--','scripts').decode().splitlines())
paths.update(['HANDOFF_20261107.md','.gitignore'])
for p in (out/'latest_home_evidence').glob('*'):
    if p.is_file():paths.add(p.relative_to(root).as_posix())
for n in ['offline_verification.txt','verification_summary.json','inventory.json','remote_gap_audit.json','publication_candidates.json','jetson_inventory.json']:
    if (out/n).is_file():paths.add((out/n).relative_to(root).as_posix())
assert not any('mac_handoff_20261107' in n for n in paths)
manifest={n:dict(bytes=(root/n).stat().st_size,sha256=hashlib.sha256((root/n).read_bytes()).hexdigest()) for n in sorted(paths)}
(out/'core_manifest.json').write_text(json.dumps(dict(files=manifest,motion_issued=False),indent=2)+'\n')
paths.add('outputs/github_sync_20261107/core_manifest.json')
git('add','--pathspec-from-file=-','--pathspec-file-nul',input=b'\0'.join(n.encode() for n in sorted(paths))+b'\0')
git('commit','-m','Record November 7 A7 acceptance, common reference library and descent gain repair')
snapshot=git('rev-parse','HEAD').decode().strip()
git('merge','--no-edit','origin/main')
head=git('rev-parse','HEAD').decode().strip()
git('push','origin','main')
remote=git('ls-remote','origin','refs/heads/main').decode().split()[0]
assert remote==head
result=dict(status='core-published',local_snapshot_commit=snapshot,verified_remote_commit=head,
    files=len(paths),motion_issued=False,other_project_commits_preserved=True,
    binary_data_status='upload-pending',private_chat_export_published=False)
(out/'core_receipt.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result),flush=True)
