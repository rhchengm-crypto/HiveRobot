"""Commit the final handoff, download index and verification receipts."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import os
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
ENV = dict(os.environ,GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never')

def git(*args,input=None):
    r = subprocess.run(['git',*args],cwd=ROOT,input=input,capture_output=True,env=ENV,timeout=180)
    if r.returncode:
        raise RuntimeError(r.stderr.decode(errors='replace')[-2500:])
    return r.stdout

def pushed_head():
    git('push','origin','main')
    head = git('rev-parse','HEAD').decode().strip()
    assert git('ls-remote','origin','refs/heads/main').decode().split()[0] == head
    return head

def main():
    receipt = json.loads((OUT/'publication_receipt.json').read_text())
    assert receipt['release_draft'] is False and receipt['release_asset_count'] == 31
    assert not git('diff','--cached','--name-only').strip()
    assert git('branch','--show-current').decode().strip() == 'main'
    paths = set(filter(None,git('ls-files','--others','--exclude-standard','-z','--','outputs/github_sync_20261108').decode().strip('\0').split('\0')))
    paths.update(git('diff','--name-only','--','outputs/github_sync_20261108','HANDOFF_20261108.md').decode().splitlines())
    paths.add('HANDOFF_20261108.md')
    for name in paths:
        p = ROOT/name
        assert p.is_file() and p.stat().st_size < 50*1024*1024 and p.suffix != '.zip', name
        assert name == 'HANDOFF_20261108.md' or name.startswith('outputs/github_sync_20261108/')
    git('add','--pathspec-from-file=-','--pathspec-file-nul',input=b'\0'.join(n.encode() for n in sorted(paths))+b'\0')
    assert set(git('diff','--cached','--name-only','-z').decode().strip('\0').split('\0')) <= paths
    git('commit','-m','Finalize November 8 handoff and checksum-verified experiment publication')
    content = pushed_head()
    proof = dict(status='verified-publication-content-commit',checked_at_utc=datetime.now(timezone.utc).isoformat(),
        verified_main_at_capture=content,publication_receipt_sha256=hashlib.sha256((OUT/'publication_receipt.json').read_bytes()).hexdigest(),
        handoff_sha256=hashlib.sha256((ROOT/'HANDOFF_20261108.md').read_bytes()).hexdigest(),
        release_url=receipt['release_url'],release_assets=31,rgbd_sessions=7,rgb_depth_pairs=3690,
        receipt_is_followup_to_content_commit=True,motion_issued=False)
    path = OUT/'final_git_verification.json'
    path.write_text(json.dumps(proof,indent=2)+'\n',encoding='utf-8')
    git('add','--',path.relative_to(ROOT).as_posix())
    git('commit','-m','Record verified November 8 publication content commit')
    final = pushed_head()
    assert not git('status','--porcelain').strip(), 'Unexpected remaining uncommitted files'
    print(json.dumps(dict(status='complete',content_commit=content,verified_final_main=final,
        clean_worktree=True,release_url=receipt['release_url'],assets=31,rgb_depth_pairs=3690,motion_issued=False)),flush=True)

if __name__ == '__main__':
    main()
