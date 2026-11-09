"""Publish reviewed current sources and handoff before large evidence assets."""
from pathlib import Path
import hashlib
import json
import os
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
ENV = dict(os.environ, GIT_TERMINAL_PROMPT='0', GCM_INTERACTIVE='never')

def git(*args, input=None):
    result = subprocess.run(['git', *args], cwd=ROOT, input=input, capture_output=True, env=ENV, timeout=180)
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors='replace')[-2500:])
    return result.stdout

def main():
    assert git('branch', '--show-current').decode().strip() == 'main'
    assert not git('diff', '--cached', '--name-only').strip(), 'Preserve existing staged changes'
    assert git('rev-list','--left-right','--count','HEAD...origin/main').decode().strip() == '0\t0', 'Review diverged main first'
    paths = set(git('diff','--name-only','--','scripts').decode().splitlines())
    paths.update(git('ls-files','--others','--exclude-standard','--','scripts').decode().splitlines())
    paths.update(['HANDOFF_20261108.md', '.gitignore'])
    for name in ['publication_preflight.md','auth_preflight.json','offline_verification.txt','verification_summary.json','publish_core.py']:
        paths.add((OUT/name).relative_to(ROOT).as_posix())
    assert all((ROOT/n).is_file() for n in paths)
    manifest = {n:dict(bytes=(ROOT/n).stat().st_size, sha256=hashlib.sha256((ROOT/n).read_bytes()).hexdigest()) for n in sorted(paths)}
    assert all(row['bytes'] < 50*1024*1024 for row in manifest.values())
    (OUT/'core_manifest.json').write_text(json.dumps(dict(files=manifest,motion_issued=False),indent=2)+'\n',encoding='utf-8')
    paths.add('outputs/github_sync_20261108/core_manifest.json')
    git('add','--pathspec-from-file=-','--pathspec-file-nul', input=b'\0'.join(n.encode() for n in sorted(paths))+b'\0')
    actual = set(git('diff','--cached','--name-only','-z').decode().strip('\0').split('\0'))
    assert actual <= paths
    git('commit','-m','Record November 8 A8 support repair, accepted non-pawn reference and offline fit')
    head = git('rev-parse','HEAD').decode().strip()
    git('push','origin','main')
    remote = git('ls-remote','origin','refs/heads/main').decode().split()[0]
    assert remote == head
    receipt = dict(status='core-published',verified_remote_commit=head,files=len(paths),motion_issued=False,
        private_chat_export_published=False,raw_data_assets_status='pending')
    (OUT/'core_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(receipt),flush=True)

if __name__ == '__main__':
    main()
