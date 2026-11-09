"""Publish bounded text batches; original oversized/media evidence uses Release."""
from pathlib import Path
import hashlib
import json
import os
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
ENV = dict(os.environ, GIT_TERMINAL_PROMPT='0', GCM_INTERACTIVE='never')

def git(*args, input=None):
    result = subprocess.run(['git', *args], cwd=ROOT, input=input, capture_output=True, env=ENV, timeout=240)
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors='replace')[-2500:])
    return result.stdout

def checkpoint(data):
    (OUT/'text_evidence_receipt.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    assert not git('diff','--cached','--name-only').strip()
    inventory = json.loads((OUT/'data_inventory.json').read_text(encoding='utf-8'))
    roots = [r['directory'] for r in inventory['directories']]
    raw = git('ls-files','--others','--exclude-standard','-z','--',*roots).decode('utf-8').strip('\0')
    paths = sorted(filter(None,raw.split('\0')))
    rows = []
    for name in paths:
        p = ROOT/name
        assert p.is_file() and p.stat().st_size < 50*1024*1024, name
        data = p.read_bytes()
        if p.suffix.lower() not in {'.py','.json','.jsonl','.md','.txt','.log','.csv','.html'}:
            # Preserve command stdout/stderr receipts as ordinary text too.
            assert '\0' not in data.decode('utf-8-sig'), name
        rows.append(dict(path=name,bytes=p.stat().st_size,sha256=hashlib.sha256(data).hexdigest()))
    groups = []
    pending = []
    size = 0
    for row in rows:
        if pending and size+row['bytes'] > 35*1024*1024:
            groups.append(pending); pending=[]; size=0
        pending.append(row); size+=row['bytes']
    if pending:
        groups.append(pending)
    receipt = dict(status='publishing',files=rows,batches=[],motion_issued=False,
        raw_media_and_oversized_log_preserved_in_release=True,private_chat_export_published=False)
    checkpoint(receipt)
    for i, group in enumerate(groups,1):
        for row in group:
            assert hashlib.sha256((ROOT/row['path']).read_bytes()).hexdigest() == row['sha256']
        names = {row['path'] for row in group}
        git('add','--pathspec-from-file=-','--pathspec-file-nul',input=b'\0'.join(n.encode() for n in sorted(names))+b'\0')
        assert set(git('diff','--cached','--name-only','-z').decode().strip('\0').split('\0')) <= names
        git('commit','-m',f'Archive November 8 experiment control and fitting evidence {i}/{len(groups)}')
        head = git('rev-parse','HEAD').decode().strip()
        git('push','origin','main')
        assert git('ls-remote','origin','refs/heads/main').decode().split()[0] == head
        receipt['batches'].append(dict(number=i,commit=head,files=len(group),bytes=sum(r['bytes'] for r in group)))
        checkpoint(receipt)
        print(f'PUBLISHED {i}/{len(groups)} {head} files={len(group)}',flush=True)
    receipt.update(status='all-selected-text-evidence-published',verified_remote_commit=head,
                   file_count=len(rows),raw_bytes=sum(r['bytes'] for r in rows))
    checkpoint(receipt)
    print(json.dumps({k:receipt[k] for k in ('status','file_count','raw_bytes','verified_remote_commit')}),flush=True)

if __name__ == '__main__':
    main()
