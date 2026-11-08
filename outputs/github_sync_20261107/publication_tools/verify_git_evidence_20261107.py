"""Read committed experiment bytes through a single git cat-file process."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/github_sync_20261107'


def read_exact(pipe, count, h):
    remaining = count
    while remaining:
        chunk = pipe.read(min(1024*1024, remaining))
        if not chunk:
            raise RuntimeError('Unexpected end of Git object stream')
        h.update(chunk)
        remaining -= len(chunk)


def check_rows(process, commit, rows, label):
    verified = []
    mismatches = []
    for index, row in enumerate(rows, 1):
        path = row['path']
        assert '\n' not in path and '\r' not in path
        process.stdin.write((commit+':'+path+'\n').encode('utf-8'))
        process.stdin.flush()
        header = process.stdout.readline().decode('utf-8', errors='replace').rstrip('\n')
        fields = header.split()
        if len(fields) != 3 or fields[1] != 'blob':
            issue = dict(path=path, reason='missing-or-nonblob', response=header)
            mismatches.append(issue)
            print('MISMATCH '+label+' '+path+' '+header, flush=True)
            continue
        blob, kind, length = fields
        length = int(length)
        h = hashlib.sha256()
        read_exact(process.stdout, length, h)
        assert process.stdout.read(1) == b'\n', 'Invalid Git batch delimiter'
        actual = dict(path=path, git_blob_oid=blob, bytes=length, sha256=h.hexdigest())
        if length != row['bytes'] or actual['sha256'] != row['sha256']:
            issue = dict(actual, expected_bytes=row['bytes'], expected_sha256=row['sha256'])
            mismatches.append(issue)
            print('MISMATCH '+label+' '+path+' expected='+str(row['bytes'])+
                  ' actual='+str(length), flush=True)
        else:
            verified.append(actual)
        if index % 100 == 0:
            print('CHECKED '+label+' '+str(index)+'/'+str(len(rows)), flush=True)
    print('COMPLETE '+label+' passed='+str(len(verified))+' mismatches='+str(len(mismatches)), flush=True)
    return dict(commit=commit, expected_files=len(rows), verified_files=len(verified),
        expected_bytes=sum(x['bytes'] for x in rows),
        verified_bytes=sum(x['bytes'] for x in verified),
        mismatches=mismatches, files=verified)


def main():
    core_receipt = json.loads((OUT/'core_receipt.json').read_text())
    core_manifest = json.loads((OUT/'core_manifest.json').read_text())
    core_manifest_bytes = (OUT/'core_manifest.json').read_bytes()
    text_receipt = json.loads((OUT/'text_evidence_receipt.json').read_text())
    core_commit = core_receipt['verified_remote_commit']
    text_commit = text_receipt['verified_remote_commit']
    assert core_commit == 'ff386d4b4b5b702402bd4453b7a74c0950643745'
    assert text_commit == 'ff5d166b575b1ad917597798e78d8f523e86d547'
    core_rows = [dict(path=k, **v) for k,v in core_manifest['files'].items()]
    text_rows = text_receipt['files']
    assert len(core_rows) == 53 and core_receipt['files'] == 54
    assert len(text_rows) == 961
    process = subprocess.Popen(['git', 'cat-file', '--batch'], cwd=ROOT,
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
    try:
        core = check_rows(process, core_commit, core_rows, 'core')
        core_manifest_self = check_rows(process, core_commit, [dict(
            path='outputs/github_sync_20261107/core_manifest.json',
            bytes=len(core_manifest_bytes), sha256=hashlib.sha256(core_manifest_bytes).hexdigest())],
            'core-manifest-self')
        text = check_rows(process, text_commit, text_rows, 'text-evidence')
    finally:
        process.stdin.close()
    assert process.wait(timeout=30) == 0, 'Git cat-file failed'
    result = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
        method='single git cat-file --batch process; raw committed blob byte count and SHA256',
        working_tree_used_as_expected_bytes=False, network_accessed=False, motion_issued=False,
        core=core, core_manifest_self=core_manifest_self, text_evidence=text,
        total_verified_files=core['verified_files']+core_manifest_self['verified_files']+text['verified_files'],
        status='passed' if not core['mismatches'] and not core_manifest_self['mismatches'] and not text['mismatches'] else 'mismatches-found')
    result['core_file_count_note'] = dict(manifest_entries=len(core_rows),
        receipt_files=core_receipt['files'],
        explanation='53 manifest-listed artifacts plus core_manifest.json itself = 54 committed core files; self bytes verified separately against the committed blob')
    destination=OUT/'git_evidence_verification.json'
    temporary=destination.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n',encoding='utf-8')
    temporary.replace(destination)
    print(json.dumps(dict(status=result['status'],files=result['total_verified_files'])), flush=True)
    if result['status']!='passed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
