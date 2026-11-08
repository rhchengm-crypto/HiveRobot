"""Upload the approved November 7 assets, without publishing the draft."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/github_sync_20261107'
HELPER = ROOT / 'tmp/github_assets_20261107.py'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def receipt(row):
    path = OUT / 'asset_receipts' / (row['asset_name'] + '.json')
    if not path.exists():
        return None
    value = json.loads(path.read_text())
    assert value['sha256'] == row['sha256']
    assert value['bytes'] == row['bytes']
    assert value['state'] == 'uploaded'
    assert value['remote_digest'] == 'sha256:' + row['sha256']
    return value


def upload(row):
    path = ROOT / row['path']
    assert path.stat().st_size == row['bytes']
    assert digest(path) == row['sha256'], 'Candidate bytes changed: ' + row['path']
    result = subprocess.run([sys.executable, '-X', 'utf8', str(HELPER), 'upload',
        str(path), '--name', row['asset_name']], cwd=ROOT, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=1800)
    if result.returncode:
        # Helper never prints credentials. Avoid exposing arbitrary error text.
        raise RuntimeError('Upload failed: ' + row['asset_name'] +
                           ' exit=' + str(result.returncode))
    value = receipt(row)
    assert value is not None
    print('VERIFIED ' + value['name'] + ' ' + str(value['bytes']), flush=True)
    return value


def save(value):
    path = OUT / 'nov7_assets_receipt.json'
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def main():
    print('Reading approved candidate inventory', flush=True)
    candidates = json.loads((OUT / 'publication_candidates.json').read_text())
    rows = candidates['proposed_release_archives']
    assert len(rows) == 40 and len({r['asset_name'] for r in rows}) == 40
    assert not any('mac_handoff' in r['path'] for r in rows)
    completed = {}
    previous_path = OUT / 'nov7_assets_receipt.json'
    previous = json.loads(previous_path.read_text()) if previous_path.exists() else {}
    history = previous.get('historical_retries', [])
    if previous.get('errors'):
        history = history + [dict(observed_utc=datetime.now(timezone.utc).isoformat(),
            failures=previous['errors'], resolution='retry original bytes after incomplete remote starter removal')]
    state = dict(release_id=438072261, tag='experiments-2026-11-07',
        publish_draft=False, maximum_parallel_uploads=2, archive_count=40,
        archive_bytes=sum(r['bytes'] for r in rows), archives=[], errors=[],
        auxiliary_media=None, status='uploading', historical_retries=history)
    for row in rows:
        value = receipt(row)
        if value is not None:
            completed[row['asset_name']] = value
    print('Existing verified receipts: ' + str(len(completed)), flush=True)
    state['archives'] = list(completed.values()); save(state)
    pending = [r for r in rows if r['asset_name'] not in completed]
    with ThreadPoolExecutor(max_workers=2) as executor:
        future_rows = {executor.submit(upload, row): row for row in pending}
        for future in as_completed(future_rows):
            row = future_rows[future]
            try:
                completed[row['asset_name']] = future.result()
            except Exception as exc:
                state['errors'].append(str(exc))
                print('FAILED ' + row['asset_name'], flush=True)
            state['archives'] = [completed[r['asset_name']] for r in rows
                                 if r['asset_name'] in completed]
            state['verified_archive_count'] = len(completed)
            save(state)
    if state['errors']:
        state['status'] = 'incomplete'; save(state)
        raise SystemExit(1)
    media = candidates['misc_ignored_media']
    assert len(media) == 24 and not any('mac_handoff' in x['path'] for x in media)
    manifest = []
    archive = OUT / 'nov7_auxiliary_media.zip'
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for row in media:
            path = ROOT / row['path']
            assert path.stat().st_size == row['bytes']
            h = digest(path)
            manifest.append(dict(path=row['path'], bytes=row['bytes'], sha256=h))
            z.write(path, row['path'])
        z.writestr('SHA256_MANIFEST.json', json.dumps(dict(
            schema=1, lossless=True, deduplicated=False, files=manifest), indent=2)+'\n')
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for row in manifest:
            assert hashlib.sha256(z.read(row['path'])).hexdigest() == row['sha256']
    extra = dict(path=archive.relative_to(ROOT).as_posix(), bytes=archive.stat().st_size,
                 sha256=digest(archive), asset_name=archive.name)
    state['auxiliary_media'] = dict(upload(extra), member_count=24,
        original_bytes=sum(r['bytes'] for r in media), lossless=True, deduplicated=False)
    state['status'] = 'complete'
    state['verified_archive_count'] = len(completed)
    state['completed_utc'] = datetime.now(timezone.utc).isoformat()
    save(state)
    print(json.dumps(dict(status=state['status'], archives=len(completed),
        auxiliary_media_members=24, receipt='outputs/github_sync_20261107/nov7_assets_receipt.json')), flush=True)


if __name__ == '__main__':
    main()
