"""Upload already inventoried November 8 original files, never publish a release."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import zipfile

import github_assets as github

ROOT, OUT = github.ROOT, github.OUT


def main():
    inventory = json.loads((OUT/'data_inventory.json').read_text(encoding='utf-8'))
    rows = [dict(local_path=row['path'], name=Path(row['path']).parts[1]+'__'+Path(row['path']).name,
                 bytes=row['bytes'], sha256=row['sha256']) for row in inventory['archives']]
    snapshot = json.loads((OUT/'final_snapshot_verification.json').read_text(encoding='utf-8'))
    if not snapshot['ok'] or not snapshot['all_members_verified'] or not snapshot['crc_verified']:
        raise RuntimeError('Final runtime snapshot verification is incomplete')
    rows.append(dict(local_path=snapshot['local_path'], name=Path(snapshot['local_path']).name,
                     bytes=snapshot['bytes'], sha256=snapshot['sha256']))
    auxiliary = OUT/'auxiliary_media_20261108.zip'
    rows.append(dict(local_path=auxiliary.relative_to(ROOT).as_posix(), name=auxiliary.name,
                     bytes=auxiliary.stat().st_size, sha256=github.digest(auxiliary)))
    if len({row['name'] for row in rows}) != len(rows):
        raise RuntimeError('Duplicate intended asset names')
    ready = []
    for row in rows:
        path = ROOT/row['local_path']
        if path.stat().st_size != row['bytes'] or github.digest(path) != row['sha256']:
            raise RuntimeError('Original file no longer matches inventory: '+row['local_path'])
        with zipfile.ZipFile(path) as archive:
            if archive.testzip() is not None:
                raise RuntimeError('ZIP CRC failed: '+row['local_path'])
        ready.append(dict(row, zip_crc_verified=True))
    github.save(OUT/'ready_asset_preupload_verification.json', dict(
        checked_at_utc=datetime.now(timezone.utc).isoformat(), assets=ready,
        count=len(ready), bytes=sum(row['bytes'] for row in ready), motion_issued=False))
    print('PREUPLOAD VERIFIED '+str(len(ready))+' assets '+str(sum(row['bytes'] for row in ready))+' bytes', flush=True)
    github.token()  # Resolve GCM once; the secret is never saved or printed.
    state = dict(status='uploading', maximum_parallel_uploads=2, assets=[], errors=[],
        intended_asset_count=len(ready), intended_bytes=sum(row['bytes'] for row in ready),
        release_tag=github.TAG, release_published_by_this_worker=False, motion_issued=False)
    destination = OUT/'ready_assets_upload_receipt.json'
    github.save(destination, state)
    completed = {}
    with ThreadPoolExecutor(max_workers=2) as executor:
        pending = {executor.submit(github.upload, ROOT/row['local_path'], row['name']): row for row in ready}
        for future in as_completed(pending):
            row = pending[future]
            try:
                completed[row['name']] = future.result()
                print('COMPLETED '+str(len(completed))+'/'+str(len(ready))+' '+row['name'], flush=True)
            except Exception as exc:
                state['errors'].append(dict(name=row['name'], error_type=type(exc).__name__))
                print('FAILED '+row['name']+' '+type(exc).__name__, flush=True)
            state['assets'] = [completed[item['name']] for item in ready if item['name'] in completed]
            state['verified_asset_count'] = len(completed)
            github.save(destination, state)
    state['status'] = 'complete' if not state['errors'] else 'incomplete'
    state['completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    github.save(destination, state)
    print(json.dumps({key:state[key] for key in ('status','verified_asset_count','intended_asset_count','intended_bytes')}), flush=True)
    if state['errors']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
