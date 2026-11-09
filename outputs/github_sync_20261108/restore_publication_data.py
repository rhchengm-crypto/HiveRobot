"""List or download checksum-verified experiment assets; never controls hardware."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import urllib.parse
import urllib.request


def digest(path):
    checksum = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            checksum.update(block)
    return checksum.hexdigest()


def checked_rows(manifest):
    rows = manifest['assets']
    names = set()
    for row in rows:
        name = row['name']
        if (not isinstance(name, str) or not name or Path(name).name != name
                or any(character in name for character in '/\\\r\n') or name in names):
            raise ValueError('Each asset must have a unique plain filename')
        names.add(name)
        if not isinstance(row['bytes'], int) or isinstance(row['bytes'], bool) or row['bytes'] < 0:
            raise ValueError('Invalid byte count: '+name)
        if not re.fullmatch('[0-9a-f]{64}', row['sha256']):
            raise ValueError('Invalid SHA256: '+name)
        url = urllib.parse.urlsplit(row['url'])
        if (url.scheme != 'https' or url.hostname != 'github.com' or url.username or url.password
                or not url.path.startswith('/rhchengm-crypto/HiveRobot/releases/download/')):
            raise ValueError('Unexpected asset URL: '+name)
        if not isinstance(row.get('group'), str) or not row['group']:
            raise ValueError('Missing asset group: '+name)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=Path(__file__).with_name('publication_manifest.json'))
    parser.add_argument('--group', default='all', help='all, or a group listed by the manifest')
    parser.add_argument('--output', type=Path, default=Path('experiment_assets_20261108'))
    parser.add_argument('--download', action='store_true', help='Transfer files; otherwise only list them')
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding='utf-8-sig'))
    all_rows = checked_rows(manifest)
    groups = sorted({row['group'] for row in all_rows})
    if args.group != 'all' and args.group not in groups:
        parser.error('Unknown group. Available: '+', '.join(['all']+groups))
    rows = [row for row in all_rows if args.group == 'all' or row['group'] == args.group]
    print(str(len(rows))+' assets; '+str(sum(row['bytes'] for row in rows))+' bytes')
    print('Groups: '+', '.join(groups))
    if not args.download:
        for row in rows:
            print(row['group'], row['name'], row['bytes'], row['sha256'])
        print('Add --download to transfer files. No archives are extracted. No robot access.')
        return
    args.output.mkdir(parents=True, exist_ok=True)
    for row in rows:
        target = args.output/row['name']
        if target.exists():
            if not target.is_file() or target.stat().st_size != row['bytes'] or digest(target) != row['sha256']:
                raise ValueError('Existing file does not match; preserved without replacement: '+row['name'])
            print('Verified existing: '+row['name'], flush=True)
            continue
        temporary = target.with_name(target.name+'.partial')
        checksum, size = hashlib.sha256(), 0
        request = urllib.request.Request(row['url'], headers={'User-Agent':'HiveRobot-evidence-restore'})
        with urllib.request.urlopen(request, timeout=180) as response, temporary.open('wb') as destination:
            for block in iter(lambda: response.read(1024*1024), b''):
                destination.write(block)
                checksum.update(block)
                size += len(block)
        if size != row['bytes'] or checksum.hexdigest() != row['sha256']:
            raise ValueError('Downloaded checksum mismatch; partial file retained: '+row['name'])
        temporary.replace(target)
        print('Downloaded and verified: '+row['name'], flush=True)
    print('All selected original files verified. No archives were extracted. No robot access.')


if __name__ == '__main__':
    main()
