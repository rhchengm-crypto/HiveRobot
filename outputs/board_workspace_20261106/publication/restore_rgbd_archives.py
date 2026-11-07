"""Run from any directory after cloning; restores only the two local RGB-D ZIPs."""
import hashlib, json
from pathlib import Path
root = Path(__file__).resolve().parents[3]
manifest = json.loads(Path(__file__).with_name('rgbd_archive_manifest.json').read_text())
for entry in manifest['archives']:
    target = root / entry['original_path']
    if target.exists():
        if hashlib.sha256(target.read_bytes()).hexdigest() != entry['sha256']:
            raise RuntimeError(f'Existing archive mismatch: {target}')
        print('Already verified:', target.name)
        continue
    temp = target.with_suffix('.zip.partial')
    digest = hashlib.sha256()
    with temp.open('wb') as out:
        for part in entry['parts']:
            data = (root / part['path']).read_bytes()
            if len(data) != part['bytes'] or hashlib.sha256(data).hexdigest() != part['sha256']:
                raise RuntimeError(f'Chunk mismatch: {part["path"]}')
            out.write(data); digest.update(data)
    if temp.stat().st_size != entry['bytes'] or digest.hexdigest() != entry['sha256']:
        raise RuntimeError('Reconstructed archive mismatch')
    temp.rename(target)
    print('Restored and verified:', target.name)
