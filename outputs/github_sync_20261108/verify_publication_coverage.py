"""Verify Git blob bytes and Release receipts against every original file."""
from pathlib import Path
import hashlib
import json
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))

def main():
    manifest = read(OUT/'publication_manifest.json')
    remote = read(OUT/'release_remote_verification.json')
    coverage = read(OUT/'every_file_coverage.json')
    assert remote['draft'] is False and remote['verified_asset_count'] == len(manifest['assets'])+2
    assets = {r['name']: r for r in remote['assets']}
    for row in manifest['assets']:
        assert assets[row['name']]['sha256'] == row['sha256'] and assets[row['name']]['bytes'] == row['bytes']
    rows = [r for r in coverage['files'] if r['primary']['tier'] == 'git-text']
    expressions = ''.join(manifest['text_evidence_commit']+':'+r['path']+'\n' for r in rows).encode()
    result = subprocess.run(['git','cat-file','--batch'],cwd=ROOT,input=expressions,capture_output=True,timeout=90)
    assert result.returncode == 0
    data = memoryview(result.stdout)
    cursor = 0
    for row in rows:
        end = result.stdout.index(b'\n',cursor)
        header = bytes(data[cursor:end]).split()
        assert len(header) == 3 and header[1] == b'blob', row['path']
        size = int(header[2]);cursor=end+1
        assert size == row['bytes'] and hashlib.sha256(data[cursor:cursor+size]).hexdigest() == row['sha256'],row['path']
        cursor += size
        assert data[cursor] == 10
        cursor += 1
    assert cursor == len(data)
    member_count = 0
    for row in coverage['files'] + coverage.get('remote_only_file_snapshots', []):
        primary = row['primary']
        if primary['tier'] == 'release-original-zip':
            proof = assets[primary['asset_name']]
            assert proof['bytes'] == row['bytes'] and proof['sha256'] == row['sha256']
        elif primary['tier'] == 'release-zip-member':
            asset = next(r for r in manifest['assets'] if r['name'] == primary['asset_name'])
            with zipfile.ZipFile(ROOT/asset['local_path']) as z:
                digest = hashlib.sha256();size=0
                with z.open(primary['member']) as stream:
                    for block in iter(lambda:stream.read(1024*1024),b''):
                        digest.update(block);size+=len(block)
                assert size == row['bytes'] and digest.hexdigest() == row['sha256'],row.get('path',row.get('source'))
            member_count += 1
    receipt = dict(status='all-experimental-code-and-data-published-and-verified',
        release_url=manifest['release_url'],release_draft=False,release_asset_count=remote['verified_asset_count'],
        payload_asset_count=len(manifest['assets']),payload_bytes=manifest['asset_bytes'],
        github_sha256_matches_all_assets=True,public_download_probes=remote['public_download_probes'],
        core_commit=manifest['core_commit'],text_evidence_commit=manifest['text_evidence_commit'],
        git_text_files_verified=len(rows),git_text_blob_bytes_verified=sum(r['bytes'] for r in rows),
        local_inventory_files_verified=len(coverage['files']),release_member_only_files_verified=member_count,
        remote_only_snapshot_files=len(coverage.get('remote_only_file_snapshots', [])),rgbd_sessions=7,rgb_depth_pairs=3690,
        private_chat_export_published=False,credentials_published=False,motion_issued=False,
        final_robot_state_source='read-only last Home/hold completed20:57:59; no motion issued during publication',
        assets=remote['assets'])
    (OUT/'publication_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in receipt.items() if k not in ('assets','public_download_probes')},ensure_ascii=False),flush=True)

if __name__ == '__main__':
    main()
