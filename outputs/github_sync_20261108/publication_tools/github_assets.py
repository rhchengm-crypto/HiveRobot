"""Publish checksum-bound November 8 evidence; credentials stay in memory.

This tool transfers files and GitHub metadata only. It does not import or
access robot controllers, serial ports, cameras, SSH, or private chat exports.
Existing assets are verified, never overwritten or deleted.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import http.client
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'outputs/github_sync_20261108'
REPO = 'rhchengm-crypto/HiveRobot'
TAG = 'experiments-2026-11-08'
USER_AGENT = 'HiveRobot-November8-experiment-publication'
TOKEN = None


def now():
    return datetime.now(timezone.utc).isoformat()


def fail(message):
    raise RuntimeError(message)


def token():
    global TOKEN
    if TOKEN is None:
        environment = dict(os.environ, GIT_TERMINAL_PROMPT='0', GCM_INTERACTIVE='never')
        result = subprocess.run(['git', 'credential', 'fill'], cwd=ROOT,
            input='protocol=https\nhost=github.com\npath='+REPO+'.git\n\n',
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=90, env=environment)
        if result.returncode:
            fail('Noninteractive GitHub credential access failed')
        fields = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
        TOKEN = fields.get('password')
        if not TOKEN:
            fail('No GitHub credential returned')
    return TOKEN


def api(method, path, body=None, missing_ok=False):
    payload = None if body is None else json.dumps(body).encode('utf-8')
    request = urllib.request.Request('https://api.github.com'+path,
        data=payload, method=method, headers={
            'Authorization': 'Bearer '+token(),
            'Accept': 'application/vnd.github+json',
            'User-Agent': USER_AGENT, 'Content-Type': 'application/json',
            'X-GitHub-Api-Version': '2022-11-28'})
    try:
        with urllib.request.urlopen(request, timeout=90) as response:
            raw = response.read()
    except urllib.error.HTTPError as exc:
        if missing_ok and exc.code == 404:
            return None
        fail('GitHub API '+method+' returned HTTP '+str(exc.code))
    return json.loads(raw) if raw else None


def digest(path):
    checksum = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            checksum.update(block)
    return checksum.hexdigest()


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def release():
    value = json.loads((OUT/'release.json').read_text(encoding='utf-8'))
    if value.get('tag_name') != TAG:
        fail('Release record has the wrong tag')
    return value


def save_release(value):
    save(OUT/'release.json', {key: value[key]
        for key in ('id', 'tag_name', 'html_url', 'draft', 'target_commitish')})


def assets(release_id):
    rows = []
    for page in range(1, 10001):
        batch = api('GET', f'/repos/{REPO}/releases/{release_id}/assets?per_page=100&page={page}')
        rows.extend(batch)
        if len(batch) < 100:
            return rows
    fail('Asset pagination exceeded the declared bound')


def asset_name(name):
    if not isinstance(name, str) or not name or Path(name).name != name or any(c in name for c in '/\\\r\n'):
        fail('Asset name must be one filename')
    return name


def remote_digest(asset, expected):
    remote = asset.get('digest')
    if remote:
        if remote != 'sha256:'+expected:
            fail('GitHub-computed SHA256 mismatch: '+asset['name'])
        return remote, 'github-computed-digest'
    request = urllib.request.Request(asset['url'], headers={
        'Authorization': 'Bearer '+token(), 'Accept': 'application/octet-stream',
        'User-Agent': USER_AGENT})
    checksum = hashlib.sha256()
    with urllib.request.urlopen(request, timeout=180) as response:
        for block in iter(lambda: response.read(1024*1024), b''):
            checksum.update(block)
    if checksum.hexdigest() != expected:
        fail('Authenticated downloaded SHA256 mismatch: '+asset['name'])
    return 'sha256:'+expected, 'authenticated-download-sha256'


def verify_asset(asset, size, checksum):
    if asset.get('state') != 'uploaded' or asset.get('size') != size:
        fail('Conflicting or incomplete asset; no deletion attempted: '+asset['name'])
    return remote_digest(asset, checksum)


def receipt_for(path, asset, checksum, verification):
    try:
        local = path.relative_to(ROOT).as_posix()
    except ValueError:
        fail('Only repository-local evidence can be published')
    result = dict(local_path=local, name=asset['name'], bytes=path.stat().st_size,
        sha256=checksum, asset_id=asset['id'], url=asset['browser_download_url'],
        remote_digest=verification[0], verification=verification[1],
        state=asset['state'], verified_at_utc=now(), motion_issued=False)
    save(OUT/'asset_receipts'/(asset['name']+'.json'), result)
    return result


def upload(path, name=None):
    path = Path(path).resolve()
    if not path.is_file() or not path.is_relative_to(ROOT):
        fail('Upload input must be a repository-local regular file')
    name = asset_name(name or path.name)
    if any(part.lower().startswith('mac_handoff') for part in path.parts):
        fail('Private migration exports cannot be published')
    size, checksum = path.stat().st_size, digest(path)
    release_id = release()['id']
    for attempt in range(1, 4):
        matches = [row for row in assets(release_id) if row['name'] == name]
        if len(matches) > 1:
            fail('Duplicate remote asset names: '+name)
        if matches:
            row = receipt_for(path, matches[0], checksum, verify_asset(matches[0], size, checksum))
            print('VERIFIED '+name+' '+str(size), flush=True)
            return row
        connection = None
        try:
            connection = http.client.HTTPSConnection('uploads.github.com', timeout=180)
            endpoint = f'/repos/{REPO}/releases/{release_id}/assets?name='+urllib.parse.quote(name, safe='')
            connection.putrequest('POST', endpoint)
            for key, value in {'Authorization': 'Bearer '+token(),
                    'Accept': 'application/vnd.github+json', 'User-Agent': USER_AGENT,
                    'Content-Type': 'application/octet-stream', 'Content-Length': str(size)}.items():
                connection.putheader(key, value)
            connection.endheaders()
            sent, last_progress = 0, 0.
            with path.open('rb') as source:
                for block in iter(lambda: source.read(1024*1024), b''):
                    connection.send(block)
                    sent += len(block)
                    if time.monotonic()-last_progress >= 20:
                        print('UPLOADING '+name+' '+str(sent)+'/'+str(size), flush=True)
                        last_progress = time.monotonic()
            response = connection.getresponse()
            payload = response.read()
            if response.status not in (200, 201):
                fail('Upload returned HTTP '+str(response.status)+': '+name)
            asset = json.loads(payload)
            row = receipt_for(path, asset, checksum, verify_asset(asset, size, checksum))
            print('VERIFIED '+name+' '+str(size), flush=True)
            return row
        except Exception:
            if attempt == 3:
                raise
            print('RETRY '+name+' '+str(attempt)+'/3; existing remote bytes will only be verified', flush=True)
            time.sleep(2)
        finally:
            if connection:
                connection.close()


def expected_receipts():
    rows = [json.loads(path.read_text(encoding='utf-8'))
            for path in sorted((OUT/'asset_receipts').glob('*.json'))]
    if not rows:
        fail('No uploaded asset receipts exist')
    if len({row['name'] for row in rows}) != len(rows):
        fail('Duplicate receipt asset names')
    return rows


def verify_manifest_coverage(receipts):
    manifest_path = OUT/'publication_manifest.json'
    if not manifest_path.is_file():
        fail('publication_manifest.json must exist before publication')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('release_tag') != TAG:
        fail('Publication manifest has the wrong release tag')
    known = {row['name']: row for row in receipts}
    for row in manifest['assets']:
        name = asset_name(row['name'])
        found = known.get(name)
        if not found or any(found[key] != row[key] for key in ('bytes', 'sha256')):
            fail('Missing or mismatched manifest asset receipt: '+name)
    for name in ('publication_manifest.json', 'restore_publication_data.py'):
        path = OUT/name
        found = known.get(name)
        if not found or found['bytes'] != path.stat().st_size or found['sha256'] != digest(path):
            fail('Latest manifest/restore helper has not been uploaded: '+name)


def public_probe(asset, row):
    request = urllib.request.Request(asset['browser_download_url'], headers={'User-Agent': USER_AGENT})
    checksum, size = hashlib.sha256(), 0
    with urllib.request.urlopen(request, timeout=180) as response:
        for block in iter(lambda: response.read(1024*1024), b''):
            checksum.update(block)
            size += len(block)
    if size != row['bytes'] or checksum.hexdigest() != row['sha256']:
        fail('Unauthenticated public download mismatch: '+asset['name'])
    return dict(name=asset['name'], bytes=size, sha256=checksum.hexdigest(),
                unauthenticated_download_sha256_verified=True)


def verify(probes=(), require_manifest=False):
    remote = api('GET', f'/repos/{REPO}/releases/'+str(release()['id']))
    if remote['tag_name'] != TAG:
        fail('Remote release has the wrong tag')
    remote_assets = assets(remote['id'])
    remote_by_name = {row['name']: row for row in remote_assets}
    if len(remote_by_name) != len(remote_assets):
        fail('Duplicate remote asset names')
    receipts = expected_receipts()
    checked = []
    for row in receipts:
        asset = remote_by_name.get(row['name'])
        if not asset:
            fail('Missing remote asset: '+row['name'])
        proof = verify_asset(asset, row['bytes'], row['sha256'])
        local = ROOT/row['local_path']
        if not local.is_file() or local.stat().st_size != row['bytes'] or digest(local) != row['sha256']:
            fail('Published local original has changed: '+row['name'])
        checked.append(dict(name=row['name'], bytes=row['bytes'], sha256=row['sha256'],
            remote_digest=proof[0], verification=proof[1], state=asset['state'],
            url=asset['browser_download_url']))
    if require_manifest:
        verify_manifest_coverage(receipts)
    selected = list(dict.fromkeys(probes))
    if selected and remote['draft']:
        fail('Public download probes require a published release')
    if not selected and not remote['draft']:
        names = {row['name'] for row in receipts}
        selected = [name for name in ('publication_manifest.json', 'restore_publication_data.py') if name in names]
        payload = [row for row in receipts if row['name'].endswith('.zip')]
        if payload:
            selected.append(min(payload, key=lambda row: row['bytes'])['name'])
    receipt_by_name = {row['name']: row for row in receipts}
    public = []
    for name in selected:
        if name not in receipt_by_name:
            fail('Public probe has no matching receipt: '+name)
        public.append(public_probe(remote_by_name[name], receipt_by_name[name]))
    result = dict(status='all-receipted-assets-verified', checked_at_utc=now(),
        release_id=remote['id'], tag=TAG, draft=remote['draft'], url=remote['html_url'],
        remote_asset_count=len(remote_assets), verified_asset_count=len(checked),
        verified_bytes=sum(row['bytes'] for row in checked), assets=checked,
        public_download_probes=public, manifest_coverage_required=require_manifest,
        motion_issued=False, credentials_saved=False)
    save(OUT/'release_remote_verification.json', result)
    print(json.dumps({key: result[key] for key in ('status','draft','verified_asset_count','verified_bytes','url')}), flush=True)
    return result


def create(commit):
    if not re.fullmatch('[0-9a-fA-F]{40}', commit):
        fail('A full 40-character published commit SHA is required')
    existing = api('GET', f'/repos/{REPO}/releases/tags/'+TAG, missing_ok=True)
    if existing is not None:
        # A draft release may not have materialized a Git tag yet. In that
        # case only an identical explicit commitish is eligible for reuse.
        reference = api('GET', f'/repos/{REPO}/git/ref/tags/'+TAG, missing_ok=True)
        if reference is None:
            if not existing['draft'] or existing['target_commitish'].lower() != commit.lower():
                fail('Existing release lacks a matching tag/commit; no modification attempted')
            save_release(existing)
            print(json.dumps(release()), flush=True)
            return
        target = reference['object']
        for _ in range(10):
            if target['type'] != 'tag':
                break
            target = api('GET', f'/repos/{REPO}/git/tags/'+target['sha'])['object']
        if target['type'] != 'commit' or target['sha'] != commit.lower():
            fail('Existing release tag points at another commit; no modification attempted')
        save_release(existing)
    else:
        value = api('POST', f'/repos/{REPO}/releases', dict(tag_name=TAG,
            target_commitish=commit.lower(), draft=True,
            name='November 8 A7 repeat and accepted non-pawn A8 experiments',
            body='Evidence upload in progress. See HANDOFF_20261108.md in the repository.'))
        save_release(value)
    print(json.dumps(release()), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='mode', required=True)
    new = subparsers.add_parser('create'); new.add_argument('--commit', required=True)
    up = subparsers.add_parser('upload'); up.add_argument('path'); up.add_argument('--name')
    publish = subparsers.add_parser('publish'); publish.add_argument('--body-file', required=True, type=Path)
    check = subparsers.add_parser('verify')
    check.add_argument('--public-download-probe', action='append', default=[])
    check.add_argument('--require-manifest', action='store_true')
    args = parser.parse_args()
    if args.mode == 'create':
        create(args.commit)
    elif args.mode == 'upload':
        upload(args.path, args.name)
    elif args.mode == 'verify':
        verify(args.public_download_probe, args.require_manifest)
    else:
        verify(require_manifest=True)
        value = api('PATCH', f'/repos/{REPO}/releases/'+str(release()['id']),
            dict(draft=False, body=args.body_file.read_text(encoding='utf-8')))
        save_release(value)
        verify(require_manifest=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        # Never print arbitrary HTTP response bodies, credential stderr, or
        # request objects, which could contain authentication material.
        if isinstance(exc, RuntimeError):
            message = str(exc)
        else:
            message = type(exc).__name__+' during publication; inspect local state and receipts before retrying'
        print('ERROR: '+message, file=sys.stderr, flush=True)
        raise SystemExit(1)
