"""Read-only scientific evidence inventory; never opens robot or network."""
from pathlib import Path
import collections, hashlib, json, re, zipfile

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'outputs/github_sync_20261108'
DATE = '20261108'
roots = sorted(d for d in (ROOT / 'outputs').iterdir() if d.is_dir() and DATE in d.name and d != OUT)
sha = lambda b: hashlib.sha256(b).hexdigest()
files, archives, findings, extension_stats, groups = [], [], [], {}, collections.defaultdict(list)
patterns = {
    'private_key_header': re.compile(rb'-----BEGIN (?:OPENSSH |RSA |EC |DSA |ENCRYPTED )?PRIVATE KEY-----'),
    'github_classic_token': re.compile(rb'gh[pousr]_[A-Za-z0-9]{36,}'),
    'github_fine_grained_token': re.compile(rb'github_pat_[A-Za-z0-9_]{50,}'),
    'openai_project_token': re.compile(rb'sk-proj-[A-Za-z0-9_-]{32,}'),
}
for d in roots:
    for p in sorted(d.rglob('*')):
        if not p.is_file() or '__pycache__' in p.parts: continue
        b = p.read_bytes()
        rel = p.relative_to(ROOT).as_posix()
        h = sha(b)
        typ = p.suffix.lower() or '(none)'
        e = extension_stats.setdefault(typ, {'files': 0, 'bytes': 0})
        e['files'] += 1; e['bytes'] += len(b)
        row = {'path': rel, 'bytes': len(b), 'sha256': h, 'extension': typ}
        row['proposed_tier'] = 'release-original-archive' if typ == '.zip' else ('release-auxiliary-media' if typ in ('.npy','.npz','.jpg','.jpeg','.png','.mp4','.avi','.webm') else ('release-covered-large-text' if len(b) >= 50*1024*1024 else 'git-text-evidence'))
        if p.name.lower() in ('id_rsa','id_ed25519','.env','credentials.json'): findings.append({'path':rel,'kind':'sensitive_filename_review'})
        if typ != '.zip' and b'\0' not in b[:8192]:
            for name, pat in patterns.items():
                if pat.search(b): findings.append({'path':rel,'kind':name,'secret_value_not_recorded':True})
        files.append(row); groups[(len(b), h)].append(rel)
        if typ == '.zip':
            with zipfile.ZipFile(p) as z:
                bad = z.testzip()
                members = [i for i in z.infolist() if not i.is_dir()]
                a = dict(row, zip_crc_ok=bad is None, members=len(members), unpacked_bytes=sum(i.file_size for i in members), rgb_frames=sum(i.filename.lower().endswith(('.jpg','.jpeg')) for i in members), depth_frames=sum(i.filename.lower().endswith('.npy') for i in members), video_files=sum(i.filename.lower().endswith(('.mp4','.avi','.webm')) for i in members))
                # Archive content scan is limited to textual members; never exposes matched bytes.
                for i in members:
                    if Path(i.filename).suffix.lower() in ('.py','.json','.jsonl','.txt','.md','.yaml','.yml','.sh','.ps1','.env'):
                        x=z.read(i)
                        for name,pat in patterns.items():
                            if pat.search(x): findings.append({'path':rel+'!'+i.filename,'kind':name,'secret_value_not_recorded':True})
                archives.append(a)

by_dir = []
for d in roots:
    prefix=d.relative_to(ROOT).as_posix()+'/'
    rows=[f for f in files if f['path'].startswith(prefix)]
    ars=[f for f in archives if f['path'].startswith(prefix)]
    by_dir.append({'directory':prefix.rstrip('/'),'files':len(rows),'bytes':sum(f['bytes'] for f in rows),'archive_count':len(ars),'archive_bytes':sum(f['bytes'] for f in ars)})
dupes=[{'bytes_each':n,'sha256':h,'paths':ps,'duplicate_storage_bytes':n*(len(ps)-1)} for (n,h),ps in groups.items() if len(ps)>1]
covered_large=[]
for f in files:
    if f['proposed_tier']!='release-covered-large-text':continue
    p=ROOT/f['path']; matches=[]
    for a in archives:
        # Only claim identity when member content SHA256 equals the external file.
        with zipfile.ZipFile(ROOT/a['path']) as z:
            for i in z.infolist():
                if i.file_size==f['bytes'] and Path(i.filename).name==p.name and sha(z.read(i))==f['sha256']:
                    matches.append({'archive':a['path'],'member':i.filename})
    covered_large.append(dict(f,verified_covering_archives=matches))
unarchived_media=[]
for f in files:
    if f['proposed_tier']!='release-auxiliary-media':continue
    matches=[]
    for a in archives:
        with zipfile.ZipFile(ROOT/a['path']) as z:
            for i in z.infolist():
                if i.file_size==f['bytes'] and Path(i.filename).name==Path(f['path']).name and sha(z.read(i))==f['sha256']:
                    matches.append({'archive':a['path'],'member':i.filename})
    f['verified_covering_archives']=matches
    if not matches:unarchived_media.append(f)
report={'date':DATE,'scope':'All files in outputs top-level directories containing 20261108, excluding this inventory directory and Python bytecode caches. Remote gap audit is a separate receipt.','motion_issued':False,'network_accessed_by_this_script':False,'total_files':len(files),'total_bytes':sum(f['bytes'] for f in files),'directories':by_dir,'extensions':extension_stats,'archives':archives,'files':files,'duplicate_content_groups':dupes,'duplicate_storage_bytes':sum(x['duplicate_storage_bytes'] for x in dupes),'large_text_coverage':covered_large,'unarchived_media':unarchived_media,'secret_hygiene_findings':findings,'secret_hygiene_limit':'High-confidence key/token signatures and sensitive filenames only; no claim of complete secret detection.','proposed_release':{'existing_original_archives':len(archives),'existing_original_archive_bytes':sum(x['bytes'] for x in archives),'max_asset_bytes':max((x['bytes'] for x in archives),default=0),'all_below_2GB':all(x['bytes']<2_000_000_000 for x in archives),'unarchived_auxiliary_files':len(unarchived_media),'unarchived_auxiliary_bytes':sum(x['bytes'] for x in unarchived_media)},'git_text_bytes_if_all_small_text_retained':sum(x['bytes'] for x in files if x['proposed_tier']=='git-text-evidence')}
OUT.mkdir(exist_ok=True)
(OUT/'data_inventory.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
lines=['# November 8 evidence publication inventory','',f"Scope: {len(roots)} experiment directories; {len(files)} files; {report['total_bytes']:,} bytes including compressed/extracted duplicates.",'','| Directory | Files | Bytes | Archives | Archive bytes |','|---|---:|---:|---:|---:|']
lines += [f"| {x['directory']} | {x['files']} | {x['bytes']:,} | {x['archive_count']} | {x['archive_bytes']:,} |" for x in by_dir]
lines += ['',f"Release: retain all {len(archives)} original ZIPs unchanged ({sum(x['bytes'] for x in archives):,} bytes). Largest {report['proposed_release']['max_asset_bytes']:,} bytes; every existing archive is below 2 GB.",f"Git text tier: {report['git_text_bytes_if_all_small_text_retained']:,} bytes before deduplication. Keep explanatory reports, source snapshots, trial models, failure diagnostics, fitting inputs/results, acceptance history, plans, and control traces below 50 MiB.",'','Do not add the 138,752,252-byte rolling motion log directly to Git. Its exact SHA256 is covered by the original A7 control ZIP and listed in data_inventory.json. The 24,801,098-byte rolling hold log is also archive-covered; Git copies are optional. Preserve run-specific traces and summaries in Git.','',f"Unarchived auxiliary media: {len(unarchived_media)} files, {sum(x['bytes'] for x in unarchived_media):,} bytes; package these in a separate ZIP if nonzero.",f"Duplicate file-content storage: {report['duplicate_storage_bytes']:,} bytes (compressed/extracted duplication not all counted here).",f"High-confidence hygiene findings: {len(findings)}. No credential value is stored in this report.",'','## Original archives','', '| Path | Bytes | SHA256 | RGB | Depth | CRC |','|---|---:|---|---:|---:|---|']
lines += [f"| {a['path']} | {a['bytes']:,} | `{a['sha256']}` | {a['rgb_frames']} | {a['depth_frames']} | {a['zip_crc_ok']} |" for a in archives]
lines += ['', 'This inventory does not publish, download, alter runtime files, query motors, or issue movement. See remote_gap_audit.json for live Jetson filesystem coverage.','']
(OUT/'inventory_summary.md').write_text('\n'.join(lines),encoding='utf-8')
print(json.dumps({k:report[k] for k in ('total_files','total_bytes','proposed_release','git_text_bytes_if_all_small_text_retained','secret_hygiene_findings')},ensure_ascii=False,indent=2))
