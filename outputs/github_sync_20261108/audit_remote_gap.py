"""Compare downloaded Jetson filesystem inventory with local evidence identities."""
from pathlib import Path
import hashlib,json,zipfile,collections
ROOT=Path(__file__).resolve().parents[2]; OUT=ROOT/'outputs/github_sync_20261108'
remote=json.loads((OUT/'jetson_inventory.json').read_text(encoding='utf-8-sig'))
inv=json.loads((OUT/'data_inventory.json').read_text(encoding='utf-8'))
index=collections.defaultdict(list)
for f in inv['files']:index[(f['bytes'],f['sha256'])].append(f['path'])
for p in (ROOT/'scripts').rglob('*'):
    if p.is_file() and '__pycache__' not in p.parts and p.stat().st_size<30*1024*1024:
        index[(p.stat().st_size,hashlib.sha256(p.read_bytes()).hexdigest())].append(p.relative_to(ROOT).as_posix())
# Archive-only manifests/receipts must not be mistaken for missing evidence.
for a in inv['archives']:
    if a['path'].endswith('rgbd_evidence.zip'):continue
    with zipfile.ZipFile(ROOT/a['path']) as z:
        for i in z.infolist():
            if i.is_dir():continue
            b=z.read(i)
            index[(len(b),hashlib.sha256(b).hexdigest())].append(a['path']+'!'+i.filename)
raw=[]; unknown=[]; coverage=[]; archive_matches=[]
for d in remote['directories']:
    for a in d['archives']:
        matches=index.get((a['bytes'],a['sha256']),[])
        archive_matches.append({'remote_path':a['path'],'bytes':a['bytes'],'sha256':a['sha256'],'local_identical':matches})
    if d['area']=='rgbd_audits':
        candidates=[]
        expected={(x['name'],x['bytes']) for x in d['raw_members']}
        for a in inv['archives']:
            if not a['path'].endswith('rgbd_evidence.zip'):continue
            with zipfile.ZipFile(ROOT/a['path']) as z:
                members={(i.filename,i.file_size) for i in z.infolist() if not i.is_dir()}
            if expected <= members:
                candidates.append({'archive':a['path'],'bytes':a['bytes'],'sha256':a['sha256'],'all_raw_member_names_and_sizes_present':True,'extra_archive_members':len(members-expected)})
        raw.append({'remote_path':d['path'],'raw_files':d['files'],'raw_bytes':d['bytes'],'rgb_frames':d['extensions'].get('.jpg',0),'depth_frames':d['extensions'].get('.npy',0),'local_complete_archives':candidates,'coverage_level':'All raw filenames and byte sizes checked; earlier download receipts additionally verify archive/member hashes.'})
    else:
        for f in d.get('members',[]):
            p=d['path']+'/'+f['name']; matches=index.get((f['bytes'],f['sha256']),[])
            record={'remote_path':p,'bytes':f['bytes'],'sha256':f['sha256'],'local_identical':matches}
            if matches:coverage.append(record)
            else:unknown.append(record)
for f in remote['top_files']:
    matches=index.get((f['bytes'],f['sha256']),[])
    record={'remote_path':f['path'],'bytes':f['bytes'],'sha256':f['sha256'],'local_identical':matches}
    if matches:coverage.append(record)
    else:unknown.append(record)
# Avoid repeat download when identical unique blobs appear in several remote contexts.
unique={}
for f in unknown:
    k=(f['bytes'],f['sha256']); unique.setdefault(k,{'bytes':f['bytes'],'sha256':f['sha256'],'remote_paths':[]})['remote_paths'].append(f['remote_path'])
report={'motion_issued':False,'remote_filesystem_only':True,'raw_rgbd_sessions':raw,'raw_rgbd_session_count':len(raw),'raw_rgbd_pairs':sum(x['rgb_frames'] for x in raw),'all_today_raw_rgbd_sessions_have_complete_local_archive':all(x['local_complete_archives'] for x in raw),'remote_archives':archive_matches,'covered_nonraw_files':len(coverage),'covered_nonraw_file_records':coverage,'unmatched_nonraw_files':unknown,'unmatched_nonraw_unique_blobs':list(unique.values()),'unmatched_nonraw_unique_bytes':sum(x['bytes'] for x in unique.values()),'note':'Unknown remote SHA does not mean lost; some are remote collector-side scripts and metadata not included in payload manifests. Capture all unknowns as a small gap ZIP for all-data publication. Root final snapshot is independently collected.'}
(OUT/'remote_gap_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:report[k] for k in ('raw_rgbd_session_count','raw_rgbd_pairs','all_today_raw_rgbd_sessions_have_complete_local_archive','covered_nonraw_files','unmatched_nonraw_unique_bytes')},indent=2))
print('UNMATCHED',len(unknown),'files',len(unique),'unique blobs')
for f in unknown:print(f['bytes'],f['remote_path'])
