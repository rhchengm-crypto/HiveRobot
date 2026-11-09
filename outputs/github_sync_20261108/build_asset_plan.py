"""Prepare lossless asset plan and every-file publication coverage, no GitHub I/O."""
from pathlib import Path
import hashlib,json,zipfile,collections
ROOT=Path(__file__).resolve().parents[2]; OUT=ROOT/'outputs/github_sync_20261108'
inv=json.loads((OUT/'data_inventory.json').read_text(encoding='utf-8'))
hash_bytes=lambda b:hashlib.sha256(b).hexdigest()
aux=OUT/'auxiliary_media_20261108.zip'
if not aux.exists():
    with zipfile.ZipFile(aux,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        for f in inv['unarchived_media']:z.write(ROOT/f['path'],f['path'])
with zipfile.ZipFile(aux) as z:
    assert z.testzip() is None
    for f in inv['unarchived_media']:assert hash_bytes(z.read(f['path']))==f['sha256']
assets=[]
for a in inv['archives']:
    parts=Path(a['path']).parts
    name=parts[1]+'__'+parts[-1]
    assets.append({'name':name,'local_path':a['path'],'bytes':a['bytes'],'sha256':a['sha256'],'role':'original-lossless-rgbd' if a['path'].endswith('rgbd_evidence.zip') else 'original-control-preparation-analysis','zip_crc_ok':a['zip_crc_ok'],'members':a['members'],'rgb_frames':a['rgb_frames'],'depth_frames':a['depth_frames']})
for p,role in ((aux,'unarchived-original-auxiliary-images'),(OUT/'final_runtime_snapshot_20261108.zip','final-runtime-and-learning-snapshot'),(OUT/'remote_gap_evidence_20261108.zip','remaining-remote-only-source-evidence')):
    if not p.exists():continue
    with zipfile.ZipFile(p) as z:
        assert z.testzip() is None
        n=len([i for i in z.infolist() if not i.is_dir()])
    assets.append({'name':p.name,'local_path':p.relative_to(ROOT).as_posix(),'bytes':p.stat().st_size,'sha256':hash_bytes(p.read_bytes()),'role':role,'zip_crc_ok':True,'members':n})
assert len({a['name'] for a in assets})==len(assets)
assert all(a['bytes']<2_000_000_000 for a in assets)
by_path={a['local_path']:a for a in assets}
member_index=collections.defaultdict(list)
for a in assets:
    if a['role']=='original-lossless-rgbd':continue
    with zipfile.ZipFile(ROOT/a['local_path']) as z:
        for i in z.infolist():
            if i.is_dir():continue
            b=z.read(i); h=hash_bytes(b)
            member_index[(len(b),h)].append({'asset_name':a['name'],'archive_path':a['local_path'],'member':i.filename,'sha256':h,'bytes':len(b)})
coverage=[]
for f in inv['files']:
    row={k:f[k] for k in ('path','bytes','sha256','extension')}
    if f['extension']=='.zip':
        row['primary']={'tier':'release-original-zip','asset_name':by_path[f['path']]['name']}
    elif f['proposed_tier']=='git-text-evidence':
        row['primary']={'tier':'git-text','repository_path':f['path']}
    else:
        matches=member_index.get((f['bytes'],f['sha256']),[])
        assert matches, f"No publication coverage for {f['path']}"
        row['primary']=dict(tier='release-zip-member',**matches[0])
    row['alternate_archive_members']=member_index.get((f['bytes'],f['sha256']),[])
    coverage.append(row)
excluded=[]
for d in inv['directories']:
    for p in (ROOT/d['directory']).rglob('*'):
        if p.is_file() and '__pycache__' in p.parts:excluded.append({'path':p.relative_to(ROOT).as_posix(),'bytes':p.stat().st_size,'reason':'Generated Python bytecode cache; source files are retained.'})
plan={'motion_issued':False,'github_mutations':False,'asset_count':len(assets),'asset_bytes':sum(a['bytes'] for a in assets),'assets':assets,'original_rgbd_archive_count':sum(a['role']=='original-lossless-rgbd' for a in assets),'original_rgbd_pairs':sum(a.get('depth_frames',0) for a in assets if a['role']=='original-lossless-rgbd'),'pending_assets':[],'no_lossy_recompression':True,'no_raw_rgbd_frame_deduplication':True}
for name in ('final_runtime_snapshot_20261108.zip','remote_gap_evidence_20261108.zip'):
    if not (OUT/name).exists():plan['pending_assets'].append(name)
receipt={'scope':inv['scope'],'snapshot_files':len(coverage),'snapshot_bytes':sum(f['bytes'] for f in coverage),'all_snapshot_files_have_primary_coverage':True,'files':coverage,'large_git_exclusions':[x for x in coverage if x['bytes']>=50*1024*1024 and x['extension']!='.zip'],'excluded_generated_caches':excluded,'private_exclusions':['outputs/mac_handoff_20261107/ (private chat/migration archive, outside today evidence scope)','Local SSH private keys and credential stores (outside evidence roots; never copied)','tmp/ and Python dependency/bytecode caches (source and mathematical reproducer scripts remain included)'],'sensitive_pattern_findings':inv['secret_hygiene_findings'],'publication_note':'This is the intended coverage plan. Root publication receipts verify actual Git objects and uploaded Release assets after publication.'}
gap_receipt=OUT/'remote_gap_capture_receipt.json'
if gap_receipt.exists():
    gr=json.loads(gap_receipt.read_text(encoding='utf-8'))
    receipt['remote_only_file_snapshots']=[{'source':f['source'],'bytes':f['bytes'],'sha256':f['sha256'],'primary':{'tier':'release-zip-member','asset_name':'remote_gap_evidence_20261108.zip','member':f['member']},'source_stable_during_read':f['source_stable_during_read'],'sealed_log':f['sealed_log'],'finite_prefix_snapshot':f['finite_prefix_snapshot'],'matches_earlier_inventory':f['matches_earlier_inventory'],'earlier_inventory_prefix_preserved':f.get('earlier_inventory_prefix_preserved')} for f in gr['files']]
    receipt['all_remote_gap_sources_have_verified_capture']=gr['all_member_sha256_verified']
(OUT/'asset_plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(OUT/'every_file_coverage.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:plan[k] for k in ('asset_count','asset_bytes','original_rgbd_pairs','pending_assets')},ensure_ascii=False,indent=2))
print('Every-file coverage',len(coverage),'files; caches excluded',len(excluded))
