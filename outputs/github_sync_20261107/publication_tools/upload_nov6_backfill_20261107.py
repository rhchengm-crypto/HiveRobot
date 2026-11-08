"""Upload only verified historical RGB-D ZIPs to the existing draft Release."""
from pathlib import Path
import concurrent.futures,importlib.util,json,os,time

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/github_sync_20261107'
for _ in range(90):
    p=OUT/'remote_rgbd/download_receipt.json'
    if p.exists():
        download=json.loads(p.read_text(encoding='utf-8'))
        if download.get('complete'):break
    time.sleep(5)
else:raise RuntimeError('Historical download is incomplete; no unverified file will be uploaded.')
assert download['downloaded_sessions']==download['expected_sessions']==15
assert download['total_source_bytes']==6_999_873_988
spec=importlib.util.spec_from_file_location('github_assets_20261107',ROOT/'tmp/github_assets_20261107.py')
helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
helper.token()  # Authenticate once; never print or save credentials.
assert helper.release()['id']==438072261
rows=[]; errors=[]
def work(row):
    assert row['verified'] and row['every_member_sha256_verified'] and row['zip_crc_verified']
    result=helper.upload(ROOT/row['local_path'],'backfill_nov6__'+row['session']+'.zip')
    assert result['sha256']==row['sha256'] and result['bytes']==row['bytes']
    return dict(result,session=row['session'],source_directory=row['source_directory'],source_bytes=row['source_bytes'],rgb_frames=row['rgb_frames'],depth_frames=row['depth_frames'],every_member_sha256_verified=True,zip_crc_verified=True)
def save():
    result={'motion_issued':False,'release_id':helper.release()['id'],'draft_published_by_this_script':False,'expected_assets':15,'verified_asset_count':len(rows),'complete':len(rows)==15 and not errors,'archive_bytes':sum(r['bytes'] for r in rows),'source_bytes':sum(r['source_bytes'] for r in rows),'verified_assets':sorted(rows,key=lambda x:x['name']),'errors':errors,'download_receipt':'outputs/github_sync_20261107/remote_rgbd/download_receipt.json'}
    p=OUT/'nov6_backfill_assets_receipt.json';tmp=p.with_suffix('.json.tmp');tmp.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');os.replace(tmp,p)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    pending={pool.submit(work,row):row['session'] for row in download['verified_downloads']}
    for future in concurrent.futures.as_completed(pending):
        try:rows.append(future.result())
        except Exception as e:errors.append({'session':pending[future],'error':repr(e)})
        save();print('Nov6 release progress '+str(len(rows))+'/15; errors '+str(len(errors)),flush=True)
if errors:raise RuntimeError('Release uploads incomplete: '+str(errors))
print('All 15 historical RGB-D archives uploaded and GitHub SHA256 verified; release remains draft.',flush=True)
