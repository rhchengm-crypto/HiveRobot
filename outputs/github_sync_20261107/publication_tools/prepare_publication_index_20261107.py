from collections import Counter
from datetime import datetime,timezone
import hashlib,json,shutil
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'outputs/github_sync_20261107'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
save=lambda p,x:p.write_text(json.dumps(x,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
nov7=read(out/'nov7_assets_receipt.json');nov6=read(out/'nov6_backfill_assets_receipt.json')
downloads=read(out/'remote_rgbd/download_receipt.json');texts=read(out/'text_evidence_receipt.json')
assert nov7['status']=='complete' and nov7['verified_archive_count']==40
assert nov6['complete'] and nov6['verified_asset_count']==15 and downloads['complete']
assert texts['status']=='all-selected-text-evidence-published'
rows=[read(p) for p in sorted((out/'asset_receipts').glob('*.json'))]
assert len(rows)==57 and len({r['name'] for r in rows})==57
base='https://github.com/rhchengm-crypto/HiveRobot/releases/download/experiments-2026-11-07/'
for r in rows:
    p=root/r['local_path'];assert p.is_file() and p.stat().st_size==r['bytes']
    assert hashlib.sha256(p.read_bytes()).hexdigest()==r['sha256']
    assert r['remote_digest']=='sha256:'+r['sha256'] and r['state']=='uploaded'
    r['upload_time_url']=r['url'];r['url']=base+r['name']
    n=r['name']
    r['group']=('nov6-rgbd-backfill' if n.startswith('backfill_nov6__') else
        'final-runtime' if n=='final_runtime_snapshot_20261107.zip' else
        'auxiliary-media' if n=='nov7_auxiliary_media.zip' else
        'nov7-rgbd' if n.endswith('__rgbd_evidence.zip') else 'nov7-control-and-preparation')
counts=dict(Counter(x['group'] for x in rows))
assert counts=={'nov7-control-and-preparation':28,'nov7-rgbd':12,'nov6-rgbd-backfill':15,'final-runtime':1,'auxiliary-media':1},counts
manifest=dict(schema=1,experimental_date='2026-11-07',timezone='America/Los_Angeles',
    created_at_utc=datetime.now(timezone.utc).isoformat(),
    repository='rhchengm-crypto/HiveRobot',release_tag='experiments-2026-11-07',
    release_url='https://github.com/rhchengm-crypto/HiveRobot/releases/tag/experiments-2026-11-07',
    core_commit=read(out/'core_receipt.json')['verified_remote_commit'],
    text_evidence_commit=texts['verified_remote_commit'],
    assets=rows,asset_count=len(rows),asset_bytes=sum(x['bytes'] for x in rows),groups=counts,
    checksum_verification='Local archive SHA256 equals GitHub-computed asset digest for every payload',
    nov7_rgb_depth_pairs=5100,nov6_backfill_rgb_depth_pairs=10360,
    private_chat_export_publicly_uploaded=False,original_data_lossless=True,motion_issued=False)
save(out/'publication_manifest.json',manifest)
# Preserve the publishing/collection tools as provenance, without chat exporters or credentials.
tool_names=['audit_publication_20261107.py','audit_publication_remote_20261107.py',
 'backfill_nov6_rgbd_20261107.py','upload_nov6_backfill_20261107.py',
 'upload_nov7_assets_20261107.py','github_assets_20261107.py','github_final_snapshot_20261107.py',
 'verify_final_snapshot_20261107.py','github_latest_home_20261107.py','extract_latest_home_20261107.py',
 'publish_core_20261107.py','publish_text_evidence_20261107.py']
toolsdir=out/'publication_tools';toolsdir.mkdir(exist_ok=True)
for n in tool_names:
    p=root/'tmp'/n
    if p.is_file():shutil.copy2(p,toolsdir/n)
header='''# November 7 experiment publication and November 6 RGB-D backfill

The repository contains current code, model JSON, validation registries, frozen per-run code/models, full text control evidence and the daily handoff. GitHub Release assets preserve original binary archives; this manifest maps each original path to its checksum and canonical download URL.

Payloads are lossless. No near-duplicate RGB/depth frames were removed. The private Mac migration ZIP and full personal chat export are not public experiment assets.

## Components

| Component | Count | Location |
|---|---:|---|
| Current controller/model/reference code | 54 files in initial publication | core_manifest.json and core_receipt.json |
| New text control evidence, frozen source/model snapshots, traces and diagnostics | 961 files | text_evidence_receipt.json; ordinary repository files |
| November 7 original RGB-D | 12 ZIPs; 5,100 RGB/depth pairs | Release group nov7-rgbd |
| November 7 original control/preparation/repair ZIPs | 28 ZIPs | Release group nov7-control-and-preparation |
| November 6 missing original RGB-D, retrieved now | 15 ZIPs; 10,360 RGB/depth pairs | Release group nov6-rgbd-backfill |
| Auxiliary images/arrays | 24 original files in 1 ZIP | nov7_auxiliary_media.zip |
| Final Jetson code/models/learning and complete run logs | 79 files in 1 ZIP | final_runtime_snapshot_20261107.zip |

The final Jetson archive includes the Home and subsequent hold after A7 acceptance, plus complete model/motion run logs. Some legacy adapters, historical local log copies and the local Clearance learning cache differ from installed files; final_snapshot_verification.json records these differences. The current installed version is frozen in the release; do not overwrite Jetson with an entire historical data directory.

The approved revised STEP is source_geometry/approved_arm_reference.step in the repository, SHA256 f7729567ef511d1d9dc38b250dcd276ab09b7aa84fff98f49c0e068971ef9718. Per-trial model/code SHA and operator observations are recorded in HANDOFF_20261107.md and the trial registries; physical acceptance does not turn requested XYZ or encoder FK into external measured TCP labels.

## Download and verify

Run from a repository clone, with Python 3:

```bash
# List all payloads without downloading.
python outputs/github_sync_20261107/restore_publication_data.py
# Download and verify only November 7 RGB/depth.
python outputs/github_sync_20261107/restore_publication_data.py --group nov7-rgbd --download
# Download every payload, preserving original ZIP bytes.
python outputs/github_sync_20261107/restore_publication_data.py --group all --download
```

Extract each archive into its own evidence directory. Original source directories are retained on Jetson; local originals also remain. This script performs file transfer/checksums only and does not access robot control.

## Exact asset index

| Group | Asset | Bytes | SHA256 |
|---|---|---:|---|
'''
table='\n'.join('| '+r['group']+' | ['+r['name']+']('+r['url']+') | '+str(r['bytes'])+' | `'+r['sha256']+'` |' for r in rows)
(out/'README.md').write_text(header+table+'\n',encoding='utf-8')
body='''November 7 seven-axis geometry/dynamics experiments, including accepted A7 v32 and the shared validated reference bank. Includes original successful and failed trial evidence and a lossless backfill of previously unpublished November 6 RGB-D.

Code, exact per-run model/source mapping and scientific limits: [HANDOFF_20261107.md](https://github.com/rhchengm-crypto/HiveRobot/blob/main/HANDOFF_20261107.md).

- 12 November 7 RGB-D ZIPs (5,100 RGB/depth pairs).
- 28 November 7 control, preparation and repair ZIPs.
- 15 November 6 RGB-D backfill ZIPs (10,360 RGB/depth pairs).
- One auxiliary-media ZIP (24 original files).
- One final Jetson runtime snapshot (79 code/model/learning/log files).

Every payload has a GitHub-computed SHA256 matching the local original. Source frames were retained losslessly. See publication_manifest.json and restore_publication_data.py for downloads and checksum verification. Full private chat/Mac migration files are excluded.

A7 v32 was accepted for empty-load approach/descent/hold; this does not certify loaded grasping, every board point, or the fault recovery branch. No robot movement was issued during publication.
'''
(out/'release_body.md').write_text(body,encoding='utf-8')
print(json.dumps(dict(payloads=len(rows),bytes=manifest['asset_bytes'],groups=counts)))
