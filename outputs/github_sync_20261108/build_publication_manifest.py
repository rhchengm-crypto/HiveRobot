"""Bind the completed Git text publication to the lossless Release payloads."""
from pathlib import Path
import hashlib
import json
import urllib.parse

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
plan = read(OUT/'asset_plan.json')
core = read(OUT/'core_receipt.json')
text = read(OUT/'text_evidence_receipt.json')
assert plan['asset_count'] == 29 and not plan['pending_assets']
assert text['status'] == 'all-selected-text-evidence-published'
groups = {
    'original-lossless-rgbd':'nov8-rgbd',
    'original-control-preparation-analysis':'nov8-control-preparation-analysis',
    'unarchived-original-auxiliary-images':'auxiliary-media',
    'final-runtime-and-learning-snapshot':'final-runtime',
    'remaining-remote-only-source-evidence':'remote-gap',
}
base = 'https://github.com/rhchengm-crypto/HiveRobot/releases/download/experiments-2026-11-08/'
assets = []
for row in plan['assets']:
    p = ROOT/row['local_path']
    assert p.stat().st_size == row['bytes'] and sha(p) == row['sha256'], p
    assets.append(dict(row,group=groups[row['role']],url=base+urllib.parse.quote(row['name'],safe='')))
manifest = dict(schema=1,date_local='2026-11-08',timezone='America/Los_Angeles',
    release_tag='experiments-2026-11-08',
    release_url='https://github.com/rhchengm-crypto/HiveRobot/releases/tag/experiments-2026-11-08',
    handoff='HANDOFF_20261108.md',core_commit=core['verified_remote_commit'],
    text_evidence_commit=text['verified_remote_commit'],text_evidence_files=text['file_count'],
    text_evidence_bytes=text['raw_bytes'],asset_count=len(assets),asset_bytes=sum(a['bytes'] for a in assets),
    assets=assets,rgbd_sessions=7,rgb_depth_pairs=3690,
    full_local_file_coverage='outputs/github_sync_20261108/every_file_coverage.json',
    source_inventory='outputs/github_sync_20261108/data_inventory.json',
    final_snapshot_verification='outputs/github_sync_20261108/final_snapshot_verification.json',
    remote_gap_receipt='outputs/github_sync_20261108/remote_gap_capture_receipt.json',
    original_frames_retained_losslessly=True,source_archives_recompressed=False,
    overlapping_snapshots_are_not_independent_trials=True,
    credentials_or_private_chat_published=False,motion_issued=False,
    acceptance=dict(A7_repeat=True,A8_lower25_nonpawn_reference=True,pawn_excluded=True,
        loaded_grasp_verified=False,mechanical_limits_measured=False,
        offline_extra1p5mm_executed=False,new_fitted_dynamics_promoted=False),
    current_model=dict(rigid_sha256='83f5e4b240d48ce4aae1c8037922696c5f2f1ed120476c8ae7da9e6ca00d6fd7',
        hold_sha256='aa91af203c5888c4d21db3d1507ea22479b63913824f8bf85145c8924b4f342f',
        reference_config_sha256='8412367b371adacde6f0f3347327a6deea4b816608e69f64b354f7fb57288d23'),
    metadata_assets=['publication_manifest.json','restore_publication_data.py'])
(OUT/'publication_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
lines = [
    '# November 8 experiment data index', '',
    '[Handoff](../../HANDOFF_20261108.md) · [GitHub Release]('+manifest['release_url']+')', '',
    'All29 payloads are downloaded to this Windows host before GitHub upload. '
    'The31 final attachments include these payloads plus the manifest and restore helper. '
    'Publication completion is established by publication_receipt.json, not this pre-upload index.', '',
    '## Three corresponding parts', '',
    '| Part | Location |', '| --- | --- |',
    '| Code, models, tests and acceptance | `scripts/`, `HANDOFF_20261108.md`; per-run SHA mapping in the handoff. Core commit `'+manifest['core_commit']+'`. |',
    '| Control and research evidence | '+str(text['file_count'])+' Git text files, '+str(text['raw_bytes'])+' raw bytes; original control packages and final runtime/gap snapshots below. |',
    '| RGB-D and auxiliary images | Seven original lossless RGB-D packages, 3,690 paired frames; four auxiliary images in their separate ZIP. |', '',
    '## Exact downloadable payloads', '',
    '| Group | Asset | Local original | Bytes | SHA256 |', '| --- | --- | --- | ---: | --- |',
]
for a in assets:
    lines.append('| '+a['group']+' | ['+a['name']+']('+a['url']+') | `'+a['local_path']+'` | '+str(a['bytes'])+' | `'+a['sha256']+'` |')
lines += ['', '## Restore', '', '```bash',
    'python outputs/github_sync_20261108/restore_publication_data.py',
    'python outputs/github_sync_20261108/restore_publication_data.py --group nov8-rgbd --download',
    'python outputs/github_sync_20261108/restore_publication_data.py --group all --download', '```', '',
    'The helper verifies original bytes and never extracts archives or controls the robot. '
    'Extract each trial into a separate evidence directory so historical same-named models remain distinct.', '',
    'A8 acceptance applies to the executed low-height non-pawn posture. '
    'The new1.5mm mathematical descent was not executed; no loaded grasp or true physical limit was measured. '
    'The four-group fit remains an offline candidate because grouped validation worsened.', '',
    'The138,752,252-byte rolling motion JSONL is retained in original control archives, not discarded. '
    'The final snapshot retains219 source/model/learning/log files and the last user Home/hold. '
    'Nine local historical/log files differ from the remote snapshot and are kept as distinct versions. '
    'The gap archive adds26 captured remote-only records; the active web log is an explicitly finite snapshot, not a sealed final log.', '',
    'Private Mac/chat archives, SSH keys, credential stores and generated Python caches are excluded.', '',
]
(OUT/'README.md').write_text('\n'.join(lines),encoding='utf-8')
body = '''November 8 seven-axis robot experiments: A7 repeat, A8 inverse planning and quasi-static support repair, accepted non-pawn A8 posture, and a four-group offline dynamics fit.

Code, exact per-run models, failures and acceptance history: [HANDOFF_20261108.md](https://github.com/rhchengm-crypto/HiveRobot/blob/main/HANDOFF_20261108.md).

- 29 lossless payloads, 571,848,294 bytes, plus publication_manifest.json and restore_publication_data.py.
- Seven complete RGB-D sessions:3,690 RGB/depth pairs, with no frame deletion.
- Original control/preparation/analysis archives, four auxiliary images, final219-file Jetson runtime snapshot, and26 remote-only evidence snapshots.
- All uploads originate from the local Windows host. SHA256, byte counts, coverage and restore instructions are indexed by the manifest.

A8 was accepted as an empty-load non-pawn posture/reference; actual closed-gripper loaded grasping was not tested. The additional1.5mm descent was not executed. Four-group fitting was completed but not promoted after worse grouped validation. No robot motion was issued during publication. Private chat exports and credentials are excluded.
'''
(OUT/'release_body.md').write_text(body,encoding='utf-8')
print(json.dumps(dict(assets=len(assets),asset_bytes=manifest['asset_bytes'],text_files=text['file_count'],
    text_commit=text['verified_remote_commit'],manifest_sha256=sha(OUT/'publication_manifest.json'))))
