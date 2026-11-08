from pathlib import Path
import json,zipfile
ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'outputs/github_sync_20261107'
inv=json.loads((out/'inventory.json').read_text(encoding='utf-8'))
remote=json.loads((out/'jetson_inventory.json').read_text(encoding='utf-8-sig'))
covered={'workspace_b3_above_1794013057155186767':'outputs/board_workspace_20261106/b3_full_rgbd.zip','workspace_b3_v3_above_1794013997642910306':'outputs/board_workspace_20261106/b3_v3_full_rgbd.zip'}
nov6=[]
for d in remote['directories']:
    if d['area']!='rgbd_audits':continue
    if not d['name'].startswith(('c5_20261106','workspace_b3_')):continue
    x=dict(d)
    x['local_complete_archive']=covered.get(d['name'])
    x['publication_status']='already_in_git_lossless_chunks' if x['local_complete_archive'] else 'raw_rgbd_not_found_locally'
    nov6.append(x)
nov7=[]
for a in inv['archives']:
    if '20261107' not in a['path'] or not a['path'].endswith('rgbd_evidence.zip'):continue
    p=ROOT/a['path']; cp=p.parent/'collection_report.json'
    c=json.loads(cp.read_text(encoding='utf-8-sig')) if cp.exists() else {}
    with zipfile.ZipFile(p) as z:
        files=[n for n in z.namelist() if not n.endswith('/')]
        count_rgb=sum(n.endswith('.jpg') for n in files)
        count_depth=sum(n.endswith('.npy') for n in files)
        expected=c.get('rgb_frames'),c.get('depth_frames')
    ctrl=next((x for x in inv['archives'] if x['path']==(p.parent/'control_evidence.zip').relative_to(ROOT).as_posix()),{})
    nov7.append({'path':a['path'],'bytes':a['bytes'],'sha256':a['sha256'],'rgbd_sha256_matches_remote_receipt':a['sha256']==c.get('rgbd',{}).get('sha256'),'control_sha256_matches_remote_receipt':ctrl.get('sha256')==c.get('control',{}).get('sha256'),'rgb_frames':count_rgb,'depth_frames':count_depth,'expected_rgb_depth':expected,'count_matches':(count_rgb,count_depth)==expected,'archive_directory':c.get('archive_directory'),'control_sha256':c.get('control',{}).get('sha256'),'control_bytes':c.get('control',{}).get('bytes')})
gaps=[d for d in nov6 if not d['local_complete_archive']]
report={'nov7_complete_rgbd_archives':nov7,'nov7_rgbd_compressed_bytes':sum(x['bytes'] for x in nov7),'nov7_total_rgb_frames':sum(x['rgb_frames'] for x in nov7),'nov7_total_depth_frames':sum(x['depth_frames'] for x in nov7),'nov6_rgbd_sessions':nov6,'nov6_unpublished_missing_raw_rgbd_sessions':gaps,'nov6_missing_raw_rgbd_bytes':sum(d['bytes'] for d in gaps),'a7_v29_trace_note':'support snapshot control_evidence.zip is not final; include control_final.zip and extracted final trace, not only initial package.','private_exclusions':['outputs/mac_handoff_20261107','private chat exports and private SSH material'],'repository_large_file_note':'42,064,424 byte A7v29 control_final trace is below GitHub 100 MiB limit but should also have compressed Release asset; Git text indexing may be retained if desired.'}
(out/'remote_gap_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
