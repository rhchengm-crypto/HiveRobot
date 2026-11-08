"""Read-only working-tree evidence audit; writes only its own inventory."""
from pathlib import Path
from collections import Counter, defaultdict
import hashlib, json, subprocess, zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/github_sync_20261107/inventory.json'
EXCLUDE = ('outputs/mac_handoff_20261107/', '.git/', 'tmp/')
def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT).decode('utf-8', 'replace')
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024), b''):h.update(b)
    return h.hexdigest()
def paths(*args):return [s for s in git(*args).split('\0') if s]
def excluded(s):return any(s.startswith(x) for x in EXCLUDE) or '/__pycache__/' in s
def entry(s, do_hash=False):
    p=ROOT/s
    d={'path':s, 'bytes':p.stat().st_size}
    if do_hash:d['sha256']=sha(p)
    return d
modified=[s for s in paths('diff','--name-only','-z') if not excluded(s)]
untracked=[s for s in paths('ls-files','--others','--exclude-standard','-z') if not excluded(s) and not s.startswith('outputs/github_sync_20261107/')]
ignored=[s for s in paths('ls-files','--others','--ignored','--exclude-standard','-z','--','outputs','scripts') if not excluded(s)]
print('git inventory read', flush=True)
current=[s for s in modified+untracked if '/20261107' in s or '20261107' in s or s.startswith('scripts/')]
dirs=sorted(p for p in (ROOT/'outputs').iterdir() if p.is_dir() and ('20261106' in p.name or '20261107' in p.name) and not p.name.startswith(('mac_', 'github_sync_')))
groups=[]
for d in dirs:
    print('directory '+d.name, flush=True)
    files=[p for p in d.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    ext=Counter(); size=Counter()
    for p in files:ext[p.suffix.lower()]+=1;size[p.suffix.lower()]+=p.stat().st_size
    groups.append({'directory':d.relative_to(ROOT).as_posix(),'files':len(files),'bytes':sum(p.stat().st_size for p in files),'extension_counts':dict(ext),'extension_bytes':dict(size)})
archives=[]
for d in dirs:
    for p in d.rglob('*.zip'):
        print('archive '+p.name+' in '+d.name, flush=True)
        zrec=entry(p.relative_to(ROOT).as_posix(),True)
        try:
            with zipfile.ZipFile(p) as z:
                info=[x for x in z.infolist() if not x.is_dir()]
                c=Counter(Path(x.filename).suffix.lower() for x in info)
                root=Counter(x.filename.split('/')[0] for x in info)
                zrec.update(entries=len(info),uncompressed_bytes=sum(x.file_size for x in info),extension_counts=dict(c),top_level=dict(root),first_entries=[x.filename for x in info[:8]])
        except Exception as e:zrec['error']=repr(e)
        archives.append(zrec)
byhash=defaultdict(list)
for a in archives:byhash[a['sha256']].append(a['path'])
large=[]
for d in dirs:
    for p in d.rglob('*'):
        if p.is_file() and p.stat().st_size>=20_000_000 and p.suffix!='.zip':large.append(entry(p.relative_to(ROOT).as_posix()))
trial_records=[]
for p in (ROOT/'scripts/data/board_workspace_trials').glob('*2026110[67]*.json'):
    try:
        x=json.loads(p.read_text(encoding='utf-8-sig'))
        trial_records.append({'path':p.relative_to(ROOT).as_posix(),'bytes':p.stat().st_size,'experiment_id':x.get('experiment_id'),'operator_validation':x.get('operator_validation'),'accepted':x.get('accepted'),'evidence':x.get('evidence'),'local_evidence_directory':x.get('local_evidence_directory')})
    except Exception as e:trial_records.append({'path':str(p),'error':repr(e)})
result={'head':git('rev-parse','HEAD').strip(),'modified':[entry(s,True) for s in modified],'untracked':[entry(s) for s in untracked],'ignored_nov6_nov7':[entry(s) for s in ignored if ('20261106' in s or '20261107' in s)],'output_directories':groups,'archives':archives,'archive_duplicate_hash_groups':[v for v in byhash.values() if len(v)>1],'large_nonarchives':large,'trial_registry':trial_records,'exclusions':list(EXCLUDE),'notes':['No robot commands or git mutations used.','Nov6 known unresolved RGB-D sessions: workspace_b3_preplanned_1794030303175762226; workspace_b3_preplanned_1794030837704929731.','Mac migration package includes private chat and duplicates; explicitly excluded from publication inventory.']}
OUT.parent.mkdir(parents=True,exist_ok=True)
OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'inventory':str(OUT),'modified':len(modified),'untracked_files':len(untracked),'untracked_bytes':sum((ROOT/s).stat().st_size for s in untracked),'ignored_nov_files':len(result['ignored_nov6_nov7']),'ignored_nov_bytes':sum(x['bytes'] for x in result['ignored_nov6_nov7']),'archive_count':len(archives),'archive_bytes':sum(a['bytes'] for a in archives),'output_directories':groups,'large_nonarchives':large},ensure_ascii=False,indent=2))
