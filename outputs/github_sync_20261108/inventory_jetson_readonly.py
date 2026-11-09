"""Executed via SSH: files/stat/hash only, no motors, motion, Web or process APIs."""
from pathlib import Path
import hashlib,json,datetime,collections
root=Path('/home/nvidia/hive_robot/DM_Control_Python')
out={'motion_issued':False,'motors_opened':False,'date':'20261108','directories':[],'top_files':[]}
for area in ('rgbd_audits','experiment_archives','board_workspace_trials'):
    base=root/'data'/area
    if not base.exists():continue
    for d in sorted(base.iterdir()):
        if not d.is_dir() or '20261108' not in d.name:continue
        fs=[p for p in d.rglob('*') if p.is_file()]
        ex=collections.Counter(p.suffix.lower() for p in fs)
        rec={'area':area,'name':d.name,'path':str(d),'files':len(fs),'bytes':sum(p.stat().st_size for p in fs),'extensions':dict(ex),'latest_mtime':max((p.stat().st_mtime for p in fs),default=None),'archives':[]}
        for p in fs:
            if p.suffix.lower()=='.zip':
                h=hashlib.sha256()
                with p.open('rb') as f:
                    for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
                rec['archives'].append({'path':str(p),'bytes':p.stat().st_size,'sha256':h.hexdigest()})
        # Per-file identities allow a genuinely complete raw-to-archive coverage audit.
        if area=='rgbd_audits':
            rec['raw_members']=[{'name':p.relative_to(d).as_posix(),'bytes':p.stat().st_size} for p in fs]
        elif area in ('board_workspace_trials','experiment_archives'):
            rec['members']=[]
            for p in fs:
                if p.suffix.lower()=='.zip':continue
                h=hashlib.sha256()
                with p.open('rb') as f:
                    for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
                rec['members'].append({'name':p.relative_to(d).as_posix(),'bytes':p.stat().st_size,'sha256':h.hexdigest()})
        out['directories'].append(rec)
base=root/'data/board_workspace_trials'
for p in sorted(base.iterdir()):
    if not p.is_file():continue
    # Explicit date files plus all top-level plans changed on robot's Nov08 day.
    if '20261108' in p.name or datetime.datetime.fromtimestamp(p.stat().st_mtime).strftime('%Y%m%d')=='20261108':
        h=hashlib.sha256(p.read_bytes()).hexdigest()
        out['top_files'].append({'path':str(p),'bytes':p.stat().st_size,'sha256':h})
print(json.dumps(out,ensure_ascii=False))
