"""List, download and SHA256-check GitHub experiment assets; never controls hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=Path(__file__).with_name('publication_manifest.json'))
    parser.add_argument('--group',default='all',choices=['all','nov7-rgbd','nov7-control-and-preparation','nov6-rgbd-backfill','auxiliary-media','final-runtime'])
    parser.add_argument('--output',type=Path,default=Path('experiment_assets_20261107'))
    parser.add_argument('--download',action='store_true')
    args=parser.parse_args()
    manifest=json.loads(args.manifest.read_text(encoding='utf-8'))
    files=[x for x in manifest['assets'] if args.group=='all' or x['group']==args.group]
    print(len(files),'assets;',sum(x['bytes'] for x in files),'bytes')
    if not args.download:
        for x in files:print(x['name'],x['bytes'],x['sha256'])
        print('Add --download to transfer files. No robot access.');return
    args.output.mkdir(parents=True,exist_ok=True)
    for row in files:
        name=row['name'];assert Path(name).name==name and name not in ('.','..')
        target=args.output/name
        if target.exists():
            assert target.stat().st_size==row['bytes'] and digest(target)==row['sha256'],'Existing file mismatch: '+name
            print('Verified existing:',name);continue
        temporary=target.with_name(name+'.partial')
        req=urllib.request.Request(row['url'],headers={'User-Agent':'HiveRobot-evidence-restore'})
        h=hashlib.sha256();size=0
        with urllib.request.urlopen(req,timeout=180) as response,temporary.open('wb') as f:
            for b in iter(lambda:response.read(1024*1024),b''):
                f.write(b);h.update(b);size+=len(b)
        assert size==row['bytes'] and h.hexdigest()==row['sha256'],'Downloaded checksum mismatch: '+name
        temporary.replace(target);print('Downloaded and verified:',name)
    print('All selected assets verified. Archives retain original bytes; unzip into separate evidence folders.')

if __name__=='__main__':main()
