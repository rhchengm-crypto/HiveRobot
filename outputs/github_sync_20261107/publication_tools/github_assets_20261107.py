"""Publish checksum-bound experimental assets using existing Git credentials.
No credentials are saved or printed. Hardware is never accessed.
"""
import argparse,hashlib,http.client,json,os,subprocess,time,urllib.error,urllib.parse,urllib.request
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/github_sync_20261107'
REPO='rhchengm-crypto/HiveRobot'
TAG='experiments-2026-11-07'
TOKEN=None
def token():
    global TOKEN
    if TOKEN is None:
        env=dict(os.environ,GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never')
        r=subprocess.run(['git','credential','fill'],input='protocol=https\nhost=github.com\npath='+REPO+'.git\n\n',
            text=True,capture_output=True,env=env,timeout=90)
        if r.returncode:raise RuntimeError('Noninteractive GitHub credential access failed')
        TOKEN=dict(x.split('=',1) for x in r.stdout.splitlines() if '=' in x).get('password')
        if not TOKEN:raise RuntimeError('No GitHub credential returned')
    return TOKEN
def api(method,path,body=None):
    raw=None if body is None else json.dumps(body).encode()
    req=urllib.request.Request('https://api.github.com'+path,data=raw,method=method,
        headers={'Authorization':'Bearer '+token(),'Accept':'application/vnd.github+json',
                 'User-Agent':'HiveRobot-experiment-publication','Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=90) as r:
        data=r.read()
        return json.loads(data) if data else None
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def save(p,x):
    p.parent.mkdir(parents=True,exist_ok=True)
    tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(x,indent=2)+'\n');os.replace(tmp,p)
def release():return json.loads((OUT/'release.json').read_text())
def remote_digest(asset,expected):
    digest=asset.get('digest')
    if digest:
        assert digest=='sha256:'+expected,'GitHub asset SHA256 mismatch'
        return digest,'github-computed-digest'
    req=urllib.request.Request(asset['url'],headers={'Authorization':'Bearer '+token(),
        'Accept':'application/octet-stream','User-Agent':'HiveRobot-experiment-publication'})
    h=hashlib.sha256()
    with urllib.request.urlopen(req,timeout=120) as r:
        for b in iter(lambda:r.read(1024*1024),b''):h.update(b)
    assert h.hexdigest()==expected,'Downloaded asset SHA256 mismatch'
    return 'sha256:'+expected,'authenticated-download-sha256'
def upload(path,name=None):
    p=Path(path).resolve();name=name or p.name;size=p.stat().st_size;expected=sha(p)
    rid=release()['id'];receipt=OUT/'asset_receipts'/(name+'.json')
    for attempt in range(3):
        assets=api('GET',f'/repos/{REPO}/releases/{rid}/assets?per_page=100')
        found=next((a for a in assets if a['name']==name),None)
        if found:
            assert found['state']=='uploaded' and found['size']==size,'Conflicting or incomplete release asset: '+name
            digest,how=remote_digest(found,expected)
            row=dict(local_path=p.relative_to(ROOT).as_posix(),name=name,bytes=size,sha256=expected,
                asset_id=found['id'],url=found['browser_download_url'],remote_digest=digest,
                verification=how,state=found['state'])
            save(receipt,row);print('VERIFIED '+name+' '+str(size),flush=True);return row
        conn=None
        try:
            conn=http.client.HTTPSConnection('uploads.github.com',timeout=180)
            url=f'/repos/{REPO}/releases/{rid}/assets?name='+urllib.parse.quote(name,safe='')
            conn.putrequest('POST',url)
            for k,v in {'Authorization':'Bearer '+token(),'Accept':'application/vnd.github+json',
                'User-Agent':'HiveRobot-experiment-publication','Content-Type':'application/octet-stream',
                'Content-Length':str(size)}.items():conn.putheader(k,v)
            conn.endheaders()
            sent=0;last=0
            with p.open('rb') as f:
                for block in iter(lambda:f.read(1024*1024),b''):
                    conn.send(block);sent+=len(block)
                    if time.monotonic()-last>20:
                        print('UPLOADING '+name+' '+str(sent)+'/'+str(size),flush=True);last=time.monotonic()
            response=conn.getresponse();raw=response.read()
            if response.status not in (200,201):raise RuntimeError('GitHub upload HTTP '+str(response.status))
            asset=json.loads(raw);assert asset['size']==size and asset['state']=='uploaded'
            digest,how=remote_digest(asset,expected)
            row=dict(local_path=p.relative_to(ROOT).as_posix(),name=name,bytes=size,sha256=expected,
                asset_id=asset['id'],url=asset['browser_download_url'],remote_digest=digest,
                verification=how,state=asset['state'])
            save(receipt,row);print('VERIFIED '+name+' '+str(size),flush=True);return row
        except Exception:
            if attempt==2:raise
            time.sleep(2)
        finally:
            if conn:conn.close()
def main():
    p=argparse.ArgumentParser();sp=p.add_subparsers(dest='mode',required=True)
    create=sp.add_parser('create');create.add_argument('--commit',required=True)
    up=sp.add_parser('upload');up.add_argument('path');up.add_argument('--name')
    finish=sp.add_parser('publish');finish.add_argument('--body-file',required=True)
    sp.add_parser('verify')
    a=p.parse_args()
    if a.mode=='create':
        existing=api('GET',f'/repos/{REPO}/releases?per_page=100')
        r=next((x for x in existing if x['tag_name']==TAG),None)
        if not r:r=api('POST',f'/repos/{REPO}/releases',dict(tag_name=TAG,target_commitish=a.commit,
            name='November 7 robot experiments and November 6 RGB-D backfill',draft=True,
            body='Experimental data publication in progress. Read HANDOFF_20261107.md in the repository.'))
        save(OUT/'release.json',{k:r[k] for k in ('id','tag_name','html_url','draft','target_commitish')})
        print(json.dumps(release()))
    elif a.mode=='upload':upload(a.path,a.name)
    elif a.mode=='publish':
        r=api('PATCH',f'/repos/{REPO}/releases/'+str(release()['id']),
              dict(draft=False,body=Path(a.body_file).read_text(encoding='utf-8')))
        save(OUT/'release.json',{k:r[k] for k in ('id','tag_name','html_url','draft','target_commitish')})
        print(json.dumps(release()))
    else:
        r=api('GET',f'/repos/{REPO}/releases/'+str(release()['id']))
        save(OUT/'release_remote_verification.json',dict(id=r['id'],tag=r['tag_name'],draft=r['draft'],
            url=r['html_url'],assets=[{k:x.get(k) for k in ('id','name','size','state','digest','browser_download_url')} for x in r['assets']]))
        print(json.dumps(dict(draft=r['draft'],assets=len(r['assets']),url=r['html_url'])))
if __name__=='__main__':main()
