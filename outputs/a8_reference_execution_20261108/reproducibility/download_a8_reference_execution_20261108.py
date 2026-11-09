from pathlib import Path
import json,subprocess,hashlib,zipfile
out=Path('outputs/a8_reference_execution_20261108')
report=json.loads((out/'collection_report.json').read_text())
common=['scp','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),
    '-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256',
    '-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5',
    '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2']
verified={}
for kind in ('control','rgbd'):
    entry=report[kind];dest=out/(kind+'_evidence.zip')
    if not(dest.exists() and dest.stat().st_size==entry['bytes'] and hashlib.sha256(dest.read_bytes()).hexdigest()==entry['sha256']):
        subprocess.run(common+['nvidia@192.168.201.199:'+entry['path'],str(dest)],check=True,timeout=240)
    assert dest.stat().st_size==entry['bytes'] and hashlib.sha256(dest.read_bytes()).hexdigest()==entry['sha256']
    with zipfile.ZipFile(dest) as z:
        assert z.testzip() is None
        manifest=json.loads(z.read('manifest.json'))
        for name,item in manifest.items():
            data=z.read(name)
            assert len(data)==item['bytes'] and hashlib.sha256(data).hexdigest()==item['sha256'],name
        if kind=='control':
            directory=out/'control_evidence';directory.mkdir(exist_ok=True)
            for name in z.namelist():assert (directory/name).resolve().is_relative_to(directory.resolve()),name
            z.extractall(directory)
        else:
            jpg=sorted(n for n in z.namelist() if n.endswith('.jpg'))[-1]
            (out/'last_recorded_frame.jpg').write_bytes(z.read(jpg))
    verified[kind]=dict(sha256=entry['sha256'],bytes=entry['bytes'],members_verified=len(manifest))
    (out/'download_verification.json').write_text(json.dumps(verified,indent=2),encoding='utf-8')
    print('DOWNLOADED_VERIFIED '+json.dumps({kind:verified[kind]}),flush=True)
