"""Run only after root COMPLETE and explicit collector_identity.json is supplied."""
from pathlib import Path
import json,re,subprocess
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
identity=json.loads((OUT/'collector_identity.json').read_text())
required=['expected_recorder_pid','expected_media_name','expected_board_log','expected_clearance_log']
assert set(required)<=set(identity)
assert isinstance(identity['expected_recorder_pid'],int) and identity['expected_recorder_pid']>1
for k in required[1:]:assert re.fullmatch(r'[A-Za-z0-9_./-]+',identity[k]),k
opts=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3']
args=['python3','-']
for name in required:args.extend(['--'+name.replace('_','-'),str(identity[name])])
r=subprocess.run(['ssh']+opts+['nvidia@192.168.201.199',' '.join(args)],input=(ROOT/'tmp/collect_a8_lower25_execution_20261108.py').read_bytes(),capture_output=True,timeout=300)
(OUT/'collector_stdout.log').write_bytes(r.stdout);(OUT/'collector_stderr.log').write_bytes(r.stderr)
assert r.returncode==0,r.stderr.decode(errors='replace')
rows=[json.loads(line) for line in r.stdout.decode().splitlines() if line.startswith('{')];assert len(rows)==1
(OUT/'collection_report.json').write_text(json.dumps(rows[0],ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(rows[0],ensure_ascii=False),flush=True)
