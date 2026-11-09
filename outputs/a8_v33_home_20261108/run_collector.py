from pathlib import Path
import json,subprocess
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
OPTS=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3']
source=(ROOT/'tmp/collect_a8_v33_home_20261108.py').read_bytes()
r=subprocess.run(['ssh']+OPTS+['nvidia@192.168.201.199','python3 -'],input=source,capture_output=True,timeout=240)
(OUT/'collector_stdout.log').write_bytes(r.stdout);(OUT/'collector_stderr.log').write_bytes(r.stderr)
assert r.returncode==0,r.stderr.decode(errors='replace')
rows=[json.loads(line) for line in r.stdout.decode().splitlines() if line.startswith('{')]
assert len(rows)==1
report=rows[0];(OUT/'collection_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False),flush=True)
