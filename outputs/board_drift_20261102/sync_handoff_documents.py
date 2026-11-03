"""Upload archival documentation only; never changes runtime files or processes."""
import base64, hashlib, io, json, shlex, subprocess, zipfile, zlib
from pathlib import Path

OUT=Path(__file__).resolve().parent; ROOT=OUT.parents[1]
files=[ROOT/'HANDOFF_20261102.md']+[OUT/n for n in ['experiment_trials.json','experiment_trials.csv','environment_record.json','records_index.json','evidence_package_validation.json','authoritative_json_sync.json','offline_reproduction_results.json','figures/README.md','figures/raw_frame_comparison.png','figures/raw_frame_comparison_sources.json','figures/experiment_sequence.svg','figures/review_video_manifest.json']]
receipt=OUT/'github_sync_receipt.json'
if receipt.exists(): files.append(receipt)
buf=io.BytesIO(); hashes={}
with zipfile.ZipFile(buf,'w',zipfile.ZIP_DEFLATED) as z:
 for p in files:
  rel=p.relative_to(ROOT).as_posix();raw=p.read_bytes();z.writestr(rel,raw);hashes[rel]=hashlib.sha256(raw).hexdigest()
code='''import base64,hashlib,io,json,sys,zipfile
from pathlib import Path
dest=Path('/home/nvidia/hive_robot/DM_Control_Python/data/handoffs/20261102/documentation')
expected=json.loads(EXPECTED)
with zipfile.ZipFile(io.BytesIO(base64.b64decode(PAYLOAD))) as z:
 for name in z.namelist():
  assert not Path(name).is_absolute() and '..' not in Path(name).parts
  raw=z.read(name);assert hashlib.sha256(raw).hexdigest()==expected[name]
  p=dest/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(raw)
  assert hashlib.sha256(p.read_bytes()).hexdigest()==expected[name]
print(json.dumps({'documentation_files_verified':len(expected),'destination':str(dest),'motion_issued':False,'runtime_modified':False}))
'''.replace('EXPECTED',repr(json.dumps(hashes))).replace('PAYLOAD',repr(base64.b64encode(buf.getvalue()).decode()))
bootstrap='import base64,zlib,sys;exec(zlib.decompress(base64.b64decode(sys.stdin.buffer.read())))'
r=subprocess.run(['ssh','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','python3 -c '+shlex.quote(bootstrap)],input=base64.b64encode(zlib.compress(code.encode(),9)),capture_output=True,timeout=60)
assert r.returncode==0,r.stderr.decode(errors='replace')
print(r.stdout.decode())
