"""Safely restore verified ZIP evidence into a separate offline directory.

Usage: python outputs/board_drift_20261102/extract_raw_evidence.py --output offline_dataset
Never point output at a live robot/project deployment.
"""
import argparse,json,hashlib,zipfile
from archive_storage import materialize_archives
from pathlib import Path
OUT=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path);a=p.parse_args();dest=a.output.resolve();dest.mkdir(parents=True,exist_ok=True)
assert not any(dest.iterdir()),'Choose a new empty offline output directory; existing files must not be overwritten'
m=materialize_archives(OUT);expected={v['path']:v for v in m['members']};count=0
for part in m['parts']:
 path=OUT/'raw_evidence'/part['file'];assert hashlib.sha256(path.read_bytes()).hexdigest()==part['sha256'],'ZIP hash mismatch'
 with zipfile.ZipFile(path) as z:
  for info in z.infolist():
   assert info.filename in expected and expected[info.filename]['archive_part']==part['file'],'Unexpected archive member'
   target=(dest/info.filename).resolve();assert target.is_relative_to(dest),'Unsafe member path'
   raw=z.read(info.filename);assert hashlib.sha256(raw).hexdigest()==expected[info.filename]['sha256'],'Member hash mismatch'
   target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(raw);count+=1
assert count==len(expected);print(json.dumps({'verified_and_restored_files':count,'output':str(dest),'motion_issued':False}))
