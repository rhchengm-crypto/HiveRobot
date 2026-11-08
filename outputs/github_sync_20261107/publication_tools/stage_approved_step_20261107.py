import hashlib,json,shutil
from pathlib import Path
root=Path(__file__).resolve().parents[1]
source=Path('C:/Users/rhche/Documents/WeChat Files/wxid_6245222452021/FileStorage/File/2026-10/RB000152_02_手臂完整(2).step')
expected='f7729567ef511d1d9dc38b250dcd276ab09b7aa84fff98f49c0e068971ef9718'
raw=source.read_bytes();assert hashlib.sha256(raw).hexdigest()==expected
target=root/'source_geometry/approved_arm_reference.step';target.parent.mkdir(exist_ok=True)
if target.exists():assert target.read_bytes()==raw
else:shutil.copy2(source,target)
manifest=dict(original_name=source.name,path='source_geometry/approved_arm_reference.step',bytes=len(raw),sha256=expected,
 approved_by_user=True,role='Approved revised seven-axis assembly and reference pose',
 note='Matches source_step_sha256 in the current common board model; source geometry, not a replay path.')
(target.parent/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(manifest,ensure_ascii=False))
