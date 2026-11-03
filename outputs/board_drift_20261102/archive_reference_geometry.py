"""Copy the exact user-approved STEP input and derived reference mesh."""
import hashlib,json,shutil
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
inspection=ROOT/'outputs/left_arm_step_analysis/revised_2/step_inspection.json';d=json.loads(inspection.read_text(encoding='utf8'))
p=Path(d['source_path']);expected='f7729567ef511d1d9dc38b250dcd276ab09b7aa84fff98f49c0e068971ef9718';assert hashlib.sha256(p.read_bytes()).hexdigest()==expected
dest=OUT/'source_geometry';dest.mkdir(exist_ok=True)
items=[(p,'approved_arm_reference.step'),(inspection,'step_inspection.json'),(ROOT/'outputs/left_arm_step_analysis/revised_2/step_vertex_preview.json','reference_mesh_vertices.json')];records=[]
for source,name in items:
 target=dest/name;shutil.copyfile(source,target);records.append({'file':name,'source':str(source),'size':target.stat().st_size,'sha256':hashlib.sha256(target.read_bytes()).hexdigest()})
(dest/'manifest.json').write_text(json.dumps({'approval':'User confirmed revised STEP orientations match physical arm; dimensions retained, no geometry editing in Nov2 experiment','files':records},indent=2));print(json.dumps({'copied_files':len(records),'STEP_sha256':expected}))
