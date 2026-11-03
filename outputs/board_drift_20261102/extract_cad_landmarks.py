"""Read circle geometry from the approved STEP; do not alter the assembly."""
import re,json,hashlib,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
inspection=json.loads((ROOT/'outputs/left_arm_step_analysis/revised_2/step_inspection.json').read_text(encoding='utf8'))
source=Path(inspection['source_path']);raw=source.read_bytes();assert hashlib.sha256(raw).hexdigest()==inspection['source_sha256']
text=raw.decode('utf8',errors='replace');entities={int(i):s.strip() for i,s in re.findall(r'#(\d+)\s*=\s*(.*?);',text,re.S)}
def refs(s):return [int(n) for n in re.findall(r'#(\d+)',s)]
def xyz(i):
 s=entities[i];v=re.findall(r'\(([^()]*)\)',s)[-1]
 return [float(x) for x in v.split(',')]
result=[]
shape_links={}
for s in entities.values():
 if s.startswith('SHAPE_REPRESENTATION_RELATIONSHIP('):
  link=refs(s)
  if len(link)==2:shape_links.setdefault(link[0],[]).append(link[1])
for index in range(len(inspection['named_geometric_parts'])):
 part=inspection['named_geometric_parts'][index];rep=part['representation_entity'];todo=[rep]+shape_links.get(rep,[]);seen=set();circles=[]
 while todo:
  i=todo.pop()
  if i in seen or i not in entities:continue
  seen.add(i);s=entities[i];todo.extend(refs(s))
  if not s.startswith('CIRCLE('):continue
  placement=refs(s)[0];pr=refs(entities[placement]);center=xyz(pr[0]);axis=xyz(pr[1]) if len(pr)>1 else [0,0,1]
  radius=float(s.rsplit(',',1)[1].rstrip(')'))
  circles.append({'entity':i,'center_cad_mm':center,'axis_cad':axis,'radius_mm':radius})
 transform=np.array(part['placement_in_cad_frame'])
 for row in circles:
  row['center_cad_mm']=(transform[:3,:3]@row['center_cad_mm']+transform[:3,3]).tolist()
  row['axis_cad']=(transform[:3,:3]@row['axis_cad']).tolist()
 result.append({'part_index':index,'name':part['name'],'motor_label':part.get('motor_label'),'circles':circles})
(OUT/'cad_circle_landmarks.json').write_text(json.dumps({'source_sha256':inspection['source_sha256'],'parts':result},indent=2))
for part in result:
 print(json.dumps({'part_index':part['part_index'],'name':part['name'],'circle_count':len(part['circles'])}))
