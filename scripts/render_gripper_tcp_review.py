"""Orthographic STEP vertex review, no generated or modified geometry."""
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw

root=Path(__file__).resolve().parents[1]
rows=json.loads((root/'outputs/left_arm_step_analysis/revised_2/step_vertex_preview.json').read_text())
points=np.array(next(r['vertices_mm'] for r in rows if r['part_index']==7))
cfg=json.loads((root/'scripts/data/left_arm_board_grasp.json').read_text())
tcp=np.array(cfg['tcp_datum']['cad_point_m'])*1000
image=Image.new('RGB',(1100,850),'white');draw=ImageDraw.Draw(image)
for panel,axis in enumerate((0,1)):
    low=points[:,[axis,2]].min(0);high=points[:,[axis,2]].max(0)
    scale=4.3
    def pixel(p):
        return (int(80+550*panel+(p[axis]-low[0])*scale),int(80+(high[1]-p[2])*scale))
    for point in points:
        x,y=pixel(point);draw.ellipse((x-1,y-1,x+1,y+1),fill='#455d73')
    x,y=pixel(tcp)
    draw.ellipse((x-7,y-7,x+7,y+7),outline='#dc382d',width=3)
    draw.line((x-15,y,x+15,y),fill='#dc382d',width=2)
    draw.line((x,y-15,x,y+15),fill='#dc382d',width=2)
    draw.text((panel*550+50,780),'Red: nominal TCP at distal finger-tip plane',fill='#b52c24')
    draw.text((panel*550+50,30),('CAD X/Z' if axis==0 else 'CAD Y/Z'),fill='black')
image.save(root/'outputs/left_arm_general_model/gripper_tcp_review.png')
