#!/usr/bin/env python3
"""Extract offline CAD check dimensions; never imports a robot controller.

P_i is the motor-axis intersection with the plane shared by concentric
radius-19 and radius-28.5 circle edges. This is a geometric housing/boss
datum, not a verified output flange, physical COM or encoder zero.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from inspect_left_arm_step import StepInspection


def datum(inspection, motor):
    axis = motor['motor_axis_candidate']
    direction = np.array(axis['direction'])
    origin = np.array(axis['point_nearest_cad_origin_mm'])
    matrix = inspection.placement(motor['representation_entity'])
    circles = {19.0: [], 28.5: []}
    for entity in inspection.reachable(motor['representation_entity']):
        value = inspection.entities[entity]
        if not value.startswith('CIRCLE('):
            continue
        radius = float(value.rsplit(',', 1)[1][:-1])
        frame = matrix @ inspection.frame(inspection.graph[entity][0])
        center = frame[:3, 3]
        residual = center - origin - np.dot(center - origin, direction) * direction
        if np.linalg.norm(np.cross(frame[:3, 2], direction)) > 1e-6 or np.linalg.norm(residual) > 1e-4:
            continue
        for target in circles:
            if abs(radius - target) < 1e-5:
                circles[target].append((float(center @ direction), entity))
    matches = [(t, entity, other) for t, entity in circles[19.0]
               for u, other in circles[28.5] if abs(t-u) < 1e-4]
    levels = sorted({round(t, 4) for t, _, _ in matches})
    if len(levels) != 1:
        raise ValueError(f"Ambiguous housing/boss datum for {motor['motor_label']}: {levels}")
    evidence = {n for _, a, b in matches for n in (a, b)}
    planar_faces = []
    for entity in inspection.reachable(motor['representation_entity']):
        if not inspection.entities[entity].startswith('ADVANCED_FACE('):
            continue
        nodes = inspection.reachable(entity)
        if len(evidence & nodes) >= 2 and any(
                inspection.entities[n].startswith('PLANE(') for n in inspection.graph[entity]):
            planar_faces.append(entity)
    if len(planar_faces) != 1:
        raise ValueError(f"Expected one actual planar datum face: {planar_faces}")
    point = origin + matches[0][0] * direction
    return {'label': motor['motor_label'], 'point_mm': point.tolist(),
            'axis_direction_arbitrary_sign': direction.tolist(),
            'axis_point_nearest_cad_origin_mm': origin.tolist(),
            'circle_evidence_entities': sorted({n for _, a, b in matches for n in (a, b)}),
            'planar_face_evidence_entity': planar_faces[0],
            'controller_joint_name': None, 'physical_check': None}


def pair_geometry(first, second):
    p, q = (np.array(item['point_mm']) for item in (first, second))
    a, b = (np.array(item['axis_direction_arbitrary_sign']) for item in (first, second))
    cross = np.cross(a, b)
    sine = np.linalg.norm(cross)
    # Infinite-axis distance is not motor-center distance or link length.
    separation = abs(float((q-p) @ cross)) / sine if sine > 1e-7 else np.linalg.norm(np.cross(q-p, a))
    return {'pair': first['label'] + '-' + second['label'],
            'datum_center_distance_mm': float(np.linalg.norm(q-p)),
            'delta_in_cad_frame_mm': (q-p).tolist(),
            'unoriented_axis_angle_deg': float(np.degrees(np.arccos(np.clip(abs(a @ b), 0, 1)))),
            'infinite_axis_shortest_distance_mm': float(separation),
            'physical_distance_mm': None, 'physical_joint_names': None}


def render(report, clouds, output):
    from PIL import Image, ImageDraw, ImageFont
    font_path = 'C:/Windows/Fonts/msyh.ttc'
    def font(size):
        return ImageFont.truetype(font_path, size)
    canvas = Image.new('RGB', (1900, 1480), 'white')
    draw = ImageDraw.Draw(canvas)
    draw.text((45, 25), '七轴手臂 STEP 尺寸复核图（单位：mm）', fill='#182535', font=font(34))
    draw.text((45, 80), 'M1–M7 为 CAD 编号；P 为轴线与 Ø57 壳体 / Ø38 凸台交接端面的交点。', fill='#37485a', font=font(22))
    draw.text((45, 115), '圆点是可定义的端面中心；不是质心。图为边界顶点投影，不能按图片比例量尺寸。', fill='#37485a', font=font(22))
    points = np.array([p for cloud in clouds for p in cloud['vertices_mm']])
    motors = report['motor_datums']
    colors = ['#d44835', '#bd7714', '#2378b7', '#1b9970', '#9254ac', '#ba407e', '#49628c']
    # Show global YZ/XZ views and a magnified shoulder view. Candidate pairs
    # are check combinations only, not an asserted moving-link topology.
    def panel(box, dims, title, limits=None):
        x0,y0,x1,y1 = box
        draw.rounded_rectangle(box, 12, outline='#c9d1db', width=2)
        draw.text((x0+18,y0+12), title, fill='#243449', font=font(24))
        selected = points[:,dims]
        lo,hi = (selected.min(axis=0), selected.max(axis=0)) if limits is None else (np.array(limits[0]),np.array(limits[1]))
        scale = min((x1-x0-90)/(hi[0]-lo[0]), (y1-y0-100)/(hi[1]-lo[1]))
        def project(point):
            uv = np.array(point)[list(dims)]
            return (x0+45+(uv[0]-lo[0])*scale, y1-40-(uv[1]-lo[1])*scale)
        for point in points:
            uv = point[list(dims)]
            if np.all(uv >= lo) and np.all(uv <= hi):
                x,y = project(point)
                draw.point((int(x),int(y)),fill='#b8c0c8')
        for index,motor in enumerate(motors):
            point=np.array(motor['point_mm']);uv=point[list(dims)]
            if not (np.all(uv >= lo) and np.all(uv <= hi)):continue
            x,y=project(point);direction=np.array(motor['axis_direction_arbitrary_sign'])
            draw.line([project(point-30*direction),project(point+30*direction)], fill=colors[index],width=3)
            draw.ellipse((x-6,y-6,x+6,y+6),fill=colors[index],outline='white',width=1)
            dx = 12 if index%2==0 else -65
            dy = -28 if index != 4 else 8
            draw.text((x+dx,y+dy),motor['label'],fill=colors[index],font=font(23))
    panel((40,170,640,1000),(1,2),'CAD Y–Z 投影（整体）')
    panel((660,170,1120,1000),(0,2),'CAD X–Z 投影（整体）')
    panel((1140,170,1860,570),(1,2),'肩部区域 Y–Z 放大',([45,120],[185,275]))
    draw.text((1160,600),'端面基准示意（不按比例）',fill='#243449',font=font(24))
    draw.rectangle((1190,665,1490,800),outline='#53657a',width=3)
    draw.rectangle((1490,690,1540,775),outline='#53657a',width=3)
    draw.line((1160,732,1640,732),fill='#2378b7',width=3)
    draw.line((1490,640,1490,820),fill='#d44835',width=3)
    draw.ellipse((1483,725,1497,739),fill='#d44835')
    draw.text((1190,810),'Ø57 壳体',fill='#243449',font=font(22))
    draw.text((1548,700),'Ø38 凸台',fill='#243449',font=font(22))
    draw.text((1510,630),'交接端面',fill='#d44835',font=font(22))
    draw.text((1510,760),'P（中心）',fill='#d44835',font=font(22))
    draw.text((1160,870),'先在实物确认这个端面，',fill='#37485a',font=font(23))
    draw.text((1160,908),'再复核下面的中心间距离。',fill='#37485a',font=font(23))
    draw.text((45,1030),'复核组合                 端面中心直线距         两轴夹角*           两轴最短距离**',fill='#243449',font=font(24))
    for index,row in enumerate(report['check_pairs']):
        y=1080+index*43
        draw.text((65,y), row['pair'], fill='#243449',font=font(22))
        draw.text((395,y),f"{row['datum_center_distance_mm']:.2f}",fill='#243449',font=font(22))
        draw.text((760,y),f"{row['unoriented_axis_angle_deg']:.2f}°",fill='#243449',font=font(22))
        draw.text((1110,y),f"{row['infinite_axis_shortest_distance_mm']:.2f}",fill='#243449',font=font(22))
    draw.text((45,1370),'* 夹角不含正负方向。  ** 无限轴线的最短距离，不能当作连杆长度。',fill='#53657a',font=font(22))
    draw.text((45,1410),'当前模型姿态下的距离；复核姿态不同，跨活动关节的端面中心距可能改变。',fill='#53657a',font=font(22))
    canvas.save(output)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    previous=json.loads((args.output_dir/'step_inspection.json').read_text(encoding='utf8'))
    inspection=StepInspection(args.step)
    import hashlib
    if hashlib.sha256(inspection.raw).hexdigest() != previous['source_sha256']:
        raise ValueError('STEP does not match inspected assembly')
    datums=[datum(inspection,m) for m in previous['motor_axis_candidates']]
    mapping_path=args.output_dir/'confirmed_joint_mapping.json'
    if mapping_path.exists():
        mapping=json.loads(mapping_path.read_text(encoding='utf8'))
        if mapping['source_sha256'] != previous['source_sha256']:
            raise ValueError('Confirmed joint mapping belongs to a different STEP')
        for item in datums:
            name=mapping['controller_joint_names'].get(item['label'])
            if name:
                item['controller_joint_name']=name
                item['joint_name_mapping_source']=mapping['mapping_source']
    lookup={m['label']:m for m in datums}
    pairs=[pair_geometry(lookup[a],lookup[b]) for a,b in
           [('M6','M5'),('M5','M4'),('M4','M7'),('M7','M3'),('M3','M2'),('M2','M1')]]
    if mapping_path.exists():
        for constraint in mapping.get('model_axis_angle_constraints',[]):
            for row in pairs:
                if set(row['pair'].split('-')) == set(constraint['pair']):
                    row['model_axis_angle_deg']=constraint['model_axis_angle_deg']
                    row['model_axis_angle_source']=constraint['source']
    report={'source_sha256':previous['source_sha256'],'units':'mm, degree',
            'reference':'original CAD assembly frame',
            'datum_definition':'axis intersection with shared radius-19/radius-28.5 circle-edge plane; housing/boss end datum, NOT verified output flange',
            'motor_datums':datums,'check_pairs':pairs,
            'limitations':['Check combinations do not establish joint order or controller names.',
                           'Center distances are for the CAD assembly pose; registration or same-rigid-body checks are required for physical comparison.',
                           'Identical motor CAD proxies may omit real motor housing differences.',
                           'No mass, physical COM, encoder direction or encoder offset inferred.'],
            'deployment_ready':False}
    if mapping_path.exists() and 'distance_verification' in mapping:
        report['distance_verification']=mapping['distance_verification']
        confirmed_pairs=set(mapping['distance_verification']['pairs'])
        for row in pairs:
            if row['pair'] in confirmed_pairs:
                row['physical_check_status']='user confirmed approximately correct'
    (args.output_dir/'cad_check_dimensions.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    with (args.output_dir/'cad_check_dimensions.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f);writer.writerow(['CAD编号组合','CAD端面中心距_mm','CAD轴线夹角_deg','CAD无限轴线最短距_mm','实物中心距_mm','实物关节名称','姿态与备注'])
        for row in pairs:writer.writerow([row['pair'],round(row['datum_center_distance_mm'],3),round(row['unoriented_axis_angle_deg'],3),round(row['infinite_axis_shortest_distance_mm'],3),'','',''])
    clouds=json.loads((args.output_dir/'step_vertex_preview.json').read_text(encoding='utf8'))
    render(report,clouds,args.output_dir/'cad_dimension_check_sheet.png')
    print(json.dumps(pairs,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
