#!/usr/bin/env python3
"""Review-only elbow articulation from the original CAD mesh and leaf manifest.

No native CAD source, controller, encoder signs or offsets are modified.
"""
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from pose_left_arm_cad_for_photo import rotation_about, transform_points


def disconnected_components(triangles):
    # The imported "atom" compound contains two distinct solids according to
    # the native BRep audit. Recover those intact mesh components, rather than
    # rotating an upstream sleeve together with the elbow's lower adapter.
    points,ids=np.unique(np.round(triangles.reshape(-1,3),7),axis=0,return_inverse=True)
    ids=ids.reshape(-1,3);parents=list(range(len(points)))
    def root(index):
        while parents[index]!=index:
            parents[index]=parents[parents[index]];index=parents[index]
        return index
    for a,b,c in ids:
        a=root(int(a));parents[root(int(b))]=a;parents[root(int(c))]=a
    groups={}
    for i,(a,_,_) in enumerate(ids):groups.setdefault(root(int(a)),[]).append(i)
    return list(groups.values())


def main():
    out=Path(__file__).resolve().parents[1]/'outputs/left_arm_step_analysis'
    manifest=json.loads((out/'photo_pose_candidate.json').read_text(encoding='utf8'))
    datums=json.loads((out/'cad_check_dimensions.json').read_text(encoding='utf8'))['motor_datums']
    lookup={d['label']:d for d in datums}
    raw=np.load(out/'photo_pose_candidate_mesh.npz')
    original=raw['original'];labels=raw['labels'];posed=original.copy()
    pivot=lookup['M7']
    rotation=rotation_about(pivot['point_mm'],pivot['axis_direction_arbitrary_sign'],90)
    offset=0;groups=[];downstream_mask=np.zeros(len(original),dtype=bool)
    for leaf in manifest['leaves']:
        count=leaf['triangles'];segment=slice(offset,offset+count)
        motor_ids=np.unique(labels[segment]);bounds=np.array(leaf['bounds_mm'])
        components=[]
        if len(motor_ids)==1 and motor_ids[0]>=0:
            move=int(motor_ids[0]) in (0,1,2)
        else:
            move=bool(bounds[1,2]<-40)
        local_mask=np.full(count,move,dtype=bool)
        if leaf['name']=='atom手臂 - 副本 (4).STEP':
            parts=disconnected_components(original[segment])
            assert len(parts)==2,'Expected two solids from native BRep audit'
            local_mask[:]=False
            for rows in parts:
                vertices=original[segment][rows].reshape(-1,3)
                low,high=vertices.min(axis=0),vertices.max(axis=0)
                component_moves=bool(high[2]<-40)
                local_mask[rows]=component_moves
                components.append({'triangles':len(rows),'bounds_mm':[low.tolist(),high.tolist()],
                                   'moves_with_elbow':component_moves})
        rows=np.flatnonzero(local_mask)+offset
        posed[rows]=transform_points(original[rows],rotation)
        downstream_mask[rows]=True
        groups.append({'name':leaf['name'],'occurrence_path':leaf['occurrence_path'],
                       'moving_triangles':len(rows),'triangles':count,'compound_components':components})
        offset+=count
    assert offset==len(original)
    assert np.array_equal(original[~downstream_mask],posed[~downstream_mask])
    for motor in (3,4,5,6):
        assert np.array_equal(original[labels==motor],posed[labels==motor])
    edges_before=np.linalg.norm(original[:,1]-original[:,0],axis=1)
    edges_after=np.linalg.norm(posed[:,1]-posed[:,0],axis=1)
    edge_error=float(np.max(abs(edges_before-edges_after)))
    assert edge_error<1e-7
    transformed_datums={d['label']:(transform_points(d['point_mm'],rotation).tolist()
                                  if d['label'] in ('M1','M2','M3') else d['point_mm']) for d in datums}
    font=lambda size:ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',size)
    image=Image.new('RGB',(1800,1180),'#f5f7fa');draw=ImageDraw.Draw(image)
    draw.text((35,20),'M7（elbow）+90° 关节姿态预览',fill='#1b344d',font=font(34))
    draw.text((35,75),'从原始 CAD 出发；撤回上一版 M5 +20°、M3 +180° 的试配。M6 及所有上游零件保持原位。',fill='#344b61',font=font(22))
    draw.text((35,114),'绕 CAD 中 M7 轴线按右手方向旋转；该“+”不代表编码器正方向。',fill='#344b61',font=font(22))
    eye=np.array([1.,0.30,0.08]);eye/=np.linalg.norm(eye)
    right=np.cross([0,0,1],eye);right/=np.linalg.norm(right);up=np.cross(eye,right)
    union=np.concatenate((original,posed))
    lo=np.array([(union@right).min(),(union@up).min()]);hi=np.array([(union@right).max(),(union@up).max()])
    scale=min(680/(hi[0]-lo[0]),800/(hi[1]-lo[1]))
    for panel,(triangles,title) in enumerate(((original,'原始 CAD'),(posed,'仅 M7 关节旋转 +90°'))):
        x0=30+panel*890;y0=170
        draw.rounded_rectangle((x0,y0,x0+850,y0+925),15,fill='white',outline='#c5cfdb',width=2)
        draw.text((x0+20,y0+15),title,font=font(26),fill='#203a54')
        def project(points):
            points=np.asarray(points)
            return np.stack([x0+70+(points@right-lo[0])*scale,
                             y0+865-(points@up-lo[1])*scale],axis=-1)
        pixels=project(triangles);centers=triangles.mean(axis=1)
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)
        light=np.array([0.4,-0.2,0.9]);light/=np.linalg.norm(light)
        shades=0.53+0.43*abs(normals@light)
        for i in np.argsort(centers@eye):
            color=([93,174,147] if panel and downstream_mask[i] else
                   [137,169,191] if labels[i]>=0 else [201,210,219])
            draw.polygon([tuple(p) for p in pixels[i]],fill=tuple(int(v*shades[i]) for v in color))
        for label in ('M6','M7','M3','M2','M1'):
            point=transformed_datums[label] if panel else lookup[label]['point_mm']
            x,y=project(point)
            draw.ellipse((x-5,y-5,x+5,y+5),fill='#dd552f')
            dx,dy=(12,-28)
            if panel and label=='M3':dy=-36
            draw.text((x+dx,y+dy),label,fill='#1b344d',font=font(23))
        x,y=project(pivot['point_mm'])
        draw.ellipse((x-19,y-19,x+19,y+19),outline='#dd552f',width=3)
        draw.text((x0+20,y0+875),'橙圈：M7 肘轴；绿色：随肘关节转动的下游组合。',fill='#3b5268',font=font(20))
    draw.text((35,1120),'仅为 CAD 预览，等待确认；未执行机械臂运动，也未写入正方向或零位偏置。',fill='#344b61',font=font(23))
    image.save(out/'m7_positive90_preview.png')
    report={'base':'original CAD; previous side/roll trial canceled','rotation_joint':'M7/elbow',
            'cad_relative_rotation_deg':90,'axis_convention':'right hand about stored original CAD axis; not encoder sign',
            'M6_unchanged':True,'all_upstream_meshes_bitwise_unchanged':True,
            'edge_length_preservation_max_error_mm':edge_error,'rigid_groups_provisional':groups,
            'transform':rotation.tolist(),'joint_datums_mm':transformed_datums,
            'status':'preview; awaiting user confirmation','motor_motion_issued':False,'encoder_calibration_written':False}
    (out/'m7_positive90_preview.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'M6_unchanged':True,'edge_error_mm':edge_error,'output':str(out/'m7_positive90_preview.png')}))


if __name__=='__main__':
    main()
