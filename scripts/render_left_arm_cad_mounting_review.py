#!/usr/bin/env python3
"""Offline STEP mounting review. Renders original geometry; does not alter CAD.

Requires the isolated OCP 8/Pillow runtime used for this workspace's CAD audit.
End colors describe geometric ends of generic STEP motor proxies, not verified
rotor/stator assignments or physical encoder-positive directions.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def build_mesh(step, outdir, kernel):
    sys.path.insert(0, str(kernel))
    from OCP.STEPControl import STEPControl_Reader
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher

    inspection=json.loads((outdir/'step_inspection.json').read_text(encoding='utf8'))
    motors=inspection['motor_axis_candidates']
    native_path=outdir/'occt_brep_verification.json'
    if native_path.exists():
        native=json.loads(native_path.read_text(encoding='utf8'))
        native_motors=[node for node in native['assembly_nodes'] if node['name']=='Wrist motor.step']
        if len(native_motors)!=len(motors):
            raise ValueError('Native motor occurrence count does not match axis inspection')
        # Analytic cylinders can extend beyond topological vertex bounds.
        bounds=np.array([node['bounds_mm'] for node in native_motors]).reshape(-1,2,3)
        match_tolerance=0.4
    else:
        bounds=np.array([motor['vertex_bounds_mm'] for motor in motors])
        # Vertex bounds are approximate. Require unique seven-way matches and
        # record exact native bounds below for independent review.
        match_tolerance=3.0
    reader=STEPControl_Reader()
    if reader.ReadFile(str(step)) != IFSelect_RetDone:
        raise ValueError('STEP read failed')
    reader.TransferRoots()
    shape=reader.OneShape()
    BRepMesh_IncrementalMesh(shape, 0.8, False, 0.35, True)
    triangles=[]; labels=[]; counts={}; motor_matches=[]
    processed=IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher()
    explorer=TopExp_Explorer(shape,TopAbs_SOLID)
    while explorer.More():
        solid=explorer.Current()
        box=Bnd_Box(); BRepBndLib.AddOptimal_s(solid,box,False,False)
        low,high=box.CornerMin(),box.CornerMax()
        bbox=np.array([[low.X(),low.Y(),low.Z()],[high.X(),high.Y(),high.Z()]])
        errors=np.linalg.norm(bounds-bbox,axis=(1,2))
        closest=int(np.argmin(errors))
        label=closest if errors[closest]<match_tolerance else -1
        if label>=0:
            if motors[label]['motor_label'] in counts:
                raise ValueError('Multiple solids matched the same motor proxy')
            motor_matches.append({'motor_label':motors[label]['motor_label'],
                                  'exact_native_bounds_mm':bbox.tolist(),
                                  'bounds_matching_error_mm':float(errors[label])})
        faces=TopExp_Explorer(solid,TopAbs_FACE)
        before=len(triangles)
        while faces.More():
            face=TopoDS.Face(faces.Current());loc=TopLoc_Location()
            processed.Add(face)
            mesh=BRep_Tool.Triangulation_s(face,loc)
            if mesh is not None:
                transform=loc.Transformation()
                nodes=[]
                for i in range(1,mesh.NbNodes()+1):
                    point=mesh.Node(i).Transformed(transform)
                    nodes.append([point.X(),point.Y(),point.Z()])
                nodes=np.array(nodes)
                for i in range(1,mesh.NbTriangles()+1):
                    indices=np.array(mesh.Triangle(i).Get(),dtype=int)-1
                    triangles.append(nodes[indices]);labels.append(label)
            faces.Next()
        if label>=0:
            counts[motors[label]['motor_label']]=len(triangles)-before
        explorer.Next()
    # Some imported gripper geometry is a shell rather than a solid. Keep
    # those faces in the review instead of silently omitting the fingers.
    faces=TopExp_Explorer(shape,TopAbs_FACE)
    while faces.More():
        face=TopoDS.Face(faces.Current())
        if not processed.Contains(face):
            processed.Add(face)
            loc=TopLoc_Location();mesh=BRep_Tool.Triangulation_s(face,loc)
            if mesh is not None:
                transform=loc.Transformation()
                nodes=[]
                for i in range(1,mesh.NbNodes()+1):
                    point=mesh.Node(i).Transformed(transform)
                    nodes.append([point.X(),point.Y(),point.Z()])
                nodes=np.array(nodes)
                for i in range(1,mesh.NbTriangles()+1):
                    indices=np.array(mesh.Triangle(i).Get(),dtype=int)-1
                    triangles.append(nodes[indices]);labels.append(-1)
        faces.Next()
    if len(counts)!=7:
        raise ValueError(f'Expected all seven motor proxy matches, got {counts}')
    triangles=np.array(triangles);labels=np.array(labels)
    np.savez_compressed(outdir/'original_cad_review_mesh.npz',triangles=triangles,labels=labels)
    (outdir/'native_render_verification.json').write_text(json.dumps({
        'source_sha256':inspection['source_sha256'],'triangles':len(triangles),
        'motor_matches':motor_matches,'source_geometry_transformed':False},indent=2),encoding='utf8')
    return triangles,labels,motors,counts


def render(triangles,labels,motors,outdir,title='原始 STEP 电机安装朝向复核',output_name='original_cad_mounting_review.png'):
    data=json.loads((outdir/'cad_check_dimensions.json').read_text(encoding='utf8'))
    datums=data['motor_datums']
    font=lambda n:ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',n)
    canvas=Image.new('RGB',(1800,1320),'#f5f7fa');draw=ImageDraw.Draw(canvas)
    draw.text((40,25),title,fill='#172b43',font=font(34))
    draw.text((40,80),'绿色：CAD 凸台端   紫色：CAD 相反端；均为几何标记，不表示编码器正方向。',fill='#34485f',font=font(23))
    draw.text((40,118),'直接显示文件内姿态，未叠加关节旋转；电机外形为 CAD 示意，不能用颜色对应实物。',fill='#34485f',font=font(23))
    centers=triangles.mean(axis=1)
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-10)
    end_categories=np.zeros(len(triangles),dtype=int)
    for i,motor in enumerate(motors):
        mask=labels==i
        d=np.array(motor['motor_axis_candidate']['direction'])
        p=np.array(datums[i]['point_mm'])
        t=float(p@d)
        vertices=triangles[mask].reshape(-1,3)@d
        lo,hi=vertices.min(),vertices.max()
        sign=1 if t>(lo+hi)/2 else -1
        front=(centers@d-t)*sign
        end_categories[mask & (front>=-1.2)]=1
        rear=lo if sign==1 else hi
        end_categories[mask & (abs(centers@d-rear)<1.1)]=2
    for panel,eye in enumerate(([1,0,0],[1,0.65,0.12])):
        eye=np.array(eye,dtype=float);eye/=np.linalg.norm(eye)
        right=np.cross([0,0,1],eye);right/=np.linalg.norm(right)
        up=np.cross(eye,right)
        x0=35+panel*880;y0=180;width=850;height=1060
        draw.rounded_rectangle((x0,y0,x0+width,y0+height),15,fill='white',outline='#c5cfda',width=2)
        title='沿 CAD +X 侧观察' if panel==0 else '从 CAD +X、+Y 侧斜看'
        draw.text((x0+20,y0+15),title,fill='#21364e',font=font(25))
        u=triangles@right;v=triangles@up
        low=np.array([u.min(),v.min()]);high=np.array([u.max(),v.max()])
        scale=min(520/(high[0]-low[0]),890/(high[1]-low[1]))
        xbase=x0+85;ybase=y0+height-65
        def project(points):
            points=np.array(points)
            return np.stack([xbase+(points@right-low[0])*scale,
                             ybase-(points@up-low[1])*scale],axis=-1)
        projected=project(triangles)
        light=np.array([0.5,-0.3,0.8]);light/=np.linalg.norm(light)
        shades=0.55+0.4*abs(normals@light)
        for index in np.argsort(centers@eye):
            base=([110,179,153] if end_categories[index]==1 else
                  [167,126,186] if end_categories[index]==2 else
                  [149,166,184] if labels[index]>=0 else [202,210,218])
            color=tuple(int(c*shades[index]) for c in base)
            draw.polygon([tuple(p) for p in projected[index]],fill=color)
        for i,datum in enumerate(datums):
            point=np.array(datum['point_mm']);x,y=project(point)
            # Keep the three shoulder labels separated on the same-side view.
            offset={'M6':-40,'M5':7,'M4':36}.get(datum['label'],0)
            textx=x0+440;texty=float(y)+offset
            draw.line((float(x),float(y),textx-12,texty+10),fill='#314b65',width=2)
            draw.ellipse((x-4,y-4,x+4,y+4),fill='#e05b34')
            draw.text((textx,texty),datum['label']+'  '+datum['controller_joint_name'],fill='#203b55',font=font(21))
    draw.text((40,1260),'这两幅是 CAD 观察视图，不是照片配准结果；遮挡处及前后连接仍需按实物确认。',fill='#34485f',font=font(23))
    canvas.save(outdir/output_name)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--kernel',type=Path,required=True)
    parser.add_argument('--title',default='原始 STEP 电机安装朝向复核')
    parser.add_argument('--output-name',default='original_cad_mounting_review.png')
    args=parser.parse_args()
    import hashlib
    source=json.loads((args.output_dir/'step_inspection.json').read_text(encoding='utf8'))
    if hashlib.sha256(args.step.read_bytes()).hexdigest()!=source['source_sha256']:
        raise ValueError('STEP source differs from audited assembly')
    triangles,labels,motors,counts=build_mesh(args.step,args.output_dir,args.kernel)
    render(triangles,labels,motors,args.output_dir,args.title,args.output_name)
    print(json.dumps({'triangles':len(triangles),'motor_proxy_triangle_counts':counts,'original_step_modified':False}))


if __name__=='__main__':
    main()
