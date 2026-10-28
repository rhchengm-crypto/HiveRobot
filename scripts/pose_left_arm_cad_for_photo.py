#!/usr/bin/env python3
"""Create a review-only articulated STEP pose; no robot/controller imports.

Rigid-group membership is an explicit visual-review hypothesis for this exact
assembly. Transform entire nonempty leaf shapes; never slice a mesh by height.
Angles are relative to the original CAD pose, not encoder offsets.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def rotation_about(point, axis, degrees):
    axis=np.array(axis,dtype=float);axis/=np.linalg.norm(axis)
    x,y,z=axis
    skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    angle=math.radians(degrees)
    rotation=np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)
    result=np.eye(4);result[:3,:3]=rotation
    point=np.array(point);result[:3,3]=point-rotation@point
    return result


def transform_points(points, matrix):
    return np.asarray(points)@matrix[:3,:3].T+matrix[:3,3]


def leaf_shapes(step):
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.collections import Sequence_TDF_Label
    from OCP.TDF import TDF_Label
    from OCP.TDataStd import TDataStd_Name
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.IFSelect import IFSelect_RetDone
    reader=STEPCAFControl_Reader()
    if reader.ReadFile(str(step))!=IFSelect_RetDone:
        raise ValueError('STEP read failed')
    doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    if not reader.Transfer(doc):
        raise ValueError('XCAF transfer failed')
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots=Sequence_TDF_Label();tool.GetFreeShapes(roots)
    leaves=[]
    def walk(label,path):
        ref=TDF_Label();resolved=ref if tool.GetReferredShape_s(label,ref) else label
        attr=TDataStd_Name()
        name=attr.Get().ToExtString() if resolved.FindAttribute(TDataStd_Name.GetID_s(),attr) else 'unnamed'
        shape=tool.GetShape_s(label)
        # For this exact export every occurrence location is identity. Refuse
        # a different assembly rather than mishandling nested placements.
        if not shape.IsNull():
            location=shape.Location().Transformation()
            matrix=np.array([[location.Value(i,j) for j in range(1,5)] for i in range(1,4)])
            if np.max(abs(matrix-np.eye(4)[:3]))>1e-9:
                raise ValueError(f'Unexpected nonidentity occurrence at {path}: {matrix.tolist()}')
        children=Sequence_TDF_Label();tool.GetComponents_s(resolved,children)
        if children.Length():
            for i in range(1,children.Length()+1):walk(children.Value(i),path+[i])
            return
        if shape.IsNull():return
        box=Bnd_Box();BRepBndLib.AddOptimal_s(shape,box,False,False)
        if box.IsVoid():return
        lo,hi=box.CornerMin(),box.CornerMax()
        leaves.append({'name':name,'occurrence_path':path,'shape':shape,
                       'bounds_mm':[[lo.X(),lo.Y(),lo.Z()],[hi.X(),hi.Y(),hi.Z()]]})
    for i in range(1,roots.Length()+1):walk(roots.Value(i),[i])
    return leaves,doc


def triangles_for_shape(shape):
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location
    explorer=TopExp_Explorer(shape,TopAbs_FACE)
    triangles=[]
    while explorer.More():
        face=TopoDS.Face(explorer.Current());loc=TopLoc_Location()
        mesh=BRep_Tool.Triangulation_s(face,loc)
        if mesh is not None:
            nodes=[]
            for i in range(1,mesh.NbNodes()+1):
                point=mesh.Node(i).Transformed(loc.Transformation())
                nodes.append([point.X(),point.Y(),point.Z()])
            nodes=np.array(nodes)
            for i in range(1,mesh.NbTriangles()+1):
                triangles.append(nodes[np.array(mesh.Triangle(i).Get(),dtype=int)-1])
        explorer.Next()
    return np.array(triangles,dtype=float).reshape(-1,3,3)


def draw_review(original,posed,labels,datums,transforms,outdir,side_deg,roll_deg):
    font=lambda n:ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',n)
    image=Image.new('RGB',(1800,1470),'#f5f7fa');draw=ImageDraw.Draw(image)
    draw.text((40,22),'照片姿态对齐候选 · 等待实物确认',font=font(34),fill='#19324a')
    draw.text((40,78),f'相对原 CAD：shoulder_side {side_deg:+g}°，arm_roll {roll_deg:+g}°；其余关节暂不调整。',font=font(24),fill='#34485e')
    draw.text((40,116),'仅做姿态试配，不是标定结果；M6 被机身遮挡，照片中未验证。',font=font(23),fill='#34485e')
    eye=np.array([1.,0.45,0.08]);eye/=np.linalg.norm(eye)
    right=np.cross([0,0,1],eye);right/=np.linalg.norm(right);up=np.cross(eye,right)
    arrays=[original,posed]
    union=np.concatenate(arrays)
    lows=np.array([(union@right).min(),(union@up).min()])
    highs=np.array([(union@right).max(),(union@up).max()])
    scale=min(470/(highs[0]-lows[0]),1000/(highs[1]-lows[1]))
    for panel,(triangles,title) in enumerate(zip(arrays,['原始 CAD 姿态','调整后的候选姿态'])):
        x0=30+panel*890;y0=180
        draw.rounded_rectangle((x0,y0,x0+850,y0+1170),14,fill='white',outline='#c4cfda',width=2)
        draw.text((x0+20,y0+15),title,font=font(26),fill='#203a54')
        def project(points):
            points=np.asarray(points)
            return np.stack([x0+45+(points@right-lows[0])*scale,
                             y0+1110-(points@up-lows[1])*scale],axis=-1)
        projected=project(triangles)
        centers=triangles.mean(axis=1)
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)
        light=np.array([0.4,-0.2,0.9]);light/=np.linalg.norm(light)
        shades=0.53+0.43*abs(normals@light)
        for i in np.argsort(centers@eye):
            base=[133,169,193] if labels[i]>=0 else [201,210,219]
            if panel and labels[i] in (0,1):base=[90,172,143]
            color=tuple(int(v*shades[i]) for v in base)
            draw.polygon([tuple(v) for v in projected[i]],fill=color)
        for datum in datums:
            label=datum['label'];point=np.array(datum['point_mm'])
            if panel:point=transform_points(point,transforms[label])
            x,y=project(point)
            off={'M6':-37,'M5':5,'M4':35}.get(label,0)
            textx=x0+455;texty=float(y)+off
            draw.line((float(x),float(y),textx-10,texty+10),fill='#37536c',width=2)
            draw.ellipse((x-4,y-4,x+4,y+4),fill='#d94b2d')
            draw.text((textx,texty),label+' '+datum['controller_joint_name'],font=font(21),fill='#1b3c54')
    draw.text((40,1380),'绿色电机：随 arm_roll 一起转动的两腕轴。支架、末端均按零件整体旋转，未逐个翻转电机。',font=font(22),fill='#34485e')
    draw.text((40,1420),'相机视角为人工近似；遮挡及单张照片的深度歧义尚未消除。不能将上面的角度写入编码器偏置。',font=font(22),fill='#34485e')
    image.save(outdir/'photo_pose_candidate_comparison.png')
    # Separate CAD-only panel for side-by-side Markdown display with the
    # user's unmodified photograph. No processing of that photograph occurs.
    image.crop((920,180,1770,1350)).save(outdir/'photo_pose_candidate_view.png')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--kernel',type=Path,required=True)
    parser.add_argument('--shoulder-side-deg',type=float,default=20.)
    parser.add_argument('--arm-roll-deg',type=float,default=180.)
    args=parser.parse_args()
    outdir=args.output_dir
    source=json.loads((outdir/'step_inspection.json').read_text(encoding='utf8'))
    if hashlib.sha256(args.step.read_bytes()).hexdigest()!=source['source_sha256']:
        raise ValueError('STEP differs from audited assembly')
    datums=json.loads((outdir/'cad_check_dimensions.json').read_text(encoding='utf8'))['motor_datums']
    lookup={row['label']:row for row in datums}
    side=rotation_about(lookup['M5']['point_mm'],lookup['M5']['axis_direction_arbitrary_sign'],args.shoulder_side_deg)
    roll=rotation_about(lookup['M3']['point_mm'],lookup['M3']['axis_direction_arbitrary_sign'],args.arm_roll_deg)
    transforms={label:np.eye(4) if label in ('M5','M6') else side@roll if label in ('M1','M2') else side for label in lookup}
    sys.path.insert(0,str(args.kernel))
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound
    from OCP.gp import gp_Trsf
    from OCP.TopLoc import TopLoc_Location
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
    from OCP.IFSelect import IFSelect_RetDone
    leaves,doc=leaf_shapes(args.step)
    native=json.loads((outdir/'occt_brep_verification.json').read_text(encoding='utf8'))
    motor_bounds=np.array([row['bounds_mm'] for row in native['assembly_nodes'] if row['name']=='Wrist motor.step']).reshape(-1,2,3)
    originals=[];posed=[];labels=[];manifest=[];native_posed=[]
    for leaf in leaves:
        bounds=np.array(leaf['bounds_mm']);motor_id=None
        if leaf['name']=='Wrist motor.step':
            errors=np.linalg.norm(motor_bounds-bounds,axis=(1,2));motor_id=int(np.argmin(errors))
            if errors[motor_id]>1e-3:raise ValueError('Unexpected motor bounds')
            label='M'+str(motor_id+1)
            matrix=transforms[label]
            group='shoulder fixed' if label in ('M5','M6') else 'arm_roll downstream' if label in ('M1','M2') else 'shoulder_side downstream'
        elif bounds[1,2]<-145:
            matrix=side@roll;group='arm_roll downstream'
        elif bounds[0,2]>195:
            matrix=np.eye(4);group='shoulder fixed'
        else:
            matrix=side;group='shoulder_side downstream'
        BRepMesh_IncrementalMesh(leaf['shape'],0.8,False,0.35,True)
        triangles=triangles_for_shape(leaf['shape'])
        originals.append(triangles);posed.append(transform_points(triangles,matrix))
        labels.extend([motor_id if motor_id is not None else -1]*len(triangles))
        transform=gp_Trsf();transform.SetValues(*matrix[:3,:].reshape(-1).tolist())
        changed=leaf['shape'].Moved(TopLoc_Location(transform))
        native_posed.append(changed)
        manifest.append({k:v for k,v in leaf.items() if k!='shape'}|{'review_rigid_group':group,'transform_from_original_cad':matrix.tolist(),'triangles':len(triangles)})
    originals=np.concatenate(originals);posed=np.concatenate(posed);labels=np.array(labels)
    # Rigid transforms must preserve every edge, including shells/fingers.
    before=np.linalg.norm(originals[:,1]-originals[:,0],axis=1)
    after=np.linalg.norm(posed[:,1]-posed[:,0],axis=1)
    error=float(np.max(abs(before-after)))
    if error>1e-7:raise ValueError('Rigid-transform edge length check failed')
    compound=TopoDS_Compound();builder=BRep_Builder();builder.MakeCompound(compound)
    for shape in native_posed:builder.Add(compound,shape)
    writer=STEPControl_Writer()
    if writer.Transfer(compound,STEPControl_AsIs)!=IFSelect_RetDone:raise ValueError('STEP transfer failed')
    target=outdir/'left_arm_photo_pose_candidate.step'
    if target.resolve()==args.step.resolve():raise ValueError('Refusing to overwrite original STEP')
    if writer.Write(str(target))!=IFSelect_RetDone:raise ValueError('STEP write failed')
    report={'source_sha256':source['source_sha256'],'candidate_step':str(target.resolve()),
            'status':'review candidate; awaiting user visual confirmation',
            'photo_evidence':'physical_assembly_photo_evidence.json',
            'cad_relative_visual_pose_angles_deg':{'shoulder_side':args.shoulder_side_deg,'arm_roll':args.arm_roll_deg},
            'angle_convention':'right-hand rotation about ORIGINAL stored CAD axis vector; not motor/encoder sign',
            'angles_from_measurement':False,'encoders_calibrated':False,'runtime_control_modified':False,
            'M6_hidden_and_not_pose_verified':True,'rigid_group_membership':'provisional assembly interpretation, inspect manifest and connection interfaces',
            'leaves':manifest,'edge_length_preservation_max_error_mm':error,
            'joint_transforms_from_original':{k:v.tolist() for k,v in transforms.items()}}
    (outdir/'photo_pose_candidate.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    np.savez_compressed(outdir/'photo_pose_candidate_mesh.npz',original=originals,posed=posed,labels=labels)
    draw_review(originals,posed,labels,datums,transforms,outdir,args.shoulder_side_deg,args.arm_roll_deg)
    print(json.dumps({'leaves':len(leaves),'triangles':len(posed),'edge_error_mm':error,'step':str(target),'status':'candidate only'}))


if __name__=='__main__':
    main()
