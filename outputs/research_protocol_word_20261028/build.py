from pathlib import Path
import math
from docx import Document
from docx.shared import Inches, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.enum.table import WD_ROW_HEIGHT_RULE

OUT = Path(__file__).resolve().parent
materials = [
 ('NVIDIA Jetson Orin computer (fanless)','1'),
 ('Development computer','1'),
 ('Damiao USB-to-CAN adapter','1'),
 ('Nuwa HP60C RGB-D camera','1'),
 ('Damiao DM4340 motors','3'),
 ('Damiao DM4310 motors','5'),
 ('Metal 3D-printed arm structure','1 set'),
 ('3D-printed main body','1 set'),
 ('Gripper palm and fingers','1 set'),
 ('4040 European-profile aluminum extrusion','TBC'),
 ('Extrusion brackets and anchors','TBC'),
 ('Bolts, nuts and washers','TBC'),
 ('Wheel-hub motors','4'),
 ('ZLAC8015D wheel-motor drivers','TBC'),
 ('24 V battery pack','TBC'),
 ('Compatible battery charger','TBC'),
 ('DC voltage converters','TBC'),
 ('Power-distribution board','TBC'),
 ('Fuses and fuse holders','TBC'),
 ('DC isolation switch','1'),
 ('Physical emergency-stop hardware','1 set'),
 ('Mechanical arm support','1'),
 ('Wheel chocks','1 set'),
 ('Motor-power cables and connectors','TBC'),
 ('CAN cables and connectors','TBC'),
 ('CAN termination components','TBC'),
 ('USB data cables','TBC'),
 ('Cable sleeves, clips and strain relief','TBC'),
 ('Camera mount','1'),
 ('Network router','1'),
 ('Ethernet cable','TBC'),
 ('Jetson storage device','TBC'),
 ('Chessboard','1'),
 ('White knight chess piece','1'),
 ('Stable worktable','1'),
 ('Task lighting','1 set'),
 ('Millimeter ruler','1'),
 ('Digital caliper','1'),
 ('Camera calibration target','1'),
 ('Hex keys and screwdrivers','1 set'),
 ('Torque driver','1'),
 ('Rated digital multimeter','1'),
 ('Impact-rated safety glasses','1 pair'),
 ('Cut-resistant handling gloves','1 pair'),
 ('Closed-toe shoes','1 pair'),
 ('Barrier or floor-marking tape','1 set'),
 ('Motion-warning sign','1'),
 ('Facility-approved battery incident equipment','1 set'),
 ('Backup storage drive','1'),
]
programs = ['Ubuntu / NVIDIA JetPack','Python 3','NumPy','PySerial','DM_CAN library','OpenCV',
 'HP60C camera driver','ROS Noetic','PyTorch','Ultralytics YOLO','SciPy','Visual Studio Code',
 'Git','SSH / SCP client','Robot Web-control application','General dynamics controller',
 'Board-grasp controller','Model Hold controller']

def doc_base(title):
 d=Document(); sec=d.sections[0]
 sec.page_width=Inches(8.5);sec.page_height=Inches(11)
 sec.top_margin=Inches(.45);sec.bottom_margin=Inches(.45)
 sec.left_margin=Inches(.45);sec.right_margin=Inches(.45)
 sec.header_distance=Inches(.15);sec.footer_distance=Inches(.15)
 normal=d.styles['Normal'];normal.font.name='Arial';normal.font.size=Pt(10)
 normal.paragraph_format.space_after=Pt(0)
 normal.paragraph_format.line_spacing=1
 for name in ['Title','Heading 1','Heading 2']:
  st=d.styles[name];st.font.name='Arial';st.font.color.rgb=RGBColor(0,0,0)
  st.paragraph_format.space_before=Pt(0);st.paragraph_format.space_after=Pt(5)
  for borders in list(st.element.iter(qn('w:pBdr'))):
   borders.getparent().remove(borders)
 d.styles['Title'].font.size=Pt(17);d.styles['Title'].font.bold=True
 d.styles['Heading 1'].font.size=Pt(11);d.styles['Heading 1'].font.bold=True
 d.add_paragraph(title,'Title')
 d.core_properties.author='';d.core_properties.title=title
 return d

def note(d,text):
 p=d.add_paragraph(text);p.paragraph_format.space_after=Pt(4)
 for r in p.runs:r.font.size=Pt(8)
 return p

def table(d,entries,header):
 count=math.ceil(len(entries)/2)
 t=d.add_table(rows=count+1,cols=4);t.autofit=False;t.alignment=WD_TABLE_ALIGNMENT.CENTER
 widths=[3.16,.64,3.16,.64]
 for c,w in zip(t.columns,widths):c.width=Inches(w)
 pr=t._tbl.tblPr
 borders=OxmlElement('w:tblBorders')
 for side in ['top','left','bottom','right','insideH','insideV']:
  e=OxmlElement('w:'+side);e.set(qn('w:val'),'single');e.set(qn('w:sz'),'4');e.set(qn('w:color'),'D9D9D9');borders.append(e)
 pr.append(borders)
 margins=OxmlElement('w:tblCellMar')
 for side,size in [('top',15),('bottom',15),('left',65),('right',65)]:
  e=OxmlElement('w:'+side);e.set(qn('w:w'),str(size));e.set(qn('w:type'),'dxa');margins.append(e)
 pr.append(margins)
 for i,row in enumerate(t.rows):
  row.height=Pt(14);row.height_rule=WD_ROW_HEIGHT_RULE.AT_LEAST
  no_split=OxmlElement('w:cantSplit');row._tr.get_or_add_trPr().append(no_split)
  if i==0:
   row._tr.get_or_add_trPr().append(OxmlElement('w:tblHeader'))
   values=[header,'Quantity',header,'Quantity']
  else:
   left=entries[i-1];right=entries[i-1+count] if i-1+count<len(entries) else ('','')
   values=[*left,*right]
  for j,(cell,txt,w) in enumerate(zip(row.cells,values,widths)):
   cell.width=Inches(w);cell.vertical_alignment=WD_CELL_VERTICAL_ALIGNMENT.CENTER
   p=cell.paragraphs[0];p.alignment=WD_ALIGN_PARAGRAPH.CENTER if j%2 else WD_ALIGN_PARAGRAPH.LEFT
   p.paragraph_format.space_before=Pt(0);p.paragraph_format.space_after=Pt(0);p.paragraph_format.line_spacing=Pt(11)
   r=p.add_run(txt);r.font.name='Arial';r.font.size=Pt(9.5)
   if i==0:
    r.bold=True;r.font.color.rgb=RGBColor(255,255,255)
   shade=OxmlElement('w:shd');shade.set(qn('w:fill'),'233E50' if i==0 else ('F1F4F6' if i%2==0 else 'FFFFFF'))
   cell._tc.get_or_add_tcPr().append(shade)
 return t

d=doc_base('HiveRobot Experiment Materials')
note(d,'Quantities are setup requirements; TBC means unconfirmed. Motor counts cover the tested left arm and gripper.')
table(d,materials,'Material')
h=d.add_paragraph('Programs','Heading 1');h.paragraph_format.space_before=Pt(6)
table(d,[(p,'') for p in programs],'Program')
note(d,'Program quantities are intentionally blank. Structural metal alloy and battery capacity remain unconfirmed.')
d.save(OUT/'HiveRobot_Materials.docx')

actions=[
 'Assign one command operator.',
 'Assign one stop observer.',
 'Wear impact-rated safety glasses.',
 'Wear closed-toe shoes.',
 'Secure loose clothing, jewelry and hair.',
 'Immobilize the wheeled base with chocks.',
 'Support the arm mechanically.',
 'Isolate motor power before inspection.',
 'Wear handling gloves for sharp, unpowered parts.',
 'Inspect the stand and arm for damage.',
 'Verify fastener tightness against specified torques.',
 'Inspect power and CAN wiring.',
 'Confirm compatibility with the 24 V battery.',
 'Remove handling gloves before motion.',
 'Verify stop behavior with the arm mechanically supported.',
 'Mark the approved motion exclusion zone.',
 'Clear all people and loose objects from that zone.',
 'Secure the chessboard.',
 'Secure the camera.',
 'Back up code, models and adaptive-learning files.',
 'Confirm that only one process owns the motor adapter.',
 'Verify retained Model Hold, learning and Clearance settings.',
 'Verify cooling suitability for the intended ML workload.',
 'Record hardware and software versions.',
 'Set approved positioning tolerances in millimeters.',
 'Calibrate the board-plane measurement scale.',
 'Restore power using the approved startup procedure.',
 'Record stationary seven-axis feedback.',
 'Run the approved Home workflow.',
 'Enter the approved Cartesian target in millimeters.',
 'Preview the model-generated path.',
 'Verify clearance along the full path.',
 'Start the fixed-camera recording.',
 'Execute Above Only with an empty path.',
 'Check the physical above-target position.',
 'Preview Touch Position from the measured current pose.',
 'Execute the approved open-claw descent.',
 'Observe the 5-second automatic Model Hold.',
 'Record independent X, Y and height measurements.',
 'Record lateral drift during descent and after takeover.',
 'Save joint-feedback and timing logs.',
 'Repeat the approved sequence three times per target.',
 'Keep claw closure blocked until grasp testing is released.',
 'Return through the approved Home workflow.',
 'Support the arm mechanically before shutdown.',
 'Isolate motor power.',
 'Archive the trial records.',
]
d=doc_base('HiveRobot Experiment Procedure')
note(d,'Before testing: resolve electrical ratings, stop behavior and the motion envelope. Three repeats are proposed.')
note(d,'Unexpected contact or instability: follow the approved stop procedure. Never isolate an unsupported load.')
style=d.styles['List Number'];style.font.name='Arial';style.font.size=Pt(10.5)
style.paragraph_format.space_after=Pt(0);style.paragraph_format.line_spacing=Pt(13)
style.paragraph_format.left_indent=Inches(.24);style.paragraph_format.first_line_indent=Inches(-.24)
for action in actions:
 p=d.add_paragraph(action,'List Number')
 p.paragraph_format.keep_with_next=False
note(d,'Clearance retains its shoulder_rotate exclusion. Existing motion watchdogs record errors; grasp closure remains unreleased.')
d.save(OUT/'HiveRobot_Procedure.docx')
print(f'Created materials ({len(materials)} physical rows, {len(programs)} programs) and procedure ({len(actions)} steps).')
