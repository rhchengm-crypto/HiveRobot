from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
from copy import deepcopy
import xml.etree.ElementTree as ET
import openpyxl

p = Path(__file__).parent / '左右臂电机ID备忘.xlsx'
ns = {'s':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
with ZipFile(p) as z:
    files = {n:z.read(n) for n in z.namelist()}
styles = ET.fromstring(files['xl/styles.xml'])
xfs = styles.find('s:cellXfs', ns)
sheet = ET.fromstring(files['xl/worksheets/sheet2.xml'])
for row in range(28,31):
    for col in ['D','F']:
        cell = sheet.find(f".//s:c[@r='{col}{row}']",ns)
        style = deepcopy(xfs[int(cell.get('s','0'))])
        style.set('numFmtId','49')
        style.set('applyNumberFormat','1')
        cell.set('s',str(len(xfs)))
        xfs.append(style)
xfs.set('count',str(len(xfs)))
files['xl/styles.xml']=ET.tostring(styles,encoding='utf-8',xml_declaration=True)
files['xl/worksheets/sheet2.xml']=ET.tostring(sheet,encoding='utf-8',xml_declaration=True)
with ZipFile(p,'w',ZIP_DEFLATED) as z:
    for name,data in files.items(): z.writestr(name,data)
w=openpyxl.load_workbook(p)
for row in range(28,31):
    for col in ['D','F']:
        c=w['全身参考'][f'{col}{row}']
        assert c.value.startswith('0x') and c.number_format=='@'
print('Hex ID cells verified as literal text with 0x prefix.')
