from pathlib import Path
import sys
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root/'qa_deps'))
import aspose.words as aw
for name in ['Materials','Procedure']:
    d=aw.Document(str(root/f'HiveRobot_{name}.docx'))
    d.update_page_layout()
    print(name,'pages',d.page_count,flush=True)
    dest=root/f'qa_{name.lower()}'
    dest.mkdir(exist_ok=True)
    for page in range(d.page_count):
        options=aw.saving.ImageSaveOptions(aw.SaveFormat.PNG)
        options.page_set=aw.saving.PageSet(page)
        options.horizontal_resolution=150
        options.vertical_resolution=150
        d.save(str(dest/f'page-{page+1}.png'),options)
