from pathlib import Path
import shutil,re,struct,zlib,sys
r=Path(r'C:\Users\rhche\Desktop\dual_esp32_macro');sys.path.insert(0,str(r/'.deps'))
from PIL import Image,ImageOps
a=r/'assets/dogs'
source=Image.open(a/'sprite_sheet.png').convert('RGBA');blink=Image.open(a/'blink_sheet.png').convert('RGBA');assert blink.size==source.size
cpp=(r/'macro_main/DogAssets.cpp').read_text();pal=[int(x,16) for x in re.findall(r'0x[0-9a-fA-F]{4}',cpp.split('const uint8_t pixels')[0])];assert len(pal)==128
rgb=[((v>>11&31)*255//31,(v>>5&63)*255//63,(v&31)*255//31) for v in pal]
cache={};data=bytearray()
for i,name in enumerate(['border_collie','golden_retriever','blue_merle']):
 cell=source.crop((512*i,0,512*(i+1),620));bbox=cell.getchannel('A').point(lambda a:255 if a>=160 else 0).getbbox()
 crop=ImageOps.contain(blink.crop((512*i,0,512*(i+1),620)).crop(bbox),(124,104),Image.Resampling.LANCZOS)
 frame=Image.new('RGBA',(128,108));frame.alpha_composite(crop,((128-crop.width)//2,106-crop.height));frame.save(a/(name+'_blink.png'))
 for p in frame.getdata():
  if p[3]<150:data.append(0);continue
  c=p[:3]
  if c not in cache:cache[c]=min(range(1,128),key=lambda j:sum((rgb[j][k]-c[k])**2 for k in range(3)))
  data.append(cache[c])
assert len(data)==41472 and max(data)<128
out=r/'sdcard/macrodesk';out.mkdir(parents=True,exist_ok=True)
(out/'blink.bin').write_bytes(b'DGB1'+struct.pack('<I',zlib.crc32(data))+data)
(a/'ANIMATION_PROMPT.txt').write_text('Built-in image_gen edit of sprite_sheet.png: close ONLY the eyes of the three awake dogs for a blink frame; preserve bodies, proportions, positions, transparent background and sleeping row. Output blink_sheet.png. Runtime uses original shared RGB565 palette; source bounding boxes preserve registration.\n')
print('Blink pack:',len(data)+8,'bytes; CRC',hex(zlib.crc32(data)))
