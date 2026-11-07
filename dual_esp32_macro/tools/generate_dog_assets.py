from pathlib import Path
import sys,shutil,json,hashlib
r=Path(__file__).resolve().parents[1];sys.path.insert(0,str(r/'.deps'))
from PIL import Image,ImageOps
m=r/'macro_main';out=r/'assets/dogs';out.mkdir(parents=True,exist_ok=True)
source=out/'sprite_sheet.png'
im=Image.open(source).convert('RGBA');assert im.size==(1536,1024)
frames=[];names=[];W,H=128,108
for row,(top,bottom) in enumerate([(0,620),(620,1024)]):
 for col,name in enumerate(['border_collie','golden_retriever','blue_merle']):
  cell=im.crop((col*512,top,(col+1)*512,bottom))
  bbox=cell.getchannel('A').point(lambda a:255 if a>=160 else 0).getbbox();assert bbox
  crop=cell.crop(bbox);crop=ImageOps.contain(crop,(W-4,H-4),Image.Resampling.LANCZOS)
  frame=Image.new('RGBA',(W,H));frame.alpha_composite(crop,((W-crop.width)//2,H-2-crop.height))
  name+=('_awake' if row==0 else '_asleep');frame.save(out/(name+'.png'));frames.append(frame);names.append(name)
# A shared 127-color palette; index zero is reserved for runtime card background.
strip=Image.new('RGB',(W,H*6),(240,235,225))
for i,frame in enumerate(frames):
 base=Image.new('RGBA',(W,H),(240,235,225,255));base.alpha_composite(frame);strip.paste(base.convert('RGB'),(0,i*H))
quant=strip.quantize(colors=127,method=Image.Quantize.MEDIANCUT,dither=Image.Dither.NONE)
pal=quant.getpalette();palette=[0]+[((pal[i*3]>>3)<<11)|((pal[i*3+1]>>2)<<5)|(pal[i*3+2]>>3) for i in range(127)]
data=[]
for i,frame in enumerate(frames):
 ids=list(quant.crop((0,i*H,W,(i+1)*H)).getdata());alpha=list(frame.getchannel('A').getdata());pixels=bytes(v+1 if a>=150 else 0 for v,a in zip(ids,alpha));assert len(pixels)==W*H and max(pixels)<128;data.append(pixels)
(m/'DogAssets.h').write_text('''#pragma once
#include <Arduino.h>
class BoardDisplay;
namespace dogart {
constexpr uint16_t kWidth=128,kHeight=108;
constexpr uint8_t kDogs=3,kFrames=6,kColors=128;
extern const uint16_t palette[kColors] PROGMEM;
extern const uint8_t pixels[kFrames][kWidth*kHeight] PROGMEM;
void draw(BoardDisplay& display,uint8_t dog,bool awake,int16_t x,int16_t y,uint16_t background);
}
''',encoding='utf-8')
with (m/'DogAssets.cpp').open('w',encoding='utf-8') as f:
 f.write('#include "DogAssets.h"\n#include "BoardDisplay.h"\nnamespace dogart {\nconst uint16_t palette[kColors] PROGMEM = {\n')
 for i in range(0,128,16):f.write('  '+','.join('0x%04x'%v for v in palette[i:i+16])+',\n')
 f.write('};\nconst uint8_t pixels[kFrames][kWidth*kHeight] PROGMEM = {\n')
 for name,pixels in zip(names,data):
  f.write(' { // '+name+'\n')
  for i in range(0,len(pixels),32):f.write('  '+','.join('0x%02x'%v for v in pixels[i:i+32])+',\n')
  f.write(' },\n')
 f.write('''};
static_assert(sizeof(pixels)==kFrames*kWidth*kHeight,"Dog asset frame sizes must match");
static_assert(sizeof(palette)==kColors*sizeof(uint16_t),"Palette size mismatch");
void draw(BoardDisplay& display,uint8_t dog,bool awake,int16_t x,int16_t y,uint16_t background) {
  if(dog>=kDogs)return;
  display.drawIndexedBitmap(x,y,pixels[(awake?0:kDogs)+dog],palette,kWidth,kHeight,background);
}
}
''')
# Round-trip RGB565 preview with actual generated palette/pixels.
for name,pixels in zip(names,data):
 p=Image.new('RGBA',(W,H));rgba=[]
 for index in pixels:
  v=palette[index];rgba.append(((v>>11&31)*255//31,(v>>5&63)*255//63,(v&31)*255//31,255 if index else 0))
 p.putdata(rgba);p.save(out/(name+'_indexed.png'))
(out/'manifest.json').write_text(json.dumps({'size':[W,H],'palette_colors':128,'flash_bytes':sum(map(len,data))+256,'frames':names,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'note':'AI-generated art; shared indexed RGB565 palette, index0 transparent'},indent=2)+'\n')
print('Six dog state frames encoded:',sum(map(len,data))+256,'flash bytes; round-trip indexed PNG previews saved.')
