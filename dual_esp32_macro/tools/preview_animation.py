from pathlib import Path
import sys,re,struct,zlib,json
r=Path(r'C:\Users\rhche\Desktop\dual_esp32_macro');sys.path.insert(0,str(r/'.deps'))
from PIL import Image,ImageDraw
cpp=(r/'macro_main/DogAssets.cpp').read_text();pal=[int(x,16) for x in re.findall(r'0x[0-9a-fA-F]{4}',cpp.split('const uint8_t pixels')[0])];raw=bytes(int(x,16) for x in re.findall(r'0x[0-9a-fA-F]{2}\b',cpp.split('const uint8_t pixels')[1].split('static_assert')[0]));assert len(raw)==6*128*108
pack=(r/'sdcard/macrodesk/blink.bin').read_bytes();assert pack[:4]==b'DGB1' and zlib.crc32(pack[8:])==struct.unpack('<I',pack[4:8])[0]
assert zlib.crc32(b'123456789')==0xcbf43926
frames=[];base=Image.open(r/'ui_preview/lights.png').convert('RGB');colors=[0xff9a,0xd6da,0xff9a]
# Use the actual preview's card background, avoiding guessed theme values.
colors=[base.getpixel((12+i*156+4,150)) for i in range(3)]
def sprite(ids,bg):
 im=Image.new('RGB',(128,108));im.putdata([bg if n==0 else (((pal[n]>>11)&31)*255//31,((pal[n]>>5)&63)*255//63,(pal[n]&31)*255//31) for n in ids]);return im
for t in range(0,4300,50):
 im=base.copy()
 for dog,on in enumerate([True,False,True]):
  phase=t+dog*730
  if on:ids=pack[8+dog*13824:8+(dog+1)*13824] if phase%4300<150 else raw[dog*13824:(dog+1)*13824]
  else:ids=raw[(3+dog)*13824:(4+dog)*13824]
  image=sprite(ids,colors[dog]);pad=0
  if not on:
   p=(phase//350)%8;pad=p if p<=4 else 8-p
   if pad:
    img=Image.new('RGB',(128,108),colors[dog]);pixels=image.load();target=img.load()
    for y in range(pad,108):
     for x in range(128):target[x,y]=pixels[x,(y-pad)*108//(108-pad)]
    image=img
  im.paste(image,(20+dog*156,114))
 frames.append(im)
frames[0].save(r/'ui_preview/dog_animation.gif',save_all=True,append_images=frames[1:],duration=50,loop=0,optimize=True)
print('Validated pack CRC/indices/size; generated 4.3-second UI animation preview.')
