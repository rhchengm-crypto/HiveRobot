from pathlib import Path
import sys,struct,re,zlib,json,hashlib
ROOT=Path(r'C:\Users\rhche\Desktop\dual_esp32_macro')
sys.path.insert(0,str(ROOT/'.deps'))
from PIL import Image,ImageOps
HERE=Path(__file__).resolve().parent
OUT=ROOT/'assets/dog_animation' if HERE.name=='tools' else HERE/'assets'
OUT.mkdir(parents=True,exist_ok=True)
source=ROOT/'macro_main/DogAssets.cpp'
text=source.read_text(encoding='utf-8')
palette_source=text.split('const uint16_t palette')[1].split('};')[0]
old_palette=[int(x,16) for x in re.findall(r'0x[0-9a-fA-F]+',palette_source)]
pixel_source=text.split('const uint8_t pixels')[1].split('};')[0]
old_pixels=bytes(int(x,16) for x in re.findall(r'0x[0-9a-fA-F]+',pixel_source))
assert len(old_palette)==128 and len(old_pixels)==6*128*108
sheet=Image.open(OUT/'alternate-sheet.png').convert('RGBA')
assert sheet.size==(1536,1024)
assert sheet.getchannel('A').getextrema()[0]==0,'Generated alpha missing'
alternates=[]
for row,(top,bottom) in enumerate([(0,620),(620,1024)]):
    for col in range(3):
        cell=sheet.crop((col*512,top,(col+1)*512,bottom))
        bounds=cell.getchannel('A').point(lambda a:255 if a>=160 else 0).getbbox()
        assert bounds
        dog=ImageOps.contain(cell.crop(bounds),(124,104),Image.Resampling.LANCZOS)
        frame=Image.new('RGBA',(128,108));frame.alpha_composite(dog,((128-dog.width)//2,106-dog.height))
        alternates.append(frame)
strip=Image.new('RGB',(128,108*6),(240,235,225))
for i,frame in enumerate(alternates):
    backdrop=Image.new('RGBA',frame.size,(240,235,225,255));backdrop.alpha_composite(frame)
    strip.paste(backdrop.convert('RGB'),(0,i*108))
quant=strip.quantize(colors=127,method=Image.Quantize.MEDIANCUT,dither=Image.Dither.NONE)
raw_palette=quant.getpalette()
new_palette=[((raw_palette[i*3]>>3)<<11)|((raw_palette[i*3+1]>>2)<<5)|(raw_palette[i*3+2]>>3) for i in range(127)]
palette=old_palette+new_palette+[0]
assert len(palette)==256
frames=[]
for i in range(6):
    original=old_pixels[i*13824:(i+1)*13824]
    ids=list(quant.crop((0,i*108,128,(i+1)*108)).getdata())
    alpha=list(alternates[i].getchannel('A').getdata())
    alternate=bytes(index+128 if a>=150 else 0 for index,a in zip(ids,alpha))
    frames.extend([original,alternate])
colors=struct.pack('<256H',*palette)
header=struct.pack('<8sHHHHI',b'DOGANIM1',128,108,12,256,13824)
header+=struct.pack('<III',zlib.crc32(header),zlib.crc32(colors),0)
pack=header+colors+b''.join(struct.pack('<I',zlib.crc32(frame))+frame for frame in frames)
assert len(pack)==166480
(OUT/'animation.dga').write_bytes(pack)
# Decode the actual pack into transparent PNGs and a small screen-scale GIF.
decoded=[]
for n,frame in enumerate(frames):
    image=Image.new('RGBA',(128,108));rgba=[]
    for index in frame:
        v=palette[index]
        rgba.append(((v>>11&31)*255//31,(v>>5&63)*255//63,(v&31)*255//31,255 if index else 0))
    image.putdata(rgba);image.save(OUT/f'frame-{n:02}.png');decoded.append(image)
preview=[]
for t in range(24):
    canvas=Image.new('RGB',(432,248),(1,20,40))
    for state in range(2):
        for dog in range(3):
            bg=(255,239,214) if state==0 else (214,214,214)
            tile=Image.new('RGBA',(140,116),bg+(255,))
            variant=(t%24>=21) if state==0 else (t%16 in (6,7,8,9))
            frame=decoded[(state*3+dog)*2+int(variant)]
            # Preview same bounded 0-2px vertical breathing as firmware.
            inset=([0,0,1,2,2,1,0,0][(t//2)%8] if state else int(t%16 in (8,9,10,11)))
            if inset:
                frame=frame.resize((128,108-inset),Image.Resampling.NEAREST)
            tile.alpha_composite(frame,(6,4+inset))
            canvas.paste(tile.convert('RGB'),(dog*144,state*124))
    preview.append(canvas)
preview[0].save(OUT/'dogs-animation-preview.gif',save_all=True,append_images=preview[1:],duration=160,loop=0)
meta={'format':'DOGANIM1','bytes':len(pack),'crc32':f'{zlib.crc32(pack):08x}',
      'sha256':hashlib.sha256(pack).hexdigest(),'frames':12,'size':[128,108],
      'order':'state awake/asleep, dog collie/golden/merle, original/alternate',
      'source':'built-in image_gen precise animation-frame edit; original static sprites retained exactly'}
(OUT/'manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps(meta))
