"""Read the firmware's display-buffer diagnostic without changing the screen."""
import sys,re,time,struct,zlib
from pathlib import Path
sys.path.insert(0,r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-asr\serial-deps')
import serial
s=serial.Serial(port=None,baudrate=115200,timeout=.5);s.dtr=True;s.rts=False;s.port='COM19';s.open()
try:
 s.reset_input_buffer();s.write(b'ui-frame\n');deadline=time.monotonic()+10;header=None
 while time.monotonic()<deadline:
  header=re.search(rb'\[UI_FRAME\] bytes=(\d+) width=(\d+) height=(\d+)',s.readline())
  if header:break
 assert header,'No frame header'
 size,width,height=map(int,header.groups());raw=bytearray()
 while len(raw)<size and time.monotonic()<deadline:raw+=s.read(size-len(raw))
 assert len(raw)==size
 pixels=bytearray()
 for y in range(width):
  pixels.append(0)
  for x in range(height):
   ox,oy=width-1-y,x
   pixels.append(0 if raw[(oy//8)*width+ox]&(1<<(oy%8)) else 255)
 def chunk(kind,data):return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data))
 png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',height,width,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(pixels))+chunk(b'IEND',b'')
 target=Path(__file__).parent/(sys.argv[1] if len(sys.argv)>1 else 'sd-restored-gpt.png');target.write_bytes(png);print(target)
finally:s.close()
