"""Safe battery diagnostics: only print explicitly selected telemetry records."""
import sys,time,re,json
from pathlib import Path
sys.path.insert(0,r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-asr\serial-deps')
import serial
s=serial.Serial(port=None,baudrate=115200,timeout=.2)
s.dtr=True;s.rts=False;s.port='COM19';s.open()
records=[]
def command(cmd):
 s.reset_input_buffer();s.write((cmd+'\n').encode());s.flush()
 end=time.monotonic()+3;data=b''
 while time.monotonic()<end:data+=s.read(4096)
 lines=data.decode('utf-8','replace').splitlines()
 for line in lines:
  if line.startswith(('[BATTERY]','[STATUS]','[SD_HISTORY]','[UI_INFO]')):
   print(line,flush=True);records.append(line)
 return lines
try:
 command('battery-info');time.sleep(5);command('battery-info')
 command('status');command('sd-info');command('ui-info')
 assert any('valid=1' in x for x in records if x.startswith('[BATTERY]')),'No calibrated voltage'
 Path('BATTERY_VALIDATION.json').write_text(json.dumps({'telemetry':records},ensure_ascii=False,indent=2),encoding='utf-8')
finally:s.close()
