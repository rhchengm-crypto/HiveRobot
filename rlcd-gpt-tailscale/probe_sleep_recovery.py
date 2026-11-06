"""Bounded sleep diagnostics; logs metadata only and never uploads audio."""
from pathlib import Path
import sys,time,json,re
sys.path.insert(0,r'C:/Users/rhche/Documents/Codex/2026-10-24/new-chat/work/rlcd-asr/serial-deps')
import serial
mode=sys.argv[1] if len(sys.argv)>1 else 'info'
allowed=('[SLEEP','[RX_KEY]','[BATTERY]','[STATUS]','[SD_INFO]','[WIFI_INFO]','[MIC_TEST]','[NEWS_SPEECH_INFO]','[NEWS_SPEECH_DONE]')
lines=[]
def connect():
 s=serial.Serial(port=None,baudrate=115200,timeout=.15);s.dtr=True;s.rts=False;s.port='COM19';s.open();return s
def read(s,seconds):
 data=b'';end=time.monotonic()+seconds
 while time.monotonic()<end:data+=s.read(8192)
 for line in data.decode('utf-8','replace').splitlines():
  if line.startswith(allowed):
   line=re.sub(r'ip=\d+(?:\.\d+){3}', 'ip=[local]', line);print(line,flush=True);lines.append(line)
 return data.decode('utf-8','replace')
def command(s,value,seconds=1):s.write((value+'\n').encode());s.flush();return read(s,seconds)
s=None if mode=='physical' else connect()
try:
 if s:read(s,.3)
 if mode in ('short','long'):
  try:command(s,'sleep-long' if mode=='long' else 'sleep-test',2)
  except serial.SerialException:print('[HOST] expected USB sleep disconnect',flush=True)
  s.close();wait=123 if mode=='long' else 5
  print('[HOST] waiting_seconds='+str(wait),flush=True)
  time.sleep(wait);end=time.monotonic()+35
  while True:
   try:s=connect();break
   except serial.SerialException:
    if time.monotonic()>end:raise
    time.sleep(1)
 elif mode=='physical':
  print('[HOST] waiting_for_physical_wake',flush=True)
  time.sleep(3);end=time.monotonic()+160
  while True:
   try:
    s=connect();data=command(s,'sleep-info',1)
    if '[SLEEP_INFO]' in data:break
    s.close()
   except serial.SerialException:pass
   if time.monotonic()>end:raise RuntimeError('No wake response before deadline')
   time.sleep(2)
 for c in ['sleep-info','battery-info','status','sd-info','wifi-info']:command(s,c)
 if mode in ('short','long','physical'):
  data=command(s,'mic',10)
  assert '[MIC_TEST] ok=1' in data,'Microphone sample read failed'
  info=command(s,'sleep-info')
  assert 'cpu_power_down=0' in info and 'result=0' in info,'Expected conservative light sleep and successful wake'
  if mode=='long':
   assert 'reason=4' in info or 'reason=7' in info,'Expected GPIO or timer wake'
   elapsed=re.search(r'slept_ms=(\d+)',info)
   assert elapsed and int(elapsed[1])>=119000,'Long test was interrupted too early'
 if mode=='tone':command(s,'speaker-test',3)
finally:
 if s:s.close()
 p=Path('SLEEP_RECOVERY_VALIDATION.json');saved=json.loads(p.read_text(encoding='utf-8')) if p.exists() else {}
 saved[mode]=lines;p.write_text(json.dumps(saved,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
