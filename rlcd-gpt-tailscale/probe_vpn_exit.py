"""Exercise actual native VPN exit while switching to non-VPN pages; no model/voice calls."""
from pathlib import Path
import sys,time,json,re
sys.path.insert(0,r'C:/Users/rhche/Documents/Codex/2026-10-24/new-chat/work/rlcd-asr/serial-deps')
import serial
s=serial.Serial(port=None,baudrate=115200,timeout=.12);s.dtr=True;s.rts=False;s.port='COM19';s.open()
lines=[];cases=[]
def read(seconds):
 data=b'';end=time.monotonic()+seconds
 while time.monotonic()<end:data+=s.read(8192)
 text=data.decode('utf-8','replace')
 for line in text.splitlines():
  if line.startswith(('[GPT_','[UI_INFO]','[WEATHER_','[STATUS]','[READY]','[SLEEP','[MIC_TEST]')) or 'Shutdown not acknowledged' in line or 'WireGuard cleanup' in line or 'Guru Meditation' in line or 'assert failed' in line:
   print(line,flush=True);lines.append(line)
 assert 'Guru Meditation' not in text and 'assert failed' not in text,'Firmware crash'
 return text
def command(value,seconds=.4):s.write((value+'\n').encode());s.flush();return read(seconds)
def ui(app):
 data=command('ui-info');assert re.search(r'\[UI_INFO\].* app='+str(app)+r'\b',data),'Page did not switch to '+str(app)
def off():
 end=time.monotonic()+65;data=''
 while time.monotonic()<end:
  data=command('gpt-info',.6)
  if 'state=0 active=0' in data:return
 raise AssertionError('VPN resources not released')
def idle():
 end=time.monotonic()+35
 while time.monotonic()<end:
  data=command('status',.6)
  if re.search(r'mode=(1|4|5)\b',data):return
 raise AssertionError('UI stayed busy')
try:
 read(1);idle();command('transcribe');ui(1)
 for label,hold in [('immediate_exit',.2),('three_second_exit',3),('ready_exit',None)]:
  off();idle();command('gpt');ui(3)
  if hold is None:
   end=time.monotonic()+85
   while time.monotonic()<end:
    data=command('gpt-info',.6)
    if 'state=2 active=1' in data:break
    if 'state=3 ' in data:raise AssertionError('VPN failed before readiness')
   else:raise AssertionError('VPN never ready')
   command('gpt-net-test',.2);end=time.monotonic()+35;data=''
   while time.monotonic()<end and '[GPT_NET_TEST]' not in data:data+=read(.5)
   assert '[GPT_NET_TEST] body_received=1' in data,'Native gateway upload regression'
   idle()
  else:read(hold)
  started=time.monotonic();command('weather');ui(4);command('tab-next');ui(5);command('tab-next');ui(1)
  off();elapsed=round((time.monotonic()-started)*1000);cases.append({'case':label,'released_ms':elapsed,'non_vpn_navigation':'4->5->1 passed'})
  print('[HOST] case='+label+' pass=1 released_ms='+str(elapsed),flush=True)
 # Send leave/re-enter in one serial batch: exercise the real fast-switch race.
 off();idle();command('gpt');ui(3);end=time.monotonic()+85
 while time.monotonic()<end:
  data=command('gpt-info',.5)
  if 'state=2 active=1' in data:break
 else:raise AssertionError('VPN not ready for re-entry test')
 reentryStart=len(lines);s.write(b'weather\ngpt\n');s.flush();data=read(.3);ui(3);end=time.monotonic()+85
 while time.monotonic()<end:
  data+=command('gpt-info',.5)
  if 'state=2 active=1' in data:break
 else:raise AssertionError('Fast re-entry did not reconnect')
 assert any('[GPT_VPN] stopped=1' in line for line in lines[reentryStart:]),'Prior session was not fully cleaned before re-entry'
 command('weather');ui(4);command('tab-next');ui(5);command('tab-next');ui(1);off()
 cases.append({'case':'immediate_leave_reenter','result':'previous session cleaned; GPT reconnected; non-VPN navigation passed'})
 print('[HOST] case=immediate_leave_reenter pass=1',flush=True)
 command('weather');ui(4);command('weather-info');command('gpt-info');command('status')
finally:
 s.close();p=Path('VPN_EXIT_VALIDATION.json');j=json.loads(p.read_text(encoding='utf-8'));j['cases']=cases;j['telemetry']=lines;j['after_status']='passed' if len(cases)==4 else 'incomplete';p.write_text(json.dumps(j,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
