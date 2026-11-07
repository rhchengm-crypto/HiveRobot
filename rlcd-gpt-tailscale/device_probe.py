"""Serial diagnostics; never record voice automatically during lifecycle tests."""
from pathlib import Path
import sys,time,json,re
sys.path.insert(0,r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-asr\serial-deps')
import serial
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
p=Path(__file__).resolve().parent
private=json.loads((Path.home()/'Desktop'/'RLCD-GPT-本地配置.json').read_text(encoding='utf-8-sig'))
secrets=[private.get(k,'') for k in ['ssh_password','openai_api_key','tailscale_auth_key','gateway_token']]
def redact(text):
 for secret in secrets:
  if secret:text=text.replace(secret,'[redacted]')
 return text
def connect():
 s=serial.Serial(port=None,baudrate=115200,timeout=.2);s.dtr=True;s.rts=False;s.port='COM19';s.open();return s
s=connect();all_data=''
def read(seconds):
 global all_data
 end=time.monotonic()+seconds;data=''
 while time.monotonic()<end:
  raw=s.read(4096)
  if raw:
   text=redact(raw.decode('utf-8','replace'));data+=text;all_data+=text
   with (p/'device-gpt.log').open('a',encoding='utf-8') as f:f.write(text)
   print(text,end='',flush=True)
 return data
def command(text,seconds=2):s.write((text+'\n').encode());s.flush();return read(seconds)
def wait_ready(seconds=75):
 end=time.monotonic()+seconds;data=''
 while time.monotonic()<end:
  data+=read(1)
  if '[GPT_VPN] ready=1' in data:return data
  if '[GPT_VPN] stopped=1' in data or 'Guru Meditation' in data or 'assert failed' in data:break
 raise RuntimeError('VPN readiness not reached; inspect device-gpt.log')
try:
 mode=sys.argv[1] if len(sys.argv)>1 else 'connect'
 if mode=='listen':read(50)
 elif mode=='news-update':
  read(8);data=command('news-refresh',.5);deadline=time.monotonic()+150
  while time.monotonic()<deadline and '[NEWS_TASK]' not in data:data+=read(1)
  assert '[NEWS_TASK] ok=1' in data,'News update failed'
  command('news-info',3);command('sd-info');command('tab-next',3);command('ui-info')
 elif mode=='pages':
  read(1);state=command('status');assert 'mode=1' in state,'Expected Ready with restored answer'
  first=command('page',3);m=re.search(r'\[PAGE\] current=(\d+) total=(\d+)',first);assert m,'Visible answer not pageable'
  current,total=map(int,m.groups());assert total>1,'Need a multi-page saved answer'
  for i in range(total):
   data=command('page',3);m=re.search(r'\[PAGE\] current=(\d+) total=(\d+)',data);assert m and int(m[1])==current%total+1,'Page sequence or wrap failed';current=int(m[1])
  while current!=1:
   data=command('page',3);m=re.search(r'\[PAGE\] current=(\d+) total=(\d+)',data);assert m;current=int(m[1])
  print('[HOST] restored_ready_answer_pagination=pass total='+str(total),flush=True)
 elif mode=='net':
  read(1)
  for i in range(3):
   data=command('gpt-net-test',.2);deadline=time.monotonic()+40
   while time.monotonic()<deadline and '[GPT_NET_TEST]' not in data:data+=read(1)
   assert '[GPT_NET_TEST] body_received=1' in data,'upload probe failed'
   print('[HOST] upload_probe='+str(i+1)+' pass=1',flush=True)
  command('gpt-info');command('sd-info')
 elif mode=='sd':
  read(6);data=command('sd-info');assert 'ready=1' in data,'SD not ready'
  command('sd-test',3);data=command('sd-info');assert 'records=1 dropped=0' in data,'SD write failed'
  command('status');command('ui-info')
 elif mode=='connect':
  read(8);command('status');command('ui-info');info=command('gpt-info');command('gpt',.3)
  if 'state=2 active=1' not in info:wait_ready()
  command('gpt-info');command('ui-info')
 elif mode=='cycles':
  read(2)
  for i in range(3):
   command('tab-next',6);data=command('gpt-info');assert 'state=0 active=0' in data,'VPN not fully released'
   command('tab-prev',.3);wait_ready();command('gpt-info')
   print('[HOST] lifecycle_cycle='+str(i+1)+' pass=1',flush=True)
 elif mode=='sleep':
  read(1)
  try:command('sleep-test',10)
  except serial.SerialException:print('[HOST] expected USB sleep disconnect',flush=True)
  s.close();deadline=time.monotonic()+20
  while True:
   try:s=connect();break
   except serial.SerialException:
    if time.monotonic()>deadline:raise
    time.sleep(.5)
  wait_ready();command('sleep-info');command('gpt-info')
 elif mode=='baseline':
  read(1);command('tab-next',7);command('gpt-info');command('weather-info');command('news-info');command('transcribe');command('ui-info');command('status')
finally:s.close()
