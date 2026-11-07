"""Collect bounded speaker diagnostics without printing news text or signed URLs."""
from pathlib import Path
import sys,time,re,json
sys.path.insert(0,r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-asr\serial-deps')
import serial
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
s=serial.Serial(port=None,baudrate=115200,timeout=.2);s.dtr=True;s.rts=False;s.port='COM19';s.open()
all_lines=[];tail=''
def read(seconds):
 global tail
 end=time.monotonic()+seconds;result=''
 while time.monotonic()<end:
  raw=s.read(4096)
  if raw:
   text=raw.decode('utf-8','replace');result+=text;tail+=text
   while '\n' in tail:
    line,tail=tail.split('\n',1);line=line.rstrip('\r')
    if line.startswith(('[NEWS_SPEECH','[SPEAKER]','[STATUS]','[NEWS_INFO]','[EXT_KEY','[MIC_TEST]','[SLEEP_INFO]','[SD_INFO]')) or any(x in line for x in ['Guru Meditation','assert failed','ES8311','i2s_channel','audio_codec','ssl_client.cpp','NetworkClientSecure.cpp','HTTPClient.cpp']):
     print(line,flush=True);all_lines.append(line)
 return result
def command(cmd,seconds=2):s.write((cmd+'\n').encode());s.flush();return read(seconds)
try:
 mode=sys.argv[1] if len(sys.argv)>1 else 'info'
 read(1);command('status');command('speech-info')
 if mode=='tone':
  data=command('speaker-test',1);end=time.monotonic()+12
  while '[NEWS_SPEECH_DONE]' not in data and time.monotonic()<end:data+=read(1)
  assert '[NEWS_SPEECH_DONE] item=0 ok=1' in data,'Tone failed'
 elif mode=='first':
  command('news');command('news-info');data=command('speech-next',1);end=time.monotonic()+110
  while '[NEWS_SPEECH_DONE]' not in data and time.monotonic()<end:data+=read(1)
  assert '[NEWS_SPEECH_DONE]' in data,'No playback completion'
 elif mode=='stream':
  command('news');command('news-info')
  for item in range(1,4):
   data=command('speech-next',.2);end=time.monotonic()+70
   while '[NEWS_SPEECH_DONE]' not in data and time.monotonic()<end:data+=read(.5)
   assert f'[NEWS_SPEECH_DONE] item={item} ok=1 cancelled=0' in data,'Playback failed on '+str(item)
 elif mode=='wrap':
  s.write(b'speech-next\n'*5);s.flush();data=read(.2);end=time.monotonic()+55
  while '[NEWS_SPEECH_DONE] item=1 ' not in data and time.monotonic()<end:data+=read(.5)
  sequence=[int(n) for n in re.findall(r'\[NEWS_SPEECH_REQUEST\] item=(\d+) ',data)]
  assert sequence==[7,8,9,10,1],str(sequence)
  assert '[NEWS_SPEECH_DONE] item=1 ok=1 cancelled=0' in data,'Wrap to cached item 1 failed'
 elif mode=='native5':
  command('news');command('news-info');s.write(b'speech-next\n'*5);s.flush();data=read(.2);end=time.monotonic()+70
  while '[NEWS_SPEECH_DONE] item=5' not in data and time.monotonic()<end:data+=read(.5)
  assert '[NEWS_SPEECH_DONE] item=5 ok=1 cancelled=0' in data,'Native streaming item 5 failed'
 elif mode=='native':
  command('news');command('news-info');s.write(b'speech-next\n'*3);s.flush();data=read(.2);end=time.monotonic()+65
  while '[NEWS_SPEECH_DONE] item=3' not in data and time.monotonic()<end:data+=read(.5)
  assert '[NEWS_SPEECH_DONE] item=3 ok=1 cancelled=0' in data,'Native streaming playback failed'
 elif mode=='retry':
  s.write(b'speech-next\n'*10);s.flush();data=read(1);end=time.monotonic()+55
  while '[NEWS_SPEECH_DONE] item=3' not in data and time.monotonic()<end:data+=read(.5)
 elif mode=='listen':read(50)
 command('speech-info');command('status');command('sd-info')
 p=Path('NEWS_SPEECH_VALIDATION.json');saved=json.loads(p.read_text(encoding='utf-8')) if p.exists() else {}
 saved[mode]=all_lines;p.write_text(json.dumps(saved,ensure_ascii=False,indent=2),encoding='utf-8')
finally:s.close()
