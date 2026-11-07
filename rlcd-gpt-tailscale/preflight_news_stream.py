from pathlib import Path
import re,json,urllib.request,time,base64
s=(Path('firmware/LocalConfig.h')).read_text(encoding='utf-8-sig')
def val(n):return json.loads(re.search(n+r'\[\]\s*=\s*("(?:[^"\\]|\\.)*")',s)[1])
host=val('ASR_ENDPOINT').split('/')[2]
req=urllib.request.Request('https://'+host+'/api/v1/services/aigc/multimodal-generation/generation',data=json.dumps({'model':'qwen3-tts-flash','input':{'text':'这是一条新闻播报测试。现在改成边生成边播放，减少按键后的等待时间。','voice':'Cherry','language_type':'Chinese'}}).encode(),headers={'Authorization':'Bearer '+val('DASHSCOPE_API_KEY'),'Content-Type':'application/json','X-DashScope-SSE':'enable'})
at=time.monotonic();first=None;chunks=0;size=0;finished=False;riff_chunks=0;audio_bytes=bytearray()
with urllib.request.urlopen(req,timeout=60) as r:
 for line in r:
  if not line.startswith(b'data:'):continue
  d=json.loads(line[5:]);audio=d.get('output',{}).get('audio',{}).get('data')
  if audio:
   raw=base64.b64decode(audio);size+=len(raw);chunks+=1;riff_chunks+=raw[:4]==b"RIFF";audio_bytes.extend(raw)
   if first is None:first=time.monotonic()-at;print(json.dumps({'first_audio_seconds':round(first,3),'first_chunk_pcm_bytes':len(raw),'riff':raw[:4]==b'RIFF'}),flush=True)
  if d.get('output',{}).get('finish_reason')=='stop':finished=True
Path('private-backup/news-stream-preflight.wav').write_bytes(audio_bytes)
print(json.dumps({'riff_chunks':riff_chunks,'chunks':chunks,'pcm_bytes':size,'finished':finished,'total_seconds':round(time.monotonic()-at,3)}))
