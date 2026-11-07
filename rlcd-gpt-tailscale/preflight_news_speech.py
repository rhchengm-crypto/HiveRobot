from pathlib import Path
import re,json,urllib.request,urllib.error,wave,io
p=Path.cwd();s=(p/'firmware/LocalConfig.h').read_text(encoding='utf-8-sig')
def value(name):return json.loads(re.search(name+r'\[\]\s*=\s*("(?:[^"\\]|\\.)*")',s)[1])
endpoint=value('ASR_ENDPOINT');host=endpoint.split('/')[2]
body={'model':'qwen3-tts-flash','input':{'text':'新闻播报测试，喇叭已准备好。','voice':'Cherry','language_type':'Chinese'}}
req=urllib.request.Request('https://'+host+'/api/v1/services/aigc/multimodal-generation/generation',data=json.dumps(body).encode(),headers={'Authorization':'Bearer '+value('DASHSCOPE_API_KEY'),'Content-Type':'application/json'})
try:
 with urllib.request.urlopen(req,timeout=60) as r:d=json.load(r)
 url=d['output']['audio']['url'];url='https:'+url[5:] if url.startswith('http:') else url
 with urllib.request.urlopen(url,timeout=30) as r:audio=r.read(2000000)
 with wave.open(io.BytesIO(audio)) as w:info={'rate':w.getframerate(),'channels':w.getnchannels(),'bits':w.getsampwidth()*8,'frames':w.getnframes()}
 (p/'private-backup/speaker-preflight.wav').write_bytes(audio)
 print(json.dumps({'tts_http':200,'model':body['model'],'wav':info,'bytes':len(audio)}))
except urllib.error.HTTPError as e:
 try:err=json.loads(e.read());print(json.dumps({'http':e.code,'code':err.get('code')}))
 except Exception:print('TTS HTTP',e.code)
 raise SystemExit(1)
