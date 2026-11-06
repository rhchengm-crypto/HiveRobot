import json,sys,urllib.request
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
p=Path(__file__).resolve().parent
c=json.loads((Path.home()/'Desktop'/'RLCD-GPT-本地配置.json').read_text(encoding='utf-8-sig'))
audio=Path(r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-asr\asr31-official-sample.wav').read_bytes()
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
req=urllib.request.Request('http://'+c['raspberry_pi_host']+':8765/v1/ask',data=audio,headers={'Authorization':'Bearer '+c['gateway_token'],'Content-Type':'audio/wav'})
try:
 with opener.open(req,timeout=145) as response:result=json.load(response)
 assert result.get('ok') and result.get('answer') and result.get('egress')=='raspberry-pi'
 out={'official_sample_audio':True,'real_openai_asr_and_gpt':True,'gateway_ms':result.get('gateway_ms'),'answer_bytes':len(result['answer'].encode()),'egress':result['egress']}
 (p/'GATEWAY_AUDIO_VALIDATION.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(out))
except Exception as e:print('PROBE_FAILED',type(e).__name__);sys.exit(1)
