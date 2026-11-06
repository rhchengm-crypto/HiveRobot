"""Real gateway web-search test; prints only the public weather answer, never secrets."""
import json,urllib.request,urllib.error,re,sys
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
p=Path(__file__).parent
c=json.loads((Path.home()/'Desktop'/'RLCD-GPT-本地配置.json').read_text(encoding='utf-8-sig'))
question='请联网查询美国华盛顿州Sammamish今天的天气，明确当地年月日、摄氏气温及降雨情况；区分当前观测与预报，并引用可靠来源。'
req=urllib.request.Request('http://'+c['raspberry_pi_host']+':8765/v1/ask-text',data=json.dumps({'text':question},ensure_ascii=False).encode(),headers={'Authorization':'Bearer '+c['gateway_token'],'Content-Type':'application/json'})
try:
 with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req,timeout=145) as response:r=json.load(response)
 assert r.get('ok') and r.get('answer') and r.get('web_search_used') is True,'No successful real web search'
 assert r.get('model','').startswith(c['gpt_model']),'Unexpected model'
 assert 'https://' in r['answer'] or 'http://' in r['answer'],'Missing visible source URL'
 result={'real_web_search':True,'requested_model':c['gpt_model'],'returned_model':r['model'],'gateway_ms':r['gateway_ms'],'answer_bytes':len(r['answer'].encode()),'question':question,'answer':r['answer']}
 (p/'GATEWAY_SEARCH_VALIDATION.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps(result,ensure_ascii=False))
except Exception as e:
 print('SEARCH_PROBE_FAILED',type(e).__name__,getattr(e,'code',''));sys.exit(1)
