"""Check the existing direct Aliyun credentials and both text API contracts privately."""
from pathlib import Path
from datetime import datetime,timezone
from zoneinfo import ZoneInfo
import re,json,urllib.request,urllib.error,time,sys
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
p=Path(__file__).parent
header=(p/'firmware'/'LocalConfig.h').read_text(encoding='utf-8')
def field(name):
 m=re.search(r'\b'+name+r'\[\]\s*=\s*("(?:\\.|[^"\\])*")',header)
 if not m:raise RuntimeError('Local field missing: '+name)
 return json.loads(m[1])
endpoint=field('ASR_ENDPOINT');key=field('DASHSCOPE_API_KEY');model='qwen3.8-flash'
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def call(url,body):
 req=urllib.request.Request(url,data=json.dumps(body,ensure_ascii=False).encode(),headers={'Authorization':'Bearer '+key,'Content-Type':'application/json'})
 with opener.open(req,timeout=65) as response:return json.load(response)
def policy(name):
 s=(p/'firmware'/name).read_text(encoding='utf-8');return re.search(r'R"(\w+)\((.*?)\)\1"',s,re.S)[2]
try:
 at=time.monotonic();stamp=datetime.now(timezone.utc).isoformat()
 body={'model':model,'stream':False,'enable_thinking':False,'temperature':0,'max_tokens':1536,'response_format':{'type':'json_object'},'enable_search':True,'search_options':{'forced_search':True,'search_strategy':'turbo'},'messages':[{'role':'system','content':policy('AnswerPolicy.h')},{'role':'user','content':json.dumps({'transcript':'请联网查询美国华盛顿州Sammamish最新天气，用中文简短回答，说明资料日期。','current_utc':stamp},ensure_ascii=False)}]}
 r=call(endpoint,body);answer=json.loads(r['choices'][0]['message']['content'])
 assert r.get('model','').startswith(model) and answer.get('answer')
 result={'target_model':model,'qa_returned_model':r['model'],'qa_json_contract':True,'qa_ms':int((time.monotonic()-at)*1000)}
 print(json.dumps(result),flush=True)
 at=time.monotonic();native=endpoint.split('/compatible-mode/')[0]+'/api/v1/services/aigc/multimodal-generation/generation'
 body={'model':model,'input':{'messages':[{'role':'system','content':policy('NewsPolicy.h')},{'role':'user','content':json.dumps({'current_utc':stamp,'request':'兼容性验证：仅提供2条最近的社会民生新闻，必须有真实检索source_index，返回JSON。'},ensure_ascii=False)}]},'parameters':{'result_format':'message','enable_thinking':False,'enable_search':True,'temperature':0,'max_tokens':1500,'response_format':{'type':'json_object'},'search_options':{'forced_search':True,'search_strategy':'turbo','freshness':7,'enable_source':True}}}
 for message in body['input']['messages']:message['content']=[{'text':message['content']}]
 r=call(native,body);raw=r['output']['choices'][0]['message']['content'];raw=''.join(x.get('text','') for x in raw) if isinstance(raw,list) else raw
 content=json.loads(raw);sources=r['output']['search_info']['search_results'];indices={x['index'] for x in sources}
 assert content.get('items') and all(x['source_index'] in indices for x in content['items'])
 result.update({'news_json_contract':True,'news_source_indices_verified':True,'news_items':len(content['items']),'news_sources':len(sources),'news_ms':int((time.monotonic()-at)*1000)})
 (p/'QWEN38_PREFLIGHT_VALIDATION.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
except Exception as e:
 if isinstance(e,urllib.error.HTTPError):
  try:
   detail=json.load(e);print('ALI_ERROR',str(detail.get('code','')),str(detail.get('message','')).replace(key,'[redacted]')[:400])
  except Exception:pass
 print('QWEN_PREFLIGHT_FAILED',type(e).__name__,getattr(e,'code',''));sys.exit(1)
