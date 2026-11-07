"""Verify GPT-6.1 access from the Pi before changing the running model."""
from pathlib import Path
import sys,json,shlex
sys.path.insert(0,r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-gpt\host-deps')
import paramiko
p=Path(__file__).parent
c=json.loads((Path.home()/'Desktop'/'RLCD-GPT-本地配置.json').read_text(encoding='utf-8-sig'))
s=paramiko.SSHClient();s.load_host_keys(str(Path.home()/'.ssh'/'known_hosts'))
source='''import json,time
from gateway import OpenAIBackend
c=json.load(open('gateway-config.json'))
b=OpenAIBackend(c)
request={'model':'gpt-6.1-sol','store':False,'reasoning':{'effort':'low'},'max_output_tokens':4096,'input':'请只用中文回答：连接成功。'}
at=time.monotonic()
r=b.call('responses',json.dumps(request).encode(),'application/json')
text=''.join(x.get('text','') for i in r.get('output',[]) if i.get('type')=='message' for x in i.get('content',[]) if x.get('type')=='output_text')
assert r.get('model','').startswith('gpt-6.1-sol') and text
print(json.dumps({'requested_model':'gpt-6.1-sol','returned_model':r['model'],'status':r.get('status'),'answer':text,'duration_ms':int((time.monotonic()-at)*1000),'usage':r.get('usage')}))
'''
try:
 s.connect(c['raspberry_pi_host'],username=c['ssh_username'],password=c['ssh_password'],look_for_keys=False,allow_agent=False,timeout=12)
 _,out,err=s.exec_command('cd /home/pi/rlcd-gpt-gateway && python3 -c '+shlex.quote(source),timeout=80)
 data=out.read();code=out.channel.recv_exit_status()
 if code:raise RuntimeError('Model preflight failed')
 result=json.loads(data)
 (p/'GPT61_PREFLIGHT_VALIDATION.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps({k:result[k] for k in ['requested_model','returned_model','status','duration_ms']},ensure_ascii=True))
except Exception as e:print('MODEL_PREFLIGHT_FAILED',type(e).__name__);sys.exit(1)
finally:s.close()
