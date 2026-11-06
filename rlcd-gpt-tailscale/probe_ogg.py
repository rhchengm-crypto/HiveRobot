"""Validate direct Ogg forwarding with the same public reference speech fixture."""
import sys,json,urllib.request
from pathlib import Path
sys.path.insert(0,r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-gpt\host-deps')
import paramiko
p=Path(__file__).parent
c=json.loads((Path.home()/'Desktop'/'RLCD-GPT-本地配置.json').read_text(encoding='utf-8-sig'))
s=paramiko.SSHClient();s.load_host_keys(str(Path.home()/'.ssh'/'known_hosts'))
try:
 s.connect(c['raspberry_pi_host'],username=c['ssh_username'],password=c['ssh_password'],look_for_keys=False,allow_agent=False,timeout=12)
 f=s.open_sftp();remote='/home/pi/rlcd-gpt-gateway/validation-sample.wav'
 f.put(r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-asr\asr31-official-sample.wav',remote)
 _,out,err=s.exec_command('/usr/bin/ffmpeg -nostdin -hide_banner -loglevel error -i '+remote+' -ac 1 -ar 16000 -c:a libopus -b:a 24k -f ogg pipe:1',timeout=15)
 audio=out.read();assert out.channel.recv_exit_status()==0 and audio.startswith(b'OggS')
 f.remove(remote);f.close()
 req=urllib.request.Request('http://'+c['raspberry_pi_host']+':8765/v1/ask',data=audio,headers={'Authorization':'Bearer '+c['gateway_token'],'Content-Type':'audio/ogg'})
 with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req,timeout=145) as response:r=json.load(response)
 assert r.get('ok') and r.get('answer') and r.get('question')
 result={'direct_ogg_forwarding':True,'real_openai_asr_and_gpt':True,'audio_bytes':len(audio),'gateway_ms':r['gateway_ms']}
 (p/'GATEWAY_OGG_VALIDATION.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
except Exception as e:print('OGG_PROBE_FAILED',type(e).__name__);sys.exit(1)
finally:s.close()
