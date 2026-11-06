import sys,json,io,shlex,time,urllib.request
from pathlib import Path
p=Path(__file__).resolve().parent
deps=Path(r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-gpt\host-deps')
sys.path.insert(0,str(deps))
import paramiko
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
c=json.loads((Path.home()/'Desktop'/'RLCD-GPT-本地配置.json').read_text(encoding='utf-8-sig'))
if not c.get('ssh_password'):raise SystemExit('请先在桌面本地配置中填写 ssh_password')
if not c.get('openai_api_key'):raise SystemExit('请先在桌面本地配置中填写 openai_api_key')
client=paramiko.SSHClient();client.load_host_keys(str(Path.home()/'.ssh'/'known_hosts'))
try:
 client.connect(c['raspberry_pi_host'],username=c['ssh_username'],password=c['ssh_password'],look_for_keys=False,allow_agent=False,timeout=12)
 def run(command,sudo=False):
  stdin,stdout,stderr=client.exec_command(('sudo -S -p "" ' if sudo else '')+command,timeout=40)
  if sudo:stdin.write(c['ssh_password']+'\n');stdin.flush()
  stdin.channel.shutdown_write();output=stdout.read().decode('utf-8','replace');error=stderr.read().decode('utf-8','replace')
  code=stdout.channel.recv_exit_status()
  if code:raise RuntimeError('Remote command failed, code '+str(code))
  return output
 home=run('printf %s "$HOME"').strip();root=home+'/rlcd-gpt-gateway'
 run('mkdir -p '+shlex.quote(root)+' && chmod 700 '+shlex.quote(root))
 run('if [ -f '+shlex.quote(root+'/gateway.py')+' ]; then cp -p '+shlex.quote(root+'/gateway.py')+' '+shlex.quote(root+'/gateway.py.previous')+'; fi')
 sftp=client.open_sftp()
 for name in ['gateway.py','test_gateway.py']:
  sftp.put(str(p/'gateway'/name),root+'/'+name);sftp.chmod(root+'/'+name,0o600)
 private={'openai_api_key':c['openai_api_key'],'gateway_token':c['gateway_token'],
          'gpt_model':c.get('gpt_model','gpt-6.1-sol'),'transcription_model':c.get('transcription_model','gpt-4o-mini-transcribe'),
          'listen_address':c['raspberry_pi_host'],'listen_port':8765}
 with sftp.open(root+'/gateway-config.json','w') as f:f.write(json.dumps(private))
 sftp.chmod(root+'/gateway-config.json',0o600)
 service=(p/'gateway'/'rlcd-gpt-gateway.service').read_text().replace('User=pi','User='+c['ssh_username']).replace('/home/pi/rlcd-gpt-gateway',root)
 with sftp.open(root+'/rlcd-gpt-gateway.service','w') as f:f.write(service)
 sftp.chmod(root+'/rlcd-gpt-gateway.service',0o600);sftp.close()
 print('SSH connected; gateway and private configuration uploaded.',flush=True)
 print(run('cd '+shlex.quote(root)+' && python3 -m unittest -q test_gateway').strip(),flush=True)
 run('install -m 644 '+shlex.quote(root+'/rlcd-gpt-gateway.service')+' /etc/systemd/system/rlcd-gpt-gateway.service',True)
 run('systemctl daemon-reload',True);run('systemctl enable --now rlcd-gpt-gateway.service',True);run('systemctl restart rlcd-gpt-gateway.service',True)
 print('System service installed and started.',flush=True)
 time.sleep(2)
 opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
 url='http://'+c['raspberry_pi_host']+':8765'
 headers={'Authorization':'Bearer '+c['gateway_token'],'Content-Type':'application/json'}
 with opener.open(urllib.request.Request(url+'/v1/health',headers=headers),timeout=15) as response:
  health=json.load(response);assert health.get('egress')=='raspberry-pi'
 print('Tailnet health request passed.',flush=True)
 with opener.open(urllib.request.Request(url+'/v1/ask-text',data=json.dumps({'text':'请只回答：家庭网关连接成功'}).encode(),headers=headers),timeout=80) as response:
  result=json.load(response);assert result.get('ok') and result.get('answer')
 assert result.get('model','').startswith(private['gpt_model']),'Returned model does not match requested model'
 print('Real GPT request passed via Raspberry Pi; duration_ms='+str(result.get('gateway_ms')),flush=True)
 (p/'GATEWAY_VALIDATION.json').write_text(json.dumps({'pi':c['raspberry_pi_host'],'service_active':True,'tailnet_health':True,'real_gpt_request':True,'requested_model':private['gpt_model'],'returned_model':result.get('model'),'gateway_ms':result.get('gateway_ms')},indent=2),encoding='utf-8')
except Exception as exc:
 # Error type is sufficient; do not echo remote output or SDK exception bodies.
 print('DEPLOY_FAILED '+type(exc).__name__,flush=True);sys.exit(1)
finally:client.close()
