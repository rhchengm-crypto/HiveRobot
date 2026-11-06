from pathlib import Path
import sys,json
sys.path.insert(0,r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-gpt\host-deps')
import paramiko
c=json.loads((Path.home()/'Desktop'/'RLCD-GPT-本地配置.json').read_text(encoding='utf-8-sig'))
s=paramiko.SSHClient();s.load_host_keys(str(Path.home()/'.ssh'/'known_hosts'))
try:
 s.connect(c['raspberry_pi_host'],username=c['ssh_username'],password=c['ssh_password'],look_for_keys=False,allow_agent=False,timeout=10)
 def run(command):
  _,out,_=s.exec_command(command,timeout=15);data=out.read().decode();assert out.channel.recv_exit_status()==0;return data
 prefs=json.loads(run('tailscale debug prefs'))
 service=run('systemctl is-active rlcd-gpt-gateway').strip()
 result={'service':service,'pi_exit_node_configured':bool(prefs.get('ExitNodeID') or prefs.get('ExitNodeIP'))}
 print(json.dumps(result))
 (Path(__file__).parent/'PI_EGRESS_VALIDATION.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
except Exception as exc:print('INSPECT_FAILED',type(exc).__name__)
finally:s.close()
