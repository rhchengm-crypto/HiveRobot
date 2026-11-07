"""Capture only TCP header summaries for the private gateway, never packet payload."""
from pathlib import Path
import sys,json
sys.path.insert(0,r'C:\Users\rhche\Documents\Codex\2026-10-24\new-chat\work\rlcd-gpt\host-deps')
import paramiko
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
c=json.loads((Path.home()/'Desktop'/'RLCD-GPT-本地配置.json').read_text(encoding='utf-8-sig'))
s=paramiko.SSHClient();s.load_host_keys(str(Path.home()/'.ssh'/'known_hosts'))
try:
 s.connect(c['raspberry_pi_host'],username=c['ssh_username'],password=c['ssh_password'],look_for_keys=False,allow_agent=False,timeout=12)
 stdin,out,err=s.exec_command('sudo -S -p "" timeout 90 tcpdump -l -n -i tailscale0 -tt -c 100 tcp port 8765',timeout=100)
 stdin.write(c['ssh_password']+'\n');stdin.flush();stdin.channel.shutdown_write()
 with (Path(__file__).parent/'transport-metadata.log').open('w',encoding='utf-8') as f:
  for line in out:
   f.write(line);f.flush();print(line.rstrip(),flush=True)
 code=out.channel.recv_exit_status();print('capture_exit=',code)
 if code not in (0,124):print(err.read().decode('utf-8','replace'))
finally:s.close()
