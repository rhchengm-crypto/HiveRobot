"""Read desktop secrets privately; write only VPN/gateway credentials to firmware."""
from pathlib import Path
import json,ipaddress
p=Path(__file__).resolve().parent
config=json.loads((Path.home()/'Desktop'/'RLCD-GPT-本地配置.json').read_text(encoding='utf-8-sig'))
host=config['raspberry_pi_host']
if ipaddress.ip_address(host) not in ipaddress.ip_network('100.64.0.0/10'):raise SystemExit('Pi address must be a Tailscale IP')
if not config.get('tailscale_auth_key','').startswith('tskey-auth-'):raise SystemExit('Use a Tailscale device auth key beginning with tskey-auth-; key was not printed')
names={'GPT_GATEWAY_HOST':host,'GPT_GATEWAY_TOKEN':config['gateway_token'],'GPT_TAILSCALE_AUTH_KEY':config.get('tailscale_auth_key','')}
header='#pragma once\n// PRIVATE: generated locally, never publish. OpenAI key stays on the Pi.\n'
for name,value in names.items():header+='static constexpr char '+name+'[]='+json.dumps(value,ensure_ascii=True)+';\n'
header+='static constexpr unsigned GPT_GATEWAY_PORT=8765;\n'
(p/'firmware'/'GatewayConfig.h').write_text(header,encoding='utf-8')
print('Firmware gateway configuration generated; no credentials displayed.')
