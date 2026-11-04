"""Run locally in a terminal: python configure.py. Secrets are never echoed."""
import getpass,json,re
from pathlib import Path

root=Path(__file__).resolve().parent
print('Waveshare ESP32-S3-RLCD-4.2 configuration')
ssid=input('2.4 GHz Wi-Fi name: ').strip()
password=getpass.getpass('Wi-Fi password (hidden): ')
key=getpass.getpass('Bailian API Key (hidden): ').strip()
if not ssid or not key:raise SystemExit('Wi-Fi name and API Key cannot be empty.')
if any(c in key for c in '\r\n\0'):raise SystemExit('Invalid API Key.')
region=input('API Key region: 1=Beijing, 2=Singapore [1]: ').strip() or '1'
if region not in ('1','2'):raise SystemExit('Choose 1 or 2.')
workspace=input('Workspace ID (optional, Enter to use legacy host): ').strip()
if workspace and not re.fullmatch(r'[A-Za-z0-9_-]+',workspace):raise SystemExit('Invalid Workspace ID.')
if workspace:
 host=f'{workspace}.cn-beijing.maas.aliyuncs.com' if region=='1' else f'{workspace}.ap-southeast-1.maas.aliyuncs.com'
else:host='dashscope.aliyuncs.com' if region=='1' else 'dashscope-intl.aliyuncs.com'
fields=dict(WIFI_SSID=ssid,WIFI_PASSWORD=password,DASHSCOPE_API_KEY=key,ASR_ENDPOINT=f'https://{host}/compatible-mode/v1/chat/completions',ASR_MODEL='qwen3-asr-flash')
content='#pragma once\n// Local secrets: do not share this file or compiled firmware.\n'
content+='\n'.join(f'static const char {k}[] = {json.dumps(v,ensure_ascii=False)};' for k,v in fields.items())+'\n'
(root/'rlcd_qwen_asr'/'LocalConfig.h').write_text(content,encoding='utf-8')
print('Saved LocalConfig.h. Secrets were not displayed. Rebuild before flashing.')
