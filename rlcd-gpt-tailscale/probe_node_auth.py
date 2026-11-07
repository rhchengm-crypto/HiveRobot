"""Real node-only resume diagnostics. Never revoke keys, reset NVS, or upload audio."""
from pathlib import Path
import sys,time,re,json
sys.path.insert(0,r"C:/Users/rhche/Documents/Codex/2026-10-24/new-chat/work/serial-deps")
import serial
sys.stdout.reconfigure(encoding="utf-8",errors="replace")
mode=sys.argv[1] if len(sys.argv)>1 else "initial"
s=serial.Serial(port=None,baudrate=115200,timeout=.1);s.dtr=True;s.rts=False;s.port="COM19";s.open();tail="";lines=[];cases=[]
def read(t):
 global tail
 end=time.monotonic()+t;out=""
 while time.monotonic()<end:
  text=s.read(8192).decode("utf-8","replace");tail+=text;out+=text
  while "\n" in tail:
   line,tail=tail.split("\n",1);line=line.rstrip("\r")
   if "[ML_AUTH]" in line:
    line=line[line.index("[ML_AUTH]"):];print(line,flush=True);lines.append(line)
    assert "auth_key_sent=1" not in line,"Unexpected enrollment-key transmission for existing identity"
   elif line.startswith(("[GPT_","[STATUS]","[UI_INFO]","[BUTTON_INFO]","[SLEEP_INFO]")):
    print(line,flush=True);lines.append(line)
 assert "Guru Meditation" not in out and "assert failed" not in out,"Firmware crash"
 return out
def cmd(c,t=.7):s.write((c+"\n").encode());s.flush();return read(t)
def info():
 for _ in range(4):
  d=cmd("gpt-info",1);m=re.search(r"state=(\d+) active=(\d+)",d)
  if m:return tuple(map(int,m.groups()))
 raise AssertionError("No GPT state")
def idle():
 end=time.monotonic()+120
 while time.monotonic()<end:
  if re.search(r"mode=(1|4|5)\b",cmd("status",1)):return
 raise AssertionError("UI stayed busy")
def ui(app):
 end=time.monotonic()+15
 while time.monotonic()<end:
  if "app="+str(app) in cmd("ui-info",1):return
 raise AssertionError("Page did not reach "+str(app))
def off():
 end=time.monotonic()+60
 while time.monotonic()<end:
  if info()[1]==0:return
 raise AssertionError("VPN cleanup timed out")
def ready():
 end=time.monotonic()+100
 while time.monotonic()<end:
  state,active=info()
  if state==2:return
  if state==3 and not active:raise AssertionError("Node-only login failed")
 raise AssertionError("Gateway readiness timeout")
try:
 read(2);idle();cmd("transcribe");off()
 for label in (["legacy_migration","page_reentry"] if mode=="initial" else ["after_chip_restart"]):
  start=len(lines);cmd("gpt",.2);ready();segment=lines[start:]
  assert any("register auth_key_sent=0" in line for line in segment),"Missing node-only RegisterRequest diagnostic"
  if label!="legacy_migration":assert any("auth_key_sent=0 enrolled=1" in line for line in segment),"Enrollment was not restored from NVS"
  cases.append({"case":label,"node_only":True,"gateway_ready":True})
  cmd("weather");ui(4);off();idle();cmd("tab-next");ui(5);idle();cmd("tab-next");ui(1);idle()
 cmd("gpt",.2);ready();cmd("keys");cmd("status");cmd("ui-info")
finally:
 s.close();p=Path("NODE_AUTH_VALIDATION.json");j=json.loads(p.read_text(encoding="utf-8")) if p.exists() else {"date":"2026-11-07"};j[mode]={"cases":cases,"telemetry":lines};p.write_text(json.dumps(j,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
