"""Real hardware GPT reconnect/exit regression; no audio or model requests."""
from pathlib import Path
import sys,time,re,json
sys.path.insert(0,r"C:/Users/rhche/Documents/Codex/2026-10-24/new-chat/work/serial-deps")
import serial
sys.stdout.reconfigure(encoding="utf-8",errors="replace")
s=serial.Serial(port=None,baudrate=115200,timeout=.1);s.dtr=True;s.rts=False;s.port="COM19";s.open();tail="";lines=[];cases=[]
def read(seconds):
 global tail
 end=time.monotonic()+seconds;result=""
 while time.monotonic()<end:
  text=s.read(8192).decode("utf-8","replace");result+=text;tail+=text
  while "\n" in tail:
   line,tail=tail.split("\n",1);line=line.rstrip("\r")
   if line.startswith(("[GPT_","[STATUS]","[UI_INFO]","[BUTTON_INFO]")):
    print(line,flush=True);lines.append(line)
 assert "Guru Meditation" not in result and "assert failed" not in result,"Firmware crash"
 return result
def cmd(value,t=.5):s.write((value+"\n").encode());s.flush();return read(t)
def info():
 for retry in range(4):
  data=cmd("gpt-info",1.2);m=re.search(r"state=(\d+) active=(\d+).*attempt=(\d+)",data)
  if m:return tuple(map(int,m.groups()))
 raise AssertionError("No diagnostic response after four polls")
def ui(app):
 end=time.monotonic()+12
 while time.monotonic()<end:
  if "app="+str(app) in cmd("ui-info",1):return
 raise AssertionError("Page did not change to "+str(app))
def idle():
 end=time.monotonic()+120
 while time.monotonic()<end:
  if re.search(r"mode=(1|4|5)\b",cmd("status",1)):return
 raise AssertionError("UI busy")
def wait_off():
 end=time.monotonic()+60
 while time.monotonic()<end:
  state,active,attempt=info()
  if active==0:return
 raise AssertionError("Cleanup did not finish")
try:
 read(2);idle();cmd("transcribe");wait_off();cmd("gpt",2);initial=info()[2]
 # Two queued retries while a session is still connecting/cleaning must coalesce.
 cmd("gpt-retry",.1);cmd("gpt-retry",.1);end=time.monotonic()+65
 while time.monotonic()<end:
  state,active,attempt=info()
  if attempt>initial and state in (1,2):break
 else:raise AssertionError("Queued retry was lost")
 cases.append({"case":"retry_while_connecting_or_cleaning","initial_attempt":initial,"new_attempt":attempt,"passed":True})
 cmd("weather");ui(4);cmd("tab-next");ui(5);cmd("tab-next");ui(1);wait_off()
 cases.append({"case":"exit_and_non_vpn_navigation","passed":True})
 idle();cmd("gpt",1);end=time.monotonic()+100
 while time.monotonic()<end:
  state,active,attempt=info()
  if state in (2,3) and not (state==3 and active):break
 else:raise AssertionError("Connect did not reach ready/failure")
 cases.append({"case":"connection_outcome","state":state,"attempt":attempt,"ready":state==2})
 if state==2:
  # Existing bounded offline diagnostic restores Wi-Fi after eight seconds.
  cmd("weather-offline",1);end=time.monotonic()+50
  while time.monotonic()<end:
   state,active,attempt=info()
   if state==3 and not active:break
  else:raise AssertionError("Offline did not finish cleanup and report failure")
  end=time.monotonic()+35
  while time.monotonic()<end:
   if "wifi=1" in cmd("status",1):break
  else:raise AssertionError("Automatic Wi-Fi restore failed")
  cases.append({"case":"bounded_wifi_loss_failure","passed":True})
 if state==3:
  before=attempt;cmd("gpt-retry",.1);state,active,attempt=info();assert attempt>before and state==1,"Retry after timeout did not start"
  cases.append({"case":"retry_after_failure","passed":True,"new_attempt":attempt})
  end=time.monotonic()+100
  while time.monotonic()<end:
   state,active,attempt=info()
   if state==2:break
   if state==3 and not active:raise AssertionError("Fresh retry still failed")
  else:raise AssertionError("Retry did not reach ready")
  cases.append({"case":"ready_after_failure_retry","passed":True})
 cmd("keys");cmd("status");cmd("ui-info")
finally:
 s.close();Path("GPT_RETRY_VALIDATION.json").write_text(json.dumps({"date":"2026-11-07","cases":cases,"telemetry":lines},ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
