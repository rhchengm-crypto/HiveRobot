"""Bounded real-device capture regression; logs metadata, never recognized text/audio."""
from pathlib import Path
import sys,time,json,re
sys.path.insert(0,r"C:/Users/rhche/Documents/Codex/2026-10-24/new-chat/work/serial-deps")
import serial
sys.stdout.reconfigure(encoding="utf-8",errors="replace")
lines=[];cases=[]
def connect():
 s=serial.Serial(port=None,baudrate=115200,timeout=.12);s.dtr=True;s.rts=False;s.port="COM19";s.open();return s
s=connect();tail=""
def read(seconds):
 global tail
 data="";end=time.monotonic()+seconds
 while time.monotonic()<end:
  text=s.read(8192).decode("utf-8","replace");data+=text;tail+=text
  while "\n" in tail:
   line,tail=tail.split("\n",1);line=line.rstrip("\r")
   if line.startswith(("[RT", "[STATUS]", "[GPT_INFO]", "[SLEEP_INFO]", "[SD_INFO]")):
    lines.append(line);print(line,flush=True)
 assert "Guru Meditation" not in data and "assert failed" not in data,"Firmware crashed"
 return data
def command(value,seconds=1):s.write((value+"\n").encode());s.flush();return read(seconds)
def run(label):
 command("transcribe");data=command("rt-start",12);data+=command("rt-stop",5)
 end=time.monotonic()+90
 while "[RT_DONE]" not in data and time.monotonic()<end:data+=read(.5)
 task=re.search(r"\[RT_CAPTURE_TASK\] created=1 .*internal_largest=(\d+)",data)
 done=re.search(r"\[RT_DONE\] ok=1 .*frames=(\d+).*ogg_bytes=(\d+)",data)
 assert task,"Capture task was not allocated"
 assert done and int(done[1])>=80 and int(done[2])>0,"Capture/Opus upload did not complete"
 cases.append({"case":label,"frames":int(done[1]),"ogg_bytes":int(done[2]),"internal_largest_before_capture":int(task[1]),"passed":True})
 print("[HOST] case="+label+" passed=1",flush=True)
try:
 read(3)
 end=time.monotonic()+120
 while time.monotonic()<end:
  state=command("status",1)
  if re.search(r"mode=(1|4|5)\b",state):break
 else:raise AssertionError("Startup background work did not finish")
 run("initial");run("repeat")
 command("gpt",3);command("weather",1)
 end=time.monotonic()+75
 while time.monotonic()<end:
  if "state=0 active=0" in command("gpt-info",.7):break
 else:raise AssertionError("VPN not released")
 run("after_vpn_exit")
 try:command("sleep-test",1)
 except serial.SerialException:pass
 s.close();time.sleep(5);end=time.monotonic()+30
 while True:
  try:s=connect();break
  except serial.SerialException:
   if time.monotonic()>end:raise
   time.sleep(.5)
 command("sleep-info");run("after_sleep");command("sd-info");command("status")
finally:
 s.close();Path("REALTIME_CAPTURE_VALIDATION.json").write_text(json.dumps({"date":"2026-11-07","cases":cases,"telemetry":lines},ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
