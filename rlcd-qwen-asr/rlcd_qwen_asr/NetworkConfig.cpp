#include "NetworkConfig.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_random.h>
namespace NetworkConfig {
static WebServer server(80);
static DNSServer dns;
static String ssid,pass,name,password,msg,candidate,candidatePass;
static bool portal=false,pending=false,trying=false,stored=false;
static uint32_t disconnected=0,queued=0,started=0,finished=0;
static const char html[] PROGMEM=R"HTML(<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>RLCD Wi-Fi 设置</title><style>body{font:18px sans-serif;max-width:480px;margin:30px auto;padding:20px}input,select,button{box-sizing:border-box;width:100%;padding:12px;margin:8px 0;font-size:18px}p{line-height:1.6}</style><h2>RLCD 更换 Wi-Fi</h2><p>手机先连接屏幕上显示的 RLCD 热点。若提示没有互联网，请选择保持连接。设备仅支持 2.4GHz Wi-Fi。</p><button onclick="scan()">扫描附近 Wi-Fi</button><select id="nets" onchange="s.value=this.value"><option>也可以手动输入名称</option></select><form id="f"><label>Wi-Fi 名称<input id="s" name="ssid" maxlength="32" required autocomplete="off"></label><label>Wi-Fi 密码<input name="password" type="password" autocomplete="new-password"></label><button>连接并保存</button></form><p id="m"></p><p>连接成功后设备会关闭热点，手机可重新连接原来的网络。失败时保留原配置。短按设备 RX 键退出设置。</p><script>async function scan(){let r=await fetch('/scan'),j=await r.json();if(j.busy){setTimeout(scan,1500);return}nets.replaceChildren();for(let n of j.networks){let o=document.createElement('option');o.value=n;o.textContent=n;nets.append(o)}}f.onsubmit=async e=>{e.preventDefault();let r=await fetch('/connect',{method:'POST',body:new URLSearchParams(new FormData(f))});let j=await r.json();m.textContent=j.message};setInterval(async()=>{try{let j=await(await fetch('/state')).json();m.textContent=j.message}catch(e){}},1500);</script></html>)HTML";
void connectSaved(){WiFi.mode(portal?WIFI_AP_STA:WIFI_STA);WiFi.setAutoReconnect(true);if(ssid.length())WiFi.begin(ssid.c_str(),pass.c_str());}
bool hasCredentials(){return ssid.length()>0;}
bool active(){return portal;}
const String& apName(){return name;}
const String& apPassword(){return password;}
const String& message(){return msg;}
void begin(const char* fallback,const char* fallbackPass){
 ssid=fallback;pass=fallbackPass;WiFi.persistent(false);
 Preferences prefs;if(prefs.begin("rlcd-net",true)){String value=prefs.getString("wifi","");prefs.end();JsonDocument j;if(!deserializeJson(j,value)){String s=j["ssid"]|"";if(s.length() && s.length()<=32){ssid=s;pass=j["password"]|"";stored=true;}}}
 connectSaved();disconnected=millis();
 server.on("/",HTTP_GET,[]{server.send_P(200,"text/html; charset=utf-8",html);});
 server.on("/state",HTTP_GET,[]{JsonDocument j;j["message"]=msg;j["connected"]=WiFi.status()==WL_CONNECTED;String body;serializeJson(j,body);server.send(200,"application/json",body);});
 server.on("/scan",HTTP_GET,[]{int n=WiFi.scanComplete();JsonDocument j;auto a=j["networks"].to<JsonArray>();if(n==WIFI_SCAN_FAILED){WiFi.scanNetworks(true);j["busy"]=true;}else if(n==WIFI_SCAN_RUNNING)j["busy"]=true;else {for(int i=0;i<n;++i){String s=WiFi.SSID(i);bool duplicate=false;for(JsonVariant v:a)if(v.as<String>()==s)duplicate=true;if(s.length()&&!duplicate)a.add(s);}WiFi.scanDelete();}String b;serializeJson(j,b);server.send(200,"application/json",b);});
 server.on("/connect",HTTP_POST,[]{
  String s=server.arg("ssid"),pw=server.arg("password");bool hex=pw.length()==64;for(unsigned i=0;hex&&i<pw.length();++i)hex=isxdigit(pw[i]);
  if(pending||trying||finished){server.send(409,"application/json",R"({"message":"正在处理，请稍候"})");return;}
  if(!s.length()||s.length()>32||!(pw.isEmpty()||(pw.length()>=8&&pw.length()<=63)||hex)){server.send(400,"application/json",R"({"message":"名称或密码长度不正确"})");return;}
  candidate=s;candidatePass=pw;pending=true;queued=millis();msg="正在连接，请等待约30秒";server.send(202,"application/json",R"({"message":"已收到，正在连接"})");
 });
 server.onNotFound([]{server.sendHeader("Location","http://192.168.4.1/",true);server.send(302,"text/plain","");});
}
void open(){if(portal)return;portal=true;finished=0;name="RLCD-Setup-"+WiFi.macAddress().substring(12);name.replace(":","");char b[12];snprintf(b,sizeof(b),"%08lu",(unsigned long)(esp_random()%100000000));password=b;msg="手机连接热点后打开网页";WiFi.mode(WIFI_AP_STA);WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));if(!WiFi.softAP(name.c_str(),password.c_str())){portal=false;msg="热点启动失败";return;}dns.start(53,"*",IPAddress(192,168,4,1));server.begin();Serial.println("[WIFI_PORTAL] active=1 address=192.168.4.1");}
void close(){if(!portal)return;bool restore=trying||pending;trying=pending=false;finished=0;candidate="";candidatePass="";dns.stop();server.stop();WiFi.softAPdisconnect(true);portal=false;WiFi.mode(WIFI_STA);if(restore)connectSaved();disconnected=millis();Serial.println("[WIFI_PORTAL] active=0");}
void tick(bool idle){
 if(!portal){if(WiFi.status()==WL_CONNECTED)disconnected=millis();else if(idle&&millis()-disconnected>45000)open();return;}
 dns.processNextRequest();server.handleClient();
 if(pending&&millis()-queued>250){pending=false;trying=true;started=millis();WiFi.setAutoReconnect(false);WiFi.disconnect(false);WiFi.begin(candidate.c_str(),candidatePass.c_str());}
 if(trying){if(WiFi.status()==WL_CONNECTED&&WiFi.SSID()==candidate){
   JsonDocument j;j["ssid"]=candidate;j["password"]=candidatePass;String value;serializeJson(j,value);Preferences prefs;bool ok=false;if(prefs.begin("rlcd-net",false)){ok=prefs.putString("wifi",value)==value.length();prefs.end();}
   trying=false;if(ok){ssid=candidate;pass=candidatePass;stored=true;msg="连接成功，已保存。热点即将关闭";finished=millis();WiFi.setAutoReconnect(true);Serial.println("[WIFI_SAVE] success=1");}else{msg="保存失败，保留原配置，请重试";connectSaved();Serial.println("[WIFI_SAVE] success=0");}candidatePass="";
  }else if(millis()-started>30000){trying=false;candidatePass="";msg="连接失败，原配置已保留，请检查密码";connectSaved();Serial.println("[WIFI_CONNECT] failed=1 original_restored=1");}}
 if(finished&&millis()-finished>10000)close();
}
void suspend(){WiFi.setAutoReconnect(false);WiFi.disconnect(false);WiFi.mode(WIFI_OFF);}
void resume(){connectSaved();disconnected=millis();}
void report(){Serial.printf("[WIFI_INFO] configured=%u stored=%u portal=%u pending=%u connected=%u ip=%s\n",hasCredentials(),stored,portal,pending||trying,WiFi.status()==WL_CONNECTED,WiFi.localIP().toString().c_str());}
}
