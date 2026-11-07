#include "GptGateway.h"
#include "GatewayConfig.h"
#include "History.h"
#include "GptOpus.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <atomic>
#include <time.h>
#include <esp_heap_caps.h>
#include "microlink.h"

namespace GptGateway {
namespace {
std::atomic<State> current{State::Off};
std::atomic<bool> wanted{false},worker{false},sending{false},cleaning{false},disconnectRequested{false};
std::atomic<unsigned> cleanupRetries{0};
microlink_t* vpn=nullptr;
portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
char detail[160]={};
char modelName[64]={};
void rememberModel(const char* value){
 if(!value||!value[0]||strlen(value)>=sizeof(modelName))return;
 for(const char* p=value;*p;++p)if(!isalnum((unsigned char)*p)&&*p!='-'&&*p!='.'&&*p!='_'&&*p!=':')return;
 portENTER_CRITICAL(&lock);strlcpy(modelName,value,sizeof(modelName));portEXIT_CRITICAL(&lock);
}
void failure(const char* value){portENTER_CRITICAL(&lock);strlcpy(detail,value,sizeof(detail));portEXIT_CRITICAL(&lock);}
bool request(const char* path,const uint8_t* body,size_t length,JsonDocument& reply,String& error,uint32_t timeout,const char* contentType="audio/wav"){
 if(!vpn||!microlink_is_connected(vpn)||!microlink_get_vpn_ip(vpn)){error="家庭VPN尚未连接";return false;}
 auto* socket=microlink_tcp_connect(vpn,microlink_parse_ip(GPT_GATEWAY_HOST),GPT_GATEWAY_PORT,10000);
 if(!socket){error="无法连接树莓派网关";return false;}
 bool ok=false;
 do {
  String headers=String(length?"POST ":"GET ")+path+" HTTP/1.1\r\nHost: "+GPT_GATEWAY_HOST+"\r\nAuthorization: Bearer "+GPT_GATEWAY_TOKEN+"\r\nConnection: close\r\n";
  if(length)headers+=String("Content-Type: ")+contentType+"\r\nContent-Length: "+String(length)+"\r\n";
  headers+="\r\n";
  if(microlink_tcp_send(socket,headers.c_str(),headers.length())!=ESP_OK){error="网关请求发送失败";break;}
  const uint32_t started=millis();bool sent=true;
  for(size_t offset=0;offset<length;){
   const size_t n=min(size_t(1024),length-offset);
   if(millis()-started>45000||microlink_tcp_send(socket,body+offset,n)!=ESP_OK){Serial.printf("[GPT_UPLOAD] failed_at=%u total=%u ms=%lu\n",unsigned(offset),unsigned(length),(unsigned long)(millis()-started));sent=false;break;}offset+=n;
  }
  if(!sent){error="录音上传失败，请重试";break;}
  String received;received.reserve(8192);char buffer[512];int split=-1;unsigned expected=0;int httpCode=0;
  const uint32_t wait=millis();
  while(millis()-wait<timeout){
   int n=microlink_tcp_recv(socket,buffer,sizeof(buffer),500);
   if(n<0)break;
   if(!n){if((!wanted||disconnectRequested)&&length==0)break;continue;}
   received.concat(buffer,n);
   if(received.length()>10000){error="网关响应过大";break;}
   if(split<0){
    split=received.indexOf("\r\n\r\n");
    if(split>=0){
     httpCode=received.substring(9,12).toInt();
     String h=received.substring(0,split);h.toLowerCase();int at=h.indexOf("\r\ncontent-length:");
     if(at<0){error="网关协议不匹配";break;}expected=h.substring(at+17).toInt();
     if(!expected||expected>8000){error="网关响应长度错误";break;}
    }else if(received.length()>1500){error="网关响应头错误";break;}
   }
   if(split>=0&&received.length()>=unsigned(split+4)+expected){
    if(deserializeJson(reply,received.c_str()+split+4,expected)){error="网关响应解析失败";break;}
    if(httpCode!=200||!(reply["ok"]|false)){error=reply["error"]|"家庭网关请求失败";break;}
    ok=true;break;
   }
  }
  if(!ok&&error.isEmpty())error="等待家庭网关超时，请重试";
 }while(false);
 microlink_tcp_close(socket);return ok;
}
void manage(void*){
 bool connected=false;String message;
 do {
  if(!GPT_TAILSCALE_AUTH_KEY[0]||!GPT_GATEWAY_TOKEN[0]){message="请先配置Tailscale和家庭网关";break;}
  if(strncmp(GPT_TAILSCALE_AUTH_KEY,"tskey-auth-",11)){message="Tailscale入网密钥格式错误";break;}
  const uint32_t networkAt=millis();
  while(wanted&&!disconnectRequested&&(WiFi.status()!=WL_CONNECTED||time(nullptr)<1700000000)&&millis()-networkAt<45000)vTaskDelay(pdMS_TO_TICKS(100));
  if(!wanted||disconnectRequested)break;
  if(WiFi.status()!=WL_CONNECTED||time(nullptr)<1700000000){message="网络或校时未就绪，按BOOT重试";break;}
  microlink_config_t config{};config.auth_key=GPT_TAILSCALE_AUTH_KEY;config.device_name="rlcd-qwen";
  config.enable_derp=true;config.enable_disco=true;config.enable_stun=true;config.max_peers=4;
  config.priority_peer_ip=microlink_parse_ip(GPT_GATEWAY_HOST);
  vpn=microlink_init(&config);
  if(!vpn||microlink_start(vpn)!=ESP_OK){message="家庭VPN初始化失败";break;}
  uint32_t start=millis();
  while(wanted&&!disconnectRequested&&!microlink_is_connected(vpn)&&millis()-start<60000&&WiFi.status()==WL_CONNECTED)vTaskDelay(pdMS_TO_TICKS(100));
  if(!wanted||disconnectRequested)break;
  if(!microlink_is_connected(vpn)){message="家庭VPN连接超时，按BOOT重试";break;}
  JsonDocument health;
  if(!request("/v1/health",nullptr,0,health,message,10000))break;
  if(!wanted||disconnectRequested)break;
  if(strcmp(health["egress"]|"","raspberry-pi")||int(health["protocol"]|0)!=1){message="家庭网关版本不匹配";break;}
  rememberModel(health["gpt_model"]|"");
  connected=true;current=State::Ready;
  Serial.printf("[GPT_VPN] ready=1 internal_free=%u largest=%u psram_free=%u\n",unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)),unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)),unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
  while(wanted&&!disconnectRequested){
   if(!sending&&WiFi.status()!=WL_CONNECTED){message="Wi-Fi断开，按BOOT重新连接";break;}
   vTaskDelay(pdMS_TO_TICKS(100));
  }
 }while(false);
 while(sending)vTaskDelay(pdMS_TO_TICKS(50));
 const bool leavingPage=disconnectRequested.load();
 if(vpn){
  cleaning=true;current=State::Stopping;cleanupRetries=0;
  // A join timeout must not abandon a live instance and permanently lock navigation.
  // Tasks may still be returning from DNS/connect. Retain ownership and retry safely.
  while(microlink_stop(vpn)!=ESP_OK){
   ++cleanupRetries;failure("家庭VPN正在退出，其他页面可继续使用");
   Serial.printf("[GPT_VPN] cleanup_retry=%u\n",cleanupRetries.load());
   vTaskDelay(pdMS_TO_TICKS(300));
  }
  microlink_destroy(vpn);vpn=nullptr;
  cleaning=false;
 }
 if(wanted&&!leavingPage){failure(message.isEmpty()?"家庭网关离线，按BOOT重试":message.c_str());current=State::Failed;}
 else current=State::Off;
 Serial.printf("[GPT_VPN] stopped=1 was_ready=%u heap=%u\n",connected,unsigned(ESP.getFreeHeap()));worker=false;vTaskDelete(nullptr);
}
}
void enter(){
 wanted=true;if(worker||vpn)return;disconnectRequested=false;current=State::Connecting;failure("");worker=true;
 if(xTaskCreatePinnedToCore(manage,"gpt-vpn",20*1024,nullptr,2,nullptr,0)!=pdPASS){worker=false;current=State::Failed;failure("家庭VPN任务内存不足");}
}
void leave(){disconnectRequested=true;wanted=false;if(worker)current=State::Stopping;}
bool stopAndWait(uint32_t timeout){leave();const uint32_t at=millis();while(worker&&millis()-at<timeout)delay(20);return !worker&&!vpn;}
bool stopping(){return cleaning||(!wanted&&worker)||(!worker&&vpn);}
bool active(){return worker||vpn;}
bool ready(){return wanted&&!disconnectRequested&&current==State::Ready&&!sending;}
State state(){return current;}
String error(){char copy[160];portENTER_CRITICAL(&lock);strlcpy(copy,detail,sizeof(copy));portEXIT_CRITICAL(&lock);return String(copy);}
String model(){char copy[64];portENTER_CRITICAL(&lock);strlcpy(copy,modelName,sizeof(copy));portEXIT_CRITICAL(&lock);return copy[0]?String(copy):String("checking...");}
bool ask(const uint8_t* wav,size_t length,String& result,String& error){
 if(current!=State::Ready||sending.exchange(true)){error="家庭网关尚未就绪";return false;}
 size_t encodedBytes=0;const uint32_t encodeAt=millis();uint8_t* encoded=GptOpus::encode(wav,length,encodedBytes);
 if(!encoded){sending=false;error="GPT录音压缩失败，请重试";return false;}
 Serial.printf("[GPT_AUDIO] wav_bytes=%u opus_bytes=%u encode_ms=%lu\n",unsigned(length),unsigned(encodedBytes),(unsigned long)(millis()-encodeAt));
 JsonDocument reply;const bool ok=request("/v1/ask",encoded,encodedBytes,reply,error,145000,"audio/ogg");free(encoded);
 if(ok){rememberModel(reply["model"]|"");result=reply["answer"]|"";History::append("gpt",result.c_str(),reply["question"]|"");Serial.printf("[GPT_REPLY] bytes=%u gateway_ms=%lu model=%s\n",unsigned(result.length()),(unsigned long)(reply["gateway_ms"]|0ul),modelName);}
 sending=false;return ok;
}
void report(){Serial.printf("[GPT_INFO] state=%u active=%u sending=%u configured=%u cleaning=%u cleanup_retries=%u\n",unsigned(current.load()),active(),sending.load(),bool(GPT_TAILSCALE_AUTH_KEY[0]&&GPT_GATEWAY_TOKEN[0]),cleaning.load(),cleanupRetries.load());}
void probeUpload(){
 if(current!=State::Ready||sending.exchange(true))return;
 auto* bytes=static_cast<uint8_t*>(heap_caps_calloc(16384,1,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
 if(bytes){JsonDocument reply;String error;const uint32_t at=millis();request("/v1/ask",bytes,16384,reply,error,30000,"application/octet-stream");Serial.printf("[GPT_NET_TEST] body_received=%u duration_ms=%lu\n",error=="需要WAV或Opus音频",(unsigned long)(millis()-at));free(bytes);}
 sending=false;
}
}
