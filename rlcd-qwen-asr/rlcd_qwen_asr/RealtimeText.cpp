#include "RealtimeText.h"
#include "Microphone.h"
#include "Transcribe.h"
#include "LocalConfig.h"
#include "Trust.h"
#include "OggOpus.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <RlcdOpus.h>
#include <ArduinoJson.h>
#include <mbedtls/base64.h>
#include <esp_heap_caps.h>
#include <atomic>
#include <new>
#include <lwip/sockets.h>
#include <mbedtls/error.h>
namespace RealtimeText {
namespace {
class RealtimeTLS:public WiFiClientSecure {
public:
 size_t write(uint8_t data)override{return write(&data,1);}
 size_t write(const uint8_t* data,size_t bytes)override {
  timeval timeout{10,0};setSocketOption(SO_SNDTIMEO,reinterpret_cast<char*>(&timeout),sizeof(timeout));sslclient->socket_timeout=10000;
  const uint32_t at=millis();const int result=send_ssl_data(sslclient.get(),data,bytes);
  if(result<0){char message[96]={};mbedtls_strerror(result,message,sizeof(message));Serial.printf("[RT_SSL_WRITE] result=%d errno=%d bytes=%u elapsed_ms=%lu detail=%s\n",result,errno,unsigned(bytes),(unsigned long)(millis()-at),message);stop();return 0;}
  return size_t(result);
 }
};
class RealtimeSocket:public WebSocketsClient {
 uint32_t nextAttempt=0;
public:
 void loop(){
  if(!_client.tcp){
   if(millis()<nextAttempt)return;
   _client.ssl=new RealtimeTLS();_client.tcp=_client.ssl;_client.ssl->setCACert(asrRootCA);
   if(_client.ssl->connect(_host.c_str(),_port,15000))connectedCb();
   else {diagnose();clientDisconnect(&_client,nullptr);nextAttempt=millis()+1000;}
   return;
  }
  WebSocketsClient::loop();
 }
 void tune(){if(_client.ssl)_client.ssl->setTimeout(10000);}
 void diagnose(){char message[96]={};int code=_client.ssl?_client.ssl->lastError(message,sizeof(message)):0;Serial.printf("[RT_TLS] code=%d detail=%s rssi=%d heap=%u\n",code,message,WiFi.RSSI(),ESP.getFreeHeap());}
};
struct Packet {uint16_t size;uint32_t created;uint8_t bytes[512];};
constexpr unsigned audioDepth=250; // 10 seconds in PSRAM for short network stalls.
QueueHandle_t audioQueue=nullptr,textQueue=nullptr;StaticQueue_t audioControl;
uint8_t* audioStorage=nullptr;
std::atomic<bool> running{false},stopping{false},fatal{false},audioDone{true},networkDone{false},textDone{true};
StateCallback stateCb;TextCallback textCb;FinishCallback finishCb;
portMUX_TYPE errorLock=portMUX_INITIALIZER_UNLOCKED;char failure[160]={};
uint32_t startedAt=0,frames=0,sentBytes=0,completed=0,converted=0,maxLag=0;
void fail(const char* message){
 portENTER_CRITICAL(&errorLock);if(!failure[0]){strncpy(failure,message,sizeof(failure)-1);failure[sizeof(failure)-1]=0;}portEXIT_CRITICAL(&errorLock);
 fatal=true;stopping=true;
}
bool push(const uint8_t* p,size_t n,uint32_t serial,uint32_t sequence,uint64_t granule,uint8_t flags){
 Packet packet;packet.size=OggOpus::page(packet.bytes,sizeof(packet.bytes),p,n,serial,sequence,granule,flags);packet.created=millis();
 if(!packet.size){fail("Opus数据帧过大，已停止");return false;}
 if(xQueueSend(audioQueue,&packet,0)!=pdTRUE){fail("上传跟不上录音，已停止；未完成部分请重录");return false;}return true;
}
void capture(void*){
 void* encoder=nullptr;uint8_t* output=nullptr;
 esp_opus_enc_config_t cfg{};cfg.sample_rate=16000;cfg.channel=1;cfg.bits_per_sample=16;cfg.bitrate=12000;cfg.frame_duration=ESP_OPUS_ENC_FRAME_DURATION_40_MS;cfg.application_mode=ESP_OPUS_ENC_APPLICATION_VOIP;cfg.complexity=3;cfg.enable_fec=false;cfg.enable_dtx=true;cfg.enable_vbr=true;
 int inSize=0,outSize=0;int16_t stereo[1280],mono[640];uint32_t seq=0;uint64_t granule=0;const uint32_t serial=esp_random();bool headerSent=false;
 do {
  const auto code=esp_opus_enc_open(&cfg,sizeof(cfg),&encoder);
  if(code!=ESP_AUDIO_ERR_OK){Serial.printf("[RT_CODEC] open_error=%d\n",int(code));fail("Opus编码器初始化失败");break;}
  if(esp_opus_enc_get_frame_size(encoder,&inSize,&outSize)!=ESP_AUDIO_ERR_OK || inSize!=1280 || outSize<1){fail("Opus编码帧配置错误");break;}
  output=static_cast<uint8_t*>(malloc(outSize));if(!output){fail("Opus编码内存不足");break;}
  Serial.printf("[RT_CODEC] pcm_bytes=%d output_capacity=%d bitrate=12000 vbr=1 dtx=1 frame_ms=40\n",inSize,outSize);
  uint8_t header[20];OggOpus::head(header);if(!push(header,19,serial,seq++,0,2))break;OggOpus::tags(header);if(!push(header,20,serial,seq++,0,0))break;headerSent=true;
  for(unsigned i=0;i<4;++i)if(Microphone::read(stereo,640)<0){fail("麦克风读取失败");break;}
  if(!fatal && !stopping && stateCb)stateCb("正在听，停顿约1秒自动转写");
  int32_t previousInput=0,previousOutput=0;
  while(!stopping && !fatal){
   if(Microphone::read(stereo,640)!=640){fail("麦克风读取失败");break;}
   int peak=0;
   for(unsigned i=0;i<640;++i){const int32_t x=stereo[2*i];int32_t y=x-previousInput+(previousOutput*32604)/32768;previousInput=x;previousOutput=y;mono[i]=int16_t(constrain(y,-32768,32767));peak=max(peak,abs(int(mono[i])));}
   // The installed microphones have low output. Raise quiet speech for VAD,
   // while limiting each frame's gain so louder speech does not clip.
   const int gain=max(1,min(8,28000/max(1,peak)));for(auto& sample:mono)sample=int16_t(int(sample)*gain);
   esp_audio_enc_in_frame_t input{reinterpret_cast<uint8_t*>(mono),1280};esp_audio_enc_out_frame_t encoded{};encoded.buffer=output;encoded.len=outSize;
   if(esp_opus_enc_process(encoder,&input,&encoded)!=ESP_AUDIO_ERR_OK){fail("Opus编码失败");break;}
   granule+=1920;if(!push(output,encoded.encoded_bytes,serial,seq++,granule,0))break;++frames;
  }
  // Flush codec lookahead with one silent frame and mark the final Ogg page.
  if(headerSent && !fatal){memset(mono,0,sizeof(mono));esp_audio_enc_in_frame_t input{reinterpret_cast<uint8_t*>(mono),1280};esp_audio_enc_out_frame_t encoded{};encoded.buffer=output;encoded.len=outSize;
   if(esp_opus_enc_process(encoder,&input,&encoded)==ESP_AUDIO_ERR_OK)push(output,encoded.encoded_bytes,serial,seq++,granule+1920,4);else fail("Opus收尾失败");}
 }while(false);
 if(encoder)esp_opus_enc_close(encoder);free(output);audioDone=true;
 Serial.printf("[RT_CAPTURE] frames=%lu audio_ms=%lu stack_free=%u\n",(unsigned long)frames,(unsigned long)(frames*40),unsigned(uxTaskGetStackHighWaterMark(nullptr)));vTaskDelete(nullptr);
}
void convert(void*){
 while(!networkDone || uxQueueMessagesWaiting(textQueue)){
  String* raw=nullptr;if(xQueueReceive(textQueue,&raw,pdMS_TO_TICKS(100))!=pdTRUE)continue;
  if(raw && !fatal){String output,error;const uint32_t at=millis();
   if(standardMandarin(*raw,output,error)){if(output.length()>3800){fail("单句转换结果过长，请分句停顿");}else{++converted;if(textCb)textCb(output.c_str());Serial.printf("[RT_MANDARIN] seq=%lu bytes=%u elapsed_ms=%lu\n",(unsigned long)converted,unsigned(output.length()),(unsigned long)(millis()-at));}}
   else {Serial.printf("[RT_CONVERT_ERROR] %s\n",error.c_str());fail("普通话转换失败；已完成文字保留，未完成部分请重录");}
  }
  delete raw;
 }
 textDone=true;Serial.printf("[RT_CONVERT_TASK] stack_free=%u\n",unsigned(uxTaskGetStackHighWaterMark(nullptr)));vTaskDelete(nullptr);
}
void networkBody(){
 const auto oldSleep=WiFi.getSleep();WiFi.setSleep(false);
 RealtimeSocket ws;bool ready=false,configured=false,captureStarted=false,finishing=false,finished=false,everConnected=false;uint32_t finishAt=0;String lastItem;
 uint32_t uploads=0,maxSendMs=0,lastNetLog=0;
 String endpoint=ASR_ENDPOINT;const int begin=endpoint.indexOf("://"),slash=endpoint.indexOf('/',begin+3);String host=endpoint.substring(begin+3,slash);
 String path="/api-ws/v1/realtime?model=qwen3-asr-flash-realtime";String headers=String("Authorization: Bearer ")+DASHSCOPE_API_KEY;
 ws.onEvent([&](WStype_t type,uint8_t* payload,size_t length){
  if(type==WStype_CONNECTED){everConnected=true;JsonDocument d;d["type"]="session.update";auto s=d["session"];s["input_audio_format"]="opus";s["sample_rate"]=16000;s["input_audio_transcription"]["language"]="zh";s["turn_detection"]["type"]="server_vad";s["turn_detection"]["threshold"]=0.2;s["turn_detection"]["silence_duration_ms"]=1000;String msg;serializeJson(d,msg);if(!ws.sendTXT(msg))fail("实时配置发送失败");configured=true;}
  else if(type==WStype_DISCONNECTED && everConnected && !finished)fail("实时连接中断；未完成部分请重录");
  else if(type==WStype_TEXT){
   JsonDocument d;if(deserializeJson(d,payload,length)){fail("实时响应解析失败");return;}const char* event=d["type"]|"";
   if(!strcmp(event,"session.updated")){ready=true;ws.tune();Serial.println("[RT] session_ready opus=ogg bitrate=12000 vbr=1 dtx=1 vad_ms=1000");}
   else if(!strcmp(event,"conversation.item.input_audio_transcription.completed")){
    String id=d["item_id"].as<String>();if(!id.isEmpty() && id==lastItem)return;lastItem=id;String transcript=d["transcript"].as<String>();transcript.trim();if(transcript.isEmpty())return;
    if(transcript.length()>3072){fail("单句过长，请分句停顿；未完成部分请重录");return;}
    String* raw=new(std::nothrow) String(transcript);if(!raw || xQueueSend(textQueue,&raw,0)!=pdTRUE){delete raw;fail("普通话转换积压，已停止；未完成部分请重录");return;}
    ++completed;Serial.printf("[RT_FINAL] seq=%lu bytes=%u session_ms=%lu pending=%u\n",(unsigned long)completed,unsigned(transcript.length()),(unsigned long)(millis()-startedAt),unsigned(uxQueueMessagesWaiting(textQueue)));
   }else if(!strcmp(event,"session.finished")){finished=true;}
   else if(!strcmp(event,"error") || !strcmp(event,"conversation.item.input_audio_transcription.failed")){Serial.printf("[RT_API_ERROR] code=%s\n",d["error"]["code"]|"unknown");fail("实时识别接口失败；已完成文字保留");}
  }
 });
 // This library leaves client-certificate pointers uninitialized for local
 // instances. Explicitly clear them; authenticate with the bearer token.
 // The library also applies this interval before its first connection after
 // boot. Once a live connection drops we fail explicitly, so this interval
 // only controls initial connection attempts, before any audio is captured.
 ws.setReconnectInterval(1000);ws.beginSslWithClientKey(host.c_str(),443,path.c_str(),asrRootCA,nullptr,nullptr,"");ws.setExtraHeaders(headers.c_str());
 while(!fatal && !finished){
  ws.loop();if(fatal || finished)break;
  if(ready && !captureStarted){
   captureStarted=true;
   if(!stopping){textDone=false;if(xTaskCreatePinnedToCore(convert,"rt-mandarin",14*1024,nullptr,2,nullptr,0)!=pdPASS){textDone=true;fail("转换任务创建失败");break;}
    audioDone=false;if(xTaskCreatePinnedToCore(capture,"rt-opus",32*1024,nullptr,3,nullptr,1)!=pdPASS){audioDone=true;fail("录音任务创建失败");break;}}
  }
  if(ready){
   // Six Opus/Ogg pages represent 240 ms. Combine them in one WebSocket
   // message (hard cap <1400 bytes including base64/JSON), hence one TLS write in
   // the vendored library, instead of 25 small writes per second.
   Packet packet;
   const unsigned queued=uxQueueMessagesWaiting(audioQueue);
   const bool have=queued && xQueuePeek(audioQueue,&packet,0)==pdTRUE;
   if(have && (queued>=6 || millis()-packet.created>=240 || (stopping && audioDone))){
    uint8_t grouped[960];size_t bytes=0;
    for(unsigned batch=0;batch<6;++batch){
     if(xQueuePeek(audioQueue,&packet,0)!=pdTRUE || bytes+packet.size>sizeof(grouped))break;
     if(xQueueReceive(audioQueue,&packet,0)!=pdTRUE)break;
    const uint32_t lag=millis()-packet.created;if(lag>maxLag)maxLag=lag;
    if(lag>12000){fail("实时上传延迟过大，已停止；未完成部分请重录");break;}
     memcpy(grouped+bytes,packet.bytes,packet.size);bytes+=packet.size;
    }
    if(fatal)break;
    char base64[1281];size_t n=0;if(mbedtls_base64_encode(reinterpret_cast<uint8_t*>(base64),sizeof(base64)-1,&n,grouped,bytes)){fail("Opus上传编码失败");break;}base64[n]=0;
    String msg=String("{\"type\":\"input_audio_buffer.append\",\"audio\":\"")+base64+"\"}";
    const uint32_t at=millis();
    if(!ws.sendTXT(msg)){ws.diagnose();fail("实时音频发送失败；未完成部分请重录");break;}sentBytes+=bytes;++uploads;maxSendMs=max(maxSendMs,millis()-at);
    if(millis()-lastNetLog>=2000){lastNetLog=millis();Serial.printf("[RT_NET] messages=%lu queued=%u max_send_ms=%lu max_lag_ms=%lu\n",(unsigned long)uploads,unsigned(uxQueueMessagesWaiting(audioQueue)),(unsigned long)maxSendMs,(unsigned long)maxLag);}
   }
   if(stopping && audioDone && !uxQueueMessagesWaiting(audioQueue) && !finishing && !fatal){String msg="{\"type\":\"session.finish\"}";if(!ws.sendTXT(msg))fail("实时识别收尾失败");finishing=true;finishAt=millis();}
  }
  if(!ready && millis()-startedAt>25000)fail(configured?"实时识别配置超时":"实时连接超时，请检查网络");
  if(finishing && millis()-finishAt>30000)fail("等待最后一句超时；已完成文字保留");
  if(stopping && !everConnected)break;
  vTaskDelay(pdMS_TO_TICKS(1));
 }
 stopping=true;while(!audioDone)vTaskDelay(pdMS_TO_TICKS(10));ws.disconnect();ws.onEvent(nullptr);networkDone=true;
 if(!fatal && stateCb)stateCb("正在整理最后的普通话文字");
 while(!textDone)vTaskDelay(pdMS_TO_TICKS(20));
 char error[160];portENTER_CRITICAL(&errorLock);strncpy(error,failure,sizeof(error));portEXIT_CRITICAL(&errorLock);
 const uint32_t seconds=max(uint32_t(1),frames*40/1000);
 if(error[0])Serial.printf("[RT_ERROR] %s\n",error);
 Serial.printf("[RT_DONE] ok=%u frames=%lu ogg_bytes=%lu avg_Bps=%lu max_lag_ms=%lu recognized=%lu converted=%lu stack_free=%u heap=%u\n",!error[0],(unsigned long)frames,(unsigned long)sentBytes,(unsigned long)(sentBytes/seconds),(unsigned long)maxLag,(unsigned long)completed,(unsigned long)converted,unsigned(uxTaskGetStackHighWaterMark(nullptr)),ESP.getFreeHeap());
 vQueueDelete(audioQueue);vQueueDelete(textQueue);audioQueue=nullptr;textQueue=nullptr;free(audioStorage);audioStorage=nullptr;
 WiFi.setSleep(oldSleep);
}
void network(void*){
 // Return normally from the body to release C++ strings and WebSocket/TLS
 // members before deleting the FreeRTOS task (which does not unwind C++).
 networkBody();char error[160];portENTER_CRITICAL(&errorLock);strncpy(error,failure,sizeof(error));portEXIT_CRITICAL(&errorLock);
 const auto done=finishCb;running=false;if(done)done(error);vTaskDelete(nullptr);
}
}
bool start(StateCallback state,TextCallback text,FinishCallback finish){
 if(running.exchange(true))return false;
 stateCb=state;textCb=text;finishCb=finish;stopping=false;fatal=false;audioDone=true;textDone=true;networkDone=false;failure[0]=0;frames=sentBytes=completed=converted=maxLag=0;startedAt=millis();
 audioStorage=static_cast<uint8_t*>(heap_caps_malloc(audioDepth*sizeof(Packet),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
 if(audioStorage)audioQueue=xQueueCreateStatic(audioDepth,sizeof(Packet),audioStorage,&audioControl);textQueue=xQueueCreate(8,sizeof(String*));
 if(!audioQueue || !textQueue || xTaskCreatePinnedToCore(network,"rt-network",16*1024,nullptr,2,nullptr,0)!=pdPASS){if(audioQueue)vQueueDelete(audioQueue);if(textQueue)vQueueDelete(textQueue);free(audioStorage);audioStorage=nullptr;audioQueue=textQueue=nullptr;running=false;return false;}
 return true;
}
void stop(){stopping=true;}
bool active(){return running;}
void codecTest(){
 if(running)return;
 esp_opus_enc_config_t cfg{};cfg.sample_rate=16000;cfg.channel=1;cfg.bits_per_sample=16;cfg.bitrate=24000;cfg.frame_duration=ESP_OPUS_ENC_FRAME_DURATION_40_MS;cfg.application_mode=ESP_OPUS_ENC_APPLICATION_VOIP;cfg.complexity=3;
 void* encoder=nullptr;int inputSize=0,outputSize=0;uint8_t* output=nullptr;uint8_t* blob=nullptr;size_t bytes=0;bool ok=false;
 do{
  if(esp_opus_enc_open(&cfg,sizeof(cfg),&encoder)!=ESP_AUDIO_ERR_OK)break;
  if(esp_opus_enc_get_frame_size(encoder,&inputSize,&outputSize)!=ESP_AUDIO_ERR_OK || inputSize!=1280 || outputSize<1)break;
  output=static_cast<uint8_t*>(malloc(outputSize));blob=static_cast<uint8_t*>(heap_caps_malloc(32768,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));if(!output || !blob)break;
  uint8_t header[20];OggOpus::head(header);bytes+=OggOpus::page(blob+bytes,32768-bytes,header,19,1234,0,0,2);OggOpus::tags(header);bytes+=OggOpus::page(blob+bytes,32768-bytes,header,20,1234,1,0,0);
  int16_t pcm[640];ok=true;
  for(unsigned frame=0;frame<50;++frame){
   for(unsigned i=0;i<640;++i)pcm[i]=int16_t(3000*sinf(2*PI*440*(frame*640+i)/16000.0f));
   esp_audio_enc_in_frame_t input{reinterpret_cast<uint8_t*>(pcm),1280};esp_audio_enc_out_frame_t encoded{};encoded.buffer=output;encoded.len=outputSize;
   if(esp_opus_enc_process(encoder,&input,&encoded)!=ESP_AUDIO_ERR_OK){ok=false;break;}
   size_t n=OggOpus::page(blob+bytes,32768-bytes,output,encoded.encoded_bytes,1234,frame+2,(frame+1)*1920,frame==49?4:0);if(!n){ok=false;break;}bytes+=n;
  }
 }while(false);
 if(ok){Serial.printf("[OPUS_TEST] bytes=%u samples=32000 bitrate=24000\n",unsigned(bytes));for(size_t i=0;i<bytes;){size_t n=Serial.write(blob+i,min(size_t(512),bytes-i));i+=n;delay(1);}Serial.println();Serial.println("[OPUS_TEST_DONE]");}
 else Serial.println("[OPUS_TEST_FAIL]");
 if(encoder)esp_opus_enc_close(encoder);free(output);free(blob);
}
}
