#include "RealtimeText.h"
#include "Microphone.h"
#include "LocalConfig.h"
#include "Trust.h"
#include "OggOpus.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <RlcdOpus.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <atomic>
#include <new>
#include <math.h>
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
QueueHandle_t audioQueue=nullptr;StaticQueue_t audioControl;
uint8_t* audioStorage=nullptr;
std::atomic<bool> running{false},stopping{false},fatal{false},audioDone{true};
StateCallback stateCb;TextCallback textCb;FinishCallback finishCb;
portMUX_TYPE errorLock=portMUX_INITIALIZER_UNLOCKED;char failure[160]={};
uint32_t startedAt=0,frames=0,sentBytes=0,completed=0,converted=0,maxLag=0;
std::atomic<uint32_t> capturedMs{0};
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
 esp_opus_enc_config_t cfg{};cfg.sample_rate=16000;cfg.channel=1;cfg.bits_per_sample=16;cfg.bitrate=24000;cfg.frame_duration=ESP_OPUS_ENC_FRAME_DURATION_40_MS;cfg.application_mode=ESP_OPUS_ENC_APPLICATION_VOIP;cfg.complexity=3;cfg.enable_fec=false;cfg.enable_dtx=false;cfg.enable_vbr=true;
 int inSize=0,outSize=0;int16_t stereo[1280],mono[640];uint32_t seq=0;uint64_t granule=0;const uint32_t serial=esp_random();bool headerSent=false;
 do {
  const auto code=esp_opus_enc_open(&cfg,sizeof(cfg),&encoder);
  if(code!=ESP_AUDIO_ERR_OK){Serial.printf("[RT_CODEC] open_error=%d\n",int(code));fail("Opus编码器初始化失败");break;}
  if(esp_opus_enc_get_frame_size(encoder,&inSize,&outSize)!=ESP_AUDIO_ERR_OK || inSize!=1280 || outSize<1){fail("Opus编码帧配置错误");break;}
  output=static_cast<uint8_t*>(malloc(outSize));if(!output){fail("Opus编码内存不足");break;}
  Serial.printf("[RT_CODEC] pcm_bytes=%d output_capacity=%d bitrate=24000 vbr=1 dtx=0 frame_ms=40\n",inSize,outSize);
  uint8_t header[20];OggOpus::head(header);if(!push(header,19,serial,seq++,0,2))break;OggOpus::tags(header);if(!push(header,20,serial,seq++,0,0))break;headerSent=true;
  for(unsigned i=0;i<4;++i)if(Microphone::read(stereo,640)<0){fail("麦克风读取失败");break;}
  if(!fatal && !stopping && stateCb)stateCb("正在听，停顿后自动转成普通话");
  int32_t previousInput=0,previousOutput=0;float gain=8.0f;
  unsigned quietFrames=0,quietLog=0;uint32_t lastAudioLog=millis();
  while(!stopping && !fatal){
   if(Microphone::read(stereo,640)!=640){fail("麦克风读取失败");break;}
   int peak=0;uint64_t energy=0;
   for(unsigned i=0;i<640;++i){const int32_t x=stereo[2*i];int32_t y=x-previousInput+(previousOutput*32604)/32768;previousInput=x;previousOutput=y;mono[i]=int16_t(constrain(y,-32768,32767));peak=max(peak,abs(int(mono[i])));energy+=int64_t(mono[i])*mono[i];}
   const unsigned rms=unsigned(sqrt(double(energy)/640));
   // Do not boost a sustained low-level noise floor by 8x. Keep all samples
   // (no hard gate), allow 200ms for soft consonants, and preserve speech gain
   // separately so the next syllable is not faded in after a pause.
   quietFrames=(rms<50 && peak<400)?min(quietFrames+1,5u):0;
   const float target=min(8.0f,28000.0f/max(1,peak));
   if(quietFrames<5)gain=target<gain?target:min(target,gain+0.10f);
   const float appliedGain=quietFrames>=5?min(gain,2.0f):gain;
   if(quietFrames>=5)++quietLog;
   for(auto& sample:mono)sample=int16_t(constrain(int32_t(sample*appliedGain),-32768,32767));
   if(millis()-lastAudioLog>=2000){lastAudioLog=millis();Serial.printf("[RT_AUDIO] rms=%u peak=%d gain_x10=%u quiet_frames=%u\n",rms,peak,unsigned(appliedGain*10),quietLog);quietLog=0;}
   esp_audio_enc_in_frame_t input{reinterpret_cast<uint8_t*>(mono),1280};esp_audio_enc_out_frame_t encoded{};encoded.buffer=output;encoded.len=outSize;
   if(esp_opus_enc_process(encoder,&input,&encoded)!=ESP_AUDIO_ERR_OK){fail("Opus编码失败");break;}
   granule+=1920;if(!push(output,encoded.encoded_bytes,serial,seq++,granule,0))break;++frames;capturedMs=frames*40;
  }
  // Flush codec lookahead with one silent frame and mark the final Ogg page.
  if(headerSent && !fatal){memset(mono,0,sizeof(mono));esp_audio_enc_in_frame_t input{reinterpret_cast<uint8_t*>(mono),1280};esp_audio_enc_out_frame_t encoded{};encoded.buffer=output;encoded.len=outSize;
   if(esp_opus_enc_process(encoder,&input,&encoded)==ESP_AUDIO_ERR_OK)push(output,encoded.encoded_bytes,serial,seq++,granule+1920,4);else fail("Opus收尾失败");}
 }while(false);
 if(encoder)esp_opus_enc_close(encoder);free(output);audioDone=true;
 Serial.printf("[RT_CAPTURE] frames=%lu audio_ms=%lu stack_free=%u\n",(unsigned long)frames,(unsigned long)(frames*40),unsigned(uxTaskGetStackHighWaterMark(nullptr)));vTaskDelete(nullptr);
}
// Message API emits final standard-Mandarin text directly. No second LLM task.
void networkBody(){
 const auto oldSleep=WiFi.getSleep();WiFi.setSleep(false);
 RealtimeSocket ws;bool ready=false,configured=false,captureStarted=false,finishing=false,finished=false,everConnected=false;
 uint32_t finishAt=0,uploads=0,maxSendMs=0,lastNetLog=0;int lastSentence=-1;
 char uuid[37];uint32_t a=esp_random(),b=esp_random(),c=esp_random(),e=esp_random();
 snprintf(uuid,sizeof(uuid),"%08lx-%04lx-4%03lx-8%03lx-%04lx%08lx",(unsigned long)a,(unsigned long)(b>>16),(unsigned long)(b&0xfff),(unsigned long)(c&0xfff),(unsigned long)(c>>16),(unsigned long)e);
 String taskId=uuid;
 String endpoint=ASR_ENDPOINT;const int begin=endpoint.indexOf("://"),slash=endpoint.indexOf('/',begin+3);String host=endpoint.substring(begin+3,slash);
 String headers=String("Authorization: Bearer ")+DASHSCOPE_API_KEY;
 String recent[3];unsigned recentCount=0;bool contextDirty=false;
 auto taskHeader=[&](JsonDocument& d,const char* action){d["header"]["action"]=action;d["header"]["task_id"]=taskId;d["header"]["streaming"]="duplex";};
 ws.onEvent([&](WStype_t type,uint8_t* payload,size_t length){
  if(type==WStype_CONNECTED){
   everConnected=true;JsonDocument d;taskHeader(d,"run-task");auto p=d["payload"];
   p["task_group"]="audio";p["task"]="asr";p["function"]="recognition";p["model"]="qwen-audio-3.1-asr-flash-message";
   auto params=p["parameters"];params["format"]="opus";params["sample_rate"]=16000;params["keep_dialect"]=false;
   params["language_hints"][0]="zh";params["vad_model"]="near_meeting_16k";params["max_sentence_silence"]=800;params["intermediate_result_enabled"]=false;
   params["disfluency_removal_enabled"]=false;params["heartbeat"]=true;
   p["input"].to<JsonObject>();String msg;serializeJson(d,msg);if(!ws.sendTXT(msg))fail("新模型任务发送失败");configured=true;
  }else if(type==WStype_DISCONNECTED && everConnected && !finished){fail("实时连接中断；未完成部分请重录");}
  else if(type==WStype_TEXT){
   JsonDocument d;if(deserializeJson(d,payload,length)){fail("实时响应解析失败");return;}
   const char* responseTask=d["header"]["task_id"]|"";if(taskId!=responseTask)return;
   const char* event=d["header"]["event"]|"";
   if(!strcmp(event,"task-started")){ready=true;ws.tune();Serial.println("[RT] model=qwen-audio-3.1-asr-flash-message task_ready opus=ogg bitrate=24000 vbr=1 dtx=0 vad=near_meeting_16k vad_ms=800 quiet_gain_max=2 keep_dialect=0");}
   else if(!strcmp(event,"result-generated")){
    auto sentence=d["payload"]["output"]["sentence"];
    if((sentence["heartbeat"]|false)||!(sentence["sentence_end"]|false))return;
    const int id=sentence["sentence_id"]|-1;if(id<0||id<=lastSentence)return;
    String text=sentence["text"]|"";text.trim();lastSentence=id;if(text.isEmpty())return;
    if(text.length()>3800){fail("单句过长，请分句停顿；未完成部分请重录");return;}
    ++completed;++converted;if(textCb)textCb(text.c_str());
    // Keep up to three short, real preceding sentences. Never inject example
    // phrases as if they were spoken, or carry context into a new recording.
    String context=text;if(context.length()>900){size_t cut=900;while(cut&&(uint8_t(context[cut])&0xc0)==0x80)--cut;context.remove(cut);}
    if(recentCount<3)recent[recentCount++]=context;else{recent[0]=recent[1];recent[1]=recent[2];recent[2]=context;}
    contextDirty=true;
    Serial.printf("[RT_FINAL] seq=%lu sentence=%d bytes=%u session_ms=%lu direct_mandarin=1\n",(unsigned long)completed,id,unsigned(text.length()),(unsigned long)(millis()-startedAt));
    const int64_t endMs=sentence["end_time"]|int64_t(-1);
    const int64_t ahead=endMs>=0?int64_t(capturedMs.load())-endMs:-1;
    Serial.printf("[RT_TIMING] sentence=%d end_ms=%lld captured_ms=%lu audio_ahead_ms=%lld\n",id,(long long)endMs,(unsigned long)capturedMs.load(),(long long)ahead);
   }else if(!strcmp(event,"task-finished")){finished=true;}
   else if(!strcmp(event,"task-failed")){
    // Log the server code only: never echo requests, headers or credentials.
    String code=d["header"]["error_code"]|"unknown";Serial.printf("[RT_API_ERROR] code=%s\n",code.c_str());
    String error=String("新模型识别失败：")+code;fail(error.c_str());
   }
  }
 });
 // Clear client-certificate pointers in this vendored WebSocket library.
 ws.setReconnectInterval(1000);ws.beginSslWithClientKey(host.c_str(),443,"/api-ws/v1/inference",asrRootCA,nullptr,nullptr,"");ws.setExtraHeaders(headers.c_str());
 while(!fatal && !finished){
  ws.loop();if(fatal||finished)break;
  if(ready&&!captureStarted){captureStarted=true;if(!stopping){audioDone=false;if(xTaskCreatePinnedToCore(capture,"rt-opus",32*1024,nullptr,3,nullptr,1)!=pdPASS){audioDone=true;fail("录音任务创建失败");break;}}}
  if(ready){
   if(contextDirty&&!finishing&&!stopping){
    JsonDocument d;taskHeader(d,"continue-task");for(unsigned i=0;i<recentCount;++i){auto item=d["payload"]["input"]["context"][i];item["role"]="user";item["content"][0]["type"]="input_text";item["content"][0]["text"]=recent[i];}
    String msg;serializeJson(d,msg);if(!ws.sendTXT(msg)){fail("识别上下文发送失败");break;}contextDirty=false;
   }
   // Batch up to 240ms of Ogg pages. Binary frames avoid base64 overhead;
   // keep each TLS write small and the existing 10-second audio queue bounded.
   Packet packet;const unsigned queued=uxQueueMessagesWaiting(audioQueue);
   const bool have=queued&&xQueuePeek(audioQueue,&packet,0)==pdTRUE;
   if(have&&(queued>=6||millis()-packet.created>=240||(stopping&&audioDone))){
    uint8_t grouped[960];size_t bytes=0;
    for(unsigned batch=0;batch<6;++batch){
     if(xQueuePeek(audioQueue,&packet,0)!=pdTRUE||bytes+packet.size>sizeof(grouped))break;
     if(xQueueReceive(audioQueue,&packet,0)!=pdTRUE)break;
     const uint32_t lag=millis()-packet.created;maxLag=max(maxLag,lag);
     if(lag>12000){fail("实时上传延迟过大，已停止；未完成部分请重录");break;}
     memcpy(grouped+bytes,packet.bytes,packet.size);bytes+=packet.size;
    }
    if(fatal)break;
    const uint32_t at=millis();if(!ws.sendBIN(grouped,bytes)){ws.diagnose();fail("实时音频发送失败；未完成部分请重录");break;}
    sentBytes+=bytes;++uploads;maxSendMs=max(maxSendMs,millis()-at);
    if(millis()-lastNetLog>=2000){lastNetLog=millis();Serial.printf("[RT_NET] messages=%lu queued=%u max_send_ms=%lu max_lag_ms=%lu\n",(unsigned long)uploads,unsigned(uxQueueMessagesWaiting(audioQueue)),(unsigned long)maxSendMs,(unsigned long)maxLag);}
   }
   if(stopping&&audioDone&&!uxQueueMessagesWaiting(audioQueue)&&!finishing&&!fatal){
    JsonDocument d;taskHeader(d,"finish-task");d["payload"]["input"].to<JsonObject>();String msg;serializeJson(d,msg);
    if(!ws.sendTXT(msg))fail("实时识别收尾失败");finishing=true;finishAt=millis();
   }
  }
  if(!ready&&millis()-startedAt>25000)fail(configured?"新模型启动超时":"实时连接超时，请检查网络");
  if(finishing&&millis()-finishAt>30000)fail("等待最后一句超时；已完成文字保留");
  if(stopping&&!everConnected)break;
  vTaskDelay(pdMS_TO_TICKS(1));
 }
 stopping=true;while(!audioDone)vTaskDelay(pdMS_TO_TICKS(10));ws.disconnect();ws.onEvent(nullptr);
 char error[160];portENTER_CRITICAL(&errorLock);strncpy(error,failure,sizeof(error));portEXIT_CRITICAL(&errorLock);
 const uint32_t seconds=max(uint32_t(1),frames*40/1000);
 if(error[0])Serial.printf("[RT_ERROR] %s\n",error);
 Serial.printf("[RT_DONE] ok=%u model=asr31 frames=%lu ogg_bytes=%lu avg_Bps=%lu max_lag_ms=%lu recognized=%lu direct_mandarin=%lu stack_free=%u heap=%u\n",!error[0],(unsigned long)frames,(unsigned long)sentBytes,(unsigned long)(sentBytes/seconds),(unsigned long)maxLag,(unsigned long)completed,(unsigned long)converted,unsigned(uxTaskGetStackHighWaterMark(nullptr)),ESP.getFreeHeap());
 vQueueDelete(audioQueue);audioQueue=nullptr;free(audioStorage);audioStorage=nullptr;WiFi.setSleep(oldSleep);
}
void network(void*){
 networkBody();char error[160];portENTER_CRITICAL(&errorLock);strncpy(error,failure,sizeof(error));portEXIT_CRITICAL(&errorLock);
 const auto done=finishCb;running=false;if(done)done(error);vTaskDelete(nullptr);
}
}
bool start(StateCallback state,TextCallback text,FinishCallback finish){
 if(running.exchange(true))return false;
 stateCb=state;textCb=text;finishCb=finish;stopping=false;fatal=false;audioDone=true;failure[0]=0;frames=sentBytes=completed=converted=maxLag=0;capturedMs=0;startedAt=millis();
 audioStorage=static_cast<uint8_t*>(heap_caps_malloc(audioDepth*sizeof(Packet),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
 if(audioStorage)audioQueue=xQueueCreateStatic(audioDepth,sizeof(Packet),audioStorage,&audioControl);
 if(!audioQueue||xTaskCreatePinnedToCore(network,"rt-network",20*1024,nullptr,2,nullptr,0)!=pdPASS){if(audioQueue)vQueueDelete(audioQueue);free(audioStorage);audioStorage=nullptr;audioQueue=nullptr;running=false;return false;}
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
