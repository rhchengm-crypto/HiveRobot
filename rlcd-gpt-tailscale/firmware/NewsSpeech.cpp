#include "NewsSpeech.h"
#include "Microphone.h"
#include "LocalConfig.h"
#include "PcmWave.h"
#include <esp_http_client.h>
#include <esp_crt_bundle.h>
#include <ArduinoJson.h>
#include <freertos/idf_additions.h>
#include <mbedtls/sha256.h>
#include <mbedtls/base64.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <atomic>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

namespace {
constexpr size_t maxAudio=2*1024*1024;
constexpr const char* model="qwen3-tts-flash";
TaskHandle_t worker=nullptr;
portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
std::atomic<bool> busy{false};std::atomic<uint32_t> version{1},generation{0};
bool pending=false,toneJob=false;
char requested[512]={},message[128]="新闻播报待命";
int requestedNumber=0,total=0,nextItem=0,lastNumber=0;
String edition,items[10];
std::atomic<uint32_t> cacheHits{0},completed{0},failed{0};

void update(const String& value){portENTER_CRITICAL(&lock);strlcpy(message,value.c_str(),sizeof(message));++version;portEXIT_CRITICAL(&lock);}
bool cancelled(uint32_t job){return generation.load()!=job;}
class Buffer:public Stream {
 public:
 uint8_t* data;size_t capacity,used=0;uint32_t job;bool overflow=false;
 Buffer(size_t cap,uint32_t id):capacity(cap),job(id){data=static_cast<uint8_t*>(heap_caps_malloc(cap,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));}
 ~Buffer(){free(data);}
 size_t write(uint8_t b)override{return write(&b,1);}
 size_t write(const uint8_t* p,size_t n)override{
  if(cancelled(job))return 0;
  if(!data||n>capacity-used){overflow=true;return 0;}memcpy(data+used,p,n);used+=n;return n;
 }
 int available()override{return 0;}int read()override{return -1;}int peek()override{return -1;}void flush()override{}
};
// ESP-IDF decodes HTTP transfer chunks before forwarding SSE bytes here.
// Only the first audio event contains a WAV header; following events append PCM.
class SsePlayer:public Stream {
 Buffer& wav;Buffer decoded;uint32_t job,started;int number;
 String line;PcmWave::View audio;
 std::atomic<size_t> frames{0};std::atomic<bool> inputDone{false},playbackOk{false};
 SemaphoreHandle_t playerDone=nullptr;bool playerStarted=false;
 unsigned chunks=0;
 void playback(){
  bool opened=false,ok=!cancelled(job);size_t outputFrame=0,used=0;int16_t pcm[1024];
  if(ok)ok=opened=Microphone::speakerOpen();
  if(opened){
   update(String("正在播报 第 ")+number+" 条");
   Serial.printf("[NEWS_SPEECH_START] item=%d stream=1 first_audio_ms=%lu\n",number,(unsigned long)(millis()-started));
  }
  while(ok&&!cancelled(job)){
   const bool final=inputDone.load();const size_t count=frames.load();
   const size_t target=uint64_t(count)*16000/audio.rate;
   if(final&&outputFrame>=target)break;
   const uint64_t position=uint64_t(outputFrame)*audio.rate;const size_t a=position/16000;
   if(outputFrame>=target||(!final&&a+1>=count)){delay(2);continue;}
   const size_t b=min(a+1,count-1);const int32_t fraction=position%16000;
   const int32_t left=int16_t(PcmWave::u16(audio.pcm+a*2)),right=int16_t(PcmWave::u16(audio.pcm+b*2));
   const int16_t sample=(left*(16000-fraction)+right*fraction)/16000;
   pcm[used*2]=pcm[used*2+1]=sample;++outputFrame;
   if(++used==512){ok=Microphone::speakerWrite(pcm,used);used=0;}
  }
  if(opened){
   if(ok&&!cancelled(job)&&used)ok=Microphone::speakerWrite(pcm,used);
   memset(pcm,0,sizeof(pcm));for(int i=0;i<4;++i)if(!Microphone::speakerWrite(pcm,512))ok=false;
   delay(130);ok=Microphone::speakerClose()&&ok;
  }
  playbackOk=ok&&!cancelled(job);xSemaphoreGive(playerDone);
 }
 bool event(){
  if(!line.startsWith("data:"))return true;
  JsonDocument json;
  if(deserializeJson(json,line.c_str()+5,line.length()-5)){error="语音事件解析失败";return false;}
  if(json["code"].is<const char*>()&&strlen(json["code"].as<const char*>())){error="语音服务返回错误";return false;}
  const char* reason=json["output"]["finish_reason"]|"";
  if(reason[0]&&strcmp(reason,"null"))Serial.printf("[NEWS_SPEECH_EVENT] finish_reason=%s\n",reason);
  const char* data=json["output"]["audio"]["data"]|"";
  if(data[0]){
   size_t count=0;
   if(mbedtls_base64_decode(decoded.data,decoded.capacity,&count,reinterpret_cast<const uint8_t*>(data),strlen(data))||!count){error="语音数据解码失败";return false;}
   if(wav.write(decoded.data,count)!=count){error="语音文件过长或已停止";return false;}
   if(wav.used<46)return true;
   PcmWave::View view;
   if(!PcmWave::parse(wav.data,wav.used,view)){error="流式语音格式异常";return false;}
   ++chunks;
   if(!playerStarted){
    audio=view;frames=view.frames;
    if(!playerDone){error="播放同步资源不足";return false;}
    if(xTaskCreatePinnedToCoreWithCaps([](void* arg){static_cast<SsePlayer*>(arg)->playback();vTaskDeleteWithCaps(nullptr);},"news-pcm",12*1024,this,3,nullptr,1,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT)!=pdPASS){error="播放任务创建失败";return false;}
    playerStarted=true;
   }else frames=view.frames;
  }
  if(!strcmp(json["output"]["finish_reason"]|"","stop"))finished=true;
  return true;
 }
 public:
 String error;bool finished=false;
 SsePlayer(Buffer& target,uint32_t id,int item,uint32_t at):wav(target),decoded(65536,id),job(id),started(at),number(item){line.reserve(32768);playerDone=xSemaphoreCreateBinary();}
 ~SsePlayer(){if(playerStarted)close(false);if(playerDone)vSemaphoreDelete(playerDone);}
 bool ready(){return decoded.data!=nullptr&&playerDone!=nullptr;}
 size_t write(uint8_t byte)override{return write(&byte,1);}
 size_t write(const uint8_t* bytes,size_t n)override{
  if(cancelled(job)||!error.isEmpty())return 0;
  for(size_t i=0;i<n;++i){
   if(bytes[i]=='\n'){
    const bool ok=event();line.remove(0);if(!ok)return 0;
   }else if(bytes[i]!='\r'){
    if(line.length()>=65536){error="语音事件过大";return 0;}line+=char(bytes[i]);
   }
  }
  return n;
 }
 bool close(bool success){
  // The final SSE event may end at EOF without a newline.
  if(line.length()&&error.isEmpty()&&!cancelled(job)){event();line.remove(0);}
  inputDone=true;
  bool ok=success&&finished&&error.isEmpty()&&!cancelled(job)&&playerStarted;
  if(playerStarted){
   xSemaphoreTake(playerDone,portMAX_DELAY);playerStarted=false;ok=playbackOk.load()&&ok;
  }
  Serial.printf("[NEWS_SPEECH_STREAM] finished=%u chunks=%u playback_ok=%u\n",finished,chunks,playbackOk.load());
  return ok;
 }
 int available()override{return 0;}int read()override{return -1;}int peek()override{return -1;}void flush()override{}
};
bool streamAudio(const String& text,Buffer& wav,String& error,uint32_t job,int number){
 const uint32_t at=millis();
 struct AwakeWifi {bool old;AwakeWifi():old(WiFi.getSleep()){WiFi.setSleep(false);}~AwakeWifi(){WiFi.setSleep(old);}} awake;
 String endpoint=ASR_ENDPOINT;const int end=endpoint.indexOf('/',8);
 if(!endpoint.startsWith("https://")||end<0){error="语音接口配置错误";return false;}
 endpoint=endpoint.substring(0,end)+"/api/v1/services/aigc/multimodal-generation/generation";
 if(cancelled(job))return false;
 esp_http_client_config_t config{};config.url=endpoint.c_str();config.method=HTTP_METHOD_POST;
 config.crt_bundle_attach=esp_crt_bundle_attach;config.timeout_ms=45000;config.buffer_size=4096;config.buffer_size_tx=1024;
 struct Client {esp_http_client_handle_t handle=nullptr;~Client(){if(handle)esp_http_client_cleanup(handle);}} client;
 client.handle=esp_http_client_init(&config);if(!client.handle){error="语音连接内存不足";return false;}
 const String authorization=String("Bearer ")+DASHSCOPE_API_KEY;
 esp_http_client_set_header(client.handle,"Authorization",authorization.c_str());
 esp_http_client_set_header(client.handle,"Content-Type","application/json");
 esp_http_client_set_header(client.handle,"Accept","text/event-stream");
 esp_http_client_set_header(client.handle,"X-DashScope-SSE","enable");
 JsonDocument request;request["model"]=model;request["input"]["text"]=text;request["input"]["voice"]="Cherry";request["input"]["language_type"]="Chinese";
 String payload;serializeJson(request,payload);
 const esp_err_t opened=esp_http_client_open(client.handle,payload.length());
 if(opened!=ESP_OK){error="语音连接失败";Serial.printf("[NEWS_SPEECH_HTTP] open_error=%d\n",int(opened));return false;}
 if(esp_http_client_write(client.handle,payload.c_str(),payload.length())!=int(payload.length())){error="语音请求发送失败";return false;}
 const int64_t header=esp_http_client_fetch_headers(client.handle);const int code=esp_http_client_get_status_code(client.handle);
 if(header<0||code!=200){error=String("语音合成 HTTP ")+code;return false;}
 SsePlayer sink(wav,job,number,at);
 if(!sink.ready()){error="流式播报内存不足";return false;}
 char buffer[4096];int received=0,lastRead=0;
 while(!cancelled(job)){
  lastRead=esp_http_client_read(client.handle,buffer,sizeof(buffer));
  if(lastRead<=0)break;
  if(sink.write(reinterpret_cast<uint8_t*>(buffer),lastRead)!=size_t(lastRead)){lastRead=-1;break;}
  received+=lastRead;if(sink.finished)break;
 }
 esp_http_client_close(client.handle);
 bool ok=sink.close(lastRead>=0);error=sink.error;
 Serial.printf("[NEWS_SPEECH_HTTP] native=1 bytes=%d last_read=%d\n",received,lastRead);
 if(!ok&&error.isEmpty()&&!cancelled(job))error="流式语音连接中断";
 Serial.printf("[NEWS_SPEECH_TTS] stream=1 http=200 ok=%u audio_bytes=%u elapsed_ms=%lu\n",ok,unsigned(wav.used),(unsigned long)(millis()-at));return ok;
}
bool cacheRead(const char* path,const uint8_t* hash,Buffer& wav){
 FILE* f=fopen(path,"rb");if(!f)return false;
 uint8_t header[40];bool ok=fread(header,1,sizeof(header),f)==sizeof(header)&&!memcmp(header,"RNS1",4)&&!memcmp(header+4,hash,32);
 const uint32_t bytes=ok?PcmWave::u32(header+36):0;ok=ok&&bytes>=44&&bytes<=wav.capacity;
 if(ok){
  uint8_t* block=static_cast<uint8_t*>(heap_caps_malloc(8192,MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA));
  if(block){
   for(size_t offset=0;offset<bytes;){const size_t n=min(size_t(8192),size_t(bytes)-offset);if(cancelled(wav.job)||fread(block,1,n,f)!=n){ok=false;break;}memcpy(wav.data+offset,block,n);offset+=n;}
   free(block);
  }else ok=fread(wav.data,1,bytes,f)==bytes;
  ok=ok&&fgetc(f)==EOF;
 }
 fclose(f);if(ok)wav.used=bytes;return ok;
}
void cacheWrite(const char* path,const uint8_t* hash,const Buffer& wav){
 if(mkdir("/sdcard/rlcd/speech",0755)!=0&&errno!=EEXIST)return;
 String temp=String(path);temp.remove(temp.length()-4);temp+=".tmp";
 FILE* f=fopen(temp.c_str(),"wb");if(!f)return;
 uint8_t header[40];memcpy(header,"RNS1",4);memcpy(header+4,hash,32);
 for(int i=0;i<4;++i)header[36+i]=uint32_t(wav.used)>>(8*i);
 bool ok=fwrite(header,1,40,f)==40;
 uint8_t* block=static_cast<uint8_t*>(heap_caps_malloc(8192,MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA));
 if(block){
  for(size_t offset=0;ok&&offset<wav.used;){const size_t n=min(size_t(8192),wav.used-offset);memcpy(block,wav.data+offset,n);ok=!cancelled(wav.job)&&fwrite(block,1,n,f)==n;offset+=n;}
  free(block);
 }else ok=ok&&fwrite(wav.data,1,wav.used,f)==wav.used;
 ok=fflush(f)==0&&ok;ok=fsync(fileno(f))==0&&ok;ok=fclose(f)==0&&ok;
 if(ok){unlink(path);ok=rename(temp.c_str(),path)==0;}if(!ok)unlink(temp.c_str());
 Serial.printf("[NEWS_SPEECH_CACHE] saved=%u\n",ok);
}
bool play(const PcmWave::View& wav,uint32_t job){
 if(!Microphone::speakerOpen())return false;
 bool ok=true;int16_t out[512*2];size_t used=0;
 const size_t frames=uint64_t(wav.frames)*16000/wav.rate;
 for(size_t i=0;i<frames&&!cancelled(job);++i){
  const uint64_t position=uint64_t(i)*wav.rate;const size_t a=position/16000,b=min(a+1,wav.frames-1);const unsigned fraction=position%16000;
  const int32_t left=int16_t(PcmWave::u16(wav.pcm+a*2)),right=int16_t(PcmWave::u16(wav.pcm+b*2));
  const int16_t sample=(left*int32_t(16000-fraction)+right*int32_t(fraction))/16000;
  out[used*2]=out[used*2+1]=sample;
  if(++used==512){if(!Microphone::speakerWrite(out,used)){ok=false;break;}used=0;}
 }
 if(ok&&!cancelled(job)&&used)ok=Microphone::speakerWrite(out,used);
 // Flush silence through DMA before muting, including cancellation (<= 128 ms).
 memset(out,0,sizeof(out));for(int i=0;i<4;++i)if(!Microphone::speakerWrite(out,512))ok=false;
 delay(130);const bool mic=Microphone::speakerClose();return ok&&mic;
}
void run(void*){
 for(;;){
  ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
  for(;;){
   char text[512];int number;bool beep;uint32_t job;
   portENTER_CRITICAL(&lock);
   if(!pending){busy=false;portEXIT_CRITICAL(&lock);break;}
   pending=false;strlcpy(text,requested,sizeof(text));number=requestedNumber;beep=toneJob;job=generation.load();portEXIT_CRITICAL(&lock);
   bool ok=false,hit=false;String error;
   if(beep){
    uint8_t pcm[3200];for(unsigned i=0;i<1600;++i){const int16_t x=3000*sinf(2*3.14159265f*660*i/16000);pcm[i*2]=x;pcm[i*2+1]=x>>8;}
    PcmWave::View tone{pcm,1600,16000};ok=play(tone,job);
   }else{
    Buffer wav(maxAudio,job);uint8_t hash[32];String identity=String(model)+"|Cherry|"+text;
    mbedtls_sha256(reinterpret_cast<const uint8_t*>(identity.c_str()),identity.length(),hash,0);
    char path[80];snprintf(path,sizeof(path),"/sdcard/rlcd/speech/%02x%02x%02x%02x.aud",hash[0],hash[1],hash[2],hash[3]);
    PcmWave::View view;
    if(!wav.data)error="播报内存不足";
    else {
     const uint32_t cacheAt=millis();hit=cacheRead(path,hash,wav)&&PcmWave::parse(wav.data,wav.used,view);
     Serial.printf("[NEWS_SPEECH_CACHE_READ] hit=%u elapsed_ms=%lu\n",hit,(unsigned long)(millis()-cacheAt));
     if(!hit){
      wav.used=0;ok=streamAudio(text,wav,error,job,number);
      if(ok&&!cancelled(job))cacheWrite(path,hash,wav);
     }else if(!cancelled(job)){
      ++cacheHits;update(String("正在播报 第 ")+number+" 条");
      Serial.printf("[NEWS_SPEECH_PLAY] item=%d cache=%u rate=%lu samples=%u\n",number,hit,(unsigned long)view.rate,unsigned(view.frames));
      ok=play(view,job);if(!ok)error="喇叭播放或麦克风恢复失败";
     }
    }
   }
   if(!cancelled(job)){
    if(ok){++completed;update(beep?"喇叭提示音测试完成":String("已播完第 ")+number+" 条 · K1下一条");}
    else {++failed;update(error.isEmpty()?"喇叭测试失败":error);Serial.printf("[NEWS_SPEECH_ERROR] %s\n",error.c_str());}
   }
   Serial.printf("[NEWS_SPEECH_DONE] item=%d ok=%u cancelled=%u\n",number,ok,cancelled(job));
  }
 }
}
int parseItems(const String& text){
 int count=0;
 for(int n=1;n<=10;++n){
  const String marker=String(n)+". ";int start=text.indexOf("\n"+marker);
  if(start>=0)++start;else if(text.startsWith(marker))start=0;else break;
  const int titleEnd=text.indexOf('\n',start),summaryEnd=titleEnd>=0?text.indexOf('\n',titleEnd+1):-1;
  if(titleEnd<0||summaryEnd<0)break;
  String item=text.substring(start,summaryEnd);item.replace("\n","，");
  if(item.length()>=sizeof(requested))break;
  items[count++]=item;
 }
 return count;
}
}
void NewsSpeech::begin(){
 if(!Microphone::speakerReady())return;
 if(xTaskCreatePinnedToCoreWithCaps(run,"news-speech",24*1024,nullptr,2,&worker,0,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT)!=pdPASS){worker=nullptr;update("播报任务创建失败");}
}
bool NewsSpeech::next(const String& bulletin,int expectedCount){
 if(!worker){update("喇叭未就绪");return false;}
 if(edition!=bulletin){edition=bulletin;total=parseItems(bulletin);nextItem=0;}
 if(total<1||total!=expectedCount){update("新闻条目尚未准备好");return false;}
 const int chosen=nextItem;nextItem=(nextItem+1)%total;lastNumber=chosen+1;
 portENTER_CRITICAL(&lock);strlcpy(requested,items[chosen].c_str(),sizeof(requested));requestedNumber=chosen+1;toneJob=false;pending=true;busy=true;++generation;portEXIT_CRITICAL(&lock);
 update(String("准备播报 第 ")+(chosen+1)+" / "+total+" 条");xTaskNotifyGive(worker);
 Serial.printf("[NEWS_SPEECH_REQUEST] item=%d next=%d count=%d\n",chosen+1,nextItem+1,total);return true;
}
bool NewsSpeech::tone(){
 if(!worker||busy)return false;
 portENTER_CRITICAL(&lock);toneJob=true;requestedNumber=0;pending=true;busy=true;++generation;portEXIT_CRITICAL(&lock);xTaskNotifyGive(worker);return true;
}
void NewsSpeech::stop(){portENTER_CRITICAL(&lock);pending=false;++generation;portEXIT_CRITICAL(&lock);}
bool NewsSpeech::active(){return busy.load();}
uint32_t NewsSpeech::revision(){return version.load();}
String NewsSpeech::label(){char copy[128];portENTER_CRITICAL(&lock);strlcpy(copy,message,sizeof(copy));portEXIT_CRITICAL(&lock);return String(copy);}
void NewsSpeech::report(){Serial.printf("[NEWS_SPEECH_INFO] ready=%u active=%u count=%d last=%d next=%d cache_hits=%lu completed=%lu failed=%lu\n",worker!=nullptr,busy.load(),total,lastNumber,nextItem+1,(unsigned long)cacheHits,(unsigned long)completed,(unsigned long)failed);}
