#include "Speech.h"
#include "SpeechOrder.h"
#include "SpeechCA.h"
#include "HotList.h"
#include "ChineseFont.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <SD.h>
#include <driver/dac_continuous.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <new>
#include <time.h>
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
namespace Speech {
namespace {
constexpr size_t maximumAudio=192*1024;
struct Job {uint32_t generation=0;char text[224]={},path[48]={};bool tv=false,tone=false;uint8_t index=0;};
constexpr size_t inputBytes=6144;
constexpr size_t outputBytes=4096;
struct Buffers {mp3dec_t* decoder=nullptr;uint8_t* input=nullptr;uint8_t* output=nullptr;mp3d_sample_t* pcm=nullptr;};
Buffers* allocateBuffers() {
  auto* b=new(std::nothrow) Buffers;if(!b)return nullptr;
  b->decoder=new(std::nothrow) mp3dec_t;b->input=new(std::nothrow) uint8_t[inputBytes];
  b->output=new(std::nothrow) uint8_t[outputBytes];b->pcm=new(std::nothrow) mp3d_sample_t[MINIMP3_MAX_SAMPLES_PER_FRAME];return b;
}
void freeBuffers(Buffers* b){if(!b)return;delete b->decoder;delete[] b->input;delete[] b->output;delete[] b->pcm;delete b;}
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
Snapshot shared;
Job requested;
SpeechOrder order;
uint32_t generation=0,handled=0;
bool activePage=false,activeTv=false,started=false;
volatile bool audioActive=false;
volatile uint32_t dmaStops=0;
bool IRAM_ATTR dacStopped(dac_continuous_handle_t,const dac_event_data_t*,void*){dmaStops=dmaStops+1;return false;}

bool current(uint32_t g){portENTER_CRITICAL(&mux);const bool result=g==generation;portEXIT_CRITICAL(&mux);return result;}
void state(const Job& j,uint8_t mode,const char* detail,uint32_t samples=0) {
  portENTER_CRITICAL(&mux);
  if(j.generation==generation) {shared.mode=mode;shared.tv=j.tv;shared.index=j.index;shared.samples=samples;strlcpy(shared.detail,detail,sizeof(shared.detail));++shared.revision;}
  portEXIT_CRITICAL(&mux);
}
void pathFor(Job& j) {
  uint64_t hash=14695981039346656037ull;
  for(const uint8_t* p=reinterpret_cast<const uint8_t*>(j.text);*p;++p){hash^=*p;hash*=1099511628211ull;}
  snprintf(j.path,sizeof(j.path),"/speech/%08lx%08lx.mp3",(unsigned long)(hash>>32),(unsigned long)hash);
}
bool cached(const char* path) {
  File f=SD.open(path,FILE_READ);uint8_t h[3];
  const bool ok=f && f.size()>=128 && f.size()<=maximumAudio && f.read(h,3)==3
    && ((!memcmp(h,"ID3",3)) || (h[0]==0xff && (h[1]&0xe0)==0xe0));
  f.close();return ok;
}
String encode(const char* p) {
  static const char hex[]="0123456789ABCDEF";String out;out.reserve(strlen(p)*3);
  while(*p) {const uint8_t c=*p++;if(isalnum(c) && c<128)out+=char(c);else{out+='%';out+=hex[c>>4];out+=hex[c&15];}}
  return out;
}
class LimitedFile:public Stream {
 public:
  File file;size_t count=0;
  int available()override{return 0;}int read()override{return -1;}int peek()override{return -1;}
  void flush()override{file.flush();}
  size_t write(uint8_t c)override{return write(&c,1);}
  size_t write(const uint8_t* data,size_t size)override {
    if(size>maximumAudio-count)return 0;
    const size_t n=file.write(data,size);count+=n;return n;
  }
};
bool download(const Job& j) {
  if(WiFi.status()!=WL_CONNECTED || time(nullptr)<1700000000)return false;
  if(!SD.exists("/speech") && !SD.mkdir("/speech"))return false;
  char temporary[56];snprintf(temporary,sizeof(temporary),"%s.tmp",j.path);SD.remove(temporary);
  LimitedFile sink;sink.file=SD.open(temporary,FILE_WRITE);if(!sink.file)return false;
  WiFiClientSecure tls;HTTPClient http;tls.setCACert(speechCA);tls.setHandshakeTimeout(8);
  http.setConnectTimeout(5000);http.setTimeout(6000);http.setReuse(false);http.useHTTP10(true);
  const String url=String("https://translate.google.com/translate_tts?ie=UTF-8&client=tw-ob&tl=zh-CN&q=")+encode(j.text);
  bool ok=http.begin(tls,url);
  if(ok) {
    http.addHeader("User-Agent","Mozilla/5.0");http.addHeader("Referer","https://translate.google.com/");
    const int code=http.GET();const int length=http.getSize();
    if(code==200 && (length<0 || length<=int(maximumAudio))) {
      const int copied=http.writeToStream(&sink);ok=copied>0 && (length<0 || copied==length);
    } else {char detail[128]={};int error=tls.lastError(detail,sizeof(detail));Serial.printf("[SPEECH] TTS HTTP=%d bytes=%d TLS=%d %s heap=%u largest=%u\n",code,length,error,detail,ESP.getFreeHeap(),heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));ok=false;}
  }
  http.end();tls.stop();sink.flush();sink.file.close();
  ok=ok && cached(temporary);
  if(ok) {if(SD.exists(j.path))SD.remove(j.path);ok=SD.rename(temporary,j.path);}
  if(!ok)SD.remove(temporary);
  Serial.printf("[SPEECH] DOWNLOAD %s bytes=%u heap=%u largest=%u\n",ok?"OK":"FAILED",unsigned(sink.count),ESP.getFreeHeap(),heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  return ok;
}
bool openDac(dac_continuous_handle_t& handle,uint32_t hz) {
  dac_continuous_config_t config{};config.chan_mask=DAC_CHANNEL_MASK_CH1;
  config.desc_num=8;config.buf_size=2048;config.freq_hz=hz;
  config.clk_src=DAC_DIGI_CLK_SRC_APLL;config.chan_mode=DAC_CHANNEL_MODE_SIMUL;
  esp_err_t result=dac_continuous_new_channels(&config,&handle);
  if(result==ESP_OK){dac_event_callbacks_t callbacks{};callbacks.on_stop=dacStopped;result=dac_continuous_register_event_callback(handle,&callbacks,nullptr);}
  if(result==ESP_OK)result=dac_continuous_enable(handle);
  if(result!=ESP_OK){Serial.printf("[SPEECH] DAC error=%s\n",esp_err_to_name(result));if(handle){dac_continuous_del_channels(handle);handle=nullptr;}return false;}
  return true;
}
bool output(dac_continuous_handle_t dac,uint8_t* data,size_t size,const Job& j) {
  size_t offset=0;uint32_t lastProgress=millis();
  while(offset<size && current(j.generation)) {
    size_t n=0;const auto result=dac_continuous_write(dac,data+offset,size-offset,&n,100);
    offset+=n;if(n)lastProgress=millis();else if(uint32_t(millis()-lastProgress)>1500)return false;
    if(result!=ESP_OK && result!=ESP_ERR_TIMEOUT)return false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return offset==size;
}
void player(void* argument) {
  const Job j=*static_cast<Job*>(argument);delete static_cast<Job*>(argument);
  if(!current(j.generation)){audioActive=false;vTaskDelete(nullptr);return;}
    audioActive=true;uint32_t samples=0,rate=0;bool ok=true;
    dmaStops=0;const uint32_t startedAt=millis();uint32_t maxReadUs=0,maxDecodeUs=0;
    dac_continuous_handle_t dac=nullptr;
    if(j.tone) {
      rate=16000;ok=openDac(dac,rate);uint8_t bytes[256];
      if(ok)state(j,Playing,"SPEAKER TEST");
      for(unsigned offset=0;ok && offset<6400 && current(j.generation);offset+=256) {
        for(unsigned i=0;i<256;++i)bytes[i]=128+int(12*sinf(2*3.14159265f*660*(offset+i)/rate));
        ok=output(dac,bytes,256,j);samples+=256;
      }
    } else {
      auto* buffers=allocateBuffers();
      File file=SD.open(j.path,FILE_READ);size_t filled=0,pending=0;int32_t previous=0,quantizationError=0;
      ok=buffers && buffers->decoder && buffers->input && buffers->output && buffers->pcm && file;
      if(!ok)Serial.printf("[SPEECH] PLAYER allocation/open failed heap=%u largest=%u file=%u\n",ESP.getFreeHeap(),heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),bool(file));
      if(ok) {
        mp3dec_init(buffers->decoder);uint8_t tag[10];
        if(file.read(tag,10)==10 && !memcmp(tag,"ID3",3)) {
          uint32_t size=((tag[6]&127)<<21)|((tag[7]&127)<<14)|((tag[8]&127)<<7)|(tag[9]&127);file.seek(10+size);
        } else file.seek(0);
      }
      while(ok && current(j.generation)) {
        if(filled<inputBytes && file.available()) {const uint32_t at=micros();const int got=file.read(buffers->input+filled,inputBytes-filled);maxReadUs=max(maxReadUs,uint32_t(micros()-at));if(got<0){ok=false;break;}filled+=got;}
        if(!filled)break;
        mp3dec_frame_info_t info{};
        const uint32_t decodeAt=micros();int n=mp3dec_decode_frame(buffers->decoder,buffers->input,int(filled),buffers->pcm,&info);maxDecodeUs=max(maxDecodeUs,uint32_t(micros()-decodeAt));
        if(info.frame_bytes>0) {filled-=info.frame_bytes;memmove(buffers->input,buffers->input+info.frame_bytes,filled);}
        else {if(filled>3){memmove(buffers->input,buffers->input+filled-3,3);filled=3;}else break;continue;}
        if(n<=0)continue;
        if(info.channels<1 || info.channels>2 || info.hz<8000 || info.hz>48000){ok=false;break;}
        if(!dac) {
          rate=info.hz*2;ok=openDac(dac,rate);if(!ok)break;
          state(j,Playing,"PLAYING");
          Serial.printf("[SPEECH] PLAY section=%s index=%u rate=%lu channels=%d GPIO=26\n",j.tv?"TV":"HOT",j.index+1,(unsigned long)rate,info.channels);
        }
        if(rate!=unsigned(info.hz*2)){ok=false;break;}
        for(int i=0;i<n;++i) {
          int32_t sample=buffers->pcm[i*info.channels];if(info.channels==2)sample=(sample+buffers->pcm[i*2+1])/2;
          // Double the sample rate and carry quantization error into the next
          // sample, moving 8-bit quantization noise above the main voice band.
          for(unsigned phase=0;phase<2;++phase) {
            const int32_t interpolated=phase?sample:(previous+sample)/2;
            const int32_t scaled=interpolated*40/100+quantizationError;
            const int32_t quantized=constrain((scaled+128)>>8,-128,127);
            quantizationError=scaled-quantized*256;
            buffers->output[pending++]=uint8_t(128+quantized);
            if(pending==outputBytes){ok=output(dac,buffers->output,pending,j);pending=0;if(!ok)break;}
          }
          previous=sample;if(!ok)break;
        }
        samples+=n*2;
        portENTER_CRITICAL(&mux);if(j.generation==generation)shared.samples=samples;portEXIT_CRITICAL(&mux);
      }
      if(ok && pending && current(j.generation))ok=output(dac,buffers->output,pending,j);
      file.close();freeBuffers(buffers);
    }
    if(dac) {
      if(current(j.generation)) {uint8_t silence[1152];memset(silence,128,sizeof(silence));output(dac,silence,sizeof(silence),j);const uint32_t drainMs=(8*2048/2)*1000/rate+25;uint32_t at=millis();while(current(j.generation) && uint32_t(millis()-at)<drainMs)vTaskDelay(pdMS_TO_TICKS(5));}
      dac_continuous_disable(dac);dac_continuous_del_channels(dac);
    }
    const bool complete=ok && samples>0;
    Serial.printf("[SPEECH] TIMING wall_ms=%lu dma_stops=%lu max_read_us=%lu max_decode_us=%lu\n",(unsigned long)(millis()-startedAt),(unsigned long)dmaStops,(unsigned long)maxReadUs,(unsigned long)maxDecodeUs);
    if(current(j.generation))state(j,complete?Done:Error,complete?"DONE":"AUDIO ERROR",samples);
    Serial.printf("[SPEECH] END %s samples=%lu duration_ms=%lu heap=%u stack=%u\n",current(j.generation)?(complete?"OK":"FAILED"):"CANCELLED",(unsigned long)samples,(unsigned long)(rate?uint64_t(samples)*1000/rate:0),ESP.getFreeHeap(),uxTaskGetStackHighWaterMark(nullptr));
  audioActive=false;vTaskDelete(nullptr);
}
bool startPlayer(const Job& j) {
  Job* copy=new(std::nothrow) Job(j);if(!copy)return false;
  audioActive=true;
  if(xTaskCreatePinnedToCore(player,"hot-speech",24576,copy,2,nullptr,1)!=pdPASS){audioActive=false;delete copy;return false;}
  return true;
}
}
bool begin() {
  if(started)return true;
  started=true;Serial.println("[SPEECH] ready: GPIO26 DAC, speaker amp, K3 in HOT&TV");return true;
}
Snapshot snapshot(){portENTER_CRITICAL(&mux);Snapshot result=shared;portEXIT_CRITICAL(&mux);return result;}
bool playing(){return audioActive;}
void stop() {
  portENTER_CRITICAL(&mux);++generation;handled=generation;shared.mode=Idle;++shared.revision;portEXIT_CRITICAL(&mux);
}
void waitStopped(){const uint32_t start=millis();while(audioActive && uint32_t(millis()-start)<500)vTaskDelay(pdMS_TO_TICKS(1));vTaskDelay(pdMS_TO_TICKS(15));}
void context(bool active,bool tv) {
  portENTER_CRITICAL(&mux);
  if(active!=activePage || (active && tv!=activeTv)) {activePage=active;activeTv=tv;order.reset();++generation;handled=generation;shared.mode=Idle;++shared.revision;}
  portEXIT_CRITICAL(&mux);
}
bool next(bool tv) {
  if(!started)return false;
  const auto feed=HotList::snapshot();Job job;
  portENTER_CRITICAL(&mux);
  if(!activePage || activeTv!=tv){activePage=true;activeTv=tv;order.reset();}
  job.generation=++generation;job.tv=tv;job.index=order.take(feed.fetchedAt);
  if(feed.valid)snprintf(job.text,sizeof(job.text),"%s，第%u条，%s。",tv?"热播剧":"娱乐热搜",job.index+1,tv?feed.dramas[job.index]:feed.topics[job.index]);
  requested=job;shared.tv=tv;shared.index=job.index;shared.samples=0;shared.mode=feed.valid?Queued:Error;
  strlcpy(shared.detail,feed.valid?"PREPARING":"NO LIST DATA",sizeof(shared.detail));++shared.revision;
  if(!feed.valid)handled=generation;
  portEXIT_CRITICAL(&mux);
  Serial.printf("[SPEECH] REQUEST section=%s index=%u valid=%u text=%s\n",tv?"TV":"HOT",job.index+1,feed.valid,job.text);
  return feed.valid;
}
bool due(){portENTER_CRITICAL(&mux);bool result=started && activePage && generation!=handled && shared.mode==Queued;portEXIT_CRITICAL(&mux);return result;}
void poll() {
  if(!due())return;
  Job job;portENTER_CRITICAL(&mux);job=requested;handled=job.generation;portEXIT_CRITICAL(&mux);
  waitStopped();pathFor(job);
  bool hit=cached(job.path);state(job,Loading,hit?"SD CACHE":"FETCHING VOICE");
  bool ok=hit || download(job);
  if(!current(job.generation))return;
  if(ok) {if(!startPlayer(job)){state(job,Error,"AUDIO MEMORY");return;}Serial.printf("[SPEECH] %s %s\n",hit?"CACHE HIT":"CACHE SAVED",job.path);}
  else state(job,Error,"VOICE UNAVAILABLE");
}
void tone() {
  if(!started)return;Job job;job.tone=true;
  portENTER_CRITICAL(&mux);job.generation=++generation;handled=generation;shared.mode=Queued;++shared.revision;portEXIT_CRITICAL(&mux);
  waitStopped();if(!startPlayer(job))state(job,Error,"AUDIO MEMORY");
}
void status() {
  const auto s=snapshot();Serial.printf("[SPEECH] STATUS mode=%u section=%s index=%u samples=%lu playing=%u detail=%s heap=%u\n",s.mode,s.tv?"TV":"HOT",s.index+1,(unsigned long)s.samples,playing(),s.detail,ESP.getFreeHeap());
}
}
