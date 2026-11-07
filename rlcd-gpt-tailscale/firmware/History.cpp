#include "History.h"
#include <ArduinoJson.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include <atomic>

namespace History {
namespace {
struct Record {char source[24],text[4096],question[2048];time_t when;};
sdmmc_card_t* card=nullptr;
QueueHandle_t queue=nullptr;
SemaphoreHandle_t files=nullptr;
std::atomic<bool> mounted{false};
std::atomic<uint32_t> written{0},dropped{0};
std::atomic<uint32_t> pending{0};
void copy(char* to,const char* from,size_t size){size_t n=min(strlen(from),size-1);while(n&&(uint8_t(from[n])&0xc0)==0x80)--n;memcpy(to,from,n);to[n]=0;}
bool save(const char* path,const String& data,bool append){
 FILE* f=fopen(path,append?"ab":"wb");if(!f)return false;
 bool ok=fwrite(data.c_str(),1,data.length(),f)==data.length();
 ok=fflush(f)==0&&ok;ok=fsync(fileno(f))==0&&ok;ok=fclose(f)==0&&ok;return ok;
}
void writer(void*){
 Record* record=nullptr;
 for(;;){
  if(xQueueReceive(queue,&record,portMAX_DELAY)!=pdTRUE)continue;
  Serial.println("[SD_HISTORY] stage=dequeued");
  struct tm local{};localtime_r(&record->when,&local);char day[24];strftime(day,sizeof(day),"%Y%m%d",&local);
  JsonDocument d;d["timestamp"]=int64_t(record->when);d["source"]=record->source;d["text"]=record->text;
  if(record->question[0])d["question"]=record->question;
  String data;serializeJson(d,data);
  String path=String("/sdcard/rlcd/history/")+(record->when>=1700000000?day:"unsynced")+".txt";
  xSemaphoreTake(files,portMAX_DELAY);
  Serial.println("[SD_HISTORY] stage=write");
  bool ok=mounted&&save(path.c_str(),data+"\n",true);
  if(ok&&!strcmp(record->source,"gpt")){
   ok=save("/sdcard/rlcd/last-gpt.tmp",data,false);
   if(ok){
    unlink("/sdcard/rlcd/last-gpt.bak");
    rename("/sdcard/rlcd/last-gpt.jsn","/sdcard/rlcd/last-gpt.bak");
    ok=rename("/sdcard/rlcd/last-gpt.tmp","/sdcard/rlcd/last-gpt.jsn")==0;
    if(!ok)rename("/sdcard/rlcd/last-gpt.bak","/sdcard/rlcd/last-gpt.jsn");
   }
  }
  xSemaphoreGive(files);
  if(ok)++written;else {++dropped;Serial.println("[SD_HISTORY] write_failed=1 text_kept_on_screen=1");}
  Serial.printf("[SD_HISTORY] saved=%u\n",ok);
  free(record);
  --pending;
 }
}
}
bool begin(){
 if(mounted)return true;
 sdmmc_host_t host=SDMMC_HOST_DEFAULT();host.max_freq_khz=10000;
 sdmmc_slot_config_t slot=SDMMC_SLOT_CONFIG_DEFAULT();slot.width=1;slot.clk=GPIO_NUM_38;slot.cmd=GPIO_NUM_21;slot.d0=GPIO_NUM_39;
 slot.flags|=SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
 esp_vfs_fat_sdmmc_mount_config_t config{};config.format_if_mount_failed=false;config.max_files=5;config.allocation_unit_size=16384;
 const esp_err_t error=esp_vfs_fat_sdmmc_mount("/sdcard",&host,&slot,&config,&card);
 if(error!=ESP_OK){Serial.printf("[SD] mounted=0 error=%d no_format=1\n",int(error));return false;}
 mkdir("/sdcard/rlcd",0755);mkdir("/sdcard/rlcd/history",0755);
 char probe[80];snprintf(probe,sizeof(probe),"/sdcard/rlcd/%08lx.tmp",(unsigned long)esp_random());
 const char payload[]="RLCD SD write/read check\n";
 bool verified=save(probe,String(payload),false);char readback[sizeof(payload)]={};FILE* f=fopen(probe,"rb");
 if(f){verified=fread(readback,1,sizeof(payload)-1,f)==sizeof(payload)-1&&verified;fclose(f);}else verified=false;
 verified=verified&&!memcmp(readback,payload,sizeof(payload)-1);unlink(probe);
 if(!verified){Serial.println("[SD] mounted=1 write_read_verified=0");esp_vfs_fat_sdcard_unmount("/sdcard",card);card=nullptr;return false;}
 queue=xQueueCreate(12,sizeof(Record*));files=xSemaphoreCreateMutex();
 if(!queue||!files){if(queue)vQueueDelete(queue);if(files)vSemaphoreDelete(files);queue=nullptr;files=nullptr;esp_vfs_fat_sdcard_unmount("/sdcard",card);card=nullptr;return false;}
 mounted=true;
 if(xTaskCreatePinnedToCore(writer,"sd-history",8192,nullptr,1,nullptr,1)!=pdPASS){mounted=false;vQueueDelete(queue);vSemaphoreDelete(files);queue=nullptr;files=nullptr;esp_vfs_fat_sdcard_unmount("/sdcard",card);card=nullptr;return false;}
 Serial.printf("[SD] mounted=1 write_read_verified=1 capacity_MB=%llu pins=38,21,39 width=1 no_format=1\n",(unsigned long long)(uint64_t(card->csd.capacity)*card->csd.sector_size/1048576));
 return true;
}
void append(const char* source,const char* text,const char* question){
 if(!mounted||!text||!text[0])return;
 auto* record=static_cast<Record*>(heap_caps_malloc(sizeof(Record),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
 if(!record){++dropped;return;}
 copy(record->source,source,sizeof(record->source));copy(record->text,text,sizeof(record->text));copy(record->question,question?question:"",sizeof(record->question));record->when=time(nullptr);
 ++pending;
 if(xQueueSend(queue,&record,0)!=pdTRUE){free(record);++dropped;--pending;}
}
bool lastGpt(String& text){
 if(!mounted)return false;
 xSemaphoreTake(files,portMAX_DELAY);FILE* f=fopen("/sdcard/rlcd/last-gpt.jsn","rb");if(!f)f=fopen("/sdcard/rlcd/last-gpt.bak","rb");bool ok=false;
 if(f){fseek(f,0,SEEK_END);long size=ftell(f);rewind(f);if(size>0&&size<14000){String data;data.reserve(size);char b[256];size_t n;while((n=fread(b,1,sizeof(b),f)))data.concat(b,n);JsonDocument d;if(!deserializeJson(d,data)){text=d["text"]|"";ok=!text.isEmpty()&&text.length()<4096;}}fclose(f);}
 xSemaphoreGive(files);return ok;
}
void report(){Serial.printf("[SD_INFO] ready=%u records=%lu dropped=%lu queued=%u\n",mounted.load(),(unsigned long)written.load(),(unsigned long)dropped.load(),queue?unsigned(uxQueueMessagesWaiting(queue)):0);}
bool flush(uint32_t timeout){const uint32_t at=millis();while(pending&&millis()-at<timeout)delay(10);return pending==0;}
}
