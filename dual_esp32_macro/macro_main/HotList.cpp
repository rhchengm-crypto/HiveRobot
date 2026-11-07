#include "HotList.h"
#include "Weather.h"
#include "HotCA.h"
#include "FeitCloud.h"
#include "ChineseFont.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_heap_caps.h>
namespace HotList {
namespace {
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
Snapshot shared;
bool started=false,requested=false,attempted=false;
bool pendingTv=false;
Snapshot candidate;
uint32_t attemptAt=0;
struct Cache { uint32_t magic=0x484f5431;Snapshot data{};uint32_t crc=0; };
uint32_t hash(const void* data,size_t n) {
  auto p=static_cast<const uint8_t*>(data);uint32_t h=2166136261u;
  while(n--){h^=*p++;h*=16777619u;}return h;
}
bool valid(const Snapshot& s) {
  if(s.fetchedAt<1700000000)return false;
  for(int i=0;i<5;++i)for(const char* p:{s.topics[i],s.dramas[i]})
    if(!p[0] || !memchr(p,0,160))return false;
  return true;
}
void save(const Snapshot& s) {
  while(ChineseFont::uploadActive())vTaskDelay(pdMS_TO_TICKS(20));
  Cache c{};c.data=s;c.data.loading=false;c.crc=hash(&c,offsetof(Cache,crc));
  Preferences p;if(p.begin("entertainment",false)) {p.putBytes("top5v1",&c,sizeof(c));p.end();}
}
bool load(Snapshot& s) {
  Cache c{};Preferences p;if(!p.begin("entertainment",true))return false;
  bool ok=p.getBytesLength("top5v1")==sizeof(c) && p.getBytes("top5v1",&c,sizeof(c))==sizeof(c);p.end();
  if(!ok || c.magic!=0x484f5431 || c.crc!=hash(&c,offsetof(Cache,crc)) || !valid(c.data))return false;
  s=c.data;s.valid=true;s.loading=false;s.lastFetchOk=false;s.revision=1;return true;
}
bool entertainment(JsonObjectConst o) {
  if(o["is_ad"].as<int>() || o["is_topic_ad"].as<int>() || o["topic_ad"].as<int>())return false;
  const char* channel=o["channel_type"]|"";const char* category=o["category"]|"";
  return !strcmp(channel,"Entertainment") || o["fun_word"].as<int>()==1
    || strstr(category,"艺人") || strstr(category,"明星") || strstr(category,"电影")
    || strstr(category,"电视剧") || strstr(category,"综艺") || strstr(category,"音乐");
}
bool fetch(bool drama,Snapshot& result) {
  WiFiClientSecure tls;HTTPClient http;tls.setCACert(hotRootCA);tls.setHandshakeTimeout(8);
  http.setConnectTimeout(5000);http.setTimeout(5000);http.setReuse(false);http.useHTTP10(true);
  const char* url=drama?"https://60s.b23.run/v2/baidu/teleplay":"https://weibo.com/ajax/statuses/hot_band?type=ent";
  if(!http.begin(tls,url))return false;
  http.addHeader("User-Agent","Mozilla/5.0 MacroDesk/1.0");
  if(!drama)http.addHeader("Referer","https://weibo.com/");
  int code=http.GET();if(code!=200 || http.getSize()>65536) {
    Serial.printf("[HOT] %s HTTP=%d\n",drama?"TV":"WEIBO",code);http.end();tls.stop();return false;
  }
  Serial.printf("[HOT] %s headers bytes=%d heap=%u largest=%u\n",drama?"TV":"WEIBO",http.getSize(),ESP.getFreeHeap(),heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  // Parse one bounded object at a time; retain only the selected five titles.
  // The 4096-bit Weibo certificate leaves too little contiguous heap for a full body.
  auto& stream=http.getStream();stream.setTimeout(5000);
  if(!stream.find(drama?"\"data\"":"\"band_list\"") || !stream.find("[")) {Serial.println("[HOT] array header unavailable");http.end();tls.stop();return false;}
  JsonDocument filter,doc;
  if(drama)filter["title"]=true;
  else for(const char* key:{"word","note","channel_type","category","fun_word","is_ad","is_topic_ad","topic_ad"})filter[key]=true;
  unsigned count=0;
  for(unsigned scanned=0;scanned<100;++scanned) {
    int ch;do {ch=stream.peek();if(ch<0){vTaskDelay(pdMS_TO_TICKS(1));ch=stream.peek();}}while(ch>=0 && isspace(ch) && stream.read()>=0);
    auto error=deserializeJson(doc,stream,DeserializationOption::Filter(filter),DeserializationOption::NestingLimit(8));
    if(error){Serial.printf("[HOT] %s item JSON=%s\n",drama?"TV":"WEIBO",error.c_str());break;}
    JsonObjectConst row=doc.as<JsonObjectConst>();
    // Consume the separator before any continue, waiting for fragmented records.
    int separator=-1;const uint32_t waitAt=millis();
    do {separator=stream.read();if(separator<0)vTaskDelay(pdMS_TO_TICKS(1));}
    while((separator<0 || isspace(separator)) && uint32_t(millis()-waitAt)<5000);
    const bool end=separator==']';
    if(separator!=',' && !end)break;
    const char* title=drama?row["title"].as<const char*>():row["word"].as<const char*>();
    if((drama || entertainment(row)) && title && title[0] && strlen(title)<160) {
      auto& titles=drama?result.dramas:result.topics;bool duplicate=false;
      for(unsigned i=0;i<count;++i)if(!strcmp(titles[i],title))duplicate=true;
      if(!duplicate)strlcpy(titles[count++],title,160);
    }
    if(count==5 || end)break;
  }
  http.end();tls.stop();
  return count==5;
}
}
void begin() {if(started)return;Snapshot c{};if(load(c)){shared=c;Serial.println("[HOT] saved TOP5 cache loaded");}started=true;}
Snapshot snapshot() {portENTER_CRITICAL(&mux);Snapshot s=shared;portEXIT_CRITICAL(&mux);return s;}
bool due() {
  if(!Weather::quiet())return false;
  if(!started || WiFi.status()!=WL_CONNECTED || time(nullptr)<1700000000 || FeitCloud::state().busy)return false;
  const auto s=snapshot();bool manual;portENTER_CRITICAL(&mux);manual=requested;portEXIT_CRITICAL(&mux);
  if(pendingTv)return true;
  if(attempted && uint32_t(millis()-attemptAt)<cooldown(manual,s.lastFetchOk))return false;
  return manual || !s.valid || scheduleSlot(time(nullptr))>scheduleSlot(s.fetchedAt);
}
void poll() {
  if(!due() || ESP.getFreeHeap()<80000 || heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)<24000)return;
  const Snapshot current=snapshot();
  bool ok;
  // One endpoint per idle slot. New lamp commands run before the second request.
  // A failed pair never replaces the previous complete set of ten titles.
  if(!pendingTv) {
    candidate=Snapshot{};
    portENTER_CRITICAL(&mux);requested=false;shared.loading=true;++shared.revision;portEXIT_CRITICAL(&mux);
    attemptAt=millis();attempted=true;
    ok=fetch(false,candidate);
    if(ok){pendingTv=true;return;}
  } else {ok=fetch(true,candidate);pendingTv=false;}
  Snapshot next=ok?candidate:current;
  if(ok){next.fetchedAt=uint32_t(time(nullptr));ok=valid(next);}
  if(!ok)next=current;
  next.loading=false;next.lastFetchOk=ok;next.valid=ok || current.valid;next.failures=current.failures+!ok;
  if(ok)save(next);
  portENTER_CRITICAL(&mux);next.revision=shared.revision+1;shared=next;portEXIT_CRITICAL(&mux);
  Serial.printf("[HOT] %s slot=%d heap=%u stack=%u\n",ok?"UPDATED 5+5":"FAILED; retained cache",scheduleSlot(time(nullptr)),ESP.getFreeHeap(),uxTaskGetStackHighWaterMark(nullptr));
}
void refresh() {portENTER_CRITICAL(&mux);requested=true;portEXIT_CRITICAL(&mux);Serial.println("[HOT] refresh queued");}
void status() {
  auto s=snapshot();Serial.printf("[HOT] STATUS valid=%u loading=%u fresh=%u fetched=%lu failures=%lu slot=%d schedule=07:00 America/Los_Angeles\n",s.valid,s.loading,s.lastFetchOk,(unsigned long)s.fetchedAt,(unsigned long)s.failures,scheduleSlot(time(nullptr)));
  if(s.valid)for(unsigned i=0;i<5;++i)Serial.printf("[HOT] %u %s | TV %s\n",i+1,s.topics[i],s.dramas[i]);
}
bool selfTest() {
  // 06:59/07:00 PST/PDT, including spring/fall transition Sundays.
  return scheduleSlot(1772978340)==20260307 && scheduleSlot(1772978400)==20260308
    && scheduleSlot(1793545140)==20261031 && scheduleSlot(1793545200)==20261101;
}
}
