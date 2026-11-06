#include "Weather.h"
#include "WeatherCA.h"
#include "FeitCloud.h"
#include "ChineseFont.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <time.h>
#include <math.h>
namespace Weather {
namespace {
constexpr char url[]="https://api.open-meteo.com/v1/forecast?latitude=47.6254&longitude=-122.0462&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max&forecast_days=3&timezone=America%2FLos_Angeles";
constexpr uint32_t interval=30*60*1000, retryInterval=60*1000;
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
Snapshot shared;
bool started=false,attempted=false;
uint32_t attemptedAt=0,networkStackMinimum=0;
bool requested=true;
uint32_t lastActivity=0;
struct Cache { uint32_t magic=0x57454131,zip=98074; Snapshot data{}; uint32_t checksum=0; };
uint32_t checksum(const uint8_t* data,size_t size) {
  uint32_t hash=2166136261u;while(size--){hash^=*data++;hash*=16777619u;}return hash;
}
bool validDays(const Snapshot& s) {
  for(unsigned i=0;i<3;++i) {
    const auto& d=s.days[i];
    if(!validDate(d.date) || !isfinite(d.high) || !isfinite(d.low) || d.high<d.low
       || d.high>70 || d.low< -90 || d.rain>100 || d.code>99)return false;
    if(i && strcmp(s.days[i-1].date,d.date)>=0)return false;
  }
  return s.utcOffset>=-12*3600 && s.utcOffset<=14*3600 && s.fetchedAt>=1700000000;
}
void save(const Snapshot& s) {
  while(ChineseFont::uploadActive())vTaskDelay(pdMS_TO_TICKS(20));
  Cache cache{};cache.data=s;
  cache.data.loading=false;
  cache.checksum=checksum(reinterpret_cast<const uint8_t*>(&cache),offsetof(Cache,checksum));
  Preferences p;
  if(p.begin("weather98074",false)) {p.putBytes("forecast1",&cache,sizeof(cache));p.end();}
}
bool load(Snapshot& s) {
  Cache cache{};Preferences p;
  if(!p.begin("weather98074",true))return false;
  const bool ok=p.getBytesLength("forecast1")==sizeof(cache) && p.getBytes("forecast1",&cache,sizeof(cache))==sizeof(cache);
  p.end();
  if(!ok || cache.magic!=0x57454131 || cache.zip!=98074
    || cache.checksum!=checksum(reinterpret_cast<const uint8_t*>(&cache),offsetof(Cache,checksum)) || !validDays(cache.data))return false;
  s=cache.data;s.loading=false;s.lastFetchOk=false;s.valid=true;s.revision=1;return true;
}
bool fetch(Snapshot& result) {
  WiFiClientSecure tls;HTTPClient http;
  tls.setCACert(weatherRootCA);tls.setHandshakeTimeout(8);
  http.setConnectTimeout(5000);http.setTimeout(5000);http.setReuse(false);
  // HTTP/1.0 prevents chunked transfer encoding when parsing the response stream.
  http.useHTTP10(true);
  if(!http.begin(tls,url))return false;
  http.addHeader("User-Agent","MacroDesk-98074/1.0");
  const int code=http.GET();
  if(code!=HTTP_CODE_OK) {
    char detail[128]={};const int tlsError=tls.lastError(detail,sizeof(detail));
    Serial.printf("[WEATHER] HTTP error %d TLS=%d %s\n",code,tlsError,detail);
    http.end();tls.stop();return false;
  }
  const int length=http.getSize();
  if(length>4096) {Serial.println("[WEATHER] response too large");http.end();tls.stop();return false;}
  JsonDocument filter;filter["utc_offset_seconds"]=true;
  const char* fields[]={"time","weather_code","temperature_2m_max","temperature_2m_min","precipitation_probability_max"};
  for(const char* field:fields)filter["daily"][field]=true;
  JsonDocument doc;
  const auto error=deserializeJson(doc,http.getStream(),DeserializationOption::Filter(filter),DeserializationOption::NestingLimit(5));
  http.end();tls.stop();
  if(error) {Serial.printf("[WEATHER] JSON error %s\n",error.c_str());return false;}
  if(!doc["utc_offset_seconds"].is<int32_t>())return false;
  result.utcOffset=doc["utc_offset_seconds"].as<int32_t>();
  const JsonObjectConst daily=doc["daily"].as<JsonObjectConst>();
  for(const char* field:fields)if(daily[field].as<JsonArrayConst>().size()!=3)return false;
  for(unsigned i=0;i<3;++i) {
    const char* date=daily["time"][i].as<const char*>();
    if(!date || strlen(date)!=10)return false;
    for(unsigned j=1;j<5;++j)if(!daily[fields[j]][i].is<float>())return false;
    const int codeValue=daily["weather_code"][i].as<int>(),rain=daily["precipitation_probability_max"][i].as<int>();
    if(codeValue<0 || codeValue>99 || rain<0 || rain>100)return false;
    auto& d=result.days[i];strlcpy(d.date,date,sizeof(d.date));
    d.high=daily["temperature_2m_max"][i].as<float>();d.low=daily["temperature_2m_min"][i].as<float>();
    d.code=codeValue;d.rain=rain;
  }
  result.fetchedAt=uint32_t(time(nullptr));
  return validDays(result);
}
}
bool begin() {
  if(started)return true;
  Snapshot cached{};if(load(cached)) {shared=cached;Serial.println("[WEATHER] saved forecast loaded (cached)");}
  started=true;return true;
}
bool due() {
  if(!started || WiFi.status()!=WL_CONNECTED || time(nullptr)<1700000000 || FeitCloud::state().busy)return false;
  const uint32_t now=millis();bool wanted;uint32_t activity;Snapshot current;
  portENTER_CRITICAL(&mux);wanted=requested;activity=lastActivity;current=shared;portEXIT_CRITICAL(&mux);
  const int64_t localNow=int64_t(time(nullptr))+current.utcOffset;
  const int64_t localFetched=int64_t(current.fetchedAt)+current.utcOffset;
  const bool dateChanged=current.valid && localNow/86400!=localFetched/86400;
  const uint32_t gap=current.lastFetchOk && !dateChanged?interval:retryInterval;
  return !((!wanted && attempted && uint32_t(now-attemptedAt)<gap) || uint32_t(now-activity)<5000
    || (wanted && attempted && uint32_t(now-attemptedAt)<15000));
}
void poll() {
  if(!due())return;
  // The caller has closed its idle cloud TLS connection, avoiding concurrent TLS heaps.
  if(ESP.getFreeHeap()<76000 || heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)<22000)return;
  const Snapshot current=snapshot();Snapshot next=current;
  portENTER_CRITICAL(&mux);requested=false;shared.loading=true;++shared.revision;portEXIT_CRITICAL(&mux);
  attemptedAt=millis();attempted=true;
  const uint32_t fetchStarted=millis();const bool ok=fetch(next);
  if(!ok)next=current;
  next.loading=false;next.lastFetchOk=ok;next.valid=ok || current.valid;
  if(!ok)++next.failures;
  portENTER_CRITICAL(&mux);next.revision=shared.revision+1;shared=next;portEXIT_CRITICAL(&mux);
  if(ok)save(next);
  networkStackMinimum=uxTaskGetStackHighWaterMark(nullptr);
  Serial.printf("[WEATHER] %s 98074 elapsed=%lu ms heap=%u\n",ok?"UPDATED":"FAILED; retained cache",(unsigned long)(millis()-fetchStarted),ESP.getFreeHeap());
  if(ok)for(const auto& d:next.days)Serial.printf("[WEATHER] %s high=%.1fC low=%.1fC code=%u rain=%u%%\n",d.date,d.high,d.low,d.code,d.rain);
}
Snapshot snapshot() {portENTER_CRITICAL(&mux);const Snapshot result=shared;portEXIT_CRITICAL(&mux);return result;}
void refresh() {
  portENTER_CRITICAL(&mux);requested=true;portEXIT_CRITICAL(&mux);
  Serial.println("[WEATHER] refresh queued");
}
bool quiet() {portENTER_CRITICAL(&mux);uint32_t activity=lastActivity;portEXIT_CRITICAL(&mux);return uint32_t(millis()-activity)>=5000;}
void userActivity() {portENTER_CRITICAL(&mux);lastActivity=millis();portEXIT_CRITICAL(&mux);}
void status() {
  const auto s=snapshot();
  Serial.printf("[WEATHER] STATUS valid=%u loading=%u fresh=%u revision=%lu failures=%lu fetched=%lu heap=%u stack=%u\n",s.valid,s.loading,s.lastFetchOk,(unsigned long)s.revision,(unsigned long)s.failures,(unsigned long)s.fetchedAt,ESP.getFreeHeap(),networkStackMinimum);
  if(s.valid)for(const auto& d:s.days)Serial.printf("[WEATHER] %s high=%.1f low=%.1f %s rain=%u%%\n",d.date,d.high,d.low,description(d.code),d.rain);
}
const char* description(uint8_t code) {
  if(code==0)return "CLEAR";if(code==1)return "MOSTLY CLEAR";if(code==2)return "PART CLOUD";if(code==3)return "OVERCAST";
  if(code==45 || code==48)return "FOG";if(code>=51 && code<=57)return "DRIZZLE";
  if(code>=61 && code<=67)return "RAIN";if(code>=71 && code<=77)return "SNOW";
  if(code>=80 && code<=82)return "SHOWERS";if(code==85 || code==86)return "SNOW SHOWERS";
  if(code>=95)return "THUNDER";return "UNKNOWN";
}
}
