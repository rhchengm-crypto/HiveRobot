#include "LocalWeather.h"
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <math.h>
namespace LocalWeather {
namespace {
class WeatherTLS:public WiFiClientSecure {
 public:WeatherTLS(){_use_ca_bundle=true;attach_ssl_certificate_bundle(sslclient.get(),true);setHandshakeTimeout(8);}
};
void copy(char* dst,const String& value,size_t capacity){size_t n=min(value.length(),capacity-1);while(n&&(uint8_t(value[n])&0xc0)==0x80)--n;memcpy(dst,value.c_str(),n);dst[n]=0;}
bool get(const String& url,JsonDocument& d,String& error){
 WeatherTLS tls;HTTPClient http;http.setConnectTimeout(7000);http.setTimeout(8000);http.setReuse(false);http.useHTTP10(true);
 if(!http.begin(tls,url)){error="天气接口初始化失败";return false;}
 http.addHeader("User-Agent","RLCD-LocalWeather/1.0");const int code=http.GET();
 if(code!=200){char detail[128]={};const int tlsCode=tls.lastError(detail,sizeof(detail));Serial.printf("[WEATHER_HTTP] status=%d tls=%d detail=%s\n",code,tlsCode,detail);http.end();error=String("天气接口 HTTP ")+code;return false;}
 if(http.getSize()>16000){http.end();error="天气数据过大";return false;}
 String text=http.getString();http.end();tls.stop();if(text.length()>16000||deserializeJson(d,text)){error="天气数据解析失败";return false;}return true;
}
bool valid(const Snapshot& s){
 if(!s.city[0]||!s.ip[0]||!s.timezone[0]||!isfinite(s.lat)||!isfinite(s.lon)||abs(s.lat)>90||abs(s.lon)>180||!isfinite(s.temp)||s.temp< -100||s.temp>70||s.offset< -43200||s.offset>50400||s.fetched<1700000000)return false;
 for(unsigned i=0;i<3;++i){const auto& d=s.days[i];if(strlen(d.date)!=10||d.date[4]!='-'||d.date[7]!='-'||!isfinite(d.low)||!isfinite(d.high)||d.low>d.high||d.low< -100||d.high>70||d.code<0||d.code>99||d.rain<0||d.rain>100||(i&&strcmp(s.days[i-1].date,d.date)>=0))return false;}
 return s.code>=0&&s.code<=99;
}
void encode(const Snapshot& s,JsonDocument& d){
 d["version"]=1;d["city"]=s.city;d["region"]=s.region;d["country"]=s.country;d["ip"]=s.ip;d["timezone"]=s.timezone;d["abbr"]=s.abbr;d["observed"]=s.observed;d["lat"]=s.lat;d["lon"]=s.lon;d["temp"]=s.temp;d["offset"]=s.offset;d["code"]=s.code;d["fetched"]=int64_t(s.fetched);
 for(unsigned i=0;i<3;++i){auto day=d["days"][i];day["date"]=s.days[i].date;day["low"]=s.days[i].low;day["high"]=s.days[i].high;day["code"]=s.days[i].code;day["rain"]=s.days[i].rain;}
}
void save(const Snapshot& s){JsonDocument d;encode(s,d);String text;serializeJson(d,text);Preferences p;if(p.begin("rlcd-weather",false)){const bool ok=p.putString("snapshot",text)==text.length();p.end();Serial.printf("[WEATHER_CACHE] saved=%u\n",ok);}}
}
bool load(Snapshot& result){
 Preferences p;if(!p.begin("rlcd-weather",true))return false;String raw=p.getString("snapshot","");p.end();JsonDocument d;if(raw.length()>6000||deserializeJson(d,raw)||d["version"]!=1)return false;
 Snapshot s;copy(s.city,d["city"]|"",sizeof(s.city));copy(s.region,d["region"]|"",sizeof(s.region));copy(s.country,d["country"]|"",sizeof(s.country));copy(s.ip,d["ip"]|"",sizeof(s.ip));copy(s.timezone,d["timezone"]|"",sizeof(s.timezone));copy(s.abbr,d["abbr"]|"",sizeof(s.abbr));copy(s.observed,d["observed"]|"",sizeof(s.observed));
 s.lat=d["lat"]|NAN;s.lon=d["lon"]|NAN;s.temp=d["temp"]|NAN;s.offset=d["offset"]|999999;s.code=d["code"]|-1;s.fetched=d["fetched"]|0LL;
 if(d["days"].size()!=3)return false;
 for(unsigned i=0;i<3;++i){auto day=d["days"][i];copy(s.days[i].date,day["date"]|"",sizeof(s.days[i].date));s.days[i].low=day["low"]|NAN;s.days[i].high=day["high"]|NAN;s.days[i].code=day["code"]|-1;s.days[i].rain=day["rain"]|-1;}
 if(!valid(s))return false;s.valid=true;s.verified=false;result=s;return true;
}
bool fetch(const Snapshot& previous,Snapshot& result,bool force,String& error){
 JsonDocument geo;if(!get("https://ipwho.is/",geo,error))return false;
 if(!(geo["success"]|false)){error="IP定位失败，稍后重试";return false;}
 Snapshot s=previous;copy(s.city,geo["city"]|"",sizeof(s.city));copy(s.region,geo["region"]|"",sizeof(s.region));copy(s.country,geo["country_code"]|"",sizeof(s.country));copy(s.ip,geo["ip"]|"",sizeof(s.ip));copy(s.timezone,geo["timezone"]["id"]|"",sizeof(s.timezone));copy(s.abbr,geo["timezone"]["abbr"]|"LOC",sizeof(s.abbr));s.offset=geo["timezone"]["offset"]|999999;s.lat=geo["latitude"]|NAN;s.lon=geo["longitude"]|NAN;
 if(!s.city[0]||!s.timezone[0]||!isfinite(s.lat)||!isfinite(s.lon)||abs(s.lat)>90||abs(s.lon)>180||s.offset< -43200||s.offset>50400){error="IP定位数据无效";return false;}
 const time_t now=time(nullptr);s.geoChecked=now;
 const time_t cityNow=now+s.offset;struct tm cityTime{};gmtime_r(&cityNow,&cityTime);char today[11];strftime(today,sizeof(today),"%Y-%m-%d",&cityTime);
 const bool moved=strcmp(s.ip,previous.ip)||strcmp(s.city,previous.city)||strcmp(s.timezone,previous.timezone)||fabs(s.lat-previous.lat)>.05||fabs(s.lon-previous.lon)>.05;
 if(!force&&previous.valid&&!moved&&now-previous.fetched<900&&!strcmp(today,previous.days[0].date)){
  s.valid=s.verified=true;result=s;Serial.println("[WEATHER_GEO] checked=1 changed=0 forecast_cached=1");return true;
 }
 String url=String("https://api.open-meteo.com/v1/forecast?latitude=")+String(s.lat,4)+"&longitude="+String(s.lon,4)+"&current=temperature_2m,weather_code&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max&forecast_days=3&timezone=auto";
 JsonDocument d;if(!get(url,d,error))return false;auto daily=d["daily"];if(daily["time"].size()!=3){error="三天天气数据不完整";return false;}
 if(d["current"]["temperature_2m"].isNull()||d["current"]["weather_code"].isNull()){error="当前天气数据不完整";return false;}
 s.temp=d["current"]["temperature_2m"];s.code=d["current"]["weather_code"];
 copy(s.observed,d["current"]["time"]|"",sizeof(s.observed));
 for(unsigned i=0;i<3;++i){auto& day=s.days[i];copy(day.date,daily["time"][i]|"",sizeof(day.date));day.low=daily["temperature_2m_min"][i]|NAN;day.high=daily["temperature_2m_max"][i]|NAN;day.code=daily["weather_code"][i]|-1;day.rain=daily["precipitation_probability_max"][i]|-1;}
 s.fetched=now;if(!valid(s)||strcmp(s.days[0].date,today)){error="天气日期或数值无效";return false;}
 s.valid=s.verified=true;result=s;save(s);
 Serial.printf("[WEATHER_FETCH] ok=1 city=%s country=%s timezone=%s moved=%u current=%.1f first=%s days=3\n",s.city,s.country,s.timezone,moved,s.temp,s.days[0].date);return true;
}
const char* condition(int c){
 if(c==0)return "晴";if(c==1)return "晴间多云";if(c==2)return "多云";if(c==3)return "阴";if(c==45||c==48)return "雾";
 if(c>=51&&c<=57)return "毛毛雨";if(c>=61&&c<=67)return "雨";if(c>=71&&c<=77)return "雪";if(c>=80&&c<=82)return "阵雨";if(c==85||c==86)return "阵雪";if(c>=95)return "雷雨";return "未知";
}
void applyTimezone(const Snapshot& s){
 // Preserve exact DST rules for common zones. Other zones use the returned
 // current offset, refreshed by the periodic IP/timezone lookup.
 const char* rule=nullptr;
 if(!strcmp(s.timezone,"America/Los_Angeles")||!strcmp(s.timezone,"America/Vancouver"))rule="PST8PDT,M3.2.0,M11.1.0";
 else if(!strcmp(s.timezone,"America/New_York")||!strcmp(s.timezone,"America/Toronto"))rule="EST5EDT,M3.2.0,M11.1.0";
 else if(!strcmp(s.timezone,"America/Chicago"))rule="CST6CDT,M3.2.0,M11.1.0";
 else if(!strcmp(s.timezone,"America/Denver"))rule="MST7MDT,M3.2.0,M11.1.0";
 else if(!strcmp(s.timezone,"Asia/Shanghai")||!strcmp(s.timezone,"Asia/Taipei"))rule="CST-8";
 else if(!strcmp(s.timezone,"Asia/Tokyo"))rule="JST-9";
 else if(!strcmp(s.timezone,"Europe/London"))rule="GMT0BST,M3.5.0/1,M10.5.0";
 char fallback[48];if(!rule){char name[10];unsigned n=0;for(unsigned i=0;s.abbr[i]&&n<8;++i)if(isalpha(s.abbr[i]))name[n++]=s.abbr[i];name[n]=0;if(n<3)strcpy(name,"LOC");int seconds=abs(s.offset);snprintf(fallback,sizeof(fallback),"%s%s%d:%02d:%02d",name,s.offset>=0?"-":"+",seconds/3600,(seconds%3600)/60,seconds%60);rule=fallback;}
 static String last;if(last==rule)return;last=rule;setenv("TZ",rule,1);tzset();Serial.printf("[WEATHER_TZ] zone=%s rule=%s\n",s.timezone,rule);
}
}
