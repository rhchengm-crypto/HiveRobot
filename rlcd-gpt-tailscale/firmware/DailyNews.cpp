#include "DailyNews.h"
#include "NewsPolicy.h"
#include "LocalConfig.h"
#include "Trust.h"
#include "UploadTransport.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
namespace DailyNews {
int editionDay(time_t now){
 struct tm local;localtime_r(&now,&local);
 if(local.tm_hour<7){--local.tm_mday;local.tm_hour=12;local.tm_isdst=-1;mktime(&local);}
 return (local.tm_year+1900)*10000+(local.tm_mon+1)*100+local.tm_mday;
}
bool load(Bulletin& n){
 Preferences p;if(!p.begin("rlcd-news",true))return false;String value=p.getString("bulletin","");p.end();
 JsonDocument d;if(value.isEmpty() || value.length()>12000 || deserializeJson(d,value))return false;
 Bulletin candidate;candidate.text=d["text"].as<String>();candidate.links=d["links"].as<String>();candidate.day=d["day"]|0;candidate.count=d["count"]|0;candidate.fetched=d["fetched"]|0LL;
 if(candidate.day<=20200000 || candidate.count<1 || candidate.count>10 || candidate.text.isEmpty() || candidate.text.length()>=4096)return false;
 n=candidate;return true;
}
bool save(const Bulletin& n){
 JsonDocument d;d["text"]=n.text;d["links"]=n.links;d["day"]=n.day;d["count"]=n.count;d["fetched"]=int64_t(n.fetched);
 String value;serializeJson(d,value);if(value.length()>12000)return false;
 Preferences p;if(!p.begin("rlcd-news",false))return false;const bool ok=p.putString("bulletin",value)==value.length();p.end();return ok;
}
bool fetchPart(int day,Bulletin& n,String& error){
 String endpoint=ASR_ENDPOINT;const int path=endpoint.indexOf("/compatible-mode/");
 if(path<0){error="新闻接口地址无效";return false;}
 endpoint=endpoint.substring(0,path)+"/api/v1/services/aigc/multimodal-generation/generation";
 const time_t now=time(nullptr);struct tm utc;gmtime_r(&now,&utc);char stamp[32];strftime(stamp,sizeof(stamp),"%Y-%m-%dT%H:%M:%SZ",&utc);
 JsonDocument request;request["model"]=NewsPolicy::model;
 request["input"]["messages"][0]["role"]="system";request["input"]["messages"][0]["content"][0]["text"]=NewsPolicy::prompt;
 JsonDocument user;user["current_utc"]=stamp;user["edition_day"]=day;user["request"]="请联网整理最新的10条热门社会新闻，返回JSON。";if(n.count){user["existing_items"]=n.text;user["request"]="请扩大到国际社会民生新闻，重点检索教育、医疗、公共安全、司法和自然灾害的最新事件；排除existing_items已有事件，提供10条新条目供筛选。返回JSON。";}String input;serializeJson(user,input);
 request["input"]["messages"][1]["role"]="user";request["input"]["messages"][1]["content"][0]["text"]=input;
 auto p=request["parameters"];p["result_format"]="message";p["enable_thinking"]=false;p["enable_search"]=true;p["temperature"]=0;p["max_tokens"]=3500;p["response_format"]["type"]="json_object";
 p["search_options"]["forced_search"]=true;p["search_options"]["search_strategy"]="turbo";p["search_options"]["freshness"]=7;p["search_options"]["enable_source"]=true;
 String payload;serializeJson(request,payload);UploadTLS tls;tls.setCACert(asrRootCA);tls.setHandshakeTimeout(15);
 HTTPClient http;http.setConnectTimeout(15000);http.setTimeout(60000);http.setReuse(false);http.useHTTP10(true);
 const uint32_t at=millis();int code=0;Serial.printf("[NEWS_MODEL] model=%s\n",NewsPolicy::model);
 for(unsigned attempt=0;attempt<2;++attempt){
  if(!http.begin(tls,endpoint)){error="新闻接口初始化失败";return false;}
  http.addHeader("Authorization",String("Bearer ")+DASHSCOPE_API_KEY);http.addHeader("Content-Type","application/json");code=http.POST(payload);
  if(code>0)break;http.end();tls.stop();if(!attempt)vTaskDelay(pdMS_TO_TICKS(400));
 }
 if(code!=200){error=String("新闻更新失败 HTTP ")+code;http.end();return false;}
 JsonDocument filter;filter["output"]["choices"][0]["message"]["content"]=true;filter["output"]["choices"][0]["finish_reason"]=true;
 filter["output"]["search_info"]["search_results"][0]["index"]=true;filter["output"]["search_info"]["search_results"][0]["site_name"]=true;filter["output"]["search_info"]["search_results"][0]["url"]=true;
 filter["usage"]["plugins"]["search"]["count"]=true;
 JsonDocument response;const String raw=http.getString();http.end();tls.stop();
 if(deserializeJson(response,raw,DeserializationOption::Filter(filter))){error="新闻响应解析失败";return false;}
 if(response["output"]["choices"][0]["finish_reason"]=="length"){error="新闻结果过长，稍后重试";return false;}
 JsonVariantConst messageContent=response["output"]["choices"][0]["message"]["content"];
 String contentText;
 if(messageContent.is<const char*>())contentText=messageContent.as<String>();
 else for(JsonObjectConst part:messageContent.as<JsonArrayConst>())if(part["text"].is<const char*>())contentText+=part["text"].as<String>();
 JsonDocument content;if(deserializeJson(content,contentText)){error="新闻格式错误";return false;}
 JsonArrayConst sources=response["output"]["search_info"]["search_results"].as<JsonArrayConst>();
 if(sources.isNull() || sources.size()==0){error="新闻未返回检索来源";return false;}
 n.day=day;n.fetched=now;
 struct tm local;localtime_r(&now,&local);char label[40];strftime(label,sizeof(label),"%m-%d %H:%M",&local);
 String body=n.text;if(body.isEmpty())body=String("更新：")+label+" 当地时间\n社会热点精选，非官方榜单\n\n";String seen="|";
 Serial.printf("[NEWS_ITEMS] received=%u existing=%d\n",unsigned(content["items"].size()),n.count);
 for(JsonObjectConst item:content["items"].as<JsonArrayConst>()){
  if(n.count==10)break;
  String title=item["title"].as<String>(),summary=item["summary"].as<String>(),date=item["date"].as<String>();
  const int index=item["source_index"]|0;String site,url;
  for(JsonObjectConst source:sources)if((source["index"]|0)==index){site=source["site_name"].as<String>();url=source["url"].as<String>();break;}
  if(title.isEmpty() || title.length()>100 || summary.isEmpty() || summary.length()>140 || date.length()!=10 || (!url.startsWith("https://") && !url.startsWith("http://")) || (seen.indexOf("|"+title+"|")>=0 || n.text.indexOf(title)>=0)){Serial.println("[NEWS_SKIP] fields_source_or_duplicate");continue;}
  // Reject malformed, future or clearly old dates. Calendar day tolerance
  // allows Chinese publication dates across the US/China timezone boundary.
  int y,m,d;if(sscanf(date.c_str(),"%d-%d-%d",&y,&m,&d)!=3 || m<1 || m>12 || d<1 || d>31)continue;
  struct tm published{};published.tm_year=y-1900;published.tm_mon=m-1;published.tm_mday=d;published.tm_hour=12;published.tm_isdst=-1;
  const time_t when=mktime(&published);if(when>now+86400 || when<now-3*86400){Serial.println("[NEWS_SKIP] date_range");continue;}
  if(site.isEmpty())site="检索来源";if(site.length()>60)site="检索来源";
  const String entry=String(n.count+1)+". "+title+"\n"+summary+"\n"+date+" "+site+"\n\n";
  if(body.length()+entry.length()+140>=4096)break;
  body+=entry;seen+=title+"|";n.links+=String(n.count+1)+" "+url+"\n";++n.count;
 }
 if(!n.count){error="未找到有来源的近期社会新闻";return false;}

 n.text=body;
 Serial.printf("[NEWS_PASS] HTTP=200 day=%d count=%d sources=%u searches=%d bytes=%u elapsed_ms=%lu\n",day,n.count,unsigned(sources.size()),response["usage"]["plugins"]["search"]["count"]|0,unsigned(n.text.length()),(unsigned long)(millis()-at));return true;
}
bool fetch(int day,Bulletin& n,String& error){
 const uint32_t at=millis();n=Bulletin{};
 if(!fetchPart(day,n,error))return false;
 unsigned passes=1;
 if(n.count<10){++passes;String supplementError;if(!fetchPart(day,n,supplementError))Serial.printf("[NEWS_SUPPLEMENT] %s\n",supplementError.c_str());}
 if(n.count<10)n.text+=String("本次可靠结果仅 ")+n.count+" 条，不凑数。";
 Serial.printf("[NEWS] HTTP=200 day=%d count=%d passes=%u bytes=%u elapsed_ms=%lu\n",day,n.count,passes,unsigned(n.text.length()),(unsigned long)(millis()-at));return true;
}
}
