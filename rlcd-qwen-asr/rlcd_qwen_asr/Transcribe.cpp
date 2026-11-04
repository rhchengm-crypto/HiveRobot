#include "Transcribe.h"
#include "LocalConfig.h"
#include "Trust.h"
#include "MandarinPolicy.h"
#include "AnswerPolicy.h"
#include "UploadTransport.h"
#include "LosslessAudio.h"
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <mbedtls/base64.h>
#include <esp_heap_caps.h>
#include <time.h>
namespace {
bool processText(const String& transcript,String& text,String& error,bool askQuestion,UploadTLS& tls,HTTPClient& http) {
  const char* field=askQuestion?"answer":"mandarin";
  const char* label=askQuestion?"千问回答":"普通话转换";
  JsonDocument request;
  request["model"]=MandarinPolicy::model;request["stream"]=false;request["enable_thinking"]=false;
  request["temperature"]=0;request["max_tokens"]=1536;request["response_format"]["type"]="json_object";
  request["messages"][0]["role"]="system";request["messages"][0]["content"]=askQuestion?AnswerPolicy::prompt:MandarinPolicy::prompt;
  if(askQuestion){
    request["enable_search"]=true;
    request["search_options"]["forced_search"]=true;
    request["search_options"]["search_strategy"]="turbo";
  }
  JsonDocument data;data["transcript"]=transcript;
  if(askQuestion){time_t now=time(nullptr);struct tm utc;gmtime_r(&now,&utc);char stamp[32];strftime(stamp,sizeof(stamp),"%Y-%m-%dT%H:%M:%SZ",&utc);data["current_utc"]=stamp;}
  String input;serializeJson(data,input);
  request["messages"][1]["role"]="user";request["messages"][1]["content"]=input;
  String payload;serializeJson(request,payload);
  http.setConnectTimeout(12000);http.setTimeout(askQuestion?60000:45000);
  // HTTPClient::setTimeout accepts uint16_t milliseconds: 90000 wraps to
  // 24464. Keep the response wait below 65536 and bound retries separately.
  const uint32_t at=millis();int code=0;
  for(unsigned attempt=0;attempt<2;++attempt) {
    if(!http.begin(tls,ASR_ENDPOINT)){error=String(label)+"接口无效";return false;}
    http.addHeader("Authorization",String("Bearer ")+DASHSCOPE_API_KEY);http.addHeader("Content-Type","application/json");
    if(askQuestion)Serial.println("[SEARCH] enabled=1 forced=1 strategy=turbo");
    code=http.POST(payload);
    if(code>0)break;
    Serial.printf("[%s] attempt=%u HTTP=%d elapsed_ms=%lu\n",askQuestion?"ASK":"MANDARIN",attempt+1,code,(unsigned long)(millis()-at));
    http.end();tls.stop();
    if(attempt==0){Serial.println("[TEXT] retrying once after transport failure");vTaskDelay(pdMS_TO_TICKS(400));}
  }
  if(code!=200){error=String(label)+(code==HTTPC_ERROR_READ_TIMEOUT?"等待超时，请再试":"失败 HTTP ");if(code!=HTTPC_ERROR_READ_TIMEOUT)error+=code;Serial.printf("[%s] HTTP=%d\n",askQuestion?"ASK":"MANDARIN",code);http.end();return false;}
  JsonDocument filter;filter["choices"][0]["message"]["content"]=true;filter["choices"][0]["finish_reason"]=true;
  JsonDocument answer;const String response=http.getString();http.end();tls.stop();
  if(deserializeJson(answer,response,DeserializationOption::Filter(filter),DeserializationOption::NestingLimit(12))){error=String(label)+"响应解析失败";return false;}
  if(answer["choices"][0]["finish_reason"]=="length"){error=String(label)+"过长，请缩短问题重试";return false;}
  const String content=answer["choices"][0]["message"]["content"].as<String>();
  JsonDocument result;
  if(deserializeJson(result,content) || !result[field].is<const char*>()){error=String(label)+"结果格式错误";return false;}
  text=result[field].as<String>();text.trim();
  if(text.isEmpty()){error=String(label)+"为空，请重试";return false;}
  if(askQuestion){
    text=String("问：")+transcript+"\n\n答："+text;
    if(text.length()>=4096){error="问答结果过长，请缩短问题";return false;}
  }
  Serial.printf("[%s] HTTP=200 input_bytes=%u text_bytes=%u elapsed_ms=%lu\n",askQuestion?"ASK":"MANDARIN",unsigned(transcript.length()),unsigned(text.length()),(unsigned long)(millis()-at));return true;
}
}
bool transcribe(const uint8_t* wav,size_t bytes,String& text,String& error,bool askQuestion,void (*recognized)()) {
  const uint32_t encodeAt=millis();size_t compressedBytes=0;
  uint8_t* compressed=bytes>=46?LosslessAudio::encode(wav+44,(bytes-44)/2,compressedBytes):nullptr;
  const bool useFlac=compressed && compressedBytes<bytes;
  const uint8_t* audio=useFlac?compressed:wav;const size_t audioBytes=useFlac?compressedBytes:bytes;
  Serial.printf("[AUDIO] format=%s pcm_bytes=%u upload_audio_bytes=%u encode_ms=%lu\n",useFlac?"flac":"wav",unsigned(bytes),unsigned(audioBytes),(unsigned long)(millis()-encodeAt));
  const String prefix=String("{\"model\":\"")+ASR_MODEL+"\",\"messages\":[{\"role\":\"user\",\"content\":[{\"type\":\"input_audio\",\"input_audio\":{\"data\":\"data:audio/"+(useFlac?"flac":"wav")+";base64,";
  const char suffix[]="\"}}]}],\"stream\":false,\"asr_options\":{\"language\":\"zh\",\"enable_itn\":false}}";
  const size_t encoded=4*((audioBytes+2)/3),total=prefix.length()+encoded+strlen(suffix);
  auto* body=static_cast<uint8_t*>(heap_caps_malloc(total+1,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
  if(!body){free(compressed);error="请求缓冲内存不足";return false;}
  memcpy(body,prefix.c_str(),prefix.length());size_t written=0;
  const int result=mbedtls_base64_encode(body+prefix.length(),encoded+1,&written,audio,audioBytes);
  free(compressed);
  if(result || written!=encoded){free(body);error="录音编码失败";return false;}
  memcpy(body+prefix.length()+written,suffix,sizeof(suffix));
  UploadTLS tls;tls.setCACert(asrRootCA);tls.setHandshakeTimeout(15);
  HTTPClient http;http.setConnectTimeout(20000);http.setTimeout(60000);http.setReuse(true);http.useHTTP10(false);
  const uint32_t at=millis();int code=0;
  for(unsigned attempt=0;attempt<2;++attempt) {
    if(!http.begin(tls,ASR_ENDPOINT)){free(body);error="接口地址无效";return false;}
    http.addHeader("Authorization",String("Bearer ")+DASHSCOPE_API_KEY);http.addHeader("Content-Type","application/json");
    code=http.POST(body,total);
    Serial.printf("[NET] stage=asr attempt=%u request_bytes=%u tls_connect_ms=%lu tls_write_ms=%lu writes=%u headers_ms=%lu\n",attempt+1,unsigned(total),(unsigned long)tls.connectMs,(unsigned long)tls.writeMs,unsigned(tls.writeCalls),(unsigned long)(millis()-at));
    if(code>0)break;
    char tlsError[96]={};const int tlsCode=tls.lastError(tlsError,sizeof(tlsError));
    Serial.printf("[ASR] upload attempt=%u HTTP=%d TLS=%d %s elapsed_ms=%lu\n",attempt+1,code,tlsCode,tlsError,(unsigned long)(millis()-at));
    http.end();tls.stop();
    if(attempt==0){Serial.println("[ASR] retrying same recording once after transport failure");vTaskDelay(pdMS_TO_TICKS(400));}
  }
  free(body);
  if(code!=200){error=code==HTTPC_ERROR_READ_TIMEOUT?String("语音识别等待超时，请再试"):String("识别请求失败 HTTP ")+code;http.end();return false;}
  // Retain only the required text to bound JSON memory use.
  JsonDocument filter;filter["choices"][0]["message"]["content"]=true;
  JsonDocument answer;
  const String response=http.getString();
  const auto parse=deserializeJson(answer,response,DeserializationOption::Filter(filter),DeserializationOption::NestingLimit(12));
  http.end();
  if(parse){error="识别响应解析失败";return false;}
  String transcript=answer["choices"][0]["message"]["content"].as<String>();transcript.trim();
  if(transcript.isEmpty()){error="没有识别到文字，请靠近再试";return false;}
  Serial.printf("[ASR] HTTP=200 text_bytes=%u elapsed_ms=%lu\n",unsigned(transcript.length()),(unsigned long)(millis()-at));
  if(recognized)recognized();
  Serial.printf("[NET] text_reuse_available=%u\n",tls.connected());
  return processText(transcript,text,error,askQuestion,tls,http);
}
bool standardMandarin(const String& transcript,String& text,String& error){
  UploadTLS tls;tls.setCACert(asrRootCA);tls.setHandshakeTimeout(15);
  HTTPClient http;http.setReuse(false);http.useHTTP10(true);
  return processText(transcript,text,error,false,tls,http);
}
