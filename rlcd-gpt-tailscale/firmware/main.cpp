#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <atomic>
#include <esp_heap_caps.h>
#include <freertos/idf_additions.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include "NetworkConfig.h"
#include "ST7305_U8g2.h"
#include "Microphone.h"
#include "Transcribe.h"
#include "AnswerPolicy.h"
#include "LosslessAudio.h"
#include "LocalConfig.h"
#include "DailyNews.h"
#include "LocalWeather.h"
#include "RealtimeText.h"
#include "ExternalKeys.h"
#include "GptGateway.h"
#include "History.h"
#include "Battery.h"
#include "NewsSpeech.h"

constexpr uint8_t recordButtonPin=0,pageButtonPin=18;
constexpr unsigned sampleRate=16000,maxSeconds=20;
constexpr size_t wavCapacity=44+sampleRate*2*maxSeconds;
ST7305_U8g2 lcd(11,12,5,40,41);
U8G2* screen=nullptr;
enum class Mode:uint8_t {Setup,Ready,Recording,Recognizing,Result,Failure,Streaming,StreamEnding};
portMUX_TYPE stateLock=portMUX_INITIALIZER_UNLOCKED;
struct Shared {Mode mode=Mode::Setup;char status[128]="正在启动";char text[4096]={};uint32_t revision=1,recorded=0;uint16_t peak=0;bool stop=false;};
Shared shared;
bool micReady=false;size_t page=0;bool followLive=false;
// Changed only while idle. The worker captures the selected mode at start.
bool askQuestion=false,newsPage=false,weatherPage=false,gptPage=false;
bool menuHome=false;uint8_t selectedCard=0;
int deferredFunction=-1;
bool deferredSleep=false,deferredAction=false,deferredPortal=false;
uint32_t deferredSleepTest=0;
Shared savedApps[3];bool hasSavedApp[3]={false,false,false};size_t savedPages[3]={0,0,0};
DailyNews::Bulletin newsCache;
Shared beforeNews,beforeWeather;
LocalWeather::Snapshot weatherCache;
bool weatherAttempted=false;std::atomic<bool> weatherPending{true};uint32_t weatherAttempt=0;
char weatherError[128]={};
int newsTarget=0;uint32_t newsAttempt=0;bool newsAttempted=false;
// Clip at a UTF-8 codepoint boundary; cloud text is not ASCII.
void copyText(char* dst,const char* src,size_t capacity);
void put16(uint8_t* p,uint16_t n);
void put32(uint8_t* p,uint32_t n);
void wavHeader(uint8_t* p,size_t bytes);
void recordTask(void*);
void showNews();
void newsTask(void*);
void newsTick();
bool navigationIdle();
void showWeather();
void weatherTask(void* force);
void weatherTick();
void enterCard();
void openFunction(uint8_t index);
void returnToCards();
void cycleMode();
void streamState(const char* message);
void streamText(const char* sentence);
void streamFinished(const char* error);
void keyPress();
void microphoneTest(void*);
void drawBold(int x,int y,const char* text);
void drawLargeBold(int x,int y,const char* text);
size_t nextChar(const char* p,size_t i);
size_t textPage(const char* text,size_t start,bool draw);
size_t pageCount(const char* text);
void advancePage();
void paginationTest();
void flacTest();
void paintKeyTest();
bool systemIdle();
void paintPortal();
void paintClock();
void paintTabs();
void paintWeather(bool updating);
void paint();
void rxPoll();
void setup();
void loop();
void copyText(char* dst,const char* src,size_t capacity) {
  size_t n=min(strlen(src),capacity-1);while(n && (uint8_t(src[n])&0xc0)==0x80)--n;memcpy(dst,src,n);dst[n]=0;
}
void status(Mode mode,const char* value,const char* text=nullptr) {
  if(mode==Mode::Failure)Serial.printf("[FAILURE] %s\n",value);
  portENTER_CRITICAL(&stateLock);shared.mode=mode;copyText(shared.status,value,sizeof(shared.status));
  if(text)copyText(shared.text,text,sizeof(shared.text));++shared.revision;portEXIT_CRITICAL(&stateLock);
}
void put16(uint8_t* p,uint16_t n){p[0]=n;p[1]=n>>8;}
void put32(uint8_t* p,uint32_t n){for(int i=0;i<4;++i)p[i]=n>>(8*i);}
void wavHeader(uint8_t* p,size_t bytes) {
  memcpy(p,"RIFF",4);put32(p+4,36+bytes);memcpy(p+8,"WAVEfmt ",8);put32(p+16,16);put16(p+20,1);put16(p+22,1);
  put32(p+24,sampleRate);put32(p+28,sampleRate*2);put16(p+32,2);put16(p+34,16);memcpy(p+36,"data",4);put32(p+40,bytes);
}
void recordTask(void*) {
  const bool ask=askQuestion;const bool useGpt=gptPage;
  const auto deleteSelf=[useGpt](){if(useGpt)vTaskDeleteWithCaps(nullptr);else vTaskDelete(nullptr);};
  auto* wav=static_cast<uint8_t*>(heap_caps_malloc(wavCapacity,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
  if(!wav){status(Mode::Failure,"PSRAM 录音内存不足");deleteSelf();return;}
  int16_t stereo[640*2];size_t count=0;int64_t energy=0;uint32_t clipped=0;bool ok=true;
  // Drop the idle DMA tail before accepting this recording.
  for(int i=0;i<4;++i)if(Microphone::read(stereo,640)<0){ok=false;break;}
  int32_t previousInput=0,previousOutput=0;
  while(ok && count<sampleRate*maxSeconds) {
    portENTER_CRITICAL(&stateLock);const bool stop=shared.stop;portEXIT_CRITICAL(&stateLock);if(stop)break;
    int frames=Microphone::read(stereo,640);if(frames<0){ok=false;break;}
    uint16_t peak=0;
    for(int i=0;i<frames && count<sampleRate*maxSeconds;++i) {
      // Use MIC1; averaging two microphones can cancel distant voice phases.
      const int32_t x=stereo[i*2];int32_t y=x-previousInput+(previousOutput*32604)/32768;
      previousInput=x;previousOutput=y;y=constrain(y,-32768,32767);
      peak=max(peak,uint16_t(abs(y)));energy+=int64_t(y)*y;if(abs(y)>32000)++clipped;
      put16(wav+44+count*2,uint16_t(int16_t(y)));++count;
    }
    portENTER_CRITICAL(&stateLock);shared.recorded=count;shared.peak=peak;++shared.revision;portEXIT_CRITICAL(&stateLock);
  }
  if(!ok)status(Mode::Failure,"麦克风读取失败，请重启检查");
  else if(count<sampleRate/3)status(Mode::Failure,"录音太短，请再说一次");
  else if(energy/int64_t(count)<64)status(Mode::Failure,"录音太轻，请靠近麦克风");
  else {
    wavHeader(wav,count*2);status(Mode::Recognizing,"正在识别，请稍候");
    Serial.printf("[MIC] samples=%u rms=%u clipped=%lu\n",unsigned(count),unsigned(sqrt(double(energy)/count)),(unsigned long)clipped);
    String result,error;
    if(WiFi.status()!=WL_CONNECTED)status(Mode::Failure,"Wi-Fi 已断开，请重新连接");
    else {
      const auto oldSleep=WiFi.getSleep();WiFi.setSleep(false);
      const bool done=useGpt?GptGateway::ask(wav,44+count*2,result,error):transcribe(wav,44+count*2,result,error,ask,[]{status(Mode::Recognizing,askQuestion?"正在联网查询并回答":"正在转换成普通话");});
      WiFi.setSleep(oldSleep);
      if(done){if(!useGpt)History::append("qwen",result.c_str());status(Mode::Result,useGpt?"GPT回答 · 家庭网络":ask?"千问回答":"普通话转换完成",result.c_str());}else status(Mode::Failure,error.c_str());
    }
  }
  free(wav);Serial.printf("[TASK] record stack_free=%u heap=%u\n",unsigned(uxTaskGetStackHighWaterMark(nullptr)),ESP.getFreeHeap());deleteSelf();
}
void showNews() {
  page=0;followLive=false;
  if(newsCache.count){String label=String("社会新闻 · ")+newsCache.count+" 条";status(Mode::Result,label.c_str(),newsCache.text.c_str());}
  else status(Mode::Ready,"等待每日新闻自动生成","");
}
void newsTask(void*) {
  DailyNews::Bulletin fresh;String error;const auto oldSleep=WiFi.getSleep();WiFi.setSleep(false);
  const bool ok=DailyNews::fetch(newsTarget,fresh,error);WiFi.setSleep(oldSleep);
  if(ok){
    const bool saved=DailyNews::save(fresh);newsCache=fresh;
    Serial.printf("[NEWS_CACHE] saved=%u day=%d count=%d\n",saved,newsCache.day,newsCache.count);
    if(!saved)Serial.println("[NEWS_ERROR] persistent cache write failed; RAM copy retained");
  }else Serial.printf("[NEWS_ERROR] %s\n",error.c_str());
  if(newsPage){
    if(ok)showNews();
    else if(newsCache.count)status(Mode::Result,"更新失败，仍显示缓存",newsCache.text.c_str());
    else status(Mode::Failure,error.c_str(),"");
  }else status(beforeNews.mode,beforeNews.status,beforeNews.text);
  Serial.printf("[NEWS_TASK] ok=%u stack_free=%u heap=%u\n",ok,unsigned(uxTaskGetStackHighWaterMark(nullptr)),ESP.getFreeHeap());vTaskDelete(nullptr);
}
void startNews(bool manual=false) {
  if(NewsSpeech::active())return;
  portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
  if(!idle || WiFi.status()!=WL_CONNECTED || time(nullptr)<1700000000 || !DASHSCOPE_API_KEY[0])return;
  if(manual && newsAttempted && millis()-newsAttempt<60000)return;
  newsAttempted=true;newsAttempt=millis();newsTarget=DailyNews::editionDay(time(nullptr));
  portENTER_CRITICAL(&stateLock);beforeNews=shared;portEXIT_CRITICAL(&stateLock);
  status(Mode::Recognizing,"正在预生成每日社会新闻");
  Serial.printf("[NEWS_START] manual=%u target=%d\n",manual,newsTarget);
  if(xTaskCreatePinnedToCore(newsTask,"daily-news",12*1024,nullptr,2,nullptr,0)!=pdPASS)status(beforeNews.mode,beforeNews.status,beforeNews.text);
}
void newsTick() {
  static uint32_t checked=0;if(millis()-checked<1000)return;checked=millis();
  portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);if(!idle)return;
  if(time(nullptr)<1700000000 || newsCache.day==DailyNews::editionDay(time(nullptr)))return;
  if(newsAttempted && millis()-newsAttempt<15UL*60*1000)return;
  startNews();
}
bool navigationIdle(){
 portENTER_CRITICAL(&stateLock);bool idle=shared.mode==Mode::Ready||shared.mode==Mode::Result||shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
 return idle&&!RealtimeText::active()&&!GptGateway::stopping();
}
void showWeather(){
 page=0;followLive=false;status(Mode::Ready,weatherCache.valid?"本地三天天气":"等待按IP获取当地天气","");
}
void weatherTask(void* force){
 LocalWeather::Snapshot previous;
 portENTER_CRITICAL(&stateLock);previous=weatherCache;portEXIT_CRITICAL(&stateLock);
 LocalWeather::Snapshot fresh;String error;const bool oldSleep=WiFi.getSleep();WiFi.setSleep(false);
 const bool ok=LocalWeather::fetch(previous,fresh,force!=nullptr,error);WiFi.setSleep(oldSleep);
 portENTER_CRITICAL(&stateLock);if(ok)weatherCache=fresh;else weatherCache.verified=false;portEXIT_CRITICAL(&stateLock);
 portENTER_CRITICAL(&stateLock);copyText(weatherError,error.c_str(),sizeof(weatherError));portEXIT_CRITICAL(&stateLock);weatherPending=!ok;
 if(weatherPage)status(Mode::Ready,ok?"本地三天天气":error.c_str(),"");
 else status(beforeWeather.mode,beforeWeather.status,beforeWeather.text);
 Serial.printf("[WEATHER_TASK] ok=%u stack_free=%u\n",ok,unsigned(uxTaskGetStackHighWaterMark(nullptr)));vTaskDelete(nullptr);
}
void startWeather(bool force=false){
 if(!navigationIdle()||NetworkConfig::active()||WiFi.status()!=WL_CONNECTED||time(nullptr)<1700000000)return;
 if(weatherAttempted&&millis()-weatherAttempt<60000)return;
 weatherAttempted=true;weatherAttempt=millis();portENTER_CRITICAL(&stateLock);beforeWeather=shared;portEXIT_CRITICAL(&stateLock);
 status(Mode::Recognizing,"正在定位并更新天气");
 if(xTaskCreatePinnedToCore(weatherTask,"local-weather",16*1024,force?reinterpret_cast<void*>(1):nullptr,2,nullptr,0)!=pdPASS)status(beforeWeather.mode,beforeWeather.status,beforeWeather.text);
}
void weatherTick(){
 static String network;
 static time_t applied=0;
 LocalWeather::Snapshot cached;portENTER_CRITICAL(&stateLock);cached=weatherCache;portEXIT_CRITICAL(&stateLock);
 if(cached.valid&&cached.verified&&cached.geoChecked!=applied){LocalWeather::applyTimezone(cached);applied=cached.geoChecked;}
 if(WiFi.status()!=WL_CONNECTED){network="";portENTER_CRITICAL(&stateLock);if(weatherCache.verified){weatherCache.verified=false;++shared.revision;}portEXIT_CRITICAL(&stateLock);return;}
 if(NetworkConfig::active())return;
 const String signature=WiFi.SSID()+"|"+WiFi.BSSIDstr()+"|"+WiFi.localIP().toString();
 if(network!=signature){network=signature;weatherPending=true;weatherAttempted=false;portENTER_CRITICAL(&stateLock);weatherCache.verified=false;++shared.revision;portEXIT_CRITICAL(&stateLock);Serial.println("[WEATHER_NET] changed=1 relocating=1");}
 if(weatherAttempted&&millis()-weatherAttempt<(weatherPending?300000u:60000u))return;
 const time_t now=time(nullptr);if(now<1700000000)return;
 if(weatherPending||!cached.valid||now-cached.geoChecked>=300||now-cached.fetched>=900)startWeather();
}
void enterCard(){
 if(!navigationIdle())return;
 menuHome=false;newsPage=selectedCard==4;weatherPage=selectedCard==3;askQuestion=selectedCard==1;gptPage=selectedCard==2;followLive=false;page=0;
 if(gptPage){
  if(hasSavedApp[2]){portENTER_CRITICAL(&stateLock);uint32_t revision=shared.revision+1;shared=savedApps[2];shared.revision=revision;portEXIT_CRITICAL(&stateLock);page=savedPages[2];}
  else status(Mode::Ready,"正在连接家庭网关","");
  GptGateway::enter();status(Mode::Ready,"正在连接家庭网关");
 }
 else if(newsPage)showNews();else if(weatherPage)showWeather();
 else {
  const unsigned index=gptPage?2:askQuestion?0:1;
  if(hasSavedApp[index]){portENTER_CRITICAL(&stateLock);uint32_t revision=shared.revision+1;shared=savedApps[index];shared.revision=revision;portEXIT_CRITICAL(&stateLock);page=savedPages[index];}
  else status(Mode::Ready,askQuestion?"按 BOOT 开始提问":"按 BOOT 开始连续转文字","");
 }
 Serial.printf("[UI] tabs=1 app=%u\n",unsigned(selectedCard+1));
}
void openFunction(uint8_t index){
 if(!navigationIdle()||index>4||index==selectedCard)return;
 if(NewsSpeech::active()){NewsSpeech::stop();deferredFunction=index;return;}
 if(!newsPage&&!weatherPage){const unsigned active=gptPage?2:askQuestion?0:1;portENTER_CRITICAL(&stateLock);savedApps[active]=shared;portEXIT_CRITICAL(&stateLock);hasSavedApp[active]=true;savedPages[active]=page;}
 const bool leavingGpt=gptPage;
 selectedCard=index;enterCard();
 if(leavingGpt)GptGateway::leave();
}
void returnToCards(){openFunction(0);}
void cycleMode(){openFunction((selectedCard+1)%5);}
void streamState(const char* message) {
  portENTER_CRITICAL(&stateLock);const auto mode=shared.mode;portEXIT_CRITICAL(&stateLock);
  status(mode==Mode::StreamEnding?Mode::StreamEnding:Mode::Streaming,message);
}
void streamText(const char* sentence) {
  History::append("transcription",sentence);
  static char previous[4096];
  portENTER_CRITICAL(&stateLock);memcpy(previous,shared.text,sizeof(previous));const auto mode=shared.mode;portEXIT_CRITICAL(&stateLock);
  String joined=previous;if(joined.length())joined+="\n\n";joined+=sentence;bool trimmed=false;
  while(joined.length()>=sizeof(shared.text)){int boundary=joined.indexOf("\n\n");if(boundary<0)break;joined.remove(0,boundary+2);trimmed=true;}
  status(mode==Mode::StreamEnding?Mode::StreamEnding:Mode::Streaming,trimmed?"正在听 · 显示最近的转写内容":"正在听，停顿后自动转写",joined.c_str());
}
void streamFinished(const char* error) {
  status(error[0]?Mode::Failure:Mode::Result,error[0]?error:"持续转写已停止 · KEY重新开始");
}
void keyPress() {
  if(NewsSpeech::active()){NewsSpeech::stop();deferredAction=true;return;}
  portENTER_CRITICAL(&stateLock);const auto mode=shared.mode;if(mode==Mode::Recording)shared.stop=true;portEXIT_CRITICAL(&stateLock);
  if(mode==Mode::Streaming){RealtimeText::stop();status(Mode::StreamEnding,"正在收尾，请稍候");return;}
  if(mode==Mode::StreamEnding)return;
  if(mode==Mode::Recording || mode==Mode::Recognizing || mode==Mode::Setup)return;
  if(GptGateway::stopping()){status(Mode::Ready,"正在退出家庭VPN，请稍候");return;}
  if(newsPage){startNews(true);return;}
  if(weatherPage){startWeather(true);return;}
  if(!micReady){status(Mode::Failure,"麦克风未就绪，请查看串口");return;}
  if(!NetworkConfig::hasCredentials() || (!gptPage&&!DASHSCOPE_API_KEY[0])){status(Mode::Failure,"请先在电脑配置 Wi-Fi 和 API Key");return;}
  if(WiFi.status()!=WL_CONNECTED){WiFi.reconnect();status(Mode::Failure,"Wi-Fi 未连接，稍后按 BOOT 重试");return;}
  if(time(nullptr)<1700000000){status(Mode::Failure,"正在同步时间，稍后按 BOOT 重试");return;}
  if(gptPage&&!GptGateway::ready()){GptGateway::enter();status(Mode::Ready,"正在连接家庭网关，请稍候");return;}
  if(!askQuestion&&!gptPage){
    page=0;followLive=true;status(Mode::Streaming,"正在连接实时识别，请稍候","");
    if(!RealtimeText::start(streamState,streamText,streamFinished))status(Mode::Failure,"持续转写任务创建失败");return;
  }
  followLive=false;
  portENTER_CRITICAL(&stateLock);shared.stop=false;shared.recorded=0;shared.peak=0;portEXIT_CRITICAL(&stateLock);
  page=0;status(Mode::Recording,"正在听，再按 BOOT 结束","");
  const BaseType_t created=gptPage?xTaskCreatePinnedToCoreWithCaps(recordTask,"record-gpt",40*1024,nullptr,3,nullptr,0,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT):xTaskCreatePinnedToCore(recordTask,"record-asr",12*1024,nullptr,3,nullptr,0);
  if(created!=pdPASS)status(Mode::Failure,"录音任务创建失败");
}
void microphoneTest(void*) {
  int16_t samples[640*2];int64_t square[2]={};uint16_t peak[2]={};bool ok=true;
  for(unsigned frame=0;frame<50;++frame) {
    if(Microphone::read(samples,640)<0){ok=false;break;}
    for(unsigned i=0;i<640;++i)for(unsigned ch=0;ch<2;++ch){const int32_t n=samples[i*2+ch];square[ch]+=int64_t(n)*n;peak[ch]=max(peak[ch],uint16_t(abs(n)));}
  }
  Serial.printf("[MIC_TEST] ok=%u frames=32000 L_rms=%u L_peak=%u R_rms=%u R_peak=%u\n",ok,unsigned(sqrt(double(square[0])/32000)),peak[0],unsigned(sqrt(double(square[1])/32000)),peak[1]);
  status(ok?Mode::Ready:Mode::Failure,ok?"麦克风采样完成，按 BOOT 录音":"麦克风采样失败");vTaskDelete(nullptr);
}
// Transparent overdraw thickens strokes by two pixels without changing
// character spacing or font coverage. Pagination reserves the extra two pixels.
void drawBold(int x,int y,const char* text) {
  screen->setFontMode(1);
  screen->drawUTF8(x,y,text);
  screen->drawUTF8(x+1,y,text);
  screen->drawUTF8(x+2,y,text);
}
// Smaller text setting: native 16px Chinese glyphs, existing bold weight.
void drawLargeBold(int x,int y,const char* text) {
  screen->setFont(u8g2_font_wqy16_t_gb2312);
  drawBold(x,y,text);
}
size_t nextChar(const char* p,size_t i) {++i;while(p[i] && (uint8_t(p[i])&0xc0)==0x80)++i;return i;}
size_t textPage(const char* text,size_t start,bool draw) {
  // Pagination must use the same Chinese font as rendering, even after the
  // English footer changed U8g2's global font on the previous frame.
  screen->setFont(u8g2_font_wqy16_t_gb2312);
  size_t i=start;int line=0;char row[256];
  while(text[i] && line<8) {
    size_t begin=i,last=i;
    while(text[i] && text[i]!='\n') {
      const size_t next=nextChar(text,i),n=next-begin;
      if(n>=sizeof(row))break;
      memcpy(row,text+begin,n);row[n]=0;
      if(screen->getUTF8Width(row)+2>348 && i>begin)break;
      last=next;i=next;
    }
    if(last==begin && text[i]!='\n' && text[i])last=i=nextChar(text,i);
    const size_t n=min(last-begin,sizeof(row)-1);memcpy(row,text+begin,n);row[n]=0;
    if(draw)drawLargeBold(24,122+line*18,row);
    if(text[i]=='\n')++i;++line;
  }
  return i;
}
size_t pageCount(const char* text) {
  size_t total=0,offset=0;
  while(text[offset]){offset=textPage(text,offset,false);++total;}
  return max(size_t(1),total);
}
void advancePage() {
  static char text[4096];
  portENTER_CRITICAL(&stateLock);
  const bool live=shared.mode==Mode::Streaming || shared.mode==Mode::StreamEnding;
  // Connection/status updates keep the visible answer, including SD restores.
  const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;
  const bool result=shared.text[0] && (idle || live);
  if(result)memcpy(text,shared.text,sizeof(text));portEXIT_CRITICAL(&stateLock);
  if(!result)return;
  const size_t total=pageCount(text);page=(page+1)%total;followLive=live && page==total-1;
  portENTER_CRITICAL(&stateLock);++shared.revision;portEXIT_CRITICAL(&stateLock);
  Serial.printf("[PAGE] current=%u total=%u\n",unsigned(page+1),unsigned(total));
}
void paginationTest() {
  portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
  if(!idle)return;
  menuHome=false;
  String fixture;
  for(unsigned i=1;i<=12;++i){fixture+=String(i)+". 这是中文翻页测试。请短按 KEY 查看下一页，末页后返回首页。";}
  screen->setFont(u8g2_font_6x10_tf);
  const size_t afterFooter=textPage(fixture.c_str(),0,false);
  screen->setFont(u8g2_font_wqy16_t_gb2312);
  const size_t afterChinese=textPage(fixture.c_str(),0,false);
  const size_t total=pageCount(fixture.c_str());
  bool valid=afterFooter==afterChinese && total>1 && afterFooter<fixture.length();
  size_t offset=0;
  while(fixture[offset]){const size_t end=textPage(fixture.c_str(),offset,false);if(end<=offset || (uint8_t(fixture[end])&0xc0)==0x80){valid=false;break;}offset=end;}
  Serial.printf("[PAGE_TEST] pass=%u pages=%u footer_end=%u chinese_end=%u bytes=%u\n",valid,unsigned(total),unsigned(afterFooter),unsigned(afterChinese),unsigned(fixture.length()));
  page=0;followLive=false;status(Mode::Result,"翻页测试：短按 KEY 翻页",fixture.c_str());
}
void flacTest() {
  portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);if(!idle)return;
  const size_t counts[]={1,1024,4097,320000};
  for(size_t n:counts){
    auto* pcm=static_cast<uint8_t*>(heap_caps_malloc(n*2,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));if(!pcm){Serial.println("[FLAC_FAIL]");return;}
    for(size_t i=0;i<n;++i){int16_t v=i<1024?0:i<2048?int16_t(int(i%400)-200):i<3072?(i&1?-32768:32767):int16_t((uint32_t(i)*1103515245u+12345u)>>16);put16(pcm+i*2,uint16_t(v));}
    size_t bytes=0;auto* encoded=LosslessAudio::encode(pcm,n,bytes);free(pcm);if(!encoded){Serial.println("[FLAC_FAIL]");return;}
    Serial.printf("[FLAC_DATA] samples=%u bytes=%u\n",unsigned(n),unsigned(bytes));Serial.write(encoded,bytes);Serial.println();free(encoded);
  }
  Serial.println("[FLAC_DONE]");
}
void paintKeyTest() {
  static uint32_t revision=0,last=0;
  if(revision==ExternalKeys::revision || millis()-last<100)return;
  revision=ExternalKeys::revision;last=millis();
  screen->clearBuffer();screen->setDrawColor(1);screen->setFont(u8g2_font_wqy16_t_gb2312);
  drawBold(12,23,"四键接线测试 · 不触发功能");screen->drawHLine(10,32,380);
  drawBold(12,57,"请从左到右逐个按下，再松开");
  for(unsigned i=0;i<4;++i){
    char label[100];snprintf(label,sizeof(label),"K%u  GPIO%u  %s  次数 %lu",i+1,ExternalKeys::pins[i],ExternalKeys::keys[i].down?"按下":"松开",(unsigned long)ExternalKeys::keys[i].presses);
    drawBold(12,96+i*43,label);
  }
  drawBold(12,278,"测试结束后恢复原有页面");screen->sendBuffer();
}
bool systemIdle(){
 portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready||shared.mode==Mode::Result||shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
 return idle&&!RealtimeText::active()&&!GptGateway::stopping()&&!NewsSpeech::active();
}
void paintPortal(){
 static String prior;static uint32_t last=0;
 String value=NetworkConfig::message()+NetworkConfig::apPassword();if(value==prior&&millis()-last<3000)return;prior=value;last=millis();
 screen->clearBuffer();screen->setDrawColor(1);screen->setFont(u8g2_font_wqy16_t_gb2312);
 drawBold(12,24,"手机网页 Wi-Fi 设置");screen->drawHLine(10,34,380);
 drawBold(12,60,"1. 手机连接下面的设备热点");
 drawBold(12,88,NetworkConfig::apName().c_str());
 String pw="密码："+NetworkConfig::apPassword();drawBold(12,116,pw.c_str());
 drawBold(12,148,"2. 提示无互联网时，保持连接");drawBold(12,178,"3. 浏览器打开下面的网址");
 drawBold(12,208,"http://192.168.4.1");drawBold(12,240,NetworkConfig::message().c_str());
 drawBold(12,280,"短按 RX 退出；仅支持 2.4GHz");screen->sendBuffer();
}
void paintClock(){
 const time_t now=time(nullptr);struct tm local{};char clock[12],date[48],zone[12];
 if(now>=1700000000){
  localtime_r(&now,&local);strftime(clock,sizeof(clock),"%H:%M:%S",&local);strftime(zone,sizeof(zone),"%Z",&local);
  const char* days[]={"周日","周一","周二","周三","周四","周五","周六"};snprintf(date,sizeof(date),"%02d-%02d %s",local.tm_mon+1,local.tm_mday,days[local.tm_wday]);
 }else{strcpy(clock,"--:--:--");strcpy(date,"正在校时");zone[0]=0;}
 screen->setFont(u8g2_font_logisoso18_tn);screen->drawStr(12,25,clock);
 screen->setFont(u8g2_font_6x10_tf);screen->drawStr(116,23,zone);
 screen->drawStr(284,11,WiFi.status()==WL_CONNECTED?"Wi-Fi OK":"Wi-Fi offline");
 char battery[20];
 if(Battery::valid())snprintf(battery,sizeof(battery),"~%d%%",Battery::percent());
 else strcpy(battery,"--%");
 if(Battery::low())screen->drawStr(284,27,"LOW");
 screen->drawFrame(344,14,42,17);screen->drawBox(386,19,3,7);
 if(Battery::valid()){
  const int fill=(Battery::percent()*38+99)/100;
  if(fill)screen->drawBox(346,16,fill,13);
 }
 // XOR keeps the centered label visible over both filled and empty areas.
 screen->setFontMode(1);screen->setDrawColor(2);
 screen->drawStr(344+(42-screen->getStrWidth(battery))/2,26,battery);
 screen->setDrawColor(1);screen->setFontMode(0);
 screen->setFont(u8g2_font_wqy16_t_gb2312);screen->drawUTF8(164,24,date);
}
void paintTabs(){
 const char* labels[]={"转文字","千问","GPT","天气","新闻"};
 for(unsigned i=0;i<5;++i){
  const int x=12+i*76;screen->drawRFrame(x,36,72,31,5);
  if(selectedCard==i){screen->drawRFrame(x+1,37,70,29,4);screen->drawBox(x+6,64,60,3);}
  screen->setFont(u8g2_font_wqy16_t_gb2312);int labelX=x+(72-screen->getUTF8Width(labels[i])-2)/2;drawBold(labelX,57,labels[i]);
 }
}
void paintWeather(bool updating){
 LocalWeather::Snapshot w;char error[128];portENTER_CRITICAL(&stateLock);w=weatherCache;memcpy(error,weatherError,sizeof(error));portEXIT_CRITICAL(&stateLock);
 screen->drawRFrame(12,77,376,194,8);screen->setFont(u8g2_font_wqy16_t_gb2312);
 if(!w.valid){
  drawBold(24,105,"本地三天天气 · IP自动定位");drawLargeBold(24,150,"正在获取当前位置和天气");
  drawLargeBold(24,190,error[0]?error:"首次联网后自动获取");drawLargeBold(24,230,"BOOT 手动重试");return;
 }
 char city[112];snprintf(city,sizeof(city),"%s / %s%s",w.city,w.country,w.verified?"":" (缓存)");
 while(city[0]&&screen->getUTF8Width(city)+2>348){size_t n=strlen(city)-1;while(n&&(uint8_t(city[n])&0xc0)==0x80)--n;city[n]=0;}
 drawBold(24,100,city);
 char current[100];snprintf(current,sizeof(current),"当前 %.1f C  %s",w.temp,LocalWeather::condition(w.code));drawBold(24,129,current);
 for(unsigned i=0;i<3;++i){
  const int x=24+i*118;screen->drawRFrame(x,143,110,101,5);
  screen->setFont(u8g2_font_7x14B_tf);screen->drawStr(x+8,163,w.days[i].date+5);
  screen->setFont(u8g2_font_wqy16_t_gb2312);screen->drawUTF8(x+8,186,LocalWeather::condition(w.days[i].code));
  char values[32];snprintf(values,sizeof(values),"%.0f~%.0f C",w.days[i].low,w.days[i].high);screen->setFont(u8g2_font_7x14B_tf);screen->drawStr(x+8,209,values);
  snprintf(values,sizeof(values),"Rain %d%%",w.days[i].rain);screen->setFont(u8g2_font_6x10_tf);screen->drawStr(x+8,233,values);
 }
 char foot[80];const char* clock=strchr(w.observed,'T');snprintf(foot,sizeof(foot),"Open-Meteo  %.5s  %s",clock?clock+1:"--:--",updating?"UPDATING":w.verified?"IP location":"CACHED / check location");
 screen->setFont(u8g2_font_6x10_tf);screen->drawStr(24,261,foot);
}
void paint() {
 if(NetworkConfig::active()){paintPortal();return;}
 if(ExternalKeys::testing){paintKeyTest();return;}
 static Shared view;static uint32_t revision=0,lastPaint=0;static time_t lastClock=-1;
 if(millis()-lastPaint<200)return;
 portENTER_CRITICAL(&stateLock);const bool dirty=revision!=shared.revision;if(dirty){view=shared;revision=shared.revision;}portEXIT_CRITICAL(&stateLock);
 const time_t now=time(nullptr);if(!dirty&&now==lastClock)return;lastClock=now;lastPaint=millis();
 if(view.text[0]){const size_t total=pageCount(view.text);page=followLive?total-1:min(page,total-1);}
 screen->clearBuffer();screen->setDrawColor(1);paintClock();paintTabs();
 if(weatherPage){paintWeather(view.mode==Mode::Recognizing);screen->setFont(u8g2_font_wqy16_t_gb2312);drawBold(12,294,"长按KEY切换功能");screen->setFont(u8g2_font_6x10_tf);screen->drawStr(280,292,"BOOT: refresh");screen->sendBuffer();return;}
 screen->drawRFrame(12,77,376,194,8);screen->setFont(u8g2_font_wqy16_t_gb2312);
 // Keep long status messages inside the rounded content card.
 char label[128];copyText(label,view.status,sizeof(label));
 while(label[0]&&screen->getUTF8Width(label)+2>348){size_t n=strlen(label)-1;while(n&&(uint8_t(label[n])&0xc0)==0x80)--n;label[n]=0;}
 drawBold(24,100,label);screen->drawHLine(24,108,352);
 if(view.mode==Mode::Recording){
  char counter[40];snprintf(counter,sizeof(counter),"%u / %u 秒",unsigned(view.recorded/sampleRate),maxSeconds);drawLargeBold(24,135,counter);
  screen->drawFrame(24,151,352,16);screen->drawBox(26,153,(uint32_t(view.peak)*348)/32768,12);
  drawLargeBold(24,200,"请对着板载麦克风说话");drawLargeBold(24,239,"再按 BOOT 结束并发送");
 }else if(view.text[0]){
  size_t offset=0;for(size_t p=0;p<page&&view.text[offset];++p)offset=textPage(view.text,offset,false);textPage(view.text,offset,true);
  char counter[32];snprintf(counter,sizeof(counter),"%u / %u",unsigned(page+1),unsigned(pageCount(view.text)));screen->setFont(u8g2_font_6x10_tf);screen->drawStr(320,264,counter);
 }else if(view.mode==Mode::Streaming||view.mode==Mode::StreamEnding){
  drawLargeBold(24,147,"请对着板载麦克风说话");drawLargeBold(24,186,"停顿后自动转成普通话");drawLargeBold(24,225,"再按 BOOT 停止本次转写");
 }else if(newsPage){
  drawLargeBold(24,147,"每天早上7点自动更新");drawLargeBold(24,186,"联网后提前生成10条新闻");drawLargeBold(24,225,"按 BOOT 可手动刷新");
 }else if(gptPage){
  drawLargeBold(24,147,"按BOOT录音，再按一次提问");drawLargeBold(24,186,"仅GPT通过家庭网关联网");drawLargeBold(24,225,"KEY翻页，长按切换功能");
 }else if(askQuestion){
  drawLargeBold(24,147,"按 BOOT 录问题，再按发送");drawLargeBold(24,186,"千问联网查询并回答");drawLargeBold(24,225,"长按 KEY 切换到转文字");
 }else{
  drawLargeBold(24,147,"按 BOOT 开始连续听写");drawLargeBold(24,186,"上海话直接转成普通话");drawLargeBold(24,225,"再按 BOOT 停止听写");
 }
 if(askQuestion||gptPage){
  String modelLabel=String("Model: ")+(gptPage?GptGateway::model():String(AnswerPolicy::model));
  screen->setFont(u8g2_font_6x10_tf);
  while(modelLabel.length()&&screen->getStrWidth(modelLabel.c_str())>276)modelLabel.remove(modelLabel.length()-1);
  screen->drawStr(24,264,modelLabel.c_str());
 }
 if(newsPage){screen->setFont(u8g2_font_6x10_tf);screen->drawStr(24,264,"K1: next news / loop");}
 screen->setFont(u8g2_font_wqy16_t_gb2312);drawBold(12,294,"KEY翻页 · 长按切换");
 screen->setFont(u8g2_font_6x10_tf);screen->drawStr(280,292,newsPage?"BOOT: refresh":"BOOT: record/stop");screen->sendBuffer();
}

uint32_t networkTestStarted=0;
bool suppressButtons=false;
int lastSleepResult=-1;unsigned lastWakeReason=0,sleepCount=0;
uint32_t lastSleepMs=0,lastRestoreMs=0;
#ifdef CONFIG_PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP
constexpr bool cpuSleepPowerDown=true;
#else
constexpr bool cpuSleepPowerDown=false;
#endif
void sleepDevice(uint32_t wakeAfterMs=0){
 if(NewsSpeech::active()){NewsSpeech::stop();deferredSleep=true;deferredSleepTest=wakeAfterMs;return;}
 if(!systemIdle()||NetworkConfig::active()){Serial.println("[SLEEP] busy=1");return;}
 // GPIO44 is a digital light-sleep wake source, not an RTC deep-sleep pin.
 // Re-establish the wake input each time, before blanking the display.
 gpio_config_t wakePin{};wakePin.pin_bit_mask=1ULL<<44;wakePin.mode=GPIO_MODE_INPUT;
 wakePin.pull_up_en=GPIO_PULLUP_ENABLE;wakePin.pull_down_en=GPIO_PULLDOWN_DISABLE;
 esp_err_t configured=gpio_config(&wakePin);
 if(configured==ESP_OK)configured=gpio_sleep_sel_dis(GPIO_NUM_44);
 if(configured==ESP_OK)configured=gpio_wakeup_enable(GPIO_NUM_44,GPIO_INTR_LOW_LEVEL);
 if(configured==ESP_OK)configured=esp_sleep_enable_gpio_wakeup();
 if(configured==ESP_OK&&wakeAfterMs)configured=esp_sleep_enable_timer_wakeup(uint64_t(wakeAfterMs)*1000);
 if(configured!=ESP_OK){
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);gpio_wakeup_disable(GPIO_NUM_44);
  Serial.printf("[SLEEP] wake_setup_failed=%d\n",int(configured));status(Mode::Failure,"休眠唤醒配置失败，请重试");return;
 }
 const auto cancelWake=[](){esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);gpio_wakeup_disable(GPIO_NUM_44);};
 Serial.printf("[SLEEP] entering light gpio=44 timer_ms=%lu cpu_power_down=%u\n",(unsigned long)wakeAfterMs,unsigned(cpuSleepPowerDown));Serial.flush();
 if(!GptGateway::stopAndWait()){cancelWake();status(Mode::Failure,"家庭VPN退出超时，请重启");return;}
 if(!History::flush()){cancelWake();status(Mode::Failure,"SD卡保存未完成，请稍后休眠");return;}
 Microphone::suspend();NetworkConfig::suspend();
 screen->clearBuffer();screen->sendBuffer();lcd.setSleep(true);
 // Drop the USB D+ pull-up before sleep so the host sees a real disconnect.
 Serial.end();delay(30);
 const int64_t sleepStarted=esp_timer_get_time();
 const esp_err_t result=esp_light_sleep_start();
 const int64_t woke=esp_timer_get_time();lastSleepMs=uint32_t((woke-sleepStarted)/1000);
 const auto reason=esp_sleep_get_wakeup_cause();lastSleepResult=int(result);lastWakeReason=unsigned(reason);++sleepCount;cancelWake();
 // Restore visible content before USB, I2S or Wi-Fi recovery can delay it.
 // Reset/reinitialize ST7305 instead of assuming its state survived a long sleep.
 lcd.fullInit();++ExternalKeys::revision;
 portENTER_CRITICAL(&stateLock);++shared.revision;portEXIT_CRITICAL(&stateLock);
 paint();
 Serial.begin(115200);
 Serial.printf("[SLEEP] display_restored=1 slept_ms=%lu reason=%u result=%d\n",(unsigned long)lastSleepMs,unsigned(reason),int(result));
 micReady=Microphone::resume();Serial.printf("[SLEEP] microphone_restored=%u\n",micReady);
 NetworkConfig::resume();lastRestoreMs=uint32_t((esp_timer_get_time()-woke)/1000);
 if(gptPage)status(Mode::Ready,"正在恢复家庭网关连接");
 // A wake press is consumed, including a held button.
 suppressButtons=true;
 portENTER_CRITICAL(&stateLock);++shared.revision;portEXIT_CRITICAL(&stateLock);
 if(!micReady)status(Mode::Failure,"唤醒后麦克风恢复失败");
 ++ExternalKeys::revision;
 Serial.printf("[SLEEP] resumed result=%d reason=%u mic=%u restore_ms=%lu\n",int(result),unsigned(reason),micReady,(unsigned long)lastRestoreMs);
}
void rxPoll(){
 static bool raw=false,down=false,handled=false;static uint32_t changed=0,pressed=0;
 bool value=digitalRead(44)==LOW;
 if(suppressButtons){raw=down=value;changed=millis();pressed=0;handled=true;return;}
 if(value!=raw){raw=value;changed=millis();}
 if(down!=raw&&millis()-changed>=35){down=raw;Serial.printf("[RX_KEY] down=%u\n",down);if(down){pressed=millis();handled=false;}else{if(!handled&&pressed){if(NetworkConfig::active())NetworkConfig::close();else sleepDevice();}pressed=0;}}
 if(down&&pressed&&!handled&&millis()-pressed>=3000){handled=true;if(NewsSpeech::active()){NewsSpeech::stop();deferredPortal=true;}else if(systemIdle())NetworkConfig::open();else Serial.println("[WIFI_PORTAL] busy=1");}
}

void setup() {
  Serial.begin(115200);pinMode(44,INPUT_PULLUP);pinMode(pageButtonPin,INPUT_PULLUP);pinMode(recordButtonPin,INPUT_PULLUP);ExternalKeys::begin();
  Battery::begin();
  lcd.begin(0,U8G2_R1);screen=lcd.getU8g2();paint();
  if(!psramFound()){status(Mode::Failure,"未检测到 PSRAM，请启用 OPI PSRAM");return;}
  History::begin();String previousGpt;
  if(History::lastGpt(previousGpt)){savedApps[2].mode=Mode::Result;copyText(savedApps[2].status,"上次GPT回答 · SD卡",sizeof(savedApps[2].status));copyText(savedApps[2].text,previousGpt.c_str(),sizeof(savedApps[2].text));hasSavedApp[2]=true;Serial.println("[SD_HISTORY] restored_gpt=1");}
  micReady=Microphone::begin();
  NewsSpeech::begin();
  NetworkConfig::begin(WIFI_SSID,WIFI_PASSWORD);
  configTzTime("PST8PDT,M3.2.0,M11.1.0","pool.ntp.org","time.nist.gov");
  const bool cached=DailyNews::load(newsCache);
  const bool weatherLoaded=LocalWeather::load(weatherCache);Serial.printf("[WEATHER_LOAD] cached=%u\n",weatherLoaded);
  Serial.printf("[NEWS_LOAD] cached=%u day=%d count=%d\n",cached,newsCache.day,newsCache.count);
  if(!micReady)status(Mode::Failure,"麦克风初始化失败");
  else if(!NetworkConfig::hasCredentials() || !DASHSCOPE_API_KEY[0])status(Mode::Failure,"请先配置 Wi-Fi 和 API Key");
  else status(Mode::Ready,"转文字：按 BOOT 开始听写");
  Serial.println("[BUTTON_MAP] BOOT_GPIO0=enter_record KEY_GPIO18=page_hold_next_tab RX_GPIO44=sleep_setup");
 Serial.printf("[READY] build=gpt-tailnet-gateway-20261106-sleep-recovery ESP32-S3-RLCD-4.2 psram=%u mic=%u config=%u\n",ESP.getPsramSize(),micReady,NetworkConfig::hasCredentials() && DASHSCOPE_API_KEY[0]);
}
void loop() {
  Battery::tick();
  if(networkTestStarted&&millis()-networkTestStarted>=8000){networkTestStarted=0;NetworkConfig::resume();Serial.println("[WEATHER_OFFLINE_TEST] reconnect=1");}
  const uint8_t extPressed=ExternalKeys::poll();
  if((extPressed&1)&&!suppressButtons&&!ExternalKeys::testing&&!NetworkConfig::active()&&newsPage&&navigationIdle())NewsSpeech::next(newsCache.text,newsCache.count);
  static uint32_t speechRevision=0;
  if(speechRevision!=NewsSpeech::revision()){
   speechRevision=NewsSpeech::revision();if(newsPage&&navigationIdle())status(Mode::Result,NewsSpeech::label().c_str());
  }
  if(!NewsSpeech::active()){
   if(deferredFunction>=0){const int index=deferredFunction;deferredFunction=-1;openFunction(index);}
   if(deferredAction){deferredAction=false;keyPress();}
   if(deferredPortal){deferredPortal=false;if(systemIdle())NetworkConfig::open();}
   if(deferredSleep){const uint32_t test=deferredSleepTest;deferredSleep=false;sleepDevice(test);}
  }
  static bool portalWas=false;
  NetworkConfig::tick(systemIdle());rxPoll();
  const bool portalNow=NetworkConfig::active();
  if(portalWas&&!portalNow){suppressButtons=true;portENTER_CRITICAL(&stateLock);++shared.revision;portEXIT_CRITICAL(&stateLock);++ExternalKeys::revision;}
  portalWas=portalNow;
  static uint8_t prior=3,stable=3;static uint32_t changed=0;static bool wifi=false;
  static uint32_t bootDown=0;static bool bootHandled=false;
  const uint8_t keys=(digitalRead(recordButtonPin)?1:0)|(digitalRead(pageButtonPin)?2:0);
  if(portalNow||suppressButtons){
    prior=stable=keys;changed=millis();bootDown=0;bootHandled=true;
    static uint32_t released=0;
    if(keys==3&&digitalRead(44)==HIGH){if(!released)released=millis();if(millis()-released>=60)suppressButtons=false;}else released=0;
  }
  if(!portalNow&&!suppressButtons&&keys!=prior){prior=keys;changed=millis();}
  if(!portalNow&&!suppressButtons&&keys!=stable && millis()-changed>=35) {
    const uint8_t pressed=stable&~keys;stable=keys;
    if(pressed&1)keyPress();
    // KEY pages content. An idle one-second hold switches the top tab.
    if(pressed&2){bootDown=millis();bootHandled=false;}
    if((stable&2) && bootDown){
      if(!bootHandled) {
        advancePage();
      }
      bootDown=0;
    }
  }
  if(!portalNow&&!suppressButtons&&!(stable&2) && bootDown && !bootHandled && millis()-bootDown>=1000) {
    bootHandled=true;
    portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
    if(idle)cycleMode();
  }
  const bool connected=WiFi.status()==WL_CONNECTED;if(wifi!=connected){wifi=connected;portENTER_CRITICAL(&stateLock);++shared.revision;portEXIT_CRITICAL(&stateLock);}
  static char command[16];static uint8_t length=0;
  while(Serial.available()) {
    const char c=Serial.read();if(c=='\r')continue;
    if(c=='\n') {
      command[length]=0;
      if(!strcmp(command,"wifi-setup")){if(systemIdle())NetworkConfig::open();}
      else if(!strcmp(command,"wifi-cancel"))NetworkConfig::close();
      else if(!strcmp(command,"wifi-info"))NetworkConfig::report();
      else if(!strcmp(command,"battery-info"))Battery::report();
      else if(!strcmp(command,"speech-info"))NewsSpeech::report();
      else if(!strcmp(command,"speech-next")){if(newsPage&&navigationIdle())NewsSpeech::next(newsCache.text,newsCache.count);}
      else if(!strcmp(command,"speech-stop"))NewsSpeech::stop();
      else if(!strcmp(command,"speaker-test")){if(systemIdle())NewsSpeech::tone();}
      else if(!strcmp(command,"sleep-info"))Serial.printf("[SLEEP_INFO] count=%u result=%d reason=%u mic=%u slept_ms=%lu restore_ms=%lu cpu_power_down=%u rx=%u\n",sleepCount,lastSleepResult,lastWakeReason,micReady,(unsigned long)lastSleepMs,(unsigned long)lastRestoreMs,unsigned(cpuSleepPowerDown),unsigned(digitalRead(44)));
      else if(!strcmp(command,"sleep"))sleepDevice();
      else if(!strcmp(command,"sleep-test"))sleepDevice(3000);
      else if(!strcmp(command,"sleep-long"))sleepDevice(120000);
      else if(NetworkConfig::active()){Serial.println("[COMMAND] close_wifi_setup_first=1");}
      else if(!strcmp(command,"home"))returnToCards();
      else if(!strcmp(command,"enter")){if(menuHome)enterCard();}
      else if(!strcmp(command,"gpt"))openFunction(2);
      else if(!strcmp(command,"gpt-info"))GptGateway::report();
      else if(!strcmp(command,"gpt-net-test")){if(gptPage&&systemIdle()&&GptGateway::ready()){status(Mode::Recognizing,"正在测试VPN上传");if(xTaskCreatePinnedToCoreWithCaps([](void*){GptGateway::probeUpload();status(Mode::Ready,"VPN上传测试完成");vTaskDeleteWithCaps(nullptr);},"gpt-net-test",16*1024,nullptr,2,nullptr,0,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT)!=pdPASS)status(Mode::Failure,"测试任务创建失败");}}
      else if(!strcmp(command,"sd-info"))History::report();
      else if(!strcmp(command,"sd-test")){History::append("diagnostic","RLCD SD history write test");}
      else if(!strcmp(command,"weather"))openFunction(3);
      else if(!strcmp(command,"weather-refresh"))startWeather(true);
      else if(!strcmp(command,"weather-offline")){if(navigationIdle()&&!networkTestStarted){networkTestStarted=millis();NetworkConfig::suspend();Serial.println("[WEATHER_OFFLINE_TEST] offline=1 duration_ms=8000");}}
      else if(!strcmp(command,"weather-info")){
       LocalWeather::Snapshot w;portENTER_CRITICAL(&stateLock);w=weatherCache;portEXIT_CRITICAL(&stateLock);
       Serial.printf("[WEATHER_INFO] valid=%u verified=%u city=%s country=%s timezone=%s current=%.1f fetched=%lld geo=%lld days=3 offset=%d\n",w.valid,w.verified,w.city,w.country,w.timezone,w.temp,(long long)w.fetched,(long long)w.geoChecked,w.offset);
      }
      else if(!strcmp(command,"tab-next"))cycleMode();
      else if(!strcmp(command,"tab-prev"))openFunction((selectedCard+4)%5);
      else if(!strcmp(command,"ui-info")){
        const time_t now=time(nullptr);struct tm local{};localtime_r(&now,&local);char stamp[64];strftime(stamp,sizeof(stamp),"%Y-%m-%d %H:%M:%S %Z",&local);
        Serial.printf("[UI_INFO] home=%u selected=%u app=%u local=%s synced=%u\n",menuHome,unsigned(selectedCard+1),unsigned(selectedCard+1),stamp,now>=1700000000);
      }
      else if(!strcmp(command,"ui-frame")){
        if(navigationIdle()){
          const size_t bytes=size_t(screen->getBufferTileWidth())*screen->getBufferTileHeight()*8;
          Serial.printf("[UI_FRAME] bytes=%u width=%u height=%u\n",unsigned(bytes),screen->getBufferTileWidth()*8,screen->getBufferTileHeight()*8);Serial.flush();
          for(size_t i=0;i<bytes;){size_t n=Serial.write(screen->getBufferPtr()+i,min(size_t(512),bytes-i));if(!n)break;i+=n;}
          Serial.println("\n[UI_FRAME_END]");
        }
      }
      else if(!strcmp(command,"mic")) {
        portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
        if(idle && micReady){status(Mode::Recognizing,"正在测试麦克风");if(xTaskCreatePinnedToCore(microphoneTest,"mic-test",8192,nullptr,3,nullptr,0)!=pdPASS)status(Mode::Failure,"采样任务创建失败");}
      }
      else if(!strcmp(command,"flac-test"))flacTest();
      else if(!strcmp(command,"opus-test")){portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);if(idle){status(Mode::Recognizing,"正在检查Opus编码器");if(xTaskCreatePinnedToCore([](void*){RealtimeText::codecTest();Serial.printf("[OPUS_TEST_STACK] free=%u\n",unsigned(uxTaskGetStackHighWaterMark(nullptr)));status(Mode::Ready,"Opus编码测试完成");vTaskDelete(nullptr);},"opus-test",32*1024,nullptr,2,nullptr,1)!=pdPASS)status(Mode::Failure,"Opus测试任务创建失败");}}
      else if(!strcmp(command,"transcribe")) {
        portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
        if(idle){menuHome=false;selectedCard=0;newsPage=false;weatherPage=false;askQuestion=false;gptPage=false;GptGateway::leave();page=0;followLive=false;status(Mode::Ready,"翻译模式：只转普通话文字","");}
      }
      else if(!strcmp(command,"rt-stop")) {if(RealtimeText::active()){RealtimeText::stop();status(Mode::StreamEnding,"正在收尾，请稍候");}}
      else if(!strcmp(command,"rt-start")) {if(!askQuestion && !newsPage && !RealtimeText::active())keyPress();}
      else if(!strcmp(command,"page-test"))paginationTest();
      else if(!strcmp(command,"page"))advancePage();
      else if(!strcmp(command,"news")) {
        portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
        if(idle){menuHome=false;selectedCard=4;weatherPage=false;gptPage=false;newsPage=true;showNews();}
      }
      else if(!strcmp(command,"news-clock")) {
        struct Check {time_t now;int day;};const Check tests[]={{1772978399,20260307},{1772978400,20260308},{1793545199,20261031},{1793545200,20261101},{1767279599,20251231},{1767279600,20260101}};bool ok=true;
        for(const auto& check:tests)if(DailyNews::editionDay(check.now)!=check.day)ok=false;
        Serial.printf("[NEWS_CLOCK] pass=%u cases=6 timezone=America/Los_Angeles daily_hour=7\n",ok);
      }
      else if(!strcmp(command,"news-refresh"))startNews(true);
      else if(!strcmp(command,"news-info")) {
        portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);if(!idle){Serial.println("[NEWS_INFO] busy=1");length=0;continue;}
        Serial.printf("[NEWS_INFO] day=%d count=%d fetched=%lld bytes=%u due=%u\n",newsCache.day,newsCache.count,(long long)newsCache.fetched,unsigned(newsCache.text.length()),newsCache.day!=DailyNews::editionDay(time(nullptr)));
      }
      else if(!strcmp(command,"cycle")) {
        portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);if(idle)cycleMode();
      }
      else if(!strcmp(command,"keys"))ExternalKeys::report();
      else if(!strcmp(command,"keys-test")) {
        portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
        if(idle){ExternalKeys::testing=true;++ExternalKeys::revision;Serial.println("[EXT_KEYS_TEST] enabled=1");ExternalKeys::report();}
        else Serial.println("[EXT_KEYS_TEST] busy=1");
      }
      else if(!strcmp(command,"keys-exit")) {ExternalKeys::testing=false;portENTER_CRITICAL(&stateLock);++shared.revision;portEXIT_CRITICAL(&stateLock);Serial.println("[EXT_KEYS_TEST] enabled=0");}
      else if(!strcmp(command,"status")) {
        portENTER_CRITICAL(&stateLock);const auto mode=shared.mode;portEXIT_CRITICAL(&stateLock);
        Serial.printf("[STATUS] chip=ESP32-S3 psram=%u mic=%u config=%u wifi=%u mode=%u width=%u height=%u ask=%u news=%u\n",ESP.getPsramSize(),micReady,NetworkConfig::hasCredentials() && DASHSCOPE_API_KEY[0],WiFi.status()==WL_CONNECTED,unsigned(mode),screen->getDisplayWidth(),screen->getDisplayHeight(),askQuestion,newsPage);
      }
      length=0;
    } else if(length<sizeof(command)-1)command[length++]=c;
  }
  if(!ExternalKeys::testing&&!NetworkConfig::active()){
   if(gptPage&&WiFi.status()==WL_CONNECTED&&!GptGateway::active()&&GptGateway::state()==GptGateway::State::Off)GptGateway::enter();
   static auto priorGpt=GptGateway::State::Off;const auto gptState=GptGateway::state();
   if(gptState!=priorGpt){priorGpt=gptState;if(gptPage&&navigationIdle()){
    if(gptState==GptGateway::State::Ready)status(Mode::Ready,"GPT已就绪 · 家庭网络");
    else if(gptState==GptGateway::State::Failed)status(Mode::Failure,GptGateway::error().c_str());
   }}
   if(!GptGateway::active()&&!NewsSpeech::active()){weatherTick();newsTick();}
  }paint();delay(5);
}

