#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <esp_heap_caps.h>
#include "ST7305_U8g2.h"
#include "Microphone.h"
#include "Transcribe.h"
#include "LosslessAudio.h"
#include "LocalConfig.h"
#include "DailyNews.h"
#include "RealtimeText.h"

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
bool askQuestion=true,newsPage=false;
DailyNews::Bulletin newsCache;
Shared beforeNews;
int newsTarget=0;uint32_t newsAttempt=0;bool newsAttempted=false;
// Clip at a UTF-8 codepoint boundary; cloud text is not ASCII.
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
  const bool ask=askQuestion;
  auto* wav=static_cast<uint8_t*>(heap_caps_malloc(wavCapacity,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
  if(!wav){status(Mode::Failure,"PSRAM 录音内存不足");vTaskDelete(nullptr);return;}
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
      const bool done=transcribe(wav,44+count*2,result,error,ask,[]{status(Mode::Recognizing,askQuestion?"正在联网查询并回答":"正在转换成普通话");});
      WiFi.setSleep(oldSleep);
      if(done)status(Mode::Result,ask?"千问回答":"普通话转换完成",result.c_str());else status(Mode::Failure,error.c_str());
    }
  }
  free(wav);Serial.printf("[TASK] record stack_free=%u heap=%u\n",unsigned(uxTaskGetStackHighWaterMark(nullptr)),ESP.getFreeHeap());vTaskDelete(nullptr);
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
void cycleMode() {
  followLive=false;
  if(newsPage){newsPage=false;askQuestion=true;page=0;status(Mode::Ready,"问答模式：按 KEY 提问","");}
  else if(askQuestion){askQuestion=false;page=0;status(Mode::Ready,"翻译模式：只转普通话文字","");}
  else {newsPage=true;showNews();}
  Serial.printf("[MODE] ask=%u news=%u\n",askQuestion,newsPage);
}
void streamState(const char* message) {
  portENTER_CRITICAL(&stateLock);const auto mode=shared.mode;portEXIT_CRITICAL(&stateLock);
  status(mode==Mode::StreamEnding?Mode::StreamEnding:Mode::Streaming,message);
}
void streamText(const char* sentence) {
  static char previous[4096];
  portENTER_CRITICAL(&stateLock);memcpy(previous,shared.text,sizeof(previous));const auto mode=shared.mode;portEXIT_CRITICAL(&stateLock);
  String joined=previous;if(joined.length())joined+="\n\n";joined+=sentence;bool trimmed=false;
  while(joined.length()>=sizeof(shared.text)){int boundary=joined.indexOf("\n\n");if(boundary<0)break;joined.remove(0,boundary+2);trimmed=true;}
  status(mode==Mode::StreamEnding?Mode::StreamEnding:Mode::Streaming,trimmed?"正在听 · 显示最近的转写内容":"正在听，停顿约1秒自动转写",joined.c_str());
}
void streamFinished(const char* error) {
  status(error[0]?Mode::Failure:Mode::Result,error[0]?error:"持续转写已停止 · KEY重新开始");
}
void keyPress() {
  portENTER_CRITICAL(&stateLock);const auto mode=shared.mode;if(mode==Mode::Recording)shared.stop=true;portEXIT_CRITICAL(&stateLock);
  if(mode==Mode::Streaming){RealtimeText::stop();status(Mode::StreamEnding,"正在收尾，请稍候");return;}
  if(mode==Mode::StreamEnding)return;
  if(mode==Mode::Recording || mode==Mode::Recognizing || mode==Mode::Setup)return;
  if(newsPage){startNews(true);return;}
  if(!micReady){status(Mode::Failure,"麦克风未就绪，请查看串口");return;}
  if(!WIFI_SSID[0] || !DASHSCOPE_API_KEY[0]){status(Mode::Failure,"请先在电脑配置 Wi-Fi 和 API Key");return;}
  if(WiFi.status()!=WL_CONNECTED){WiFi.reconnect();status(Mode::Failure,"Wi-Fi 未连接，稍后按 KEY 重试");return;}
  if(time(nullptr)<1700000000){status(Mode::Failure,"正在同步时间，稍后按 KEY 重试");return;}
  if(!askQuestion){
    page=0;followLive=true;status(Mode::Streaming,"正在连接实时识别，请稍候","");
    if(!RealtimeText::start(streamState,streamText,streamFinished))status(Mode::Failure,"持续转写任务创建失败");return;
  }
  followLive=false;
  portENTER_CRITICAL(&stateLock);shared.stop=false;shared.recorded=0;shared.peak=0;portEXIT_CRITICAL(&stateLock);
  page=0;status(Mode::Recording,"正在听，再按 KEY 结束","");
  if(xTaskCreatePinnedToCore(recordTask,"record-asr",12*1024,nullptr,3,nullptr,0)!=pdPASS)status(Mode::Failure,"录音任务创建失败");
}
void microphoneTest(void*) {
  int16_t samples[640*2];int64_t square[2]={};uint16_t peak[2]={};bool ok=true;
  for(unsigned frame=0;frame<50;++frame) {
    if(Microphone::read(samples,640)<0){ok=false;break;}
    for(unsigned i=0;i<640;++i)for(unsigned ch=0;ch<2;++ch){const int32_t n=samples[i*2+ch];square[ch]+=int64_t(n)*n;peak[ch]=max(peak[ch],uint16_t(abs(n)));}
  }
  Serial.printf("[MIC_TEST] ok=%u frames=32000 L_rms=%u L_peak=%u R_rms=%u R_peak=%u\n",ok,unsigned(sqrt(double(square[0])/32000)),peak[0],unsigned(sqrt(double(square[1])/32000)),peak[1]);
  status(ok?Mode::Ready:Mode::Failure,ok?"麦克风采样完成，按 KEY 录音":"麦克风采样失败");vTaskDelete(nullptr);
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
  while(text[i] && line<9) {
    size_t begin=i,last=i;
    while(text[i] && text[i]!='\n') {
      const size_t next=nextChar(text,i),n=next-begin;
      if(n>=sizeof(row))break;
      memcpy(row,text+begin,n);row[n]=0;
      if(screen->getUTF8Width(row)+2>376 && i>begin)break;
      last=next;i=next;
    }
    if(last==begin && text[i]!='\n' && text[i])last=i=nextChar(text,i);
    const size_t n=min(last-begin,sizeof(row)-1);memcpy(row,text+begin,n);row[n]=0;
    if(draw)drawLargeBold(12,92+line*19,row);
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
  portENTER_CRITICAL(&stateLock);const bool live=shared.mode==Mode::Streaming || shared.mode==Mode::StreamEnding;const bool result=shared.mode==Mode::Result || live;if(result)memcpy(text,shared.text,sizeof(text));portEXIT_CRITICAL(&stateLock);
  if(!result)return;
  const size_t total=pageCount(text);page=(page+1)%total;followLive=live && page==total-1;
  portENTER_CRITICAL(&stateLock);++shared.revision;portEXIT_CRITICAL(&stateLock);
  Serial.printf("[PAGE] current=%u total=%u\n",unsigned(page+1),unsigned(total));
}
void paginationTest() {
  portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
  if(!idle)return;
  String fixture;
  for(unsigned i=1;i<=12;++i){fixture+=String(i)+". 这是中文翻页测试。请短按 BOOT 查看下一页，末页后返回首页。";}
  screen->setFont(u8g2_font_6x10_tf);
  const size_t afterFooter=textPage(fixture.c_str(),0,false);
  screen->setFont(u8g2_font_wqy16_t_gb2312);
  const size_t afterChinese=textPage(fixture.c_str(),0,false);
  const size_t total=pageCount(fixture.c_str());
  bool valid=afterFooter==afterChinese && total>1 && afterFooter<fixture.length();
  size_t offset=0;
  while(fixture[offset]){const size_t end=textPage(fixture.c_str(),offset,false);if(end<=offset || (uint8_t(fixture[end])&0xc0)==0x80){valid=false;break;}offset=end;}
  Serial.printf("[PAGE_TEST] pass=%u pages=%u footer_end=%u chinese_end=%u bytes=%u\n",valid,unsigned(total),unsigned(afterFooter),unsigned(afterChinese),unsigned(fixture.length()));
  page=0;followLive=false;status(Mode::Result,"翻页测试：短按 BOOT 翻页",fixture.c_str());
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
void paint() {
  static Shared view;static uint32_t revision=0,lastPaint=0;
  if(millis()-lastPaint<200)return;
  portENTER_CRITICAL(&stateLock);const bool dirty=revision!=shared.revision;if(dirty){view=shared;revision=shared.revision;}portEXIT_CRITICAL(&stateLock);
  if(!dirty)return;lastPaint=millis();
  if(view.text[0]){const size_t total=pageCount(view.text);page=followLive?total-1:min(page,total-1);}
  screen->clearBuffer();screen->setDrawColor(1);screen->setFont(u8g2_font_wqy16_t_gb2312);
  drawBold(12,23,newsPage?"3/3 每日社会新闻":askQuestion?"1/3 千问问答 · 联网搜索":"2/3 上海话转普通话");screen->drawHLine(10,32,380);
  drawBold(12,55,view.status);
  if(view.mode==Mode::Recording) {
    char label[64];snprintf(label,sizeof(label),"%u / %u 秒",unsigned(view.recorded/sampleRate),maxSeconds);drawLargeBold(12,88,label);
    screen->drawFrame(12,106,376,18);screen->drawBox(14,108,(uint32_t(view.peak)*372)/32768,14);
    drawLargeBold(12,154,"请对着板载麦克风说话");drawLargeBold(12,190,"最长20秒，结束后识别");
  } else if(view.text[0]) {
    size_t offset=0;for(size_t p=0;p<page && view.text[offset];++p)offset=textPage(view.text,offset,false);
    const size_t end=textPage(view.text,offset,true);
    char footer[100];snprintf(footer,sizeof(footer),"第 %u / %u 页 · BOOT 翻页",unsigned(page+1),unsigned(pageCount(view.text)));screen->setFont(u8g2_font_wqy16_t_gb2312);drawBold(12,278,footer);
  } else if(view.mode==Mode::Streaming || view.mode==Mode::StreamEnding){drawLargeBold(12,112,"请根据上方状态开始讲话");drawLargeBold(12,150,"将自动追加普通话文字");drawLargeBold(12,192,"再按 KEY 停止本次转写");}
  else if(newsPage){drawLargeBold(12,112,"每天早上7点自动更新");drawLargeBold(12,150,"首次联网后自动生成");drawLargeBold(12,192,"长按 BOOT 返回问答");}
  else if(askQuestion){drawLargeBold(12,112,"按 KEY 录问题，再按发送");drawLargeBold(12,150,"千问联网查询并回答");drawLargeBold(12,192,"长按 BOOT 切换到翻译");}
  else {drawLargeBold(12,112,"按 KEY 开始持续听写");drawLargeBold(12,150,"停顿后自动转成普通话");drawLargeBold(12,192,"仅翻译，不回答所说问题");drawLargeBold(12,230,"再按 KEY 停止听写");}
  screen->drawHLine(10,283,380);screen->setFont(u8g2_font_6x10_tf);drawBold(12,296,WiFi.status()==WL_CONNECTED?(newsPage?"KEY: refresh   BOOT: page / hold: mode":"Wi-Fi OK   KEY: record/stop   BOOT: page"):"Wi-Fi offline");
  screen->sendBuffer();
}
void setup() {
  Serial.begin(115200);pinMode(18,INPUT_PULLUP);pinMode(0,INPUT_PULLUP);
  lcd.begin(0,U8G2_R1);screen=lcd.getU8g2();paint();
  if(!psramFound()){status(Mode::Failure,"未检测到 PSRAM，请启用 OPI PSRAM");return;}
  micReady=Microphone::begin();
  WiFi.mode(WIFI_STA);WiFi.setAutoReconnect(true);
  if(WIFI_SSID[0])WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  configTzTime("PST8PDT,M3.2.0,M11.1.0","pool.ntp.org","time.nist.gov");
  const bool cached=DailyNews::load(newsCache);
  Serial.printf("[NEWS_LOAD] cached=%u day=%d count=%d\n",cached,newsCache.day,newsCache.count);
  if(!micReady)status(Mode::Failure,"麦克风初始化失败");
  else if(!WIFI_SSID[0] || !DASHSCOPE_API_KEY[0])status(Mode::Failure,"请先配置 Wi-Fi 和 API Key");
  else status(Mode::Ready,"问答模式：按 KEY 提问");
  Serial.printf("[READY] ESP32-S3-RLCD-4.2 psram=%u mic=%u config=%u\n",ESP.getPsramSize(),micReady,WIFI_SSID[0] && DASHSCOPE_API_KEY[0]);
}
void loop() {
  static uint8_t prior=3,stable=3;static uint32_t changed=0;static bool wifi=false;
  static uint32_t bootDown=0;static bool bootHandled=false;
  const uint8_t keys=(digitalRead(18)?1:0)|(digitalRead(0)?2:0);
  if(keys!=prior){prior=keys;changed=millis();}
  if(keys!=stable && millis()-changed>=35) {
    const uint8_t pressed=stable&~keys;stable=keys;
    if(pressed&1)keyPress();
    // BOOT short release pages; a one-second hold changes mode only while idle.
    if(pressed&2){bootDown=millis();bootHandled=false;}
    if((stable&2) && bootDown){
      if(!bootHandled) {
        advancePage();
      }
      bootDown=0;
    }
  }
  if(!(stable&2) && bootDown && !bootHandled && millis()-bootDown>=1000) {
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
      if(!strcmp(command,"mic")) {
        portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
        if(idle && micReady){status(Mode::Recognizing,"正在测试麦克风");if(xTaskCreatePinnedToCore(microphoneTest,"mic-test",8192,nullptr,3,nullptr,0)!=pdPASS)status(Mode::Failure,"采样任务创建失败");}
      }
      else if(!strcmp(command,"flac-test"))flacTest();
      else if(!strcmp(command,"opus-test")){portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);if(idle){status(Mode::Recognizing,"正在检查Opus编码器");if(xTaskCreatePinnedToCore([](void*){RealtimeText::codecTest();Serial.printf("[OPUS_TEST_STACK] free=%u\n",unsigned(uxTaskGetStackHighWaterMark(nullptr)));status(Mode::Ready,"Opus编码测试完成");vTaskDelete(nullptr);},"opus-test",32*1024,nullptr,2,nullptr,1)!=pdPASS)status(Mode::Failure,"Opus测试任务创建失败");}}
      else if(!strcmp(command,"transcribe")) {
        portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
        if(idle){newsPage=false;askQuestion=false;page=0;followLive=false;status(Mode::Ready,"翻译模式：只转普通话文字","");}
      }
      else if(!strcmp(command,"rt-stop")) {if(RealtimeText::active()){RealtimeText::stop();status(Mode::StreamEnding,"正在收尾，请稍候");}}
      else if(!strcmp(command,"rt-start")) {if(!askQuestion && !newsPage && !RealtimeText::active())keyPress();}
      else if(!strcmp(command,"page-test"))paginationTest();
      else if(!strcmp(command,"page"))advancePage();
      else if(!strcmp(command,"news")) {
        portENTER_CRITICAL(&stateLock);const bool idle=shared.mode==Mode::Ready || shared.mode==Mode::Result || shared.mode==Mode::Failure;portEXIT_CRITICAL(&stateLock);
        if(idle){newsPage=true;showNews();}
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
      else if(!strcmp(command,"status")) {
        portENTER_CRITICAL(&stateLock);const auto mode=shared.mode;portEXIT_CRITICAL(&stateLock);
        Serial.printf("[STATUS] chip=ESP32-S3 psram=%u mic=%u config=%u wifi=%u mode=%u width=%u height=%u ask=%u news=%u\n",ESP.getPsramSize(),micReady,WIFI_SSID[0] && DASHSCOPE_API_KEY[0],WiFi.status()==WL_CONNECTED,unsigned(mode),screen->getDisplayWidth(),screen->getDisplayHeight(),askQuestion,newsPage);
      }
      length=0;
    } else if(length<sizeof(command)-1)command[length++]=c;
  }
  newsTick();paint();delay(5);
}
