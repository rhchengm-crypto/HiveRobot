#include "LocalClock.h"
#include "Weather.h"
#include "HotList.h"
#include "Speech.h"
#include "ChineseFont.h"
#include <time.h>
#include <math.h>
#include "DogAnimation.h"
#include "ScreenUI.h"
#include "DogAssets.h"
#include <Preferences.h>
namespace {
constexpr uint16_t BG=0x0105, PANEL=0x1948, INK=0xffff, MUTED=0xadb5, ACCENT=0x07ff;
constexpr uint16_t CREAM=0xff7a, SLEEP=0xd6ba, UNKNOWN=0xcdda, DARK=0x0145, AMBER=0xfd20;
constexpr TouchPoint targets[]={{35,65},{445,65},{35,285},{445,285}};
constexpr uint32_t calibrationMagic=0x4c434131; // Landscape rotation 1, format v1.
struct StoredCalibration { uint32_t magic; TouchCalibration data; };
int cloudBits(const ScreenModel& m) {
  return m.cloud.wifi | (m.online<<1) | (m.cloud.busy<<2) | (m.cloud.known<<3)
    | (m.cloud.power<<4) | (m.cloud.secondKnown<<5) | (m.cloud.secondPower<<6)
    | (m.cloud.thirdKnown<<7) | (m.cloud.thirdPower<<8);
}
}
void ScreenUI::begin() {
  Preferences prefs; StoredCalibration stored{};
  bool loaded=false;
  if (prefs.begin("macroTouch",true)) {
    loaded=prefs.getBytesLength("landscape1")==sizeof(stored)
      && prefs.getBytes("landscape1",&stored,sizeof(stored))==sizeof(stored)
      && stored.magic==calibrationMagic && stored.data.valid();
    prefs.end();
  }
  if (loaded) { calibration_=stored.data; Serial.println("[TOUCH] Saved landscape calibration loaded"); }
  else calibrate();
  dirty_=true;
}
void ScreenUI::selectPage(uint8_t page) {
  if (calibrating_ || page>3 || page_==page) return;
  page_=page; dirty_=true;
  // A physical page switch cannot turn an already-held screen touch into a tap.
  originCard_=-1;
  if(wasDown_)latched_=true;
  Serial.printf("[UI] Page=%s\n",page_==0?"LIGHTS":page_==1?"WEATHER":page_==2?"HOTTV":"KEYS");
}
void ScreenUI::changePage(int8_t direction) {
  const int next=int(page_)+(direction<0?-1:direction>0?1:0);
  if(next>=0 && next<=3)selectPage(next);
}
void ScreenUI::calibrate() {
  calibrating_=true; point_=samples_=0; waitRelease_=true;
  dirty_=true; latched_=true; retryCalibration_=false;
  Serial.println("[TOUCH] Calibration: hold each cross until it moves; lift between points (4 points)");
}
void ScreenUI::saveCalibration() {
  Preferences prefs; const StoredCalibration stored{calibrationMagic,calibration_};
  bool saved=false;
  if (prefs.begin("macroTouch",false)) {
    saved=prefs.putBytes("landscape1",&stored,sizeof(stored))==sizeof(stored); prefs.end();
  }
  Serial.println(saved?"[TOUCH] Calibration saved":"[TOUCH] Calibration active, but NVS save failed");
}
void ScreenUI::touch(bool down,uint16_t x,uint16_t y,uint32_t now) {
  rawX_=x; rawY_=y;
  if (!down) {
    // Resistive contact can briefly drop while held: re-arm after stable release.
    if (!wasDown_) { waitRelease_=false; return; }
    if (!releasing_) { releasing_=true; releaseAt_=now; }
    if (uint32_t(now-releaseAt_)<40) return;
    wasDown_=false; latched_=false; waitRelease_=false; samples_=0; originCard_=-1;
    return;
  }
  releasing_=false;
  if (!wasDown_) {
    ++touchCount_; wasDown_=true; pressedAt_=now; samples_=0; latched_=false;
    originCard_=-1;
    TouchPoint origin;
    if (!calibrating_ && calibration_.map(x,y,origin)) originCard_=touchBulbAt(page_,origin.x,origin.y);
  }
  if (calibrating_) {
    if (waitRelease_) return;
    if (!samples_ || x<minX_ || x>maxX_ || y<minY_ || y>maxY_) {
      samples_=0;sumX_=sumY_=0;
      minX_=x>60?x-60:0; maxX_=x+60; minY_=y>60?y-60:0; maxY_=y+60;
      pressedAt_=now;
    }
    sumX_+=x;sumY_+=y;++samples_;
    if (samples_<12 || uint32_t(now-pressedAt_)<120) return;
    const TouchPoint measured{int16_t(sumX_/samples_),int16_t(sumY_/samples_)};
    waitRelease_=true; samples_=0;
    if (point_<3) {
      calibration_.raw[point_]=measured; ++point_; dirty_=true;
      if (point_==3 && !calibration_.valid()) { point_=0; retryCalibration_=true; }
    } else {
      TouchPoint check;
      const bool mapped=calibration_.map(measured.x,measured.y,check);
      const int dx=check.x-targets[3].x,dy=check.y-targets[3].y;
      if (!mapped || dx*dx+dy*dy>900) {
        point_=0;retryCalibration_=true;dirty_=true;
        Serial.println("[TOUCH] Validation failed; retry calibration");
      } else {
        saveCalibration(); calibrating_=false; page_=0; dirty_=true; latched_=true;
      }
    }
    return;
  }
  if (latched_) return;
  TouchPoint p;
  if (!calibration_.map(x,y,p)) return;
  if (p.x>=430 && p.x<468 && p.y>=4 && p.y<28 && uint32_t(now-pressedAt_)>=800) {
    calibrate(); return;
  }
  const int card=touchBulbAt(page_,p.x,p.y);
  if (card!=originCard_) originCard_=-1; // Dragging into/away from a card does not toggle it.
  if (uint32_t(now-pressedAt_)<60) return;
  const int page=touchPageAt(p.x,p.y);
  if (page>=0) {
    latched_=true;
    selectPage(page);
    return;
  }
  if(page_==2 && p.y>=80 && p.y<110 && p.x>=12 && p.x<468) {
    latched_=true;hotTv_=p.x>=240;dirty_=true;return;
  }
  if(page_==2 && p.x>=350 && p.x<468 && p.y>=280 && p.y<312) {
    latched_=true;HotList::refresh();return;
  }
  if(touchWeatherRefresh(page_,p.x,p.y)) {
    latched_=true;Weather::refresh();return;
  }
  if (card>=0 && card==originCard_) {
    latched_=true; // One enqueue until a stable release, even if queueing fails.
    const bool queued=FeitCloud::toggleFromTouch(card);
    Serial.printf("[UI] Tap BULB%u queued=%s\n",card+1,queued?"yes":"no");
  }
}
void ScreenUI::drawCalibration() {
  d_.fillScreen(BG); d_.setTextColor(INK,BG);d_.setTextSize(2);d_.setCursor(12,6);
  d_.printf("TOUCH CALIBRATION %u/4",point_+1);
  d_.setTextSize(1);d_.setCursor(12,30);
  d_.print(retryCalibration_?"Retry: hold the CENTER of each cross, then lift.":"Hold cross until it moves, then lift. Keys remain active.");
  const auto p=targets[point_];
  d_.fillRect(p.x-14,p.y-2,29,5,ACCENT); d_.fillRect(p.x-2,p.y-14,5,29,ACCENT);
  d_.fillRect(p.x-2,p.y-2,5,5,INK);
}
void ScreenUI::chrome() {
  d_.fillScreen(BG); d_.setTextColor(INK,BG);
  d_.fillCircle(20,22,5,ACCENT);d_.fillCircle(12,13,2,ACCENT);d_.fillCircle(18,9,2,ACCENT);
  d_.fillCircle(24,9,2,ACCENT);d_.fillCircle(29,14,2,ACCENT);
  d_.fillRect(430,4,38,24,PANEL);d_.setTextSize(1);d_.setCursor(440,12);d_.print("CAL");
  const char* names[]={"LIGHTS","WEATHER","HOT&TV","9 KEYS"};
  for (uint8_t i=0;i<4;++i) {
    const int x=12+i*116; const uint16_t color=page_==i?CREAM:PANEL;
    d_.fillRoundRect(x,36,108,36,8,color);d_.setTextColor(page_==i?DARK:INK,color);d_.setTextSize(2);
    d_.setCursor(x+(108-strlen(names[i])*12)/2,46);d_.print(names[i]);
    if(page_==i)d_.fillRect(x,69,108,3,ACCENT);
  }
}
void ScreenUI::drawClock(bool force) {
  const time_t utc=time(nullptr);
  if(!force && oldClockSecond_==int64_t(utc))return;
  oldClockSecond_=utc;
  d_.fillRect(34,0,180,32,BG);
  d_.setTextColor(INK,BG);d_.setTextSize(2);d_.setCursor(36,5);
  tm local{};
  if(!LocalClock::valid(utc) || !localtime_r(&utc,&local)) {
    d_.print("--:--:--");d_.setTextSize(1);d_.setTextColor(MUTED,BG);d_.setCursor(36,23);d_.print("WAITING FOR TIME");return;
  }
  char text[24];strftime(text,sizeof(text),"%H:%M:%S",&local);d_.print(text);
  strftime(text,sizeof(text),"%Z",&local);d_.setTextSize(1);d_.setTextColor(ACCENT,BG);d_.setCursor(142,9);d_.print(text);
  strftime(text,sizeof(text),"%a %m/%d/%Y",&local);d_.setTextColor(MUTED,BG);d_.setCursor(36,23);d_.print(text);
}
void ScreenUI::render(const ScreenModel& m) {
  doganim::visible(page_==0 && !calibrating_);
  if (calibrating_) { if(dirty_){drawCalibration();dirty_=false;} return; }
  const bool force=dirty_; const int bits=cloudBits(m);
  if (force) chrome();
  drawClock(force);
  if (force || oldBits_!=bits) {
    d_.fillRect(222,4,200,24,BG);d_.setTextColor(m.cloud.wifi?ACCENT:MUTED,BG);d_.setTextSize(1);d_.setCursor(224,8);
    d_.printf("WiFi:%s  I2C:%s",m.cloud.wifi?"OK":"WAIT",m.online?"OK":"OFF");
    d_.setCursor(224,19);d_.print(m.cloud.busy?"CLOUD REQUEST IN PROGRESS":"READY");
  }
  if (page_==0) {
    const bool known[]={m.cloud.known,m.cloud.secondKnown,m.cloud.thirdKnown};
    const bool power[]={m.cloud.power,m.cloud.secondPower,m.cloud.thirdPower};
    const char* names[]={"FEIT","ROOM C2","ROOM C1"};
    const uint8_t keyNames[]={9,8,2};
    for(uint8_t i=0;i<3;++i) {
      const int stateMask=(1<<(3+2*i))|(1<<(4+2*i));
      if(!force && !((oldBits_^bits)&stateMask))continue;
      const int x=12+i*156;
      const bool on=known[i]&&power[i];
      const uint16_t color=!known[i]?UNKNOWN:on?CREAM:SLEEP;
      d_.fillRoundRect(x,82,144,188,10,color);
      d_.setTextColor(DARK,color);d_.setTextSize(2);d_.setCursor(x+10,90);d_.print(names[i]);
      d_.setTextSize(1);d_.setCursor(x+10,109);d_.printf("K%u",keyNames[i]);
      d_.fillCircle(x+129,98,5,on?AMBER:0x7bcf);
      dogart::draw(d_,i,on,x+8,114,color);
      const uint16_t pill=on?AMBER:known[i]?0x5aeb:0x7bcf;
      d_.fillRoundRect(x+12,226,120,24,11,pill);
      d_.setTextSize(2);d_.setTextColor(on?DARK:INK,pill);
      d_.setCursor(x+(!known[i]?66:on?60:54),230);d_.print(!known[i]?"?":on?"ON":"OFF");
      d_.setTextSize(1);d_.setTextColor(0x5acb,color);d_.setCursor(x+33,257);d_.print("TAP TO TOGGLE");
    }
    doganim::update(d_,known,power,force,wasDown_);
    if(force || oldBits_!=bits) {
      const bool allKnown=known[0]&&known[1]&&known[2];const bool anyOn=power[0]||power[1]||power[2];
      d_.fillRect(12,278,228,34,BG);d_.setTextColor(INK,BG);d_.setTextSize(1);d_.setCursor(12,279);
      if(allKnown)d_.printf("K4: PRESS FOR %s",anyOn?"ALL OFF":"ALL ON"); else d_.print("K4: STATE UNKNOWN");
      d_.setTextColor(MUTED,BG);d_.setCursor(12,300);d_.print("K1 PINK / K6 ORANGE");
    }
    if(force || oldPot_!=m.pot) {
      d_.fillRect(252,276,216,36,BG);d_.setTextColor(INK,BG);d_.setTextSize(2);d_.setCursor(252,279);
      d_.printf("BRIGHT %u%%",(m.brightness+5)/10);
      d_.fillRoundRect(252,302,216,9,4,PANEL);
      const uint16_t filled=uint32_t(m.pot)*216/4095;
      if(filled>4)d_.fillRoundRect(252,303,filled,7,3,ACCENT); else if(filled)d_.fillRect(252,303,filled,7,ACCENT);
    }
  } else if(page_==1) {
    drawWeather(force,m.cloud.wifi);
  } else if(page_==2) {
    drawHot(force,m.cloud.wifi);
  } else {
    for (uint8_t i=0;i<9;++i) {
      if(force || ((oldMask_^m.mask)&(1u<<i)) || oldCounts_[i]!=m.counts[i]) {
        const int x=12+(i%3)*154,y=86+(i/3)*56;
        const bool down=m.mask&(1u<<i);const uint16_t color=down?0x0328:PANEL;
        d_.fillRect(x,y,148,50,color);d_.setTextColor(INK,color);d_.setTextSize(2);d_.setCursor(x+8,y+6);d_.printf("K%u",i+1);
        d_.setTextSize(1);d_.setCursor(x+80,y+10);d_.print(down?"DOWN":"UP");
        d_.setCursor(x+8,y+33);d_.printf("PRESS #%lu",(unsigned long)m.counts[i]);
      }
    }
    if(force || oldPot_!=m.pot) {
      d_.fillRect(12,258,456,18,BG);d_.setTextSize(1);d_.setTextColor(INK,BG);d_.setCursor(12,262);
      d_.printf("POT %u/4095    BRIGHT TARGET %u/1000    HELD",m.pot,m.brightness);
    }
    if(force || oldMask_!=m.mask || oldErrors_!=m.errors || oldOverflows_!=m.overflows || oldTouches_!=touchCount_ || oldBits_!=bits || wasDown_) {
      d_.fillRect(12,278,456,34,BG);d_.setTextSize(1);d_.setTextColor(MUTED,BG);d_.setCursor(12,280);
      d_.printf("HELD 0x%03x  I2C:%s  ERR:%lu OV:%lu",m.mask,m.online?"ONLINE":"OFFLINE",(unsigned long)m.errors,(unsigned long)m.overflows);
      d_.setCursor(12,299);d_.printf("TOUCH #%lu  RAW X=%u Y=%u  CAL: hold top right",(unsigned long)touchCount_,rawX_,rawY_);
    }
  }
  oldBits_=bits;oldMask_=m.mask;oldPot_=m.pot;oldErrors_=m.errors;oldOverflows_=m.overflows;oldTouches_=touchCount_;
  for(uint8_t i=0;i<9;++i)oldCounts_[i]=m.counts[i];
  dirty_=false;
}

namespace {
void weatherIcon(BoardDisplay& d,int x,int y,uint8_t code) {
  if(code<=2) {
    const int sy=y+5;
    d.fillCircle(x-6,sy-6,13,AMBER);
    for(int s=-1;s<=1;s+=2) {
      d.drawLine(x-6+s*18,sy-6,x-6+s*23,sy-6,AMBER);
      d.drawLine(x-6,sy-6+s*18,x-6,sy-6+s*23,AMBER);
      d.drawLine(x-6+s*13,sy-19,x-6+s*17,sy-23,AMBER);
      d.drawLine(x-6+s*13,sy+7,x-6+s*17,sy+11,AMBER);
    }
  }
  if(code>=2) {
    d.fillCircle(x-13,y+1,11,MUTED);d.fillCircle(x+1,y-7,16,INK);
    d.fillCircle(x+17,y+2,11,MUTED);d.fillRoundRect(x-21,y,45,13,6,INK);
  }
  const bool snow=(code>=71 && code<=77) || code==85 || code==86;
  const bool rain=(code>=51 && code<=67) || (code>=80 && code<=82);
  if(rain)for(int n=-1;n<=1;++n)d.drawLine(x+n*14,y+19,x+n*14-4,y+27,ACCENT);
  if(snow)for(int n=-1;n<=1;++n) {
    d.drawLine(x+n*14-3,y+23,x+n*14+3,y+23,INK);
    d.drawLine(x+n*14,y+20,x+n*14,y+26,INK);
  }
  if(code==45 || code==48) {d.drawFastHLine(x-23,y+20,48,MUTED);d.drawFastHLine(x-17,y+26,36,MUTED);}
  if(code>=95) {d.fillTriangle(x+1,y+14,x-9,y+24,x+5,y+20,AMBER);d.fillTriangle(x-1,y+20,x+9,y+17,x-6,y+29,AMBER);}
}
}
void ScreenUI::drawWeather(bool force,bool wifi) {
  const auto weather=Weather::snapshot();
  const uint32_t minute=millis()/60000;
  if(!force && weather.revision==oldWeatherRevision_ && minute==oldWeatherMinute_ && (oldBits_&1)==wifi)return;
  oldWeatherRevision_=weather.revision;oldWeatherMinute_=minute;
  d_.fillRect(0,78,480,242,BG);
  d_.setTextSize(2);d_.setTextColor(INK,BG);d_.setCursor(12,84);d_.print("SAMMAMISH");
  d_.setCursor(396,84);d_.print("98074");
  d_.setTextSize(1);d_.setTextColor(MUTED,BG);d_.setCursor(12,106);d_.print("3-DAY FORECAST / CELSIUS");
  static const char* weekdays[]={"SUN","MON","TUE","WED","THU","FRI","SAT"};
  for(unsigned i=0;i<3;++i) {
    const int x=12+i*156;d_.fillRoundRect(x,122,144,148,9,PANEL);
    d_.setTextColor(INK,PANEL);d_.setTextSize(2);d_.setCursor(x+18,132);
    if(!weather.valid) {
      d_.print("-- --/--");d_.setCursor(x+48,184);d_.print("--");
      d_.setTextSize(1);d_.setTextColor(MUTED,PANEL);d_.setCursor(x+27,238);d_.print("WAITING DATA");continue;
    }
    const auto& day=weather.days[i];
    d_.setTextSize(1);d_.setCursor(x+63,130);d_.print(weekdays[Weather::weekday(day.date)]);
    d_.setTextSize(2);d_.setCursor(x+42,142);d_.printf("%.2s/%.2s",day.date+5,day.date+8);
    weatherIcon(d_,x+72,184,day.code);
    d_.setTextSize(1);d_.setCursor(x+(144-strlen(Weather::description(day.code))*6)/2,216);d_.print(Weather::description(day.code));
    d_.setTextSize(2);d_.setTextColor(INK,PANEL);d_.setCursor(x+12,231);
    d_.printf("%d/%d C",int(lroundf(day.high)),int(lroundf(day.low)));
    d_.setTextSize(1);d_.setTextColor(ACCENT,PANEL);d_.setCursor(x+33,255);d_.printf("RAIN %u%%",day.rain);
  }
  const time_t utc=time(nullptr),local=utc+weather.utcOffset;
  tm current{};gmtime_r(&local,&current);char today[11];strftime(today,sizeof(today),"%Y-%m-%d",&current);
  const bool stale=weather.valid && (utc<1700000000 || utc<weather.fetchedAt
    || uint32_t(utc-weather.fetchedAt)>3600 || strcmp(today,weather.days[0].date)!=0);
  const char* status=weather.loading?"UPDATING...":!wifi?(weather.valid?"OFFLINE - CACHED":"OFFLINE - NO DATA"):
    !weather.valid?"WAITING FOR FORECAST":!weather.lastFetchOk?"CACHED - RETRYING":stale?"STALE - RETRYING":"UPDATED EVERY 30 MIN";
  d_.setTextSize(1);d_.setTextColor(stale || !wifi?AMBER:MUTED,BG);d_.setCursor(12,281);d_.print(status);
  d_.setCursor(12,294);
  if(weather.valid) {
    const time_t fetched=weather.fetchedAt+weather.utcOffset;tm t{};gmtime_r(&fetched,&t);
    d_.printf("Updated %02d/%02d %02d:%02d %s",t.tm_mon+1,t.tm_mday,t.tm_hour,t.tm_min,weather.utcOffset==-25200?"PDT":weather.utcOffset==-28800?"PST":"LOCAL");
  } else d_.print("WiFi + time sync required");
  d_.setCursor(12,308);d_.print("Source: Open-Meteo");
  d_.fillRoundRect(350,280,118,32,7,CREAM);d_.setTextColor(DARK,CREAM);d_.setTextSize(2);d_.setCursor(367,289);d_.print("REFRESH");
}

void ScreenUI::drawHot(bool force,bool wifi) {
  const auto hot=HotList::snapshot();const auto voice=Speech::snapshot();const uint32_t minute=millis()/60000;const bool font=ChineseFont::ready();
  if(!force && hot.revision==oldHotRevision_ && minute==oldHotMinute_ && font==oldFontReady_ && voice.revision==oldSpeechRevision_ && (oldBits_&1)==wifi)return;
  oldHotRevision_=hot.revision;oldHotMinute_=minute;oldFontReady_=font;oldSpeechRevision_=voice.revision;
  const uint32_t started=millis();d_.fillRect(0,78,480,242,BG);
  for(int i=0;i<2;++i) {
    const int x=12+232*i;const uint16_t color=hotTv_==bool(i)?CREAM:PANEL;
    d_.fillRoundRect(x,80,224,28,6,color);
    if(font)ChineseFont::text(d_,i?"热播剧 TOP5":"娱乐热搜 TOP5",x+12,86,200,1,hotTv_==bool(i)?DARK:INK,color);
    else {d_.setTextSize(2);d_.setTextColor(hotTv_==bool(i)?DARK:INK,color);d_.setCursor(x+12,87);d_.print(i?"TV TOP 5":"HOT TOP 5");}
  }
  for(int i=0;i<5;++i) {
    int y=112+33*i;const uint16_t rowColor=(voice.tv==hotTv_ && voice.index==i && voice.mode>=Speech::Queued && voice.mode<=Speech::Playing)?0x0328:PANEL;d_.fillRoundRect(12,y,456,32,4,rowColor);
    d_.setTextSize(2);d_.setTextColor(ACCENT,rowColor);d_.setCursor(20,y+8);d_.print(i+1);
    if(font && hot.valid)ChineseFont::text(d_,hotTv_?hot.dramas[i]:hot.topics[i],42,y,418,2,INK,rowColor);
    else {d_.setTextSize(1);d_.setTextColor(MUTED,rowColor);d_.setCursor(44,y+12);d_.print(!font?"SD CHINESE FONT REQUIRED":"WAITING FOR ONLINE TOP 5");}
  }
  const bool stale=hot.valid && HotList::scheduleSlot(time(nullptr))>HotList::scheduleSlot(hot.fetchedAt);
  const char* state=hot.loading?"UPDATING...":!wifi?"OFFLINE / CACHED":!hot.valid?"WAITING / RETRYING":!hot.lastFetchOk?"CACHED / RETRYING":stale?"STALE / RETRYING":"DAILY 07:00 LOCAL";
  d_.setTextSize(1);d_.setTextColor(stale || !wifi?AMBER:MUTED,BG);d_.setCursor(12,282);
  if(voice.tv==hotTv_ && voice.mode!=Speech::Idle)d_.printf("K3 #%u: %s",voice.index+1,voice.detail);else d_.print(state);
  d_.setCursor(12,295);
  if(hot.valid){time_t when=hot.fetchedAt;tm t{};localtime_r(&when,&t);char text[40];strftime(text,sizeof(text),"Updated %m/%d %H:%M %Z",&t);d_.print(text);}
  else d_.print("WiFi + time sync required");
  d_.setCursor(12,308);d_.print(hotTv_?"Baidu TV / 60s   K3: NEXT":"Weibo entertainment   K3: NEXT");
  d_.fillRoundRect(350,280,118,32,7,CREAM);d_.setTextColor(DARK,CREAM);d_.setTextSize(2);d_.setCursor(367,289);d_.print("REFRESH");
  Serial.printf("[HOT] DRAW section=%s valid=%u font=%u elapsed=%lu ms\n",hotTv_?"TV":"HOT",hot.valid,font,(unsigned long)(millis()-started));
}

