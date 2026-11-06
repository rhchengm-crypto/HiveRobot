#include "ScreenIdle.h"
#include "Weather.h"
#include "HotList.h"
#include "Speech.h"
#include "ChineseFont.h"
#include "DogAnimation.h"
#include <Arduino.h>
#include "KeyBridge.h"
#include "BoardDisplay.h"
#include "PotControl.h"
#include "FeitCloud.h"
#include "BrightnessControl.h"
#include "ScreenUI.h"
#include "SdDiagnostic.h"

// All application mappings stay on this board. Set true if the outer pot wires
// are reversed; no secondary firmware change is needed.
constexpr bool POT_REVERSED = false;
KeyBridge keys;
BoardDisplay display;
uint32_t pressCount[macrokeys::kCount] = {};
uint16_t mappedPot = 0;
PotControl potControl;
uint32_t uiReadyAt = 0;
ScreenUI ui(display);
ScreenIdle screenIdle;
bool backlightPending=false,discardWakeTouch=false;
uint32_t wakeTouchReleasedAt=0;
void screenActivity(uint32_t now,const char* source) {
  if(!screenIdle.activity(now))return;
  // Redraw before restoring the light; the original key callback keeps running.
  backlightPending=true;ui.invalidate();
  Serial.printf("[SCREEN] WAKE source=%s t=%lu\n",source,(unsigned long)now);
}
void screenOff(const char* reason) {
  screenIdle.sleepNow();backlightPending=false;display.setBacklight(false);doganim::visible(false);
  Serial.printf("[SCREEN] OFF reason=%s idle_ms=%lu backlight=%u\n",reason,(unsigned long)screenIdle.idleFor(millis()),display.backlightEnabled());
}
void screenStatus() {
  Serial.printf("[SCREEN] STATUS asleep=%u backlight=%u pin27=%d idle_ms=%lu timeout_ms=%lu held=0x%03x\n",screenIdle.asleep(),display.backlightEnabled(),digitalRead(27),(unsigned long)screenIdle.idleFor(millis()),(unsigned long)ScreenIdle::timeout,keys.held());
}

void onMacroKey(uint8_t key, bool down) {
  if (down) { screenActivity(millis(),"KEY"); ++pressCount[key - 1]; Weather::userActivity(); }
  Serial.printf("[KEY] K%u %s t=%lu ms\n", key, down ? "PRESS" : "RELEASE", (unsigned long)millis());
  if (key == 1 && down) FeitCloud::toggleAllColor();
  else if (key == 6 && down) FeitCloud::toggleAllOrangeColor();
  else if (key == 7 && down) ui.changePage(-1);
  else if (key == 5 && down) ui.changePage(1);
  else if (key == 4 && down) FeitCloud::toggleAll();
  else if (key == 9 && down) FeitCloud::toggle();
  else if (key == 8 && down) FeitCloud::toggleSecond();
  else if (key == 2 && down) FeitCloud::toggleThird();
  else if(key==3 && down && ui.hotPageActive())Speech::next(ui.hotShowsTv());
}
void onPotChanged(uint16_t raw) {
  const uint16_t value = POT_REVERSED ? macrokeys::kPotMax - raw : raw;
  if (!potControl.update(value)) return;
  const unsigned percent = potControl.level();
  mappedPot = (static_cast<uint32_t>(percent) * macrokeys::kPotMax + 50) / 100;
  Serial.printf("[POT] raw=%u value=%u percent=%u t=%lu ms\n", raw, mappedPot, percent, (unsigned long)millis());
  // First report is the boot baseline, not a user's brightness action.
  static bool baselineReady = false;
  if (!baselineReady) { baselineReady = true; return; }
  screenActivity(millis(),"POT");
  FeitCloud::setBrightnessPercent(percent);
}
void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(115200);
  if (!keys.begin(onMacroKey, onPotChanged)) {
    Serial.println("FATAL: key bridge init failed");
    while (true) delay(1000);
  }
  display.begin();
  ui.begin();
  mountSdCard();
  doganim::begin();
  ChineseFont::begin();
  if(!Speech::begin())Serial.println("[SPEECH] initialization failed");
  display.diagnose("before WiFi");
  uiReadyAt = millis();
  screenIdle.begin(uiReadyAt);
  Serial.println("[SCREEN] idle timeout=300000 ms; keys wake and execute original function");
  Serial.println("Main ready: 9KEY+POT fixed 8-byte protocol-v1; cold-start-v2 retained");
  Serial.println("Compile-time tests passed: nine-key debounce/edges, CRC/frames, ADC filter");
}
void loop() {
  keys.service();
  FeitCloud::serviceSerial();
  const int8_t requestedPage=FeitCloud::takePageRequest();
  if(requestedPage>=0)ui.showPage(requestedPage);
  const int8_t hotView=FeitCloud::takeHotViewRequest();if(hotView>=0)ui.showHotSection(hotView!=0);
  Speech::context(ui.hotPageActive(),ui.hotShowsTv());
  const auto speechRequest=FeitCloud::takeSpeechRequest();
  if(speechRequest==FeitCloud::SpeechRequest::Next && ui.hotPageActive()) {screenActivity(millis(),"SPEECH");Speech::next(ui.hotShowsTv());}
  else if(speechRequest==FeitCloud::SpeechRequest::Tone) {screenActivity(millis(),"SPEAKER TEST");Speech::tone();}
  const uint32_t now = millis();
  switch(FeitCloud::takeScreenRequest()) {
    case FeitCloud::ScreenRequest::Status:screenStatus();break;
    case FeitCloud::ScreenRequest::Off:screenOff("SERIAL");break;
    case FeitCloud::ScreenRequest::On:screenActivity(now,"SERIAL");break;
    default:break;
  }
  static bool cloudStarted = false, previousWifi = false, recoverPending = false;
  static uint32_t wifiUpAt = 0;
  bool forceRedraw = false;
  // Paint a complete frame before starting radio/network initialization.
  if (!cloudStarted && uint32_t(now - uiReadyAt) >= 500) {
    cloudStarted = true;
    HotList::begin();
    if(!Weather::begin())Serial.println("[WEATHER] worker initialization failed");
    if (!FeitCloud::begin()) Serial.println("[FEIT] ERROR: worker initialization failed");
  }
  const bool wifi = FeitCloud::state().wifi;
  if (wifi && !previousWifi) { wifiUpAt = now; recoverPending = true; }
  if (!wifi) recoverPending = false;
  previousWifi = wifi;
  const bool manualRecovery = FeitCloud::takeDisplayResetRequest();
  if (manualRecovery || (recoverPending && uint32_t(now - wifiUpAt) >= 500)) {
    recoverPending = false;
    display.diagnose("before recovery");
    display.begin();
    ui.invalidate();
    display.diagnose("after recovery");
    forceRedraw = true;
    Serial.println("[LCD] Full redraw requested; serial lcd repeats recovery");
  }
  static uint32_t touchAt=0, paintAt=0;
  if (static_cast<uint32_t>(now-touchAt)>=10) {
    touchAt=now;
    uint16_t x=0,y=0;const bool down=display.readTouch(x,y);
    if(down) {
      Weather::userActivity();
      if(screenIdle.asleep()){discardWakeTouch=true;wakeTouchReleasedAt=0;}
      screenActivity(now,"TOUCH");
    }
    if(discardWakeTouch) {
      if(down)wakeTouchReleasedAt=0;
      else if(!wakeTouchReleasedAt)wakeTouchReleasedAt=now;
      else if(uint32_t(now-wakeTouchReleasedAt)>=40)discardWakeTouch=false;
    } else ui.touch(down,x,y,now);
  }
  if(screenIdle.tick(now,keys.held()!=0))screenOff("IDLE");
  if (!screenIdle.asleep() && (forceRedraw || backlightPending || static_cast<uint32_t>(now-paintAt)>=50)) {
    paintAt=now;
    ui.render({FeitCloud::state(),keys.online(),keys.held(),mappedPot,
               BrightnessControl::mapPercent(potControl.level()),keys.errors(),keys.overflows(),pressCount});
    if(backlightPending) {
      display.setBacklight(true);backlightPending=false;
      Serial.println("[SCREEN] READY backlight=1");
    }
  }
  if(!screenIdle.asleep())doganim::service();
  delay(1);
}
