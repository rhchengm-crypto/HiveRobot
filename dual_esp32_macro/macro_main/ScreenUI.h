#pragma once
#include "BoardDisplay.h"
#include "FeitCloud.h"
#include "TouchGeometry.h"
struct ScreenModel {
  FeitCloud::State cloud;
  bool online;
  uint16_t mask, pot, brightness;
  uint32_t errors, overflows;
  const uint32_t* counts;
};
class ScreenUI {
 public:
  explicit ScreenUI(BoardDisplay& display): d_(display) {}
  void begin(); // Display hardware already initialized in landscape.
  void changePage(int8_t direction); // Negative: left; positive: right; stop at page boundaries.
  void showPage(uint8_t page) { selectPage(page); }
  void invalidate() { dirty_=true; }
  bool hotPageActive() const {return !calibrating_ && page_==2;}
  bool hotShowsTv() const {return hotTv_;}
  void showHotSection(bool tv) {if(calibrating_)return;selectPage(2);if(hotTv_!=tv){hotTv_=tv;dirty_=true;}}
  void touch(bool down, uint16_t rawX, uint16_t rawY, uint32_t now);
  void render(const ScreenModel& model);
 private:
  void selectPage(uint8_t page);
  void calibrate();
  void drawCalibration();
  void chrome();
  void drawClock(bool force);
  void drawWeather(bool force,bool wifi);
  void drawHot(bool force,bool wifi);
  bool hotTv_=false;
  uint32_t oldHotRevision_=UINT32_MAX,oldHotMinute_=UINT32_MAX,oldSpeechRevision_=UINT32_MAX;
  bool oldFontReady_=false;
  void saveCalibration();
  BoardDisplay& d_;
  TouchCalibration calibration_{};
  bool calibrating_=false, dirty_=true, wasDown_=false, latched_=false, waitRelease_=false;
  bool retryCalibration_=false;
  bool releasing_=false;
  int8_t originCard_=-1;
  uint32_t releaseAt_=0;
  uint8_t page_=0, point_=0, samples_=0;
  uint32_t pressedAt_=0, sumX_=0, sumY_=0, touchCount_=0;
  uint16_t minX_=0,maxX_=0,minY_=0,maxY_=0, rawX_=0,rawY_=0;
  uint32_t oldWeatherRevision_=UINT32_MAX,oldWeatherMinute_=UINT32_MAX;
  int64_t oldClockSecond_=-1;
  int oldBits_=-1, oldMask_=-1, oldPot_=-1;
  uint32_t oldErrors_=UINT32_MAX, oldOverflows_=UINT32_MAX, oldTouches_=UINT32_MAX;
  uint32_t oldCounts_[9]={};
};
