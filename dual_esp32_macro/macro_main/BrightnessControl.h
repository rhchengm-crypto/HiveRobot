#pragma once
#include <stdint.h>

// One latest-value slot; caller supplies cross-core locking. No network queue flood.
class BrightnessControl {
 public:
  static constexpr uint16_t mapPercent(uint8_t p) {
    return 10 + (static_cast<uint32_t>(p > 100 ? 100 : p) * 990 + 50) / 100;
  }
  constexpr void update(uint8_t percent, uint32_t now) {
    const uint16_t value = mapPercent(percent);
    if (hasValue_ && value == value_) return;
    if (!dirty_) firstAt_ = now;
    value_ = value; hasValue_ = dirty_ = true; changedAt_ = now;
  }
  constexpr bool take(uint32_t now, uint16_t& value) {
    if (!dirty_ || (sent_ && static_cast<uint32_t>(now - sentAt_) < 750)) return false;
    // Send a settled position after 150ms, or periodic latest value during a turn.
    if (static_cast<uint32_t>(now - changedAt_) < 150 &&
        static_cast<uint32_t>(now - firstAt_) < 1000) return false;
    value = value_; dirty_ = false; sent_ = true; sentAt_ = now; return true;
  }
  constexpr void cancel() { dirty_ = false; }
 private:
  bool hasValue_ = false, dirty_ = false, sent_ = false;
  uint16_t value_ = 10;
  uint32_t firstAt_ = 0, changedAt_ = 0, sentAt_ = 0;
};
constexpr bool testBrightnessControl() {
  if (BrightnessControl::mapPercent(0) != 10 || BrightnessControl::mapPercent(100) != 1000) return false;
  BrightnessControl c; uint16_t v = 0;
  c.update(10, 0); c.update(50, 100);
  if (c.take(249, v) || !c.take(250, v) || v != 505 || c.take(251, v)) return false;
  // Input arriving during a blocking request must remain as the next target.
  c.update(100, 300);
  if (c.take(999, v) || !c.take(1000, v) || v != 1000) return false;
  c.update(0, 1100); c.cancel(); if (c.take(3000, v)) return false;
  BrightnessControl continuous;
  for (uint32_t t = 0; t < 1000; t += 100) {
    continuous.update(t / 100, t); if (continuous.take(t, v)) return false;
  }
  continuous.update(99, 1000);
  if (!continuous.take(1000, v) || v != BrightnessControl::mapPercent(99)) return false;
  BrightnessControl wrap;
  wrap.update(0, UINT32_MAX - 99);
  if (wrap.take(49, v) || !wrap.take(50, v) || v != 10) return false;
  return true;
}
static_assert(testBrightnessControl(), "Brightness coalescing/rate-limit/wrap tests failed");

constexpr bool testIndependentBrightnessSlots() {
  BrightnessControl lamps[2]; uint16_t value=0;
  for (auto& lamp: lamps) lamp.update(30,0);
  if (!lamps[0].take(150,value) || value!=307) return false;
  // Lamp 1 may be OFF/busy: its pending value must survive lamp 0's send.
  for (auto& lamp: lamps) lamp.update(80,200);
  if (!lamps[1].take(350,value) || value!=802) return false;
  if (lamps[0].take(899,value) || !lamps[0].take(900,value) || value!=802) return false;
  for (auto& lamp: lamps) lamp.update(100,1000);
  lamps[0].cancel();
  return !lamps[0].take(2000,value) && lamps[1].take(2000,value) && value==1000;
}
static_assert(testIndependentBrightnessSlots(), "Two-lamp latest brightness isolation/deferred send tests failed");

constexpr bool testThreeLampBrightnessSlots() {
  BrightnessControl lamps[3]; uint16_t value=0;
  for (auto& lamp: lamps) lamp.update(40,0);
  if (!lamps[0].take(150,value) || value!=406) return false;
  for (auto& lamp: lamps) lamp.update(90,200);
  if (!lamps[1].take(350,value) || value!=901) return false;
  // Third lamp may remain OFF: retain its latest target independently.
  if (!lamps[2].take(5000,value) || value!=901) return false;
  if (!lamps[0].take(5000,value) || value!=901) return false;
  for (auto& lamp: lamps) lamp.update(100,5100);
  lamps[0].cancel();
  return !lamps[0].take(6000,value) && lamps[1].take(6000,value) && value==1000
      && lamps[2].take(6000,value) && value==1000;
}
static_assert(testThreeLampBrightnessSlots(), "Three-lamp latest-value and independent cancellation tests failed");
