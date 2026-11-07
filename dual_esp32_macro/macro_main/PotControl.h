#pragma once
#include <stdint.h>

// Application-side Schmitt quantizer. No changes to the secondary/protocol.
class PotControl {
 public:
  constexpr bool update(uint16_t raw) {
    if (raw <= 16) raw = 0;
    if (raw >= 4079) raw = 4095;
    const int32_t scaled = static_cast<int32_t>(raw) * 100;
    const uint8_t rounded = static_cast<uint8_t>((scaled + 2047) / 4095);
    if (!ready_) { ready_ = true; level_ = rounded; return true; }
    // Additional 20 ADC counts of hysteresis around the half-step boundary.
    const int32_t center = static_cast<int32_t>(level_) * 4095;
    if ((rounded > level_ && scaled >= center + 2047 + 2000) ||
        (rounded < level_ && scaled <= center - 2047 - 2000)) {
      level_ = rounded;
      return true;
    }
    return false;
  }
  constexpr uint8_t level() const { return level_; }
 private:
  bool ready_ = false;
  uint8_t level_ = 0;
};

constexpr bool testPotControl() {
  PotControl q;
  if (!q.update(2048) || q.level() != 50) return false;
  for (unsigned i = 0; i < 50; ++i) {
    if (q.update(2030) || q.update(2065)) return false;
  }
  if (!q.update(2095) || q.level() != 51) return false;
  for (unsigned i = 0; i < 50; ++i) {
    if (q.update(2070) || q.update(2090)) return false;
  }
  if (!q.update(2040) || q.level() != 50) return false;
  if (!q.update(0) || q.level() != 0 || q.update(12)) return false;
  if (!q.update(4095) || q.level() != 100 || q.update(4080)) return false;
  return true;
}
static_assert(testPotControl(), "Pot quantization/hysteresis/endpoints tests failed");
