#pragma once
#include <stdint.h>
namespace colorpreset {
struct Preset { uint16_t hue,saturation; };
constexpr Preset kPink{6,118},kOrange{30,1000};
constexpr uint16_t kWarmTemperature=0;
constexpr Preset forKey(uint8_t key) { return key==6?kOrange:kPink; }
static_assert(forKey(1).hue==6 && forKey(1).saturation==118
  && forKey(6).hue==30 && forKey(6).saturation==1000,"Pink/orange key mapping failed");
class Toggle {
 public:
  constexpr bool colored() const { return colored_; }
  // Advance on each processed color-key event, independently of device replies.
  constexpr void advance() { colored_=!colored_; }
 private:
  bool colored_=true;
};
constexpr bool testToggle() {
  Toggle t;
  for (uint8_t press=0;press<10;++press) {
    if(t.colored()!=(press%2==0))return false;
    t.advance();
  }
  return t.colored();
}
static_assert(testToggle(), "K1 must alternate each event independently of acknowledgements");
}

namespace colorpreset {
constexpr bool testIndependentColorKeys() {
  Toggle pink, orange;
  pink.advance();
  if(pink.colored() || !orange.colored())return false;
  orange.advance();
  if(pink.colored() || orange.colored())return false;
  pink.advance();
  return pink.colored() && !orange.colored() && kPink.hue!=kOrange.hue;
}
static_assert(testIndependentColorKeys(), "K1 and K6 color/warm toggles must be independent");
}
