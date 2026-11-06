#pragma once
#include <stdint.h>
// Display-only inactivity: network and the key bridge remain running.
class ScreenIdle {
 public:
  static constexpr uint32_t timeout=5u*60u*1000u;
  constexpr void begin(uint32_t now) {last_=now;asleep_=false;}
  constexpr bool activity(uint32_t now) {
    const bool woke=asleep_;last_=now;asleep_=false;return woke;
  }
  constexpr bool tick(uint32_t now,bool held=false) {
    if(asleep_)return false;
    if(held){last_=now;return false;}
    if(idleFor(now)<timeout)return false;
    asleep_=true;return true;
  }
  constexpr void sleepNow(){asleep_=true;}
  constexpr bool asleep() const {return asleep_;}
  constexpr uint32_t idleFor(uint32_t now) const {return uint32_t(now-last_);}
 private:
  uint32_t last_=0;
  bool asleep_=false;
};
constexpr bool testScreenIdle() {
  ScreenIdle idle;idle.begin(100);
  if(idle.tick(300099) || !idle.tick(300100) || !idle.asleep())return false;
  if(!idle.activity(300101) || idle.asleep() || idle.idleFor(300101))return false;
  if(idle.tick(600100) || idle.tick(600101,true) || idle.tick(900100))return false;
  if(!idle.tick(900101))return false;
  // The same wake event remains available to every original key handler.
  for(unsigned key=1;key<=9;++key) {
    idle.sleepNow();if(!idle.activity(key) || idle.asleep())return false;
  }
  idle.begin(0xffffff00u);
  return !idle.tick(uint32_t(0xffffff00u+299999u))
    && idle.tick(uint32_t(0xffffff00u+300000u));
}
static_assert(testScreenIdle(),"Five-minute boundary, activity, held keys and millis wrap failed");
