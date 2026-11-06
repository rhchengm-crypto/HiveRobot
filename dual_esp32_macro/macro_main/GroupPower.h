#pragma once
#include <stdint.h>
struct GroupTarget { bool valid; bool on; };
constexpr GroupTarget groupPowerTarget(uint8_t known, uint8_t on, uint8_t members) {
  return {members != 0 && (known & members) == members, (on & members) == 0};
}
constexpr bool testGroupPower() {
  // Both OFF -> ON; either/both ON -> OFF. Unknown state must not be inverted blindly.
  if (!groupPowerTarget(3,0,3).valid || !groupPowerTarget(3,0,3).on) return false;
  for (uint8_t on=1; on<4; ++on)
    if (!groupPowerTarget(3,on,3).valid || groupPowerTarget(3,on,3).on) return false;
  for (uint8_t known=0; known<3; ++known)
    if (groupPowerTarget(known,0,3).valid) return false;
  if (groupPowerTarget(0,0,0).valid) return false;
  // A separately controlled lamp outside the group must not affect the decision.
  return groupPowerTarget(3,4,3).on && groupPowerTarget(1,2,1).on;
}
static_assert(testGroupPower(), "Master power mixed/unknown/all-off state tests failed");

constexpr bool testThreeLampGroupPower() {
  for (uint8_t on=0; on<8; ++on) {
    const auto target=groupPowerTarget(7,on,7);
    if (!target.valid || target.on != (on==0)) return false;
  }
  for (uint8_t known=0; known<7; ++known)
    for (uint8_t on=0; on<8; ++on)
      if (groupPowerTarget(known,on,7).valid) return false;
  return groupPowerTarget(7,8,7).on;
}
static_assert(testThreeLampGroupPower(), "Three-lamp all 8 power combinations and unknown-state tests failed");
