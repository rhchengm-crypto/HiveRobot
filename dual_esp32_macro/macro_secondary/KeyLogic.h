#pragma once
#include <stdint.h>
#include <stddef.h>

namespace macrokeys {
constexpr uint8_t kCount = 9;
constexpr uint16_t kMask = 0x01ff;
constexpr uint8_t kAddress = 0x42;
constexpr uint32_t kDebounceMs = 10;
constexpr uint8_t kMagic = 0xa9;
constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kFrameSize = 8;
constexpr uint16_t kPotMax = 4095;
constexpr uint16_t kPotDeadband = 8;
struct InputState { uint16_t keys = 0; uint16_t pot = 0; uint8_t sequence = 0; };
constexpr uint16_t pressedEdges(uint16_t oldMask, uint16_t newMask) {
  return static_cast<uint16_t>((~oldMask & newMask) & kMask);
}
constexpr uint16_t releasedEdges(uint16_t oldMask, uint16_t newMask) {
  return static_cast<uint16_t>((oldMask & ~newMask) & kMask);
}
constexpr uint16_t distance(uint16_t a, uint16_t b) { return a > b ? a - b : b - a; }
// CRC-8/SMBUS: polynomial 0x07, init 0, no reflection, xorout 0.
constexpr uint8_t crc8(const uint8_t* bytes, size_t count) {
  uint8_t crc = 0;
  for (size_t i = 0; i < count; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = static_cast<uint8_t>((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
  }
  return crc;
}
constexpr void encodeFrame(InputState state, uint8_t* frame) {
  frame[0] = kMagic; frame[1] = kProtocolVersion;
  frame[2] = state.keys & 0xff; frame[3] = state.keys >> 8;
  frame[4] = state.pot & 0xff; frame[5] = state.pot >> 8;
  frame[6] = state.sequence; frame[7] = crc8(frame, 7);
}
constexpr bool decodeFrame(const uint8_t* frame, size_t count, InputState& state) {
  if (count != kFrameSize || frame[0] != kMagic || frame[1] != kProtocolVersion ||
      frame[7] != crc8(frame, 7)) return false;
  const uint16_t keys = frame[2] | (static_cast<uint16_t>(frame[3]) << 8);
  const uint16_t pot = frame[4] | (static_cast<uint16_t>(frame[5]) << 8);
  if ((keys & ~kMask) != 0 || pot > kPotMax) return false;
  state = {keys, pot, frame[6]};
  return true;
}
class Debouncer {
 public:
  constexpr uint16_t update(uint16_t raw, uint32_t now) {
    for (uint8_t i = 0; i < kCount; ++i) {
      const uint16_t bit = static_cast<uint16_t>(1u << i);
      if ((raw & bit) != (candidate_ & bit)) { candidate_ ^= bit; since_[i] = now; }
      if ((candidate_ & bit) != (stable_ & bit) &&
          static_cast<uint32_t>(now - since_[i]) >= kDebounceMs) stable_ ^= bit;
    }
    return stable_;
  }
 private:
  uint16_t candidate_ = 0, stable_ = 0;
  uint32_t since_[kCount] = {};
};
// Integer EMA, updated every 4ms. Rounding converges even at the ADC endpoints.
class PotFilter {
 public:
  constexpr uint16_t update(uint16_t raw) {
    if (raw > kPotMax) raw = kPotMax;
    const int32_t target = static_cast<int32_t>(raw) * 16;
    if (!ready_) { value_ = target; ready_ = true; }
    else {
      const int32_t delta = target - value_;
      value_ += delta >= 0 ? (delta + 3) / 4 : -((-delta + 3) / 4);
    }
    return static_cast<uint16_t>((value_ + 8) / 16);
  }
 private:
  int32_t value_ = 0;
  bool ready_ = false;
};
constexpr bool testDebounce() {
  for (uint16_t mask = 0; mask <= kMask; ++mask) {
    Debouncer d;
    if (d.update(mask, 1) != 0 || d.update(mask, 10) != 0 || d.update(mask, 11) != mask) return false;
    if (d.update(0, 20) != mask || d.update(0, 29) != mask || d.update(0, 30) != 0) return false;
  }
  Debouncer bounce;
  bounce.update(3, 0); bounce.update(2, 5);
  if (bounce.update(2, 10) != 2) return false;
  bounce.update(3, 11);
  if (bounce.update(3, 21) != 3) return false;
  Debouncer wrap; wrap.update(kMask, 0xfffffffaUL);
  return wrap.update(kMask, 4) == kMask;
}
constexpr bool testEdges() {
  for (uint16_t a = 0; a <= kMask; ++a) {
    for (uint8_t i = 0; i < kCount; ++i) {
      const uint16_t b = a ^ (1u << i);
      if ((pressedEdges(a, b) & releasedEdges(a, b)) != 0 ||
          static_cast<uint16_t>((a | pressedEdges(a, b)) & ~releasedEdges(a, b)) != b) return false;
    }
    const uint16_t b = a ^ kMask;
    if (static_cast<uint16_t>((a | pressedEdges(a, b)) & ~releasedEdges(a, b)) != b) return false;
  }
  return true;
}
constexpr bool testProtocol() {
  const uint8_t reference[] = {'1','2','3','4','5','6','7','8','9'};
  if (crc8(reference, 9) != 0xf4) return false;
  const uint16_t masks[] = {0, 1, 0x100, kMask};
  const uint16_t pots[] = {0, 1, 2048, kPotMax};
  for (uint16_t mask : masks) for (uint16_t pot : pots) {
    uint8_t frame[kFrameSize] = {}; InputState state;
    encodeFrame({mask, pot, 255}, frame);
    if (!decodeFrame(frame, kFrameSize, state) || state.keys != mask || state.pot != pot || state.sequence != 255) return false;
    for (uint8_t byte = 0; byte < kFrameSize; ++byte) for (uint8_t bit = 0; bit < 8; ++bit) {
      frame[byte] ^= (1u << bit);
      if (decodeFrame(frame, kFrameSize, state)) return false;
      frame[byte] ^= (1u << bit);
    }
    if (decodeFrame(frame, kFrameSize - 1, state)) return false;
  }
  uint8_t frame[kFrameSize] = {}; InputState state;
  encodeFrame({0x0200, 0, 0}, frame);
  if (decodeFrame(frame, kFrameSize, state)) return false;
  encodeFrame({0, 4096, 0}, frame);
  return !decodeFrame(frame, kFrameSize, state);
}
constexpr bool testFilter() {
  PotFilter filter;
  if (filter.update(0) != 0) return false;
  uint16_t previous = 0;
  for (uint8_t i = 0; i < 100; ++i) {
    uint16_t next = filter.update(kPotMax);
    if (next < previous || next > kPotMax) return false;
    previous = next;
  }
  if (previous != kPotMax) return false;
  for (uint8_t i = 0; i < 100; ++i) previous = filter.update(0);
  return previous == 0;
}
static_assert(testDebounce(), "Nine-key debounce/chords/rollover test failed");
static_assert(testEdges(), "Nine-key edge transition test failed");
static_assert(testProtocol(), "Protocol/CRC/corruption/range test failed");
static_assert(testFilter(), "ADC filter convergence test failed");
} // namespace macrokeys
