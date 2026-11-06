#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "KeyLogic.h"

class KeyBridge {
 public:
  using EventHandler = void (*)(uint8_t key, bool down);
  using PotHandler = void (*)(uint16_t raw);
  bool begin(EventHandler handler, PotHandler potHandler = nullptr);
  void service(); // Call from the existing UI loop; never performs an I2C read.
  uint16_t held() const { return deliveredMask_; }
  uint16_t pot() const { return deliveredPot_; }
  bool online() const { return deliveredOnline_; }
  uint32_t errors() const;
  uint32_t overflows() const;
 private:
  struct Snapshot { uint16_t mask; bool online; uint16_t pot = 0; };
  static void taskEntry(void* arg);
  void pollingTask();
  void deliver(Snapshot state);
  TwoWire bus_{1}; // Dedicated hardware I2C controller, independent of SPI/touch.
  QueueHandle_t queue_ = nullptr;
  EventHandler handler_ = nullptr;
  PotHandler potHandler_ = nullptr;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  Snapshot latest_{0, false};
  uint32_t errors_ = 0, overflows_ = 0;
  bool resync_ = false;
  uint16_t deliveredMask_ = 0, deliveredPot_ = 0;
  bool deliveredOnline_ = false;
};
