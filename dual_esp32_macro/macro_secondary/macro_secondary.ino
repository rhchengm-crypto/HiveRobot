#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>
#include "KeyLogic.h"

// Classic ESP32-WROOM DevKit: no flash pins, input-only pins or boot straps.
constexpr uint8_t KEY_PINS[macrokeys::kCount] = {13, 14, 25, 26, 27, 18, 33, 23, 32};
constexpr int POT_PIN = 34;
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
static_assert(configTICK_RATE_HZ >= 1000, "1ms scan requires FreeRTOS tick rate >= 1000Hz");
// This compiler does not guarantee always-lock-free C++ atomics, even at 32 bits.
// Use the native ESP32 cross-core critical section for a fixed-size snapshot copy only.
uint8_t publishedFrame[macrokeys::kFrameSize] = {};
macrokeys::PotFilter potFilter;
uint16_t filteredPot = 0;
uint8_t sequence = 0;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t lastRequestAt = 0, requestCallbacks = 0;
bool slaveReady = false;
uint32_t lastInitAt = 0;
unsigned recoveryCount = 0;
macrokeys::Debouncer debounce;

// Wire invokes this in its slave service task. No scanning, serial, allocation,
// delay or macro execution here. Only a precomputed 8-byte snapshot is copied into Wire's TX buffer.
void replyKeys() {
  portENTER_CRITICAL(&stateMux);
  uint8_t snapshot[macrokeys::kFrameSize];
  memcpy(snapshot, publishedFrame, sizeof(snapshot));
  lastRequestAt = millis();
  ++requestCallbacks;
  portEXIT_CRITICAL(&stateMux);
  Wire.write(snapshot, sizeof(snapshot));
}

bool startSlave() {
  lastInitAt = millis();
  if (slaveReady) { Wire.end(); slaveReady = false; }
  pinMode(SDA_PIN, INPUT_PULLUP); pinMode(SCL_PIN, INPUT_PULLUP);
  if (digitalRead(SDA_PIN) == LOW || digitalRead(SCL_PIN) == LOW) {
    Serial.printf("[SLAVE] Init deferred: SDA=%d SCL=%d\n", digitalRead(SDA_PIN), digitalRead(SCL_PIN));
    return false;
  }
  Wire.onRequest(replyKeys);
  Wire.setTimeOut(2);
  if (!Wire.begin(static_cast<uint8_t>(macrokeys::kAddress), SDA_PIN, SCL_PIN, 400000)) {
    Serial.println("[SLAVE] Init failed; retry later (no permanent halt)");
    return false;
  }
  slaveReady = true;
#if CONFIG_IDF_TARGET_ESP32
  // Original ESP32 requires initial TX FIFO preloading. Later callbacks refill
  // the next response; a master can see the previous snapshot for one poll.
  portENTER_CRITICAL(&stateMux);
  uint8_t initial[macrokeys::kFrameSize];
  memcpy(initial, publishedFrame, sizeof(initial));
  portEXIT_CRITICAL(&stateMux);
  if (Wire.slaveWrite(initial, sizeof(initial)) != sizeof(initial)) {
    Wire.end(); slaveReady = false;
    Serial.println("[SLAVE] Initial TX preload failed; retry later");
    return false;
  }
#endif
  portENTER_CRITICAL(&stateMux); lastRequestAt = millis(); portEXIT_CRITICAL(&stateMux);
  Serial.printf("[SLAVE] I2C ready, recovery=%u\n", recoveryCount);
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.printf("[BOOT] Secondary 9KEY+POT protocol-v1 / cold-start-v2 / K7=GPIO33 reset_reason=%d\n", (int)esp_reset_reason());
  for (uint8_t pin : KEY_PINS) pinMode(pin, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(POT_PIN, ADC_11db);
  filteredPot = potFilter.update(analogRead(POT_PIN));
  macrokeys::encodeFrame({0, filteredPot, sequence}, publishedFrame);
  delay(300); // Allow the shared 5V boost and secondary regulator to settle.
  if (startSlave()) Serial.println("Secondary ready: addr=0x42 SDA=21 SCL=22; 9 keys + ADC34; 8-byte protocol-v1");
  else Serial.println("[SLAVE] Startup pending; loop will retry I2C initialization");
  Serial.println("1ms scan, independent 10ms debounce; compile-time logic tests passed");
}

void loop() {
  static TickType_t wake = xTaskGetTickCount();
  uint16_t raw = 0;
  for (uint8_t i = 0; i < macrokeys::kCount; ++i)
    if (digitalRead(KEY_PINS[i]) == LOW) raw |= static_cast<uint16_t>(1u << i);
  const uint32_t sampleAt = millis();
  const uint16_t next = debounce.update(raw, sampleAt);
  static uint32_t potAt = 0;
  if (static_cast<uint32_t>(sampleAt - potAt) >= 4) {
    potAt = sampleAt;
    filteredPot = potFilter.update(analogRead(POT_PIN));
  }
  uint8_t frame[macrokeys::kFrameSize];
  macrokeys::encodeFrame({next, filteredPot, ++sequence}, frame);
  portENTER_CRITICAL(&stateMux);
  memcpy(publishedFrame, frame, sizeof(frame));
  const uint32_t lastActivity = lastRequestAt;
  const uint32_t callbacks = requestCallbacks;
  portEXIT_CRITICAL(&stateMux);
  const uint32_t now = millis();
  if ((!slaveReady && static_cast<uint32_t>(now - lastInitAt) >= 1000) ||
      (slaveReady && static_cast<uint32_t>(now - lastActivity) >= 3000 &&
       static_cast<uint32_t>(now - lastInitAt) >= (recoveryCount < 3 ? 3000u : 15000u))) {
    ++recoveryCount;
    Serial.printf("[SLAVE] Reinitialize I2C only; request_callbacks=%lu\n", (unsigned long)callbacks);
    startSlave();
    wake = xTaskGetTickCount();
  }
  vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
}
