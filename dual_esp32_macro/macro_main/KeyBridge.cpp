#include "KeyBridge.h"

namespace {
constexpr uint16_t kMasterPotThreshold = 32; // Main-side only; protocol unchanged.
bool potMoved(uint16_t value, uint16_t previous) {
  return macrokeys::distance(value, previous) >= kMasterPotThreshold ||
         (value <= 16 && previous > 16) || (value >= 4079 && previous < 4079);
}
// This is the only master on this dedicated bus. Never drive a 3.3V bus HIGH
// push-pull; release open-drain outputs and respect a slave holding SCL LOW.
bool waitSclHigh() {
  const uint32_t at = micros();
  while (digitalRead(22) == LOW) {
    if (static_cast<uint32_t>(micros() - at) >= 2000) return false;
    delayMicroseconds(5);
  }
  return true;
}
bool clearBus() {
  digitalWrite(21, HIGH); digitalWrite(22, HIGH);
  pinMode(21, OUTPUT_OPEN_DRAIN); pinMode(22, OUTPUT_OPEN_DRAIN);
  bool ok = waitSclHigh();
  for (unsigned n = 0; ok && digitalRead(21) == LOW && n < 9; ++n) {
    digitalWrite(22, LOW); delayMicroseconds(5);
    digitalWrite(22, HIGH); ok = waitSclHigh(); delayMicroseconds(5);
  }
  if (ok) {
    digitalWrite(22, LOW); digitalWrite(21, LOW); delayMicroseconds(5);
    digitalWrite(22, HIGH); ok = waitSclHigh(); delayMicroseconds(5);
    digitalWrite(21, HIGH); delayMicroseconds(5); // STOP
    ok = ok && digitalRead(21) == HIGH;
  }
  pinMode(21, INPUT_PULLUP); pinMode(22, INPUT_PULLUP);
  return ok;
}
} // namespace

bool KeyBridge::begin(EventHandler handler, PotHandler potHandler) {
  handler_ = handler;
  potHandler_ = potHandler;
  queue_ = xQueueCreate(64, sizeof(Snapshot));
  if (!queue_) return false;
  bus_.setTimeOut(2); // A broken bus must not block the UI loop.
  if (!bus_.begin(21, 22, 400000)) {
    vQueueDelete(queue_); queue_ = nullptr; return false;
  }
  if (xTaskCreatePinnedToCore(taskEntry, "macro-i2c", 4096, this, 2, nullptr, 0) != pdPASS) {
    bus_.end(); vQueueDelete(queue_); queue_ = nullptr; return false;
  }
  return true;
}
void KeyBridge::taskEntry(void* arg) { static_cast<KeyBridge*>(arg)->pollingTask(); }
void KeyBridge::pollingTask() {
  Serial.println("[I2C] Cold-start grace 1000ms; waiting for secondary power/startup");
  vTaskDelay(pdMS_TO_TICKS(1000));
  TickType_t wake = xTaskGetTickCount();
  uint32_t lastGood = millis();
  uint32_t recoveryAt = lastGood, diagnosticAt = lastGood - 1000;
  unsigned failures = 0;
  uint32_t lastPotQueuedAt = 0;
  bool addressed = false;
  Snapshot lastQueued{0, false};
  for (;;) {
    uint8_t frame[macrokeys::kFrameSize] = {};
    macrokeys::InputState input;
    uint8_t received = 0xff;
    size_t count = 0;
    uint8_t probe = 0xff;
    if (!addressed) {
      // Probe before a read so a powering-up slave is not immediately asked to
      // transmit from an unprimed FIFO. NACK is a normal startup condition.
      bus_.beginTransmission(macrokeys::kAddress);
      probe = bus_.endTransmission(true);
      if (probe == 0) {
        addressed = true;
        Serial.println("[I2C] 0x42 ACK; allowing TX FIFO to settle 20ms");
        vTaskDelay(pdMS_TO_TICKS(20)); wake = xTaskGetTickCount();
      }
    }
    if (addressed) count = bus_.requestFrom(macrokeys::kAddress, macrokeys::kFrameSize, true);
    size_t copied = 0;
    while (bus_.available() && copied < sizeof(frame)) frame[copied++] = static_cast<uint8_t>(bus_.read());
    if (copied) received = frame[0];
    while (bus_.available()) bus_.read();
    const bool valid = count == macrokeys::kFrameSize && copied == macrokeys::kFrameSize &&
                       macrokeys::decodeFrame(frame, copied, input);
    Snapshot current = lastQueued;
    if (valid) {
      failures = 0; lastGood = millis(); current = {input.keys, true, input.pot};
    } else {
      ++failures;
      if (failures >= 5) addressed = false;
      portENTER_CRITICAL(&mux_); ++errors_; portEXIT_CRITICAL(&mux_);
      if (static_cast<uint32_t>(millis() - lastGood) >= 100) current = {0, false, lastQueued.pot};
      if (static_cast<uint32_t>(millis() - diagnosticAt) >= 1000) {
        diagnosticAt = millis();
        Serial.printf("[I2C] offline probe=%u bytes=%u value=0x%02x SDA=%d SCL=%d\n",
                      probe, (unsigned)count, received, digitalRead(21), digitalRead(22));
      }
    }
    portENTER_CRITICAL(&mux_); latest_ = current; portEXIT_CRITICAL(&mux_);
    const bool keyOrLinkChanged = current.mask != lastQueued.mask || current.online != lastQueued.online;
    const bool potChanged = current.online && potMoved(current.pot, lastQueued.pot) &&
                            static_cast<uint32_t>(millis() - lastPotQueuedAt) >= 20;
    if (keyOrLinkChanged || potChanged) {
      if (xQueueSend(queue_, &current, 0) == pdTRUE) {
        lastQueued = current;
        lastPotQueuedAt = millis();
      }
      else {
        portENTER_CRITICAL(&mux_); ++overflows_; resync_ = true; portEXIT_CRITICAL(&mux_);
      }
    }
    if (failures >= 5 && static_cast<uint32_t>(millis() - lastGood) >= 3000 &&
        static_cast<uint32_t>(millis() - recoveryAt) >= 5000) {
      recoveryAt = millis();
      bus_.end();
      const bool idle = clearBus();
      const bool begun = bus_.begin(21, 22, 400000);
      bus_.setTimeOut(2);
      addressed = false;
      Serial.printf("[I2C] bus recovery idle=%d driver=%d SDA=%d SCL=%d\n",
                    idle, begun, digitalRead(21), digitalRead(22));
      vTaskDelay(pdMS_TO_TICKS(100)); wake = xTaskGetTickCount();
    }
    const TickType_t period = pdMS_TO_TICKS(failures >= 5 ? 20 : 2);
    if (static_cast<int32_t>(xTaskGetTickCount() - wake) >= static_cast<int32_t>(period))
      wake = xTaskGetTickCount(); // No catch-up spin after a timeout.
    vTaskDelayUntil(&wake, period);
  }
}
void KeyBridge::deliver(Snapshot state) {
  const uint16_t releases = macrokeys::releasedEdges(deliveredMask_, state.mask);
  const uint16_t presses = macrokeys::pressedEdges(deliveredMask_, state.mask);
  const bool notifyPot = state.online && (!deliveredOnline_ ||
      potMoved(state.pot, deliveredPot_));
  deliveredMask_ = state.mask; deliveredOnline_ = state.online;
  if (notifyPot) deliveredPot_ = state.pot;
  for (uint8_t i = 0; i < macrokeys::kCount; ++i)
    if (handler_ && (releases & (1u << i))) handler_(i + 1, false);
  for (uint8_t i = 0; i < macrokeys::kCount; ++i)
    if (handler_ && (presses & (1u << i))) handler_(i + 1, true);
  if (notifyPot && potHandler_) potHandler_(deliveredPot_);
}
void KeyBridge::service() {
  Snapshot state;
  bool recover;
  portENTER_CRITICAL(&mux_); recover = resync_; resync_ = false; state = latest_; portEXIT_CRITICAL(&mux_);
  if (recover) {
    // A blocking application lost transitions: discard stale history, release
    // delivered keys and resynchronize. Never silently leave a macro held down.
    xQueueReset(queue_); deliver({0, false, deliveredPot_}); deliver(state);
  }
  for (unsigned n = 0; n < 64 && xQueueReceive(queue_, &state, 0) == pdTRUE; ++n) deliver(state);
}
uint32_t KeyBridge::errors() const {
  auto* self = const_cast<KeyBridge*>(this);
  portENTER_CRITICAL(&self->mux_); uint32_t value = errors_; portEXIT_CRITICAL(&self->mux_); return value;
}
uint32_t KeyBridge::overflows() const {
  auto* self = const_cast<KeyBridge*>(this);
  portENTER_CRITICAL(&self->mux_); uint32_t value = overflows_; portEXIT_CRITICAL(&self->mux_); return value;
}
