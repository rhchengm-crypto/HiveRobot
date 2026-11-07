#pragma once
#include <Arduino.h>
namespace FeitCloud {
struct State { bool wifi=false, known=false, power=false, busy=false; bool secondKnown=false, secondPower=false; bool thirdKnown=false, thirdPower=false; uint32_t completed=0, failures=0; };
bool begin();
void setBrightnessPercent(uint8_t percent); // Same latest target for all configured bulbs; non-blocking.
bool toggleFromTouch(uint8_t bulb); // 0..2; same queue/cache as physical keys.
bool toggle(); // K9: toggle the configured Feit bulb; non-blocking.
bool toggleSecond(); // K8: toggle the second configured bulb.
bool toggleThird(); // K2: toggle the third configured bulb.
bool toggleAllOrangeColor(); // K6: orange / warm white across all bulbs; independent of K1.
bool toggleAllColor(); // K1: pink / warm white across all configured bulbs.
bool toggleAll(); // K4: any ON -> all OFF; all OFF -> all ON.
void serviceSerial(); // Bounded input parsing, never performs network I/O.
State state();
bool takeDisplayResetRequest(); // UI task only; serial lcd requests local recovery.
int8_t takePageRequest(); // Serial page 0..3, consumed by the UI task.
enum class ScreenRequest : uint8_t {None,Status,Off,On};
enum class SpeechRequest:uint8_t {None,Next,Tone};
SpeechRequest takeSpeechRequest();
int8_t takeHotViewRequest();
ScreenRequest takeScreenRequest(); // Display-only serial diagnostics, UI task.
}

