#pragma once
#include <Arduino.h>
namespace Microphone {
bool begin();
// Returns stereo 16-bit PCM at 16 kHz. -1 means a device/read failure.
int read(int16_t* stereo,size_t frames);
}
