#pragma once
#include <Arduino.h>
class BoardDisplay;
namespace dogart {
constexpr uint16_t kWidth=128,kHeight=108;
constexpr uint8_t kDogs=3,kFrames=6,kColors=128;
extern const uint16_t palette[kColors] PROGMEM;
extern const uint8_t pixels[kFrames][kWidth*kHeight] PROGMEM;
void draw(BoardDisplay& display,uint8_t dog,bool awake,int16_t x,int16_t y,uint16_t background);
}
