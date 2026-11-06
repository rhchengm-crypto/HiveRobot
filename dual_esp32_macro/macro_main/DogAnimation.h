#pragma once
#include <Arduino.h>
class BoardDisplay;
namespace doganim {
void begin();
void suspend();
void service();
bool receiving();
void startUpload(uint32_t bytes, uint32_t crc);
void verify();
void status();
void visible(bool value);
void update(BoardDisplay& display,const bool* known,const bool* power,bool force,bool touching);
}
