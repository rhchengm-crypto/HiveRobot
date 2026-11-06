#pragma once
#include "BoardDisplay.h"
namespace ChineseFont {
void begin();
bool ready();
bool uploadActive();
void status();
void startUpload(uint32_t bytes,uint32_t crc);
bool receiving();
void text(BoardDisplay& d,const char* utf8,int x,int y,int width,int lines,uint16_t ink,uint16_t bg);
}
