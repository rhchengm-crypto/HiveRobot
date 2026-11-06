#pragma once
#include <Arduino.h>
namespace History {
bool begin();
void append(const char* source,const char* text,const char* question=nullptr);
bool lastGpt(String& text);
bool flush(uint32_t timeout=5000);
void report();
}
