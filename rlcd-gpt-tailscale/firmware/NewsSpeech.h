#pragma once
#include <Arduino.h>
namespace NewsSpeech {
void begin();
bool next(const String& bulletin,int expectedCount);
void stop();
bool active();
uint32_t revision();
String label();
void report();
bool tone();
}
