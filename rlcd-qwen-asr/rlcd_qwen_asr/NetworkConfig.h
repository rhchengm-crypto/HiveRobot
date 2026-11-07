#pragma once
#include <Arduino.h>
namespace NetworkConfig {
void begin(const char*,const char*);
void tick(bool idle);
bool hasCredentials();
bool active();
void open();
void close();
void suspend();
void resume();
const String& apName();
const String& apPassword();
const String& message();
void report();
}
