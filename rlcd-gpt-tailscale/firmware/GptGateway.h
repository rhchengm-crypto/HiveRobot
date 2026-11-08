#pragma once
#include <Arduino.h>
namespace GptGateway {
enum class State:uint8_t {Off,Connecting,Ready,Failed,Stopping};
void enter();
void leave();
void retry();
bool needsRestart();
String connectionStatus();
bool stopAndWait(uint32_t timeout=35000);
bool stopping();
bool active();
bool ready();
State state();
String error();
String model();
bool ask(const uint8_t* wav,size_t length,String& result,String& failure);
void report();
void probeUpload();
}
