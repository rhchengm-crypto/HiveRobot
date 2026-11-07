#pragma once
#include <Arduino.h>
namespace Speech {
enum Mode:uint8_t {Idle,Queued,Loading,Playing,Done,Error};
struct Snapshot {
  uint32_t revision=0,samples=0;
  uint8_t mode=Idle,index=0;
  bool tv=false;
  char detail[40]={};
};
bool begin();
void context(bool active,bool tv);
bool next(bool tv);
bool due();
void poll(); // Existing network worker only; closes idle lamp TLS first.
bool playing();
void stop();
void waitStopped();
void tone();
Snapshot snapshot();
void status();
}
