#pragma once
#include <Arduino.h>
#include <time.h>
namespace HotList {
struct Snapshot {
  char topics[5][160]={}, dramas[5][160]={};
  uint32_t fetchedAt=0,revision=0,failures=0;
  bool valid=false,loading=false,lastFetchOk=false;
};
// Calendar arithmetic via mktime keeps the 07:00 boundary correct across DST.
inline int scheduleSlot(time_t utc) {
  tm t{};if(utc<1700000000 || !localtime_r(&utc,&t))return 0;
  if(t.tm_hour<7) {--t.tm_mday;t.tm_hour=12;t.tm_isdst=-1;mktime(&t);}
  return (t.tm_year+1900)*10000+(t.tm_mon+1)*100+t.tm_mday;
}
constexpr uint32_t cooldown(bool manual,bool lastFetchOk) {
  return manual?15000u:lastFetchOk?0u:300000u;
}
static_assert(cooldown(false,true)==0 && cooldown(false,false)==300000
  && cooldown(true,true)==15000,"Successful pre-07:00 fetch must not defer the daily boundary");
void begin();
bool due();
void poll();
void refresh();
Snapshot snapshot();
void status();
bool selfTest();
}
