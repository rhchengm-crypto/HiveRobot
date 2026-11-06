#pragma once
#include <Arduino.h>
#include <time.h>
namespace LocalClock {
// America/Los_Angeles: US DST starts second Sunday March, ends first Sunday November.
constexpr char kTimezone[]="PST8PDT,M3.2.0/2,M11.1.0/2";
inline bool valid(time_t utc) { return utc>=1700000000; }
inline void status() {
  const time_t utc=time(nullptr);tm local{};char text[48]={};
  if(valid(utc) && localtime_r(&utc,&local))strftime(text,sizeof(text),"%Y-%m-%d %H:%M:%S %Z",&local);
  else strlcpy(text,"WAITING FOR NTP",sizeof(text));
  Serial.printf("[CLOCK] valid=%u utc=%llu local=%s timezone=America/Los_Angeles\n",valid(utc),(unsigned long long)utc,text);
}
inline bool selfTest() {
  struct Sample {time_t utc;int month,day,hour,minute,dst;};
  const Sample samples[]={
    {1772963940,3,8,1,59,0},{1772964000,3,8,3,0,1},
    {1793523540,11,1,1,59,1},{1793523600,11,1,1,0,0}
  };
  for(const auto& s:samples) {
    tm t{};if(!localtime_r(&s.utc,&t) || t.tm_year!=126 || t.tm_mon+1!=s.month
      || t.tm_mday!=s.day || t.tm_hour!=s.hour || t.tm_min!=s.minute || t.tm_isdst!=s.dst)return false;
  }
  return true;
}
}
