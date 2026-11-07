#pragma once
#include <Arduino.h>
namespace Weather {
struct Day { char date[11]={}; float high=0,low=0; uint8_t code=0,rain=0; };
struct Snapshot {
  Day days[3];
  int32_t utcOffset=0;
  uint32_t fetchedAt=0,revision=0,failures=0;
  bool valid=false,loading=false,lastFetchOk=false;
};
bool begin();
bool due(); // Called by the existing network worker while lamp queues are idle.
void poll(); // Background worker only; never the UI/serial task.
Snapshot snapshot();
void refresh();
void userActivity();
bool quiet();
void status();
const char* description(uint8_t code);
constexpr bool validDate(const char* d) {
  for(unsigned i=0;i<10;++i) {
    if(i==4 || i==7) {if(d[i]!='-')return false;}
    else if(d[i]<'0' || d[i]>'9')return false;
  }
  if(d[10])return false;
  const int year=(d[0]-'0')*1000+(d[1]-'0')*100+(d[2]-'0')*10+d[3]-'0';
  const int month=(d[5]-'0')*10+d[6]-'0',day=(d[8]-'0')*10+d[9]-'0';
  constexpr uint8_t lengths[]={31,28,31,30,31,30,31,31,30,31,30,31};
  const bool leap=year%4==0 && (year%100!=0 || year%400==0);
  return year>=2020 && year<=2100 && month>=1 && month<=12 && day>=1
    && day<=lengths[month-1]+(month==2 && leap);
}
constexpr int weekday(const char* d) {
  int y=(d[0]-'0')*1000+(d[1]-'0')*100+(d[2]-'0')*10+d[3]-'0';
  const int m=(d[5]-'0')*10+d[6]-'0',day=(d[8]-'0')*10+d[9]-'0';
  constexpr int t[]={0,3,2,5,0,3,5,1,4,6,2,4};
  y-=m<3;return (y+y/4-y/100+y/400+t[m-1]+day)%7;
}
static_assert(validDate("2028-02-29") && !validDate("2026-02-29") && !validDate("2026-13-01")
  && weekday("2026-10-24")==6 && weekday("2026-10-25")==0,"Weather date/calendar validation failed");
}
