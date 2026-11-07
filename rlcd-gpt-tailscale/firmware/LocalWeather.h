#pragma once
#include <Arduino.h>
#include <time.h>
namespace LocalWeather {
struct Day {char date[11]={};float low=0,high=0;int code=0,rain=0;};
struct Snapshot {bool valid=false,verified=false;char city[64]={},region[64]={},country[4]={},ip[48]={},timezone[64]={},abbr[12]={},observed[24]={};float lat=0,lon=0,temp=0;int offset=0,code=0;time_t fetched=0,geoChecked=0;Day days[3];};
bool load(Snapshot&);
bool fetch(const Snapshot&,Snapshot&,bool force,String& error);
const char* condition(int code);
void applyTimezone(const Snapshot&);
}
