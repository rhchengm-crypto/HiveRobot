#pragma once
#include <Arduino.h>
#include <time.h>
namespace DailyNews {
struct Bulletin {String text,links;int day=0,count=0;time_t fetched=0;};
// POSIX TZ is set to America/Los_Angeles by the app, including DST.
int editionDay(time_t now);
bool load(Bulletin& news);
bool save(const Bulletin& news);
bool fetch(int day,Bulletin& news,String& error);
}
