#pragma once
namespace RealtimeText {
using StateCallback=void(*)(const char*);
using TextCallback=void(*)(const char*);
using FinishCallback=void(*)(const char*);
bool start(StateCallback state,TextCallback text,FinishCallback finish);
void stop();
bool active();
void codecTest();
}
