#pragma once
#include <Arduino.h>
// WAV data includes its 44-byte header. Does not log keys or uploaded audio.
bool transcribe(const uint8_t* wav,size_t bytes,String& text,String& error,bool askQuestion,void (*recognized)()=nullptr);
bool standardMandarin(const String& transcript,String& text,String& error);
