#pragma once
#include <Arduino.h>
namespace ExternalKeys {
constexpr uint8_t pins[4]={1,2,3,17};
struct Key {bool raw=false,down=false;uint32_t changed=0,presses=0;};
Key keys[4];uint32_t revision=1;bool testing=false;
void begin(){
 for(unsigned i=0;i<4;++i){pinMode(pins[i],INPUT_PULLUP);keys[i].raw=keys[i].down=digitalRead(pins[i])==LOW;keys[i].changed=millis();}
 Serial.println("[EXT_KEYS_READY] pins=1,2,3,17 active_low=1 debounce_ms=35 actions=diagnostic_only");
}
void poll(){
 const uint32_t now=millis();
 for(unsigned i=0;i<4;++i){
  auto& k=keys[i];const bool down=digitalRead(pins[i])==LOW;
  if(down!=k.raw){k.raw=down;k.changed=now;}
  if(k.down!=k.raw && uint32_t(now-k.changed)>=35){
   k.down=k.raw;if(k.down)++k.presses;++revision;
   Serial.printf("[EXT_KEY] key=K%u gpio=%u state=%s presses=%lu\n",i+1,pins[i],k.down?"DOWN":"UP",(unsigned long)k.presses);
  }
 }
}
void report(){for(unsigned i=0;i<4;++i)Serial.printf("[EXT_KEY_STATUS] key=K%u gpio=%u down=%u presses=%lu\n",i+1,pins[i],keys[i].down,(unsigned long)keys[i].presses);}
}
