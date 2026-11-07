#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace PcmWave {
struct View {const uint8_t* pcm=nullptr;size_t frames=0;uint32_t rate=0;};
inline uint16_t u16(const uint8_t* p){return uint16_t(p[0])|(uint16_t(p[1])<<8);}
inline uint32_t u32(const uint8_t* p){return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
inline bool parse(const uint8_t* bytes,size_t length,View& out){
 out=View{};
 if(length<44||memcmp(bytes,"RIFF",4)||memcmp(bytes+8,"WAVE",4))return false;
 bool format=false;uint32_t rate=0;
 for(size_t offset=12;offset<=length-8;){
  const uint8_t* chunk=bytes+offset;const uint32_t declared=u32(chunk+4);offset+=8;
  const size_t remaining=length-offset;
  if(!memcmp(chunk,"fmt ",4)){
   if(declared<16||declared>remaining||u16(bytes+offset)!=1||u16(bytes+offset+2)!=1||u16(bytes+offset+14)!=16)return false;
   rate=u32(bytes+offset+4);if(rate!=16000&&rate!=24000)return false;format=true;
  }else if(!memcmp(chunk,"data",4)){
   // The TTS service emits streaming WAVs with a placeholder 0x7fffffff size.
   // Bound PCM by the completed HTTP/cache payload instead of trusting that size.
   if(!format)return false;
   if(declared>remaining&&declared<0x7fff0000u)return false;
   const size_t size=declared<remaining?declared:remaining;
   if(size<2||size%2)return false;
   out.pcm=bytes+offset;out.frames=size/2;out.rate=rate;return true;
  }
  if(declared>remaining)return false;
  const size_t skip=size_t(declared)+(declared&1);
  if(skip>remaining)return false;offset+=skip;
 }
 return false;
}
}
