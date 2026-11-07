#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <esp_heap_caps.h>
// Small mono/16-bit/16kHz FLAC encoder. RFC 9639: fixed order-0 Rice
// subframes, with verbatim fallback for incompressible blocks. No samples
// are resampled, discarded or quantized. Output is owned by the caller.
namespace LosslessAudio {
struct Writer {
 uint8_t* data;size_t cap,pos=0;unsigned used=0;uint8_t byte=0;bool ok=true;
 void putBit(unsigned b){byte=uint8_t((byte<<1)|(b&1));if(++used==8){put(byte);used=0;byte=0;}}
 void bits(uint32_t v,unsigned n){while(n)putBit(v>>(--n));}
 void put(uint8_t b){if(pos<cap)data[pos++]=b;else ok=false;}
 void align(){while(used)putBit(0);}
};
inline uint8_t crc8(const uint8_t* p,size_t n){uint8_t c=0;while(n--){c^=*p++;for(int j=0;j<8;++j)c=(c&128)?uint8_t((c<<1)^7):uint8_t(c<<1);}return c;}
inline uint16_t crc16(const uint8_t* p,size_t n){uint16_t c=0;while(n--){c^=uint16_t(*p++)<<8;for(int j=0;j<8;++j)c=(c&0x8000)?uint16_t((c<<1)^0x8005):uint16_t(c<<1);}return c;}
inline int32_t sample(const uint8_t* pcm,size_t i){return int16_t(uint16_t(pcm[2*i])|(uint16_t(pcm[2*i+1])<<8));}
inline uint32_t fold(int32_t v){return v>=0?uint32_t(v)*2:uint32_t(-v)*2-1;}
inline void number(Writer& w,unsigned n){if(n<128)w.put(n);else if(n<2048){w.put(0xc0|(n>>6));w.put(0x80|(n&63));}else{w.put(0xe0|(n>>12));w.put(0x80|((n>>6)&63));w.put(0x80|(n&63));}}
inline uint8_t* encode(const uint8_t* pcm,size_t samples,size_t& bytes){
 bytes=0;if(!samples || samples>320000)return nullptr;
 const size_t cap=samples*2+((samples+1023)/1024)*32+42;
 auto* out=static_cast<uint8_t*>(heap_caps_malloc(cap,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));if(!out)return nullptr;
 Writer w{out,cap};w.put('f');w.put('L');w.put('a');w.put('C');w.put(0x80);w.put(0);w.put(0);w.put(34);
 // STREAMINFO: block sizes (last frame may be smaller), unknown frame sizes,
 // 16000 Hz, one channel, 16 bits, exact sample count; unspecified MD5 = zero.
 w.put(4);w.put(0);w.put(4);w.put(0);for(int j=0;j<6;++j)w.put(0);
 const uint64_t info=(uint64_t(16000)<<44)|(uint64_t(15)<<36)|samples;
 for(int j=7;j>=0;--j)w.put(uint8_t(info>>(8*j)));for(int j=0;j<16;++j)w.put(0);
 unsigned frame=0;
 for(size_t at=0;at<samples;at+=1024,++frame){
   const size_t n=(samples-at<1024)?samples-at:1024,start=w.pos;
   w.put(0xff);w.put(0xf8);w.put(0x75);w.put(8);number(w,frame);w.put((n-1)>>8);w.put(n-1);w.put(crc8(out+start,w.pos-start));
   uint64_t best=uint64_t(n)*16;unsigned parameter=15;
   for(unsigned k=0;k<15;++k){uint64_t cost=8;for(size_t j=0;j<n;++j)cost+=(fold(sample(pcm,at+j))>>k)+1+k;if(cost<best){best=cost;parameter=k;}}
   if(parameter==15){w.put(2);for(size_t j=0;j<n;++j)w.bits(uint16_t(sample(pcm,at+j)),16);}
   else {w.put(0x10);w.bits(0,2);w.bits(0,4);w.bits(parameter,4);
     for(size_t j=0;j<n;++j){const uint32_t v=fold(sample(pcm,at+j));for(uint32_t q=v>>parameter;q;--q)w.putBit(0);w.putBit(1);w.bits(v,parameter);}}
   w.align();const uint16_t crc=crc16(out+start,w.pos-start);w.put(crc>>8);w.put(crc);
 }
 if(!w.ok){free(out);return nullptr;}bytes=w.pos;return out;
}
}
