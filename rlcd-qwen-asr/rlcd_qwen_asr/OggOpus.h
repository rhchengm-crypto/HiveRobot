#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace OggOpus {
inline void le(uint8_t* p,uint64_t v,unsigned n){for(unsigned i=0;i<n;++i)p[i]=uint8_t(v>>(8*i));}
inline size_t page(uint8_t* out,size_t capacity,const uint8_t* packet,size_t bytes,uint32_t serial,uint32_t sequence,uint64_t granule,uint8_t flags){
 const size_t segments=bytes/255+1,total=27+segments+bytes;if(total>capacity || segments>255)return 0;
 memset(out,0,27+segments);memcpy(out,"OggS",4);out[5]=flags;le(out+6,granule,8);le(out+14,serial,4);le(out+18,sequence,4);out[26]=segments;
 for(size_t i=0;i<segments;++i)out[27+i]=i+1<segments?255:bytes%255;
 memcpy(out+27+segments,packet,bytes);uint32_t crc=0;
 for(size_t i=0;i<total;++i){crc^=uint32_t(out[i])<<24;for(unsigned b=0;b<8;++b)crc=(crc&0x80000000)?(crc<<1)^0x04c11db7:crc<<1;}
 le(out+22,crc,4);return total;
}
inline void head(uint8_t* p){memset(p,0,19);memcpy(p,"OpusHead",8);p[8]=1;p[9]=1;le(p+10,312,2);le(p+12,16000,4);}
inline void tags(uint8_t* p){memset(p,0,20);memcpy(p,"OpusTags",8);le(p+8,4,4);memcpy(p+12,"RLCD",4);}
}
