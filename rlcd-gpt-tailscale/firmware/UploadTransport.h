#pragma once
#include <WiFiClientSecure.h>
#include <Stream.h>
#include <esp_heap_caps.h>
// Turn off Nagle after the socket exists, rather than before TLS connects.
class UploadTLS:public WiFiClientSecure {
 uint8_t* uploadBuffer_=nullptr;
 public:
  ~UploadTLS(){free(uploadBuffer_);}
  using WiFiClientSecure::connect;
  size_t write(uint8_t byte)override{return write(&byte,1);}
  uint32_t connectMs=0,writeMs=0;size_t sentBytes=0,writeCalls=0;
  size_t write(const uint8_t* data,size_t bytes)override {
    // Feed mbedTLS in its 4096-byte chunks rather than HTTPClient's 1460-byte
    // Stream writes. Copy PSRAM into internal RAM for predictable TLS access.
    if(!uploadBuffer_)uploadBuffer_=static_cast<uint8_t*>(heap_caps_malloc(4096,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
    if(!uploadBuffer_)return 0;
    size_t sent=0;const uint32_t at=millis();
    while(sent<bytes){const size_t n=min(size_t(4096),bytes-sent);memcpy(uploadBuffer_,data+sent,n);
      const size_t done=WiFiClientSecure::write(uploadBuffer_,n);sent+=done;++writeCalls;
      if(done!=n)break;}
    sentBytes+=sent;writeMs+=millis()-at;return sent;
  }
  int connect(const char* host,uint16_t port,int32_t timeout)override {
    const uint32_t at=millis();const int ok=WiFiClientSecure::connect(host,port,timeout);connectMs+=millis()-at;if(ok)setNoDelay(true);return ok;
  }
};
// HTTPClient's Stream path copies PSRAM into a bounded internal-RAM send buffer.
class AudioPayload:public Stream {
 const uint8_t* data_;size_t size_,position_=0;
 public:
 AudioPayload(const uint8_t* data,size_t size):data_(data),size_(size){}
 int available()override{return int(size_-position_);}
 int read()override{return position_<size_?data_[position_++]:-1;}
 int peek()override{return position_<size_?data_[position_]:-1;}
 size_t readBytes(char* target,size_t bytes)override {
   const size_t n=min(bytes,size_-position_);memcpy(target,data_+position_,n);position_+=n;return n;
 }
 size_t write(uint8_t)override{return 0;}
 void flush()override{}
};
