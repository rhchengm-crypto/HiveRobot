#pragma once
#include <Arduino.h>
#include <RlcdOpus.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include "OggOpus.h"
namespace GptOpus {
inline uint8_t* encode(const uint8_t* wav,size_t length,size_t& bytes){
 bytes=0;if(length<46)return nullptr;
 const size_t samples=(length-44)/2,capacity=128*1024;
 auto* blob=static_cast<uint8_t*>(heap_caps_malloc(capacity,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
 if(!blob)return nullptr;
 esp_opus_enc_config_t cfg{};cfg.sample_rate=16000;cfg.channel=1;cfg.bits_per_sample=16;cfg.bitrate=24000;cfg.frame_duration=ESP_OPUS_ENC_FRAME_DURATION_40_MS;cfg.application_mode=ESP_OPUS_ENC_APPLICATION_VOIP;cfg.complexity=3;cfg.enable_vbr=true;
 void* encoder=nullptr;uint8_t* output=nullptr;int inputSize=0,outputSize=0;bool ok=false;
 do{
  if(esp_opus_enc_open(&cfg,sizeof(cfg),&encoder)!=ESP_AUDIO_ERR_OK)break;
  if(esp_opus_enc_get_frame_size(encoder,&inputSize,&outputSize)!=ESP_AUDIO_ERR_OK||inputSize!=1280||outputSize<1)break;
  output=static_cast<uint8_t*>(malloc(outputSize));if(!output)break;
  uint8_t header[20];const uint32_t serial=esp_random();uint32_t seq=0;
  OggOpus::head(header);bytes+=OggOpus::page(blob,capacity,header,19,serial,seq++,0,2);
  OggOpus::tags(header);bytes+=OggOpus::page(blob+bytes,capacity-bytes,header,20,serial,seq++,0,0);
  int16_t pcm[640];ok=true;
  for(size_t at=0;at<samples;at+=640){
   size_t n=min(size_t(640),samples-at);memset(pcm,0,sizeof(pcm));memcpy(pcm,wav+44+at*2,n*2);
   esp_audio_enc_in_frame_t input{reinterpret_cast<uint8_t*>(pcm),1280};esp_audio_enc_out_frame_t encoded{};encoded.buffer=output;encoded.len=outputSize;
   if(esp_opus_enc_process(encoder,&input,&encoded)!=ESP_AUDIO_ERR_OK){ok=false;break;}
   const bool last=at+n==samples;size_t added=OggOpus::page(blob+bytes,capacity-bytes,output,encoded.encoded_bytes,serial,seq++,uint64_t(at+n)*3,last?4:0);
   if(!added){ok=false;break;}bytes+=added;delay(1);
  }
 }while(false);
 if(encoder)esp_opus_enc_close(encoder);free(output);
 if(!ok){free(blob);bytes=0;return nullptr;}return blob;
}
}
