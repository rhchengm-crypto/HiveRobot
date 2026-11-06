#include "DogAnimation.h"
#include "BoardDisplay.h"
#include <SD.h>
#include <esp_heap_caps.h>
namespace doganim {
namespace {
constexpr uint32_t kPixels=128*108, kHeader=32, kPalette=512, kRecord=kPixels+4;
constexpr uint32_t kSize=kHeader+kPalette+12*kRecord;
constexpr const char* kPath="/dogs/animation.dga";
File asset, incoming;
uint8_t pixels[kPixels];
uint16_t palette[256];
bool available=false, shown=false, loading=false, ready=false, uploading=false;
bool knownState[3]={}, powerState[3]={};
int lastKey[3]={-1,-1,-1}, verifyIndex=-1;
uint8_t pendingDog=0, pendingFrame=0, pendingInset=0, cursor=0;
int pendingKey=-1;
uint32_t offset=0, frameCrc=0, expectedFrameCrc=0;
uint32_t uploadBytes=0, uploadReceived=0, uploadCrc=0, expectedUploadCrc=0, uploadAt=0;
uint8_t uploadBuffer[512];
size_t uploadBuffered=0;
uint32_t draws=0, errors=0, maxReadUs=0, maxDrawUs=0;
uint32_t crc32(uint32_t crc,const uint8_t* data,size_t size) {
  crc=~crc;
  while(size--) {crc^=*data++;for(unsigned b=0;b<8;++b)crc=(crc>>1)^(0xedb88320u & (0u-(crc&1)));}
  return ~crc;
}
uint32_t le32(const uint8_t* p) {return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
uint16_t le16(const uint8_t* p) {return p[0]|(uint16_t(p[1])<<8);}
void fail(const char* message) {
  ++errors; available=false;loading=ready=false; verifyIndex=-1;
  if(asset)asset.close();
  Serial.printf("[DOG] %s; static artwork retained\n",message);
}
int desired(uint8_t dog,uint32_t now) {
  if(!knownState[dog])return -1;
  const uint32_t phase=now+uint32_t(dog)*1100;
  uint8_t variant=0,inset=0;
  if(powerState[dog]) {
    variant=(phase%4800)>=4350;
    inset=((phase/600)%4)==2?1:0;
  } else {
    static constexpr uint8_t breathing[]={0,0,1,2,2,1,0,0};
    const uint8_t step=(phase/400)%8;
    variant=step==3 || step==4; inset=breathing[step];
  }
  const uint8_t frame=((powerState[dog]?0:3)+dog)*2+variant;
  return frame*4+inset;
}
void startFrame(uint8_t frame) {
  uint8_t checksum[4];
  if(!asset.seek(kHeader+kPalette+uint32_t(frame)*kRecord) || asset.read(checksum,4)!=4) {
    fail("frame seek/read failed");return;
  }
  pendingFrame=frame; expectedFrameCrc=le32(checksum);frameCrc=0;offset=0;
  loading=true; ready=false;
}
void abortUpload(const char* reason) {
  if(incoming)incoming.close();
  SD.remove("/dogs/animation.tmp");uploading=false;
  Serial.printf("[DOG] UPLOAD FAIL %s\n",reason);begin();
}
void finishUpload() {
  incoming.flush();incoming.close();uploading=false;
  if(uploadCrc!=expectedUploadCrc) {abortUpload("CRC mismatch");return;}
  // Keep the prior valid pack until the new file is complete and CRC checked.
  const bool previous=SD.exists(kPath);
  if(previous) {
    if(SD.exists("/dogs/animation.prev") && !SD.remove("/dogs/animation.prev")) {
      abortUpload("cannot remove prior backup");return;
    }
    if(!SD.rename(kPath,"/dogs/animation.prev")) {abortUpload("cannot back up pack");return;}
  }
  if(!SD.rename("/dogs/animation.tmp",kPath)) {
    if(previous)SD.rename("/dogs/animation.prev",kPath);
    abortUpload("cannot activate pack");return;
  }
  Serial.printf("[DOG] UPLOAD OK bytes=%lu crc=%08lx\n",(unsigned long)uploadReceived,(unsigned long)uploadCrc);
  begin();
}
}
void suspend() {
  available=false; loading=ready=false; verifyIndex=-1;
  if(asset)asset.close();
}
void begin() {
  suspend();for(auto& key:lastKey)key=-1;
  if(!SD.exists(kPath) && SD.exists("/dogs/animation.prev"))SD.rename("/dogs/animation.prev",kPath);
  asset=SD.open(kPath,FILE_READ);
  if(!asset) {Serial.println("[DOG] SD animation pack absent; using static artwork");return;}
  uint8_t header[kHeader],colors[kPalette];
  if(asset.size()!=kSize || asset.read(header,kHeader)!=kHeader || memcmp(header,"DOGANIM1",8)
    || le16(header+8)!=128 || le16(header+10)!=108 || le16(header+12)!=12
    || le16(header+14)!=256 || le32(header+16)!=kPixels
    || le32(header+20)!=crc32(0,header,20)
    || asset.read(colors,kPalette)!=kPalette || le32(header+24)!=crc32(0,colors,kPalette)) {
    fail("invalid animation pack header/palette");return;
  }
  for(unsigned i=0;i<256;++i)palette[i]=le16(colors+i*2);
  available=true;
  Serial.printf("[DOG] SD animation READY: 3 dogs, 12 frames, %lu bytes\n",(unsigned long)kSize);
}
bool receiving() {
  if(!uploading)return false;
  if(uint32_t(millis()-uploadAt)>8000) {abortUpload("receive timeout");return true;}
  while(uploadBuffered<sizeof(uploadBuffer) && uploadReceived+uploadBuffered<uploadBytes && Serial.available()) {
    uploadBuffer[uploadBuffered++]=Serial.read();uploadAt=millis();
  }
  if(uploadBuffered && (uploadBuffered==sizeof(uploadBuffer) || uploadReceived+uploadBuffered==uploadBytes)) {
    if(incoming.write(uploadBuffer,uploadBuffered)!=uploadBuffered) {abortUpload("SD write failed");return true;}
    uploadCrc=crc32(uploadCrc,uploadBuffer,uploadBuffered);uploadReceived+=uploadBuffered;uploadBuffered=0;
    // Host sends at most 512 bytes, waits for this cumulative acknowledgement.
    Serial.printf("[DOG] ACK %lu\n",(unsigned long)uploadReceived);
    if(uploadReceived==uploadBytes)finishUpload();
  }
  return true;
}
void startUpload(uint32_t bytes,uint32_t crc) {
  if(uploading || bytes!=kSize) {Serial.println("[DOG] UPLOAD FAIL invalid size/busy");return;}
  suspend();
  if(!SD.exists("/dogs") && !SD.mkdir("/dogs")) {Serial.println("[DOG] UPLOAD FAIL no SD directory");begin();return;}
  if(SD.exists("/dogs/animation.tmp") && !SD.remove("/dogs/animation.tmp")) {
    Serial.println("[DOG] UPLOAD FAIL stale temporary file");begin();return;
  }
  incoming=SD.open("/dogs/animation.tmp",FILE_WRITE);
  if(!incoming) {Serial.println("[DOG] UPLOAD FAIL cannot create file");begin();return;}
  uploading=true;uploadBytes=bytes;expectedUploadCrc=crc;uploadReceived=uploadCrc=0;uploadBuffered=0;uploadAt=millis();
  Serial.printf("[DOG] UPLOAD READY %lu\n",(unsigned long)bytes);
}
void service() {
  if(!available || uploading)return;
  const uint32_t at=micros();
  if(loading) {
    const size_t count=min(uint32_t(512),kPixels-offset);
    if(asset.read(pixels+offset,count)!=count) {fail("short frame read");return;}
    frameCrc=crc32(frameCrc,pixels+offset,count);offset+=count;
    if(offset==kPixels) {
      loading=false;
      if(frameCrc!=expectedFrameCrc) {fail("frame CRC mismatch");return;}
      ready=true;
    }
  }
  if(verifyIndex>=0 && ready) {
    ready=false;
    if(++verifyIndex==12) {verifyIndex=-1;Serial.println("[DOG] VERIFY PASS: all 12 SD frames CRC OK");}
    else startFrame(verifyIndex);
  }
  const uint32_t elapsed=micros()-at;if(elapsed>maxReadUs)maxReadUs=elapsed;
}
void verify() {
  if(!available || uploading) {Serial.println("[DOG] VERIFY unavailable");return;}
  verifyIndex=0;startFrame(0);Serial.println("[DOG] VERIFY started");
}
void status() {
  Serial.printf("[DOG] STATUS ready=%u visible=%u upload=%u draws=%lu errors=%lu maxReadUs=%lu maxDrawUs=%lu heap=%u largest=%u\n",
    available,shown,uploading,(unsigned long)draws,(unsigned long)errors,(unsigned long)maxReadUs,
    (unsigned long)maxDrawUs,ESP.getFreeHeap(),heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}
void visible(bool value) {
  if(shown!=value) {shown=value;if(verifyIndex<0)loading=ready=false;for(auto& key:lastKey)key=-1;}
}
void update(BoardDisplay& display,const bool* known,const bool* power,bool force,bool touching) {
  for(unsigned i=0;i<3;++i) {
    if(force || knownState[i]!=known[i] || powerState[i]!=power[i])lastKey[i]=-1;
    knownState[i]=known[i];powerState[i]=power[i];
  }
  if(!shown || !available || uploading || verifyIndex>=0 || touching)return;
  const uint32_t now=millis();
  if(ready) {
    if(desired(pendingDog,now)==pendingKey) {
      const uint16_t background=powerState[pendingDog]?0xff7a:0xd6ba;
      const uint32_t at=micros();
      display.drawIndexedBitmap(20+pendingDog*156,114,pixels,palette,128,108,background,pendingInset);
      const uint32_t elapsed=micros()-at;if(elapsed>maxDrawUs)maxDrawUs=elapsed;
      lastKey[pendingDog]=pendingKey;++draws;
    }
    ready=false;
  }
  if(!loading) {
    for(unsigned n=0;n<3;++n) {
      const uint8_t dog=(cursor+n)%3;const int key=desired(dog,now);
      if(key<0 || key==lastKey[dog])continue;
      pendingDog=dog;pendingKey=key;pendingInset=key%4;cursor=(dog+1)%3;startFrame(key/4);break;
    }
  }
}
}
