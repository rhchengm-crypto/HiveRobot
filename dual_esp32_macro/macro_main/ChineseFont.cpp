#include "ChineseFont.h"
#include "DogAnimation.h"
#include <SD.h>
namespace ChineseFont {
namespace {
constexpr uint32_t kBytes=16+0x7000*32;
constexpr char path[]="/fonts/cjk16.bin",temp[]="/fonts/cjk16.tmp",backup[]="/fonts/cjk16.prev";
File font,incoming;
volatile bool uploading=false;
uint32_t expected=0,received=0,crc=0,at=0;
uint8_t buffer[512];size_t buffered=0;
struct Glyph { uint16_t code=0;uint8_t bits[32]={}; };
Glyph cache[128];uint8_t cursor=0;
bool readGlyph(uint16_t c,uint8_t* bits) {
  for(const auto& item:cache)if(item.code==c){memcpy(bits,item.bits,32);return true;}
  if(!font.seek(16+uint32_t(c-0x3000)*32) || font.read(bits,32)!=32)return false;
  auto& item=cache[cursor++%128];item.code=c;memcpy(item.bits,bits,32);return true;
}
uint32_t crc32(uint32_t c,const uint8_t* p,size_t n) {
  c=~c;while(n--){c^=*p++;for(unsigned b=0;b<8;++b)c=(c>>1)^(0xedb88320u & (0u-(c&1)));}return ~c;
}
void abort(const char* reason) {
  incoming.close();SD.remove(temp);uploading=false;Serial.printf("[FONT] UPLOAD FAIL %s\n",reason);begin();doganim::begin();
}
uint32_t next(const char*& p) {
  const uint8_t a=uint8_t(*p++);if(a<128)return a;
  unsigned n=a>=0xf0?3:a>=0xe0?2:a>=0xc0?1:0;uint32_t c=a&((1u<<(6-n))-1);
  if(!n)return '?';
  while(n--) {const uint8_t b=uint8_t(*p);if((b&0xc0)!=0x80)return '?';++p;c=(c<<6)|(b&63);}
  return c;
}
}
void begin() {
  for(auto& item:cache)item.code=0;cursor=0;
  font.close();if(!SD.exists(path) && SD.exists(backup))SD.rename(backup,path);
  font=SD.open(path,FILE_READ);uint8_t header[16];
  if(!font || font.size()!=kBytes || font.read(header,16)!=16 || memcmp(header,"CJK16V1",8))font.close();
  Serial.printf("[FONT] ready=%u bytes=%lu\n",bool(font),font?(unsigned long)font.size():0);
}
bool uploadActive(){return uploading;}
bool ready(){return bool(font) && !uploading;}
void status(){Serial.printf("[FONT] STATUS ready=%u upload=%u bytes=%lu\n",ready(),uploading,font?(unsigned long)font.size():0);}
void startUpload(uint32_t bytes,uint32_t checksum) {
  if(uploading || bytes!=kBytes){Serial.println("[FONT] UPLOAD FAIL size/busy");return;}
  doganim::suspend();font.close();
  if(!SD.exists("/fonts") && !SD.mkdir("/fonts")){abort("directory");return;}
  if(SD.exists(temp) && !SD.remove(temp)){abort("temporary file");return;}
  incoming=SD.open(temp,FILE_WRITE);if(!incoming){abort("open");return;}
  uploading=true;expected=checksum;received=crc=buffered=0;at=millis();Serial.printf("[FONT] UPLOAD READY %lu\n",(unsigned long)bytes);
}
bool receiving() {
  if(!uploading)return false;
  if(uint32_t(millis()-at)>8000){abort("timeout");return true;}
  while(buffered<512 && received+buffered<kBytes && Serial.available()){buffer[buffered++]=Serial.read();at=millis();}
  if(buffered && (buffered==512 || received+buffered==kBytes)) {
    if(incoming.write(buffer,buffered)!=buffered){abort("write");return true;}
    crc=crc32(crc,buffer,buffered);received+=buffered;buffered=0;
    Serial.printf("[FONT] ACK %lu\n",(unsigned long)received);
    if(received==kBytes) {
      incoming.flush();incoming.close();
      if(crc!=expected){abort("CRC");return true;}
      // Validate the format before activating; keep the previous valid font on failure.
      File check=SD.open(temp,FILE_READ);uint8_t header[16];
      bool ok=check && check.size()==kBytes && check.read(header,16)==16 && !memcmp(header,"CJK16V1",8);check.close();
      if(!ok){abort("header");return true;}
      if(SD.exists(path)) {
        if((SD.exists(backup) && !SD.remove(backup)) || !SD.rename(path,backup)){abort("backup");return true;}
      }
      if(!SD.rename(temp,path)){if(SD.exists(backup))SD.rename(backup,path);abort("activate");return true;}
      uploading=false;Serial.printf("[FONT] UPLOAD OK bytes=%lu crc=%08lx\n",(unsigned long)received,(unsigned long)crc);begin();doganim::begin();
    }
  }
  return true;
}
void text(BoardDisplay& d,const char* p,int x,int y,int width,int lines,uint16_t ink,uint16_t bg) {
  int dx=0,line=0;uint8_t glyph[32];
  d.setTextSize(1);d.setTextColor(ink,bg);
  while(*p && line<lines) {
    const uint32_t c=next(p);const int w=c<128?8:16;
    if(dx+w>width){dx=0;++line;if(line>=lines)break;}
    if(c<128) {d.drawChar(x+dx,y+line*17+4,c,ink,bg,1);}
    else if(ready() && c>=0x3000 && c<0xa000 && readGlyph(c,glyph))
      { uint8_t pixels[256];uint16_t colors[2]={bg,ink};
        for(unsigned i=0;i<256;++i)pixels[i]=(glyph[i/8]>>(7-i%8))&1;
        d.drawIndexedBitmap(x+dx,y+line*17,pixels,colors,16,16,bg); }
    else d.drawRect(x+dx+2,y+line*17+2,12,12,ink);
    dx+=w;
  }
  if(*p && line>=lines) {d.fillRect(x+width-18,y+(lines-1)*17,18,16,bg);d.setCursor(x+width-18,y+(lines-1)*17+4);d.print("...");}
}
}
