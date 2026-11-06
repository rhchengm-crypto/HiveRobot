#include "BoardDisplay.h"

void BoardDisplay::command(uint8_t cmd, const uint8_t* data, size_t count) {
  spi_.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(kCs, LOW); digitalWrite(kDc, LOW); spi_.transfer(cmd);
  digitalWrite(kDc, HIGH);
  if (count) spi_.writeBytes(data, count);
  digitalWrite(kCs, HIGH); spi_.endTransaction();
}
uint32_t BoardDisplay::readRegister(uint8_t cmd, uint8_t count) {
  spi_.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(kTouchCs, HIGH); digitalWrite(kCs, LOW);
  digitalWrite(kDc, LOW); spi_.transfer(cmd); digitalWrite(kDc, HIGH);
  uint32_t value = 0;
  for (uint8_t i = 0; i < count; ++i) value = (value << 8) | spi_.transfer(0);
  digitalWrite(kCs, HIGH); spi_.endTransaction();
  return value;
}
void BoardDisplay::diagnose(const char* stage) {
  // Raw readings include dummy bytes. Some panels do not expose readable SDO;
  // 0x00/0xff is diagnostic information, not proof of a dead panel.
  const uint32_t id = readRegister(0xd3, 4);
  const uint32_t power = readRegister(0x0a, 2);
  const uint32_t format = readRegister(0x0c, 2);
  Serial.printf("[LCD] %s BL27=%d ID_D3=%08lx POWER_0A=%04lx FORMAT_0C=%04lx (raw)\n",
                stage, digitalRead(kBacklight), (unsigned long)id,
                (unsigned long)power, (unsigned long)format);
}
void BoardDisplay::setRotation(uint8_t orientation) {
  // Match ST7796 rotation registers to Adafruit_GFX's logical dimensions.
  static const uint8_t madctl[] = {0x48, 0x28, 0x88, 0xe8};
  Adafruit_GFX::setRotation(orientation & 3);
  command(0x36, &madctl[orientation & 3], 1);
}
void BoardDisplay::begin() {
  pinMode(kCs, OUTPUT); digitalWrite(kCs, HIGH);
  pinMode(kTouchCs, OUTPUT); digitalWrite(kTouchCs, HIGH);
  pinMode(5, OUTPUT); digitalWrite(5, HIGH); // Keep the unused SD SPI device deselected.
  pinMode(kDc, OUTPUT); pinMode(kBacklight, OUTPUT); digitalWrite(kBacklight, LOW);
  pinMode(36, INPUT); // Resistive touch IRQ, input-only GPIO.
  spi_.begin(14, 12, 13, kCs);
  command(0x01); delay(120); command(0x11); delay(120);
  const uint8_t unlock1[] = {0xc3}, unlock2[] = {0x96}, madctl[] = {0x48}, format[] = {0x55};
  command(0xf0, unlock1, 1); command(0xf0, unlock2, 1);
  command(0x36, madctl, 1); command(0x3a, format, 1);
  const uint8_t inversion[] = {1}, function[] = {0x80, 0x02, 0x3b};
  const uint8_t output[] = {0x40, 0x8a, 0, 0, 0x29, 0x19, 0xa5, 0x33};
  const uint8_t power2[] = {6}, power3[] = {0xa7}, vcom[] = {0x18};
  command(0xb4, inversion, 1); command(0xb6, function, sizeof(function));
  command(0xe8, output, sizeof(output)); command(0xc1, power2, 1);
  command(0xc2, power3, 1); command(0xc5, vcom, 1);
  delay(120);
  const uint8_t positive[] = {0xf0, 9, 0x0b, 6, 4, 0x15, 0x2f, 0x54, 0x42, 0x3c, 0x17, 0x14, 0x18, 0x1b};
  const uint8_t negative[] = {0xe0, 9, 0x0b, 6, 4, 3, 0x2b, 0x43, 0x42, 0x3b, 0x16, 0x14, 0x17, 0x1b};
  command(0xe0, positive, sizeof(positive)); command(0xe1, negative, sizeof(negative));
  delay(120);
  const uint8_t lock1[] = {0x3c}, lock2[] = {0x69};
  command(0xf0, lock1, 1); command(0xf0, lock2, 1); delay(120); command(0x29); delay(20);
  setRotation(1); // Landscape 480x320, restored after every LCD recovery.
  fillScreen(0); digitalWrite(kBacklight, backlightEnabled_?HIGH:LOW); setTextWrap(false);
  Serial.println("[LCD] ST7796 initialized; SPI14/13/12 CS15 DC2 BL27");
}
void BoardDisplay::window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  const uint16_t right = x + w - 1, bottom = y + h - 1;
  digitalWrite(kDc, LOW); spi_.transfer(0x2a); digitalWrite(kDc, HIGH);
  spi_.transfer16(x); spi_.transfer16(right);
  digitalWrite(kDc, LOW); spi_.transfer(0x2b); digitalWrite(kDc, HIGH);
  spi_.transfer16(y); spi_.transfer16(bottom);
  digitalWrite(kDc, LOW); spi_.transfer(0x2c); digitalWrite(kDc, HIGH);
}
void BoardDisplay::drawPixel(int16_t x, int16_t y, uint16_t color) { fillRect(x, y, 1, 1, color); }
void BoardDisplay::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (x < 0) { w += x; x = 0; } if (y < 0) { h += y; y = 0; }
  if (x >= width() || y >= height() || w <= 0 || h <= 0) return;
  if (x + w > width()) w = width() - x;
  if (y + h > height()) h = height() - y;
  uint8_t block[128];
  for (unsigned i = 0; i < sizeof(block); i += 2) { block[i] = color >> 8; block[i + 1] = color; }
  spi_.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0)); digitalWrite(kCs, LOW);
  window(x, y, w, h);
  uint32_t remaining = static_cast<uint32_t>(w) * h * 2;
  while (remaining) {
    size_t count = remaining > sizeof(block) ? sizeof(block) : remaining;
    spi_.writeBytes(block, count); remaining -= count;
  }
  digitalWrite(kCs, HIGH); spi_.endTransaction();
}
void BoardDisplay::drawIndexedBitmap(int16_t x,int16_t y,const uint8_t* pixels,const uint16_t* palette,
                                     uint16_t w,uint16_t h,uint16_t background,uint8_t verticalInset) {
  if(!pixels || !palette || !w || !h || x<0 || y<0 || x+w>width() || y+h>height())return;
  // One bounded 128-byte transfer buffer, no heap allocation or framebuffer.
  uint8_t block[128];
  spi_.beginTransaction(SPISettings(20000000,MSBFIRST,SPI_MODE0));digitalWrite(kCs,LOW);
  window(x,y,w,h);
  const uint32_t total=uint32_t(w)*h;
  for(uint32_t offset=0;offset<total;) {
    const uint32_t count=(total-offset)>64?64:total-offset;
    for(uint32_t j=0;j<count;++j) {
      const uint32_t output=offset+j, row=output/w;
      const uint32_t sourceRow=verticalInset && row>=verticalInset ? (row-verticalInset)*h/(h-verticalInset) : row;
      const uint8_t index=(verticalInset && row<verticalInset)?0:pgm_read_byte(pixels+sourceRow*w+output%w);
      const uint16_t color=index?pgm_read_word(palette+index):background;
      block[j*2]=color>>8;block[j*2+1]=color;
    }
    spi_.writeBytes(block,count*2);offset+=count;
  }
  digitalWrite(kCs,HIGH);spi_.endTransaction();
}
uint16_t BoardDisplay::adc(uint8_t cmd) {
  spi_.transfer(cmd);
  uint16_t value = static_cast<uint16_t>(spi_.transfer(0)) << 8;
  value |= spi_.transfer(0); return value >> 3;
}
bool BoardDisplay::readTouch(uint16_t& x, uint16_t& y) {
  spi_.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(kTouchCs, LOW);
  uint16_t z = adc(0xb0);
  x = adc(0xd0); y = adc(0x90);
  adc(0x00); // Power down ADC and re-enable PENIRQ.
  digitalWrite(kTouchCs, HIGH); spi_.endTransaction();
  return z > 150 && x > 50 && x < 4050 && y > 50 && y < 4050;
}
