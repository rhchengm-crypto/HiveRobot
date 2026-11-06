#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>

// Reference UI only. Existing applications can use KeyBridge without this driver.
// ESP32-3248S035R: ST7796 + resistive XPT2046, sharing HSPI pins 12/13/14.
class BoardDisplay : public Adafruit_GFX {
 public:
  BoardDisplay() : Adafruit_GFX(320, 480), spi_(HSPI) {}
  void begin();
  void setBacklight(bool enabled) {backlightEnabled_=enabled;digitalWrite(kBacklight,enabled?HIGH:LOW);}
  bool backlightEnabled() const {return backlightEnabled_;}
  void setRotation(uint8_t rotation) override;
  void diagnose(const char* stage);
  void drawPixel(int16_t x, int16_t y, uint16_t color) override;
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override;
  void drawFastHLine(int16_t x,int16_t y,int16_t w,uint16_t color) override { fillRect(x,y,w,1,color); }
  void drawFastVLine(int16_t x,int16_t y,int16_t h,uint16_t color) override { fillRect(x,y,1,h,color); }
  void drawIndexedBitmap(int16_t x,int16_t y,const uint8_t* pixels,const uint16_t* palette,
                         uint16_t w,uint16_t h,uint16_t background,uint8_t verticalInset=0);
  bool readTouch(uint16_t& x, uint16_t& y);
 private:
  uint32_t readRegister(uint8_t cmd, uint8_t count);
  void command(uint8_t cmd, const uint8_t* data = nullptr, size_t count = 0);
  void window(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  uint16_t adc(uint8_t cmd);
  SPIClass spi_;
  bool backlightEnabled_=true;
  static constexpr int kCs = 15, kDc = 2, kBacklight = 27, kTouchCs = 33;
};
