#include "Microphone.h"
#include <driver/i2c_master.h>
#include <driver/i2s_std.h>
#include "src/esp_codec_dev/include/esp_codec_dev.h"
#include "src/esp_codec_dev/include/esp_codec_dev_defaults.h"
namespace Microphone {
static esp_codec_dev_handle_t device=nullptr;
bool begin() {
  i2c_master_bus_config_t busConfig{};
  busConfig.i2c_port=I2C_NUM_0;busConfig.sda_io_num=GPIO_NUM_13;busConfig.scl_io_num=GPIO_NUM_14;
  busConfig.clk_source=I2C_CLK_SRC_DEFAULT;busConfig.glitch_ignore_cnt=7;busConfig.flags.enable_internal_pullup=true;
  i2c_master_bus_handle_t bus=nullptr;
  if(i2c_new_master_bus(&busConfig,&bus)!=ESP_OK)return false;
  if(i2c_master_probe(bus,0x40,200)!=ESP_OK){Serial.println("[MIC] ES7210 not found at 0x40");return false;}
  i2s_chan_config_t channel=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0,I2S_ROLE_MASTER);
  channel.dma_desc_num=8;channel.dma_frame_num=256;
  i2s_chan_handle_t rx=nullptr;
  if(i2s_new_channel(&channel,nullptr,&rx)!=ESP_OK)return false;
  i2s_std_config_t standard{};
  standard.clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(16000);
  standard.slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,I2S_SLOT_MODE_STEREO);
  standard.gpio_cfg.mclk=GPIO_NUM_16;standard.gpio_cfg.bclk=GPIO_NUM_9;standard.gpio_cfg.ws=GPIO_NUM_45;
  standard.gpio_cfg.din=GPIO_NUM_10;standard.gpio_cfg.dout=I2S_GPIO_UNUSED;
  if(i2s_channel_init_std_mode(rx,&standard)!=ESP_OK || i2s_channel_enable(rx)!=ESP_OK)return false;
  audio_codec_i2c_cfg_t controlConfig{};controlConfig.port=0;controlConfig.addr=ES7210_CODEC_DEFAULT_ADDR;controlConfig.bus_handle=bus;
  const auto* control=audio_codec_new_i2c_ctrl(&controlConfig);if(!control)return false;
  es7210_codec_cfg_t adcConfig{};adcConfig.ctrl_if=control;
  // The official Waveshare input configuration selects MIC1 and MIC3 for stereo.
  adcConfig.mic_selected=ES7120_SEL_MIC1|ES7120_SEL_MIC3;
  const auto* adc=es7210_codec_new(&adcConfig);if(!adc)return false;
  audio_codec_i2s_cfg_t dataConfig{};dataConfig.port=0;dataConfig.rx_handle=rx;
  const auto* data=audio_codec_new_i2s_data(&dataConfig);if(!data)return false;
  esp_codec_dev_cfg_t devConfig{};devConfig.dev_type=ESP_CODEC_DEV_TYPE_IN;devConfig.codec_if=adc;devConfig.data_if=data;
  device=esp_codec_dev_new(&devConfig);if(!device)return false;
  esp_codec_dev_sample_info_t format{};format.sample_rate=16000;format.channel=2;format.bits_per_sample=16;
  if(esp_codec_dev_open(device,&format)!=ESP_CODEC_DEV_OK)return false;
  if(esp_codec_dev_set_in_gain(device,24)!=ESP_CODEC_DEV_OK)return false;
  // Keep the unused power amplifier disabled: this version only listens.
  pinMode(46,OUTPUT);digitalWrite(46,LOW);
  Serial.println("[MIC] ES7210 stereo 16kHz/16bit; MCLK16 BCLK9 WS45 DIN10");return true;
}
int read(int16_t* stereo,size_t frames) {
  if(!device || esp_codec_dev_read(device,stereo,frames*4)!=ESP_CODEC_DEV_OK)return -1;
  return int(frames);
}
}
