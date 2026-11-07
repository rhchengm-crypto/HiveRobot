#include "Microphone.h"
#include <driver/i2c_master.h>
#include <driver/i2s_std.h>
#include "src/esp_codec_dev/include/esp_codec_dev.h"
#include "src/esp_codec_dev/include/esp_codec_dev_defaults.h"
namespace Microphone {
static esp_codec_dev_handle_t device=nullptr;
static esp_codec_dev_handle_t speaker=nullptr;
static bool outputOpen=false;
bool begin() {
  i2c_master_bus_config_t busConfig{};
  busConfig.i2c_port=I2C_NUM_0;busConfig.sda_io_num=GPIO_NUM_13;busConfig.scl_io_num=GPIO_NUM_14;
  busConfig.clk_source=I2C_CLK_SRC_DEFAULT;busConfig.glitch_ignore_cnt=7;busConfig.flags.enable_internal_pullup=true;
  i2c_master_bus_handle_t bus=nullptr;
  if(i2c_new_master_bus(&busConfig,&bus)!=ESP_OK)return false;
  if(i2c_master_probe(bus,0x40,200)!=ESP_OK){Serial.println("[MIC] ES7210 not found at 0x40");return false;}
  i2s_chan_config_t channel=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0,I2S_ROLE_MASTER);
  channel.dma_desc_num=8;channel.dma_frame_num=256;
  channel.auto_clear_after_cb=true; // Silence, rather than repeated audio, on a network underrun.
  i2s_chan_handle_t rx=nullptr,tx=nullptr;
  if(i2s_new_channel(&channel,&tx,&rx)!=ESP_OK)return false;
  i2s_std_config_t standard{};
  standard.clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(16000);
  standard.slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,I2S_SLOT_MODE_STEREO);
  standard.gpio_cfg.mclk=GPIO_NUM_16;standard.gpio_cfg.bclk=GPIO_NUM_9;standard.gpio_cfg.ws=GPIO_NUM_45;
  standard.gpio_cfg.din=GPIO_NUM_10;standard.gpio_cfg.dout=GPIO_NUM_8;
  if(i2s_channel_init_std_mode(rx,&standard)!=ESP_OK || i2s_channel_init_std_mode(tx,&standard)!=ESP_OK)return false;
  audio_codec_i2c_cfg_t controlConfig{};controlConfig.port=0;controlConfig.addr=ES7210_CODEC_DEFAULT_ADDR;controlConfig.bus_handle=bus;
  const auto* control=audio_codec_new_i2c_ctrl(&controlConfig);if(!control)return false;
  es7210_codec_cfg_t adcConfig{};adcConfig.ctrl_if=control;
  // The official Waveshare input configuration selects MIC1 and MIC3 for stereo.
  adcConfig.mic_selected=ES7120_SEL_MIC1|ES7120_SEL_MIC3;
  const auto* adc=es7210_codec_new(&adcConfig);if(!adc)return false;
  audio_codec_i2s_cfg_t dataConfig{};dataConfig.port=0;dataConfig.rx_handle=rx;dataConfig.tx_handle=tx;
  const auto* data=audio_codec_new_i2s_data(&dataConfig);if(!data)return false;
  esp_codec_dev_cfg_t devConfig{};devConfig.dev_type=ESP_CODEC_DEV_TYPE_IN;devConfig.codec_if=adc;devConfig.data_if=data;
  device=esp_codec_dev_new(&devConfig);if(!device)return false;
  esp_codec_dev_sample_info_t format{};format.sample_rate=16000;format.channel=2;format.bits_per_sample=16;
  if(esp_codec_dev_open(device,&format)!=ESP_CODEC_DEV_OK)return false;
  if(esp_codec_dev_set_in_gain(device,24)!=ESP_CODEC_DEV_OK)return false;
  // Both codecs share the same 16kHz stereo bus. Speech is resampled before output.
  pinMode(46,OUTPUT);digitalWrite(46,LOW);
  if(i2c_master_probe(bus,0x18,200)==ESP_OK){
    audio_codec_i2c_cfg_t outControl{};outControl.port=0;outControl.addr=ES8311_CODEC_DEFAULT_ADDR;outControl.bus_handle=bus;
    const auto* ctrl=audio_codec_new_i2c_ctrl(&outControl);
    const auto* gpio=audio_codec_new_gpio();
    if(ctrl&&gpio){
      es8311_codec_cfg_t config{};config.ctrl_if=ctrl;config.gpio_if=gpio;config.codec_mode=ESP_CODEC_DEV_WORK_MODE_DAC;
      config.pa_pin=46;config.use_mclk=true;config.no_dac_ref=true;
      const auto* codec=es8311_codec_new(&config);
      if(codec){esp_codec_dev_cfg_t output{};output.dev_type=ESP_CODEC_DEV_TYPE_OUT;output.codec_if=codec;output.data_if=data;speaker=esp_codec_dev_new(&output);}
    }
  }
  Serial.printf("[SPEAKER] ready=%u codec=ES8311 dout=8 amp=46 rate=16000\n",speaker!=nullptr);
  Serial.println("[MIC] ES7210 stereo 16kHz/16bit; MCLK16 BCLK9 WS45 DIN10");return true;
}
void suspend(){if(device)esp_codec_dev_close(device);}
bool resume(){if(!device)return false;esp_codec_dev_sample_info_t format{};format.sample_rate=16000;format.channel=2;format.bits_per_sample=16;return esp_codec_dev_open(device,&format)==ESP_CODEC_DEV_OK && esp_codec_dev_set_in_gain(device,24)==ESP_CODEC_DEV_OK;}
int read(int16_t* stereo,size_t frames) {
  if(!device || esp_codec_dev_read(device,stereo,frames*4)!=ESP_CODEC_DEV_OK)return -1;
  return int(frames);
}
bool speakerReady(){return speaker!=nullptr;}
bool speakerOpen(){
 if(!speaker||outputOpen)return false;
 suspend();
 esp_codec_dev_sample_info_t format{};format.sample_rate=16000;format.channel=2;format.bits_per_sample=16;
 outputOpen=esp_codec_dev_open(speaker,&format)==ESP_CODEC_DEV_OK;
 if(outputOpen){esp_codec_dev_set_out_vol(speaker,70);esp_codec_dev_set_out_mute(speaker,false);}
 else {digitalWrite(46,LOW);resume();}
 return outputOpen;
}
bool speakerWrite(int16_t* stereo,size_t frames){return outputOpen&&esp_codec_dev_write(speaker,stereo,frames*4)==ESP_CODEC_DEV_OK;}
bool speakerClose(){
 if(outputOpen){esp_codec_dev_set_out_mute(speaker,true);esp_codec_dev_close(speaker);outputOpen=false;}
 digitalWrite(46,LOW);const bool ok=resume();Serial.printf("[SPEAKER] closed=1 mic_resumed=%u\n",ok);return ok;
}
}
