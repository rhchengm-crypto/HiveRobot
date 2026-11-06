#include "Battery.h"
#include <Arduino.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <algorithm>

namespace {
adc_oneshot_unit_handle_t adc=nullptr;
adc_cali_handle_t calibration=nullptr;
float volts=0;
bool available=false;
uint32_t lastSample=0;
int rawMean=0;

void sample() {
  if(!adc || !calibration)return;
  int readings[16];
  for(auto& reading:readings) {
    if(adc_oneshot_read(adc,ADC_CHANNEL_3,&reading)!=ESP_OK){available=false;return;}
  }
  std::sort(readings,readings+16);
  int sum=0;
  for(int i=2;i<14;++i)sum+=readings[i];
  rawMean=(sum+6)/12;
  int millivolts=0;
  if(adc_cali_raw_to_voltage(calibration,rawMean,&millivolts)!=ESP_OK){available=false;return;}
  // Waveshare GPIO4: 200k upper / 100k lower resistor divider.
  const float measured=millivolts*.003f;
  if(measured<2.0f || measured>4.6f){available=false;return;}
  volts=available?volts*.75f+measured*.25f:measured;
  available=true;
}
}

void Battery::begin() {
  adc_oneshot_unit_init_cfg_t unit{};unit.unit_id=ADC_UNIT_1;
  esp_err_t result=adc_oneshot_new_unit(&unit,&adc);
  if(result==ESP_OK){
    adc_oneshot_chan_cfg_t channel{};channel.atten=ADC_ATTEN_DB_12;channel.bitwidth=ADC_BITWIDTH_12;
    result=adc_oneshot_config_channel(adc,ADC_CHANNEL_3,&channel);
  }
  if(result==ESP_OK){
    adc_cali_curve_fitting_config_t curve{};curve.unit_id=ADC_UNIT_1;curve.chan=ADC_CHANNEL_3;
    curve.atten=ADC_ATTEN_DB_12;curve.bitwidth=ADC_BITWIDTH_12;
    result=adc_cali_create_scheme_curve_fitting(&curve,&calibration);
  }
  if(result!=ESP_OK){
    Serial.printf("[BATTERY] init_failed=%d\n",int(result));
    if(adc){adc_oneshot_del_unit(adc);adc=nullptr;}
    return;
  }
  sample();lastSample=millis();report();
}
void Battery::tick(){if(millis()-lastSample>=5000){lastSample=millis();sample();}}
bool Battery::valid(){return available;}
float Battery::voltage(){return volts;}
int Battery::percent(){
  // Approximate resting-voltage curve. Load, temperature and charging affect it.
  static const float v[]={3.0f,3.3f,3.5f,3.6f,3.7f,3.8f,3.9f,4.0f,4.1f,4.2f};
  static const int p[]={0,5,10,20,40,60,75,85,95,100};
  if(volts<=v[0])return 0;
  for(unsigned i=1;i<sizeof(v)/sizeof(v[0]);++i)
    if(volts<=v[i])return p[i-1]+int((volts-v[i-1])/(v[i]-v[i-1])*(p[i]-p[i-1])+.5f);
  return 100;
}
bool Battery::low(){return available&&volts<=3.5f;}
void Battery::report(){Serial.printf("[BATTERY] valid=%u gpio=4 raw=%d voltage=%.3f estimated_percent=%d low=%u\n",available,rawMean,double(volts),available?percent():-1,low());}
