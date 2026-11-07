#pragma once
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// Dedicated VSPI bus; LCD/touch use HSPI. Never format the user's card.
inline bool mountSdCard() {
  static SPIClass sdSpi(VSPI);
  SD.end();
  sdSpi.begin(18, 19, 23, 5);
  Serial.println("[SD] Detecting: CLK18 MISO19 MOSI23 CS5, 4MHz");
  uint32_t frequency = 4000000;
  if (!SD.begin(5, sdSpi, frequency, "/sd", 5, false)) {
    SD.end(); frequency = 1000000;
    Serial.println("[SD] Retrying at 1MHz");
    if (!SD.begin(5, sdSpi, frequency, "/sd", 5, false)) {
      Serial.println("[SD] FAIL: mount failed; check seating/power/filesystem. No formatting performed.");
      return false;
    }
  }
  const auto type = SD.cardType();
  if (type == CARD_NONE) { Serial.println("[SD] FAIL: no card"); return false; }
  const uint64_t total = SD.totalBytes(), used = SD.usedBytes();
  Serial.printf("[SD] MOUNT OK type=%s clock=%lu card=%llu bytes filesystem=%llu used=%llu free=%llu\n",
    type==CARD_SDHC ? "SDHC" : type==CARD_SD ? "SDSC" : "MMC", (unsigned long)frequency,
    (unsigned long long)SD.cardSize(), (unsigned long long)total,
    (unsigned long long)used, (unsigned long long)(total >= used ? total-used : 0));
  return true;
}
inline void runSdDiagnostic() {
  if(!mountSdCard())return;
  File root = SD.open("/");
  if (!root || !root.isDirectory()) { Serial.println("[SD] FAIL: root directory unreadable"); return; }
  unsigned count = 0;
  for (File file = root.openNextFile(); file && count < 16; file = root.openNextFile()) {
    Serial.printf("[SD] %s %s %lu bytes\n", file.isDirectory()?"DIR":"FILE", file.name(), (unsigned long)file.size());
    file.close(); ++count;
  }
  root.close();
  char path[48];
  unsigned attempt=0;
  do { snprintf(path,sizeof(path),"/codex_sd_check_%08lx_%u.tmp",(unsigned long)esp_random(),attempt++); }
  while (SD.exists(path) && attempt < 10);
  if (SD.exists(path)) { Serial.println("[SD] FAIL: no unused test filename"); return; }
  uint8_t buffer[512];
  const uint32_t start = millis();
  File test = SD.open(path, FILE_WRITE);
  bool good = bool(test);
  for (unsigned block=0; good && block<128; ++block) {
    for (unsigned i=0;i<sizeof(buffer);++i) buffer[i]=uint8_t(i*37+block*13);
    good = test.write(buffer,sizeof(buffer))==sizeof(buffer);
  }
  if (test) { test.flush(); test.close(); }
  const uint32_t written = millis();
  test = SD.open(path, FILE_READ);
  good = good && bool(test) && test.size()==65536;
  for (unsigned block=0; good && block<128; ++block) {
    good = test.read(buffer,sizeof(buffer))==sizeof(buffer);
    for (unsigned i=0;good && i<sizeof(buffer);++i) good=buffer[i]==uint8_t(i*37+block*13);
  }
  if(test) test.close();
  const uint32_t read = millis();
  const bool removed = SD.remove(path);
  Serial.printf("[SD] %s: 65536-byte write/read comparison; write=%lu ms read=%lu ms cleanup=%s\n",
    good?"PASS":"FAIL",(unsigned long)(written-start),(unsigned long)(read-written),removed?"OK":"FAILED");
  if (!removed) Serial.printf("[SD] Test file may remain: %s\n",path);
}
