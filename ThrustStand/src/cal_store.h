#pragma once
//
// EEPROM-backed calibration store: tare offset, counts-per-newton, and a run
// counter. Same magic + version + checksum scheme as ../../src/hw/config_store.h,
// so a half-written record or a fresh chip falls back to flagged-uncalibrated
// defaults instead of silently producing wrong newtons.
//
// Calibration is the one number a thrust stand cannot guess. If this record is
// invalid the firmware keeps running but reports UNCAL, and the CSV header
// records it so a bad run can never be mistaken for a good one later.
//
#include <Arduino.h>
#include <EEPROM.h>

#include "../config.h"

class CalStore {
 public:
  struct Data {
    uint32_t magic = kMagic;
    uint16_t version = 1;
    int32_t tareOffset = 0;
    float countsPerN = stand::DEFAULT_COUNTS_PER_N;
    uint8_t calibrated = 0;
    uint16_t runCount = 0;
    uint16_t checksum = 0;
  };

  void load() {
    EEPROM.get(stand::EEPROM_BASE_ADDR, data_);
    if (data_.magic != kMagic || data_.version != 1 ||
        data_.checksum != computeChecksum(data_) ||
        !(data_.countsPerN > 1.0f)) {
      data_ = Data{};
      save();
    }
  }

  void save() {
    data_.magic = kMagic;
    data_.checksum = computeChecksum(data_);
    // EEPROM.put only rewrites bytes that actually changed, so calling this on
    // every calibration is cheap against the ~100k write endurance.
    EEPROM.put(stand::EEPROM_BASE_ADDR, data_);
  }

  int32_t tareOffset() const { return data_.tareOffset; }
  float countsPerN() const { return data_.countsPerN; }
  bool calibrated() const { return data_.calibrated != 0; }

  void setCal(int32_t tare, float countsPerN, bool valid) {
    data_.tareOffset = tare;
    data_.countsPerN = countsPerN;
    data_.calibrated = valid ? 1 : 0;
    save();
  }

  void setTare(int32_t tare) {
    data_.tareOffset = tare;
    save();
  }

  uint16_t nextRunNumber() {
    ++data_.runCount;
    save();
    return data_.runCount;
  }
  uint16_t runCount() const { return data_.runCount; }

 private:
  static constexpr uint32_t kMagic = 0x54535444;  // "TSTD"

  static uint16_t computeChecksum(const Data& d) {
    Data tmp = d;
    tmp.checksum = 0;
    const uint8_t* p = (const uint8_t*)&tmp;
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(Data); ++i) sum = (uint16_t)(sum * 31 + p[i]);
    return sum;
  }

  Data data_;
};
