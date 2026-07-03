#pragma once
//
// EEPROM-backed persistent settings: flight mode, servo trims, and the
// flight-log counter. Guarded by magic + checksum; falls back to defaults on
// first boot or corruption.
//
#include <Arduino.h>
#include <EEPROM.h>

#include "../config.h"

class ConfigStore {
 public:
  struct Data {
    uint32_t magic = kMagic;
    uint16_t version = 1;
    uint8_t mode = (uint8_t)cfg::DEFAULT_MODE;
    uint16_t flightCount = 0;
    float trimAUs = 0, trimBUs = 0;
    uint16_t checksum = 0;
  };

  void load() {
    EEPROM.get(cfg::EEPROM_BASE_ADDR, data_);
    if (data_.magic != kMagic || data_.checksum != computeChecksum(data_)) {
      data_ = Data{};
      save();
    }
  }

  void save() {
    data_.magic = kMagic;
    data_.checksum = computeChecksum(data_);
    EEPROM.put(cfg::EEPROM_BASE_ADDR, data_);
  }

  cfg::FlightMode mode() const { return (cfg::FlightMode)data_.mode; }
  void setMode(cfg::FlightMode m) { data_.mode = (uint8_t)m; save(); }

  int nextFlightNumber() {
    ++data_.flightCount;
    save();
    return data_.flightCount;
  }

  float trimAUs() const { return data_.trimAUs; }
  float trimBUs() const { return data_.trimBUs; }
  void setTrims(float a, float b) {
    data_.trimAUs = a;
    data_.trimBUs = b;
    save();
  }

 private:
  static constexpr uint32_t kMagic = 0x524B4643;  // "RKFC"

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
