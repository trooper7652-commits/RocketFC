#pragma once
//
// EEPROM-backed persistent settings: flight mode, servo trims, the
// flight-log counter, and the chute latch angles + last position. Guarded by
// magic + checksum; falls back to defaults on first boot or corruption. A
// version-1 record (before the chute fields) is migrated, keeping the mode,
// trims and flight counter.
//
#include <Arduino.h>
#include <EEPROM.h>

#include "../config.h"

class ConfigStore {
 public:
  // Last chute latch position commanded on the ground (matches
  // Actuators::ChutePos values).
  enum : uint8_t { CHUTE_LOCK = 0, CHUTE_RELEASE = 1, CHUTE_JOG = 2 };

  struct Data {
    uint32_t magic = kMagic;
    uint16_t version = 2;
    uint8_t mode = (uint8_t)cfg::DEFAULT_MODE;
    uint16_t flightCount = 0;
    float trimAUs = 0, trimBUs = 0;
    float chuteLockDeg = cfg::CHUTE_LOCK_DEG_DEFAULT;
    float chuteReleaseDeg = cfg::CHUTE_RELEASE_DEG_DEFAULT;
    uint8_t chuteLastPos = CHUTE_RELEASE;
    float chuteLastDeg = cfg::CHUTE_RELEASE_DEG_DEFAULT;
    uint16_t checksum = 0;
  };

  void load() {
    EEPROM.get(cfg::EEPROM_BASE_ADDR, data_);
    if (data_.magic == kMagic && data_.version == 2 &&
        data_.checksum == computeChecksum(data_))
      return;

    DataV1 v1;
    EEPROM.get(cfg::EEPROM_BASE_ADDR, v1);
    data_ = Data{};
    if (v1.magic == kMagic && v1.version == 1 &&
        v1.checksum == computeChecksum(v1)) {
      data_.mode = v1.mode;
      data_.flightCount = v1.flightCount;
      data_.trimAUs = v1.trimAUs;
      data_.trimBUs = v1.trimBUs;
    }
    save();
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

  // ---- chute latch ----
  float chuteLockDeg() const { return data_.chuteLockDeg; }
  float chuteReleaseDeg() const { return data_.chuteReleaseDeg; }
  uint8_t chuteLastPos() const { return data_.chuteLastPos; }
  float chuteLastDeg() const { return data_.chuteLastDeg; }
  void setChuteLockDeg(float d) { data_.chuteLockDeg = d; save(); }
  void setChuteReleaseDeg(float d) { data_.chuteReleaseDeg = d; save(); }
  // Ground moves only -- never call in flight (EEPROM write = flash write).
  void setChuteLast(uint8_t pos, float deg) {
    if (data_.chuteLastPos == pos && data_.chuteLastDeg == deg) return;
    data_.chuteLastPos = pos;
    data_.chuteLastDeg = deg;
    save();
  }
  void resetChute() {
    data_.chuteLockDeg = cfg::CHUTE_LOCK_DEG_DEFAULT;
    data_.chuteReleaseDeg = cfg::CHUTE_RELEASE_DEG_DEFAULT;
    save();
  }

 private:
  static constexpr uint32_t kMagic = 0x524B4643;  // "RKFC"

  // The version-1 layout, kept only to migrate old EEPROM contents.
  struct DataV1 {
    uint32_t magic;
    uint16_t version;
    uint8_t mode;
    uint16_t flightCount;
    float trimAUs, trimBUs;
    uint16_t checksum;
  };

  template <class T>
  static uint16_t computeChecksum(const T& d) {
    T tmp = d;
    tmp.checksum = 0;
    const uint8_t* p = (const uint8_t*)&tmp;
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(T); ++i) sum = (uint16_t)(sum * 31 + p[i]);
    return sum;
  }

  Data data_;
};
