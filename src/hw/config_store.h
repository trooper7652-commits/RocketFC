#pragma once
//
// EEPROM-backed persistent settings: flight mode, servo trims, the
// flight-log counter, and the chute latch angles (LOCK / SET / OPEN) + last
// position. Guarded by magic + checksum; falls back to defaults on first boot
// or corruption. Older records are migrated, keeping the mode, trims and
// flight counter: version 1 (before the chute fields) and version 2 (LOCK and
// OPEN only, placeholder angles -- reset to the current defaults).
//
#include <Arduino.h>
#include <EEPROM.h>

#include "../config.h"

class ConfigStore {
 public:
  // Last chute latch position commanded on the ground (matches
  // Actuators::ChutePos values).
  enum : uint8_t { CHUTE_LOCK = 0, CHUTE_RELEASE = 1, CHUTE_JOG = 2, CHUTE_SET = 3 };

  struct Data {
    uint32_t magic = kMagic;
    uint16_t version = 3;
    uint8_t mode = (uint8_t)cfg::DEFAULT_MODE;
    uint16_t flightCount = 0;
    float trimAUs = 0, trimBUs = 0;
    float chuteLockDeg = cfg::CHUTE_LOCK_DEG_DEFAULT;
    float chuteReleaseDeg = cfg::CHUTE_RELEASE_DEG_DEFAULT;
    uint8_t chuteLastPos = CHUTE_RELEASE;
    float chuteLastDeg = cfg::CHUTE_RELEASE_DEG_DEFAULT;
    float chuteSetDeg = cfg::CHUTE_SET_DEG_DEFAULT;
    uint16_t checksum = 0;
  };

  void load() {
    EEPROM.get(cfg::EEPROM_BASE_ADDR, data_);
    if (data_.magic == kMagic && data_.version == 3 &&
        data_.checksum == computeChecksum(data_))
      return;

    // Older layouts: keep the mode, trims, flight counter and (v2) the last
    // latch position; the chute angles start from the current defaults.
    DataV2 v2;
    EEPROM.get(cfg::EEPROM_BASE_ADDR, v2);
    DataV1 v1;
    EEPROM.get(cfg::EEPROM_BASE_ADDR, v1);
    data_ = Data{};
    if (v2.magic == kMagic && v2.version == 2 &&
        v2.checksum == computeChecksum(v2)) {
      data_.mode = v2.mode;
      data_.flightCount = v2.flightCount;
      data_.trimAUs = v2.trimAUs;
      data_.trimBUs = v2.trimBUs;
      data_.chuteLastPos = v2.chuteLastPos;
      data_.chuteLastDeg = v2.chuteLastDeg;
    } else if (v1.magic == kMagic && v1.version == 1 &&
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
  float chuteSetDeg() const { return data_.chuteSetDeg; }
  float chuteReleaseDeg() const { return data_.chuteReleaseDeg; }
  uint8_t chuteLastPos() const { return data_.chuteLastPos; }
  float chuteLastDeg() const { return data_.chuteLastDeg; }
  void setChuteLockDeg(float d) { data_.chuteLockDeg = d; save(); }
  void setChuteSetDeg(float d) { data_.chuteSetDeg = d; save(); }
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
    data_.chuteSetDeg = cfg::CHUTE_SET_DEG_DEFAULT;
    data_.chuteReleaseDeg = cfg::CHUTE_RELEASE_DEG_DEFAULT;
    save();
  }

 private:
  static constexpr uint32_t kMagic = 0x524B4643;  // "RKFC"

  // Older layouts, kept only to migrate old EEPROM contents.
  struct DataV1 {
    uint32_t magic;
    uint16_t version;
    uint8_t mode;
    uint16_t flightCount;
    float trimAUs, trimBUs;
    uint16_t checksum;
  };
  struct DataV2 {
    uint32_t magic;
    uint16_t version;
    uint8_t mode;
    uint16_t flightCount;
    float trimAUs, trimBUs;
    float chuteLockDeg;
    float chuteReleaseDeg;
    uint8_t chuteLastPos;
    float chuteLastDeg;
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
