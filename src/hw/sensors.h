#pragma once
//
// Sensor layer: BMI088 IMU (Bolder Flight library) + custom non-blocking
// MS5611 barometer driver, axis mapping into the body frame, gyro-bias /
// level calibration, and the pad ground-reference (p0 + measured baro noise
// for the Kalman filter's R).
//
// The MS5611 driver is a small state machine: start a conversion, come back
// later to collect it. A blocking read (~5 ms at OSR 2048) would stall the
// 2 ms control loop, which is why the stock libraries aren't used.
//
#include <Arduino.h>
#include <Wire.h>

#include "BMI088.h"
#include "../config.h"
#include "../core/quat.h"

// ---------------------------------------------------------------------------
// MS5611 async driver
// ---------------------------------------------------------------------------
class Ms5611 {
 public:
  bool begin(TwoWire& w) {
    wire_ = &w;
    for (uint8_t a : {(uint8_t)0x77, (uint8_t)0x76}) {
      addr_ = a;
      if (probe()) { ok_ = true; return true; }
    }
    ok_ = false;
    return false;
  }

  // Call every loop; cheap. Sets newData() when a pressure sample lands.
  void tick(uint32_t ms) {
    if (!ok_) return;
    switch (phase_) {
      case Phase::START:
        startConversion(tempDue() ? CMD_CONV_D2 : CMD_CONV_D1, ms);
        break;
      case Phase::WAIT_D1:
        if (ms - convStartMs_ >= kConvMs) {
          const uint32_t d1 = readAdc();
          if (d1 != 0) {
            d1_ = d1;
            compute();
            newData_ = true;
            ++cycle_;
            errs_ = 0;
          } else if (++errs_ > 10) {
            ok_ = false;
          }
          phase_ = Phase::START;
        }
        break;
      case Phase::WAIT_D2:
        if (ms - convStartMs_ >= kConvMs) {
          const uint32_t d2 = readAdc();
          if (d2 != 0) { d2_ = d2; errs_ = 0; }
          else if (++errs_ > 10) { ok_ = false; }
          phase_ = Phase::START;
        }
        break;
    }
  }

  bool newData() {  // consuming read
    const bool n = newData_;
    newData_ = false;
    return n;
  }
  float pressurePa() const { return pressurePa_; }
  float temperatureC() const { return temperatureC_; }
  bool ok() const { return ok_; }
  uint8_t address() const { return addr_; }

 private:
  enum class Phase { START, WAIT_D1, WAIT_D2 };
  static constexpr uint8_t CMD_RESET = 0x1E;
  static constexpr uint8_t CMD_CONV_D1 = 0x46;  // pressure, OSR 2048
  static constexpr uint8_t CMD_CONV_D2 = 0x56;  // temperature, OSR 2048
  static constexpr uint8_t CMD_ADC_READ = 0x00;
  static constexpr uint8_t CMD_PROM_READ = 0xA0;
  static constexpr uint32_t kConvMs = 6;  // OSR 2048 max 4.6 ms + margin

  bool tempDue() const { return d2_ == 0 || (cycle_ % cfg::BARO_TEMP_EVERY) == 0; }

  void startConversion(uint8_t cmd, uint32_t ms) {
    if (!sendCmd(cmd)) { if (++errs_ > 10) ok_ = false; return; }
    convStartMs_ = ms;
    phase_ = (cmd == CMD_CONV_D1) ? Phase::WAIT_D1 : Phase::WAIT_D2;
  }

  bool probe() {
    if (!sendCmd(CMD_RESET)) return false;
    delay(4);  // reset needs 2.8 ms
    uint16_t sum = 0;
    for (int i = 0; i < 8; ++i) {
      prom_[i] = readProm(i);
      sum |= prom_[i];
    }
    if (sum == 0) return false;
    return crc4Ok();
  }

  bool sendCmd(uint8_t cmd) {
    wire_->beginTransmission(addr_);
    wire_->write(cmd);
    return wire_->endTransmission() == 0;
  }

  uint16_t readProm(int i) {
    wire_->beginTransmission(addr_);
    wire_->write((uint8_t)(CMD_PROM_READ + 2 * i));
    if (wire_->endTransmission() != 0) return 0;
    if (wire_->requestFrom(addr_, (uint8_t)2) != 2) return 0;
    const uint16_t hi = wire_->read(), lo = wire_->read();
    return (uint16_t)(hi << 8 | lo);
  }

  uint32_t readAdc() {
    wire_->beginTransmission(addr_);
    wire_->write(CMD_ADC_READ);
    if (wire_->endTransmission() != 0) return 0;
    if (wire_->requestFrom(addr_, (uint8_t)3) != 3) return 0;
    uint32_t v = 0;
    for (int i = 0; i < 3; ++i) v = (v << 8) | wire_->read();
    return v;
  }

  // Datasheet PROM CRC-4.
  bool crc4Ok() {
    uint16_t prom[8];
    memcpy(prom, prom_, sizeof(prom));
    const uint8_t crcStored = prom[7] & 0x000F;
    prom[7] &= 0xFF00;
    uint16_t rem = 0;
    for (int i = 0; i < 16; ++i) {
      rem ^= (i % 2 == 1) ? (prom[i >> 1] & 0x00FF) : (prom[i >> 1] >> 8);
      for (int b = 8; b > 0; --b)
        rem = (rem & 0x8000) ? (rem << 1) ^ 0x3000 : (rem << 1);
    }
    return ((rem >> 12) & 0xF) == crcStored;
  }

  // First + second order compensation per MS5611 datasheet.
  void compute() {
    const int32_t dT = (int32_t)d2_ - ((int32_t)prom_[5] << 8);
    int64_t TEMP = 2000 + ((int64_t)dT * prom_[6] >> 23);
    int64_t OFF = ((int64_t)prom_[2] << 16) + (((int64_t)prom_[4] * dT) >> 7);
    int64_t SENS = ((int64_t)prom_[1] << 15) + (((int64_t)prom_[3] * dT) >> 8);
    if (TEMP < 2000) {
      const int64_t t2 = ((int64_t)dT * dT) >> 31;
      const int64_t dt2k = TEMP - 2000;
      int64_t off2 = 5 * dt2k * dt2k / 2;
      int64_t sens2 = 5 * dt2k * dt2k / 4;
      if (TEMP < -1500) {
        const int64_t dtm15 = TEMP + 1500;
        off2 += 7 * dtm15 * dtm15;
        sens2 += 11 * dtm15 * dtm15 / 2;
      }
      TEMP -= t2;
      OFF -= off2;
      SENS -= sens2;
    }
    const int64_t P = ((((int64_t)d1_ * SENS) >> 21) - OFF) >> 15;
    pressurePa_ = (float)P;          // 0.01 mbar == Pa
    temperatureC_ = (float)TEMP / 100.0f;
  }

  TwoWire* wire_ = nullptr;
  uint8_t addr_ = 0x77;
  uint16_t prom_[8] = {0};
  Phase phase_ = Phase::START;
  uint32_t convStartMs_ = 0, d1_ = 0, d2_ = 0, cycle_ = 0;
  float pressurePa_ = 101325.0f, temperatureC_ = 20.0f;
  bool ok_ = false, newData_ = false;
  int errs_ = 0;
};

// ---------------------------------------------------------------------------
// Sensors facade
// ---------------------------------------------------------------------------
class Sensors {
 public:
  bool begin() {
    Wire.begin();
    Wire.setClock(cfg::I2C_HZ);

    // The BMI088 breakout may be strapped to the alternate addresses; probe
    // both. (Heap-allocated once at init — the library objects aren't
    // copyable because of const members.)
    imuOk_ = false;
    for (uint8_t aa : {(uint8_t)0x18, (uint8_t)0x19}) {
      auto* a = new Bmi088Accel(Wire, aa);
      if (a->begin() > 0) { bmiAccel_ = a; accelAddr_ = aa; break; }
      delete a;
    }
    if (bmiAccel_ != nullptr) {
      for (uint8_t ga : {(uint8_t)0x68, (uint8_t)0x69}) {
        auto* g = new Bmi088Gyro(Wire, ga);
        if (g->begin() > 0) { bmiGyro_ = g; gyroAddr_ = ga; imuOk_ = true; break; }
        delete g;
      }
    }
    if (imuOk_) {
      bmiAccel_->setRange(Bmi088Accel::RANGE_24G);
      bmiAccel_->setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ);
      bmiGyro_->setRange(Bmi088Gyro::RANGE_2000DPS);
      bmiGyro_->setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
    }

    baroOk_ = ms5611_.begin(Wire);
    return imuOk_ && baroOk_;
  }

  // ---- IMU (call at FAST_HZ) ----
  void readImu() {
    if (!imuOk_ || bmiAccel_ == nullptr || bmiGyro_ == nullptr) return;
    bmiAccel_->readSensor();
    bmiGyro_->readSensor();
    const Vec3 aRaw{bmiAccel_->getAccelX_mss(), bmiAccel_->getAccelY_mss(),
                    bmiAccel_->getAccelZ_mss()};
    const Vec3 gRaw{bmiGyro_->getGyroX_rads(), bmiGyro_->getGyroY_rads(),
                    bmiGyro_->getGyroZ_rads()};
    accelRaw_ = mapToBody(aRaw);
    gyroRaw_ = mapToBody(gRaw);

    // Staleness check: a wedged sensor returns identical raw values forever.
    if (gRaw.x == lastGyroRaw_.x && gRaw.y == lastGyroRaw_.y &&
        gRaw.z == lastGyroRaw_.z && aRaw.x == lastAccelRaw_.x) {
      if (++staleCount_ > 250) imuFresh_ = false;  // 0.5 s frozen
    } else {
      staleCount_ = 0;
      imuFresh_ = true;
    }
    lastGyroRaw_ = gRaw;
    lastAccelRaw_ = aRaw;

    if (calRunning_) calAccumulate();
  }

  Vec3 accel() const { return accelRaw_; }
  Vec3 gyro() const { return gyroRaw_ - gyroBias_; }
  Vec3 gyroBias() const { return gyroBias_; }
  bool imuOk() const { return imuOk_ && imuFresh_; }
  uint8_t accelAddr() const { return accelAddr_; }
  uint8_t gyroAddr() const { return gyroAddr_; }

  // ---- gyro bias + level calibration (vehicle must be still) ----
  void startImuCal() {
    calRunning_ = true;
    calDone_ = false;
    calCount_ = 0;
    calGyroSum_ = calAccelSum_ = Vec3{};
  }
  bool imuCalRunning() const { return calRunning_; }
  bool imuCalDone() const { return calDone_; }
  Vec3 accelLevelAvg() const { return calAccelAvg_; }

  // ---- barometer ----
  void baroTick(uint32_t ms) {
    ms5611_.tick(ms);
    if (ms5611_.newData()) {
      pendingBaro_ = true;
      if (gndCollecting_) gndAccumulate(ms5611_.pressurePa());
    }
  }
  bool baroNew() {  // consuming
    const bool n = pendingBaro_;
    pendingBaro_ = false;
    return n;
  }
  float baroAltitude() const {
    return 44330.0f * (1.0f - powf(ms5611_.pressurePa() / p0_, 0.190295f));
  }
  float baroPressure() const { return ms5611_.pressurePa(); }
  float baroTemperature() const { return ms5611_.temperatureC(); }
  bool baroOk() const { return ms5611_.ok(); }

  // Pad ground reference: collects BARO_GROUND_SAMPLES, sets p0 and measures
  // the baro noise (Kalman R).
  void startGroundRef() {
    gndCollecting_ = true;
    gndDone_ = false;
    gndN_ = 0;
    gndMean_ = gndM2_ = 0;
  }
  bool groundRefRunning() const { return gndCollecting_; }
  bool groundRefDone() const { return gndDone_; }
  float baroAltNoiseVar() const { return altNoiseVar_; }

 private:
  Vec3 mapToBody(const Vec3& s) const {
    const float* R = cfg::IMU_R_SB;
    return {R[0] * s.x + R[1] * s.y + R[2] * s.z,
            R[3] * s.x + R[4] * s.y + R[5] * s.z,
            R[6] * s.x + R[7] * s.y + R[8] * s.z};
  }

  void calAccumulate() {
    // Restart if the vehicle moves during calibration.
    if (gyroRaw_.norm() > cfg::GYRO_STILL_DPS * cfg::DEG2RAD * 3.0f &&
        calCount_ > 0) {
      startImuCal();
      return;
    }
    calGyroSum_ = calGyroSum_ + gyroRaw_;
    calAccelSum_ = calAccelSum_ + accelRaw_;
    if (++calCount_ >= (int)(cfg::GYRO_CAL_SECONDS * cfg::FAST_HZ)) {
      const float inv = 1.0f / (float)calCount_;
      gyroBias_ = calGyroSum_ * inv;
      calAccelAvg_ = calAccelSum_ * inv;
      calRunning_ = false;
      calDone_ = true;
    }
  }

  void gndAccumulate(float p) {
    // Welford running mean/variance on pressure.
    ++gndN_;
    const float d = p - gndMean_;
    gndMean_ += d / (float)gndN_;
    gndM2_ += d * (p - gndMean_);
    if (gndN_ >= cfg::BARO_GROUND_SAMPLES) {
      p0_ = gndMean_;
      const float varP = gndM2_ / (float)(gndN_ - 1);
      const float mPerPa = 8434.6f / p0_;  // d(alt)/d(p) near the pad
      altNoiseVar_ = varP * mPerPa * mPerPa;
      if (altNoiseVar_ < 0.0025f) altNoiseVar_ = 0.0025f;  // floor: (5 cm)^2
      gndCollecting_ = false;
      gndDone_ = true;
    }
  }

  Bmi088Accel* bmiAccel_ = nullptr;
  Bmi088Gyro* bmiGyro_ = nullptr;
  Ms5611 ms5611_;
  uint8_t accelAddr_ = 0x18, gyroAddr_ = 0x68;

  Vec3 accelRaw_{0, 0, cfg::G0}, gyroRaw_{};
  Vec3 lastAccelRaw_{}, lastGyroRaw_{};
  Vec3 gyroBias_{}, calGyroSum_{}, calAccelSum_{}, calAccelAvg_{0, 0, cfg::G0};
  bool imuOk_ = false, imuFresh_ = true, baroOk_ = false;
  int staleCount_ = 0;

  bool calRunning_ = false, calDone_ = false;
  int calCount_ = 0;

  bool pendingBaro_ = false;
  bool gndCollecting_ = false, gndDone_ = false;
  int gndN_ = 0;
  float gndMean_ = 0, gndM2_ = 0;
  float p0_ = 101325.0f, altNoiseVar_ = cfg::BARO_R_FALLBACK_M2;
};
