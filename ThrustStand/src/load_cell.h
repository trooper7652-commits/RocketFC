#pragma once
//
// HX711 load-cell front end.
//
// Deliberately NOT the common bogde/HX711 library: its read() busy-waits on
// DOUT until a conversion is ready, which at 80 SPS means parking the CPU for
// up to 12.5 ms. During a burn that would stall the IR decoder (so you could
// not abort) and freeze the LCD. This driver is a poll-and-return state
// machine in the shape of Ms5611::tick() in ../../src/hw/sensors.h.
//
// Interrupts are intentionally NOT disabled around the bit-bang. The HX711
// powers itself down if SCK is held high longer than 60 us, and IRremote's
// 50 us sampling ISR can land inside a pulse -- but that ISR is a few
// microseconds against a ~10 us pulse and a 60 us limit, so the margin is
// large. Masking interrupts instead would drop IR samples and corrupt remote
// frames, which is the far worse failure: a missed abort.
//
// Tare and calibration are non-blocking accumulators (the pattern from
// calAccumulate()/gndAccumulate() in ../../src/hw/sensors.h) so the UI stays
// alive while they run. Sums are int64 rather than float because on AVR a
// double IS a float, and 160 samples of a 24-bit converter overflow a 24-bit
// mantissa.
//
#include <Arduino.h>

#include "../config.h"

enum class CalPhase : uint8_t {
  NONE = 0,
  TARE,       // averaging the zero-load offset
  CAL_WAIT,   // waiting for the operator to enter the reference mass
  CAL_LOAD,   // averaging with the reference mass in place
};

class LoadCell {
 public:
  void begin() {
    pinMode(stand::PIN_HX711_DOUT, INPUT);
    pinMode(stand::PIN_HX711_SCK, OUTPUT);
    digitalWrite(stand::PIN_HX711_SCK, LOW);
    lastReadyMs_ = millis();
  }

  // -------------------------------------------------------------------------
  // Sampling
  // -------------------------------------------------------------------------

  // Returns true on the loop pass where a NEW sample was taken.
  bool tick(uint32_t ms) {
    if (digitalRead(stand::PIN_HX711_DOUT) != LOW) {
      if (ms - lastReadyMs_ > stand::HX711_TIMEOUT_MS) fault_ = true;
      return false;
    }
    const int32_t raw = shiftRaw_();
    const uint32_t us = micros();

    // Measured conversion period, lightly smoothed -> 10 vs 80 SPS detection.
    if (haveSample_) {
      const float dtMs = (us - lastSampleUs_) * 0.001f;
      if (dtMs > 0.5f && dtMs < 500.0f) periodMs_ += 0.2f * (dtMs - periodMs_);
    }
    lastSampleUs_ = us;
    lastReadyMs_ = ms;
    fault_ = false;

    raw_ = raw;
    thrustN_ = (float)(raw - tareOffset_) / countsPerN_;

    // One-pole IIR, same form as control.h: alpha = dt / (dt + 1/(2*pi*fc)).
    const float dt = periodMs_ * 0.001f;
    if (!haveSample_) {
      thrustFiltN_ = thrustN_;
    } else if (dt > 0.0f) {
      const float alpha =
          dt / (dt + 1.0f / (2.0f * stand::PI_F * stand::LPF_HZ));
      thrustFiltN_ += alpha * (thrustN_ - thrustFiltN_);
    }
    haveSample_ = true;

    accumulate_(raw);
    return true;
  }

  int32_t raw() const { return raw_; }
  float thrustN() const { return thrustN_; }
  float thrustFiltN() const { return thrustFiltN_; }
  bool fault() const { return fault_; }
  bool haveSample() const { return haveSample_; }
  float periodMs() const { return periodMs_; }
  float sampleRateHz() const {
    return periodMs_ > 0.01f ? 1000.0f / periodMs_ : 0.0f;
  }
  bool fastRate() const {
    return periodMs_ > 0.01f && periodMs_ <= stand::RATE_FAST_MAX_MS;
  }

  bool calibrated() const { return calibrated_; }
  float countsPerN() const { return countsPerN_; }
  int32_t tareOffset() const { return tareOffset_; }
  float noiseN() const { return noiseN_; }

  bool overloaded() const {
    return fabs(thrustN_) > stand::CELL_CAPACITY_N * stand::OVERLOAD_WARN_FRAC;
  }

  void applyCal(int32_t tare, float countsPerN, bool valid) {
    tareOffset_ = tare;
    if (countsPerN > 1.0f) countsPerN_ = countsPerN;
    calibrated_ = valid;
  }

  // -------------------------------------------------------------------------
  // Tare / calibration -- non-blocking; poll phase() and the done flags.
  // -------------------------------------------------------------------------

  void startTare() { beginAccum_(CalPhase::TARE, stand::TARE_SAMPLES); }

  // Call once the reference mass is resting on the cell, with its true mass in
  // grams (weigh it on a kitchen scale). Tare must have been run first.
  bool startCalLoad(float grams) {
    if (grams < stand::CAL_MIN_GRAMS || grams > stand::CAL_MAX_GRAMS) {
      return false;
    }
    calGrams_ = grams;
    beginAccum_(CalPhase::CAL_LOAD, stand::CAL_SAMPLES);
    return true;
  }

  void awaitCalMass() { phase_ = CalPhase::CAL_WAIT; }
  void cancelCal() { phase_ = CalPhase::NONE; }

  CalPhase phase() const { return phase_; }
  uint16_t progress() const { return count_; }
  uint16_t progressTarget() const { return target_; }
  float calGrams() const { return calGrams_; }

  // Set for one loop pass when an accumulation completes.
  bool tareComplete() const { return tareComplete_; }
  bool calComplete() const { return calComplete_; }
  void clearFlags() { tareComplete_ = calComplete_ = false; }

 private:
  // Clock out 24 bits MSB-first, then the gain-select pulses. Every SCK high
  // pulse must stay well under 60 us -- do not "optimize" this into a direct
  // PORT burst without re-checking that, and do not add anything slow here.
  int32_t shiftRaw_() {
    uint32_t v = 0;
    for (uint8_t i = 0; i < 24; ++i) {
      digitalWrite(stand::PIN_HX711_SCK, HIGH);
      delayMicroseconds(1);
      v = (v << 1) | (uint32_t)digitalRead(stand::PIN_HX711_DOUT);
      digitalWrite(stand::PIN_HX711_SCK, LOW);
      delayMicroseconds(1);
    }
    for (uint8_t i = 24; i < stand::HX711_GAIN_PULSES; ++i) {
      digitalWrite(stand::PIN_HX711_SCK, HIGH);
      delayMicroseconds(1);
      digitalWrite(stand::PIN_HX711_SCK, LOW);
      delayMicroseconds(1);
    }
    if (v & 0x800000UL) v |= 0xFF000000UL;  // sign-extend 24 -> 32
    return (int32_t)v;
  }

  void beginAccum_(CalPhase p, uint16_t target) {
    phase_ = p;
    target_ = target;
    count_ = 0;
    sum_ = 0;
    sumSq_ = 0;
    first_ = 0;
  }

  void accumulate_(int32_t raw) {
    if (phase_ != CalPhase::TARE && phase_ != CalPhase::CAL_LOAD) return;
    if (count_ == 0) first_ = raw;
    const int32_t d = raw - first_;  // shifted origin keeps the sums exact
    sum_ += (int64_t)d;
    sumSq_ += (int64_t)d * (int64_t)d;
    if (++count_ < target_) return;

    const float n = (float)count_;
    const float mean = (float)first_ + (float)sum_ / n;
    const float var =
        ((float)sumSq_ - (float)sum_ * (float)sum_ / n) / (n - 1.0f);

    if (phase_ == CalPhase::TARE) {
      tareOffset_ = (int32_t)(mean + 0.5f);
      noiseN_ = (var > 0.0f ? sqrt(var) : 0.0f) / countsPerN_;
      tareComplete_ = true;
    } else {
      const float deltaCounts = mean - (float)tareOffset_;
      const float refN = calGrams_ * 0.001f * stand::G0;
      // A near-zero delta means the mass never made it onto the cell.
      if (refN > 0.0f && fabs(deltaCounts) > 100.0f) {
        countsPerN_ = deltaCounts / refN;
        calibrated_ = true;
      }
      calComplete_ = true;
    }
    phase_ = CalPhase::NONE;
  }

  int32_t raw_ = 0, tareOffset_ = 0, first_ = 0;
  float thrustN_ = 0, thrustFiltN_ = 0;
  float countsPerN_ = stand::DEFAULT_COUNTS_PER_N;
  float noiseN_ = 0, calGrams_ = 0, periodMs_ = 12.5f;
  uint32_t lastSampleUs_ = 0, lastReadyMs_ = 0;
  int64_t sum_ = 0, sumSq_ = 0;
  uint16_t count_ = 0, target_ = 0;
  CalPhase phase_ = CalPhase::NONE;
  bool calibrated_ = false, fault_ = false, haveSample_ = false;
  bool tareComplete_ = false, calComplete_ = false;
};
