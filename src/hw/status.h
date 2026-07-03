#pragma once
//
// Buzzer + LED status patterns, computed statelessly from (state, faults,
// mode, time) so the sequencer can't get stuck. See README for the beep code
// table. Silent during flight states (nobody can hear you at 100 m).
//
//   IDLE, healthy:   one short chirp every 3 s, slow LED blink
//   IDLE, faulted:   N low beeps every 4 s (N = fault code), fast LED blink
//   ARMED:           2 beeps (CHUTE_TEST) / 3 beeps (FULL_LANDING) every 2.5 s
//   TOUCHDOWN:       loud locate beacon: 3 long beeps every 3 s, LED strobe
//
#include <Arduino.h>

#include "../config.h"
#include "../core/flight_state.h"

enum Fault : uint8_t {
  FAULT_IMU = 1,         // 1 beep
  FAULT_BARO = 2,        // 2 beeps
  FAULT_SD = 3,          // 3 beeps
  FAULT_VBAT = 4,        // 4 beeps
  FAULT_CONT_CHUTE = 5,  // 5 beeps
  FAULT_CONT_LAND = 6,   // 6 beeps
};

class StatusIndicator {
 public:
  void begin() {
    pinMode(cfg::PIN_BUZZER, OUTPUT);
    pinMode(cfg::PIN_LED, OUTPUT);
    // Boot chirp: proof of life before anything else runs.
    tone(cfg::PIN_BUZZER, 1500, 80);
  }

  void setFaultCode(uint8_t code) { faultCode_ = code; }  // 0 = healthy

  void tick(uint32_t ms, FlightState state, cfg::FlightMode mode) {
    bool buzz = false, led = false;
    switch (state) {
      case FlightState::IDLE:
        if (faultCode_ == 0) {
          buzz = pattern(ms, 3000, 1, 70, 0);
          led = (ms % 2000) < 100;
          freq_ = 1200;
        } else {
          buzz = pattern(ms, 4000, faultCode_, 140, 260);
          led = (ms % 250) < 125;
          freq_ = 400;
        }
        break;
      case FlightState::ARMED:
        buzz = pattern(ms, 2500, mode == cfg::FlightMode::CHUTE_TEST ? 2 : 3,
                       90, 180);
        led = (ms % 400) < 200;
        freq_ = 2000;
        break;
      case FlightState::TOUCHDOWN:
        buzz = pattern(ms, 3000, 3, 400, 250);
        led = (ms % 150) < 75;
        freq_ = 2700;
        break;
      default:  // in flight: silent, LED solid
        led = true;
        break;
    }
    applyBuzzer(buzz);
    digitalWrite(cfg::PIN_LED, led ? HIGH : LOW);
  }

 private:
  // True while a beep should sound: `count` beeps of `onMs` separated by
  // `gapMs`, repeating every `periodMs`.
  static bool pattern(uint32_t ms, uint32_t periodMs, uint8_t count,
                      uint32_t onMs, uint32_t gapMs) {
    const uint32_t t = ms % periodMs;
    const uint32_t slot = onMs + gapMs;
    if (t >= (uint32_t)count * slot) return false;
    return (t % slot) < onMs;
  }

  void applyBuzzer(bool on) {
    if (on == buzzing_) return;
    buzzing_ = on;
    if (on) tone(cfg::PIN_BUZZER, freq_);
    else noTone(cfg::PIN_BUZZER);
  }

  uint8_t faultCode_ = 0;
  bool buzzing_ = false;
  uint16_t freq_ = 1200;
};
