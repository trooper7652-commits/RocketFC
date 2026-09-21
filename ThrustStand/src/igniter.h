#pragma once
//
// Igniter gate. This is the only code in the project that can start a fire, so
// it is written to be boring and to fail safe.
//
// THE PULLDOWN IS NOT OPTIONAL. A 10k resistor from gate to ground is what
// holds the MOSFET off between power-on and the first line of setup(), while
// the AVR pin is still a floating input. No amount of firmware can cover that
// window. See WIRING.md.
//
// Three defences layered on top of it:
//
//  1. safeInit() drives the pin LOW *before* making it an OUTPUT. On AVR the
//     opposite order emits a brief glitch as the port latch is applied.
//  2. tick() re-asserts LOW on every pass whenever we are not deliberately
//     firing, so a corrupted port register or a stray write cannot leave the
//     gate hot unnoticed.
//  3. FIRE_PULSE_MS is a hard cap enforced against millis(). Nothing -- not a
//     lost IR link, not a hung UI, not a stuck state machine -- can hold the
//     gate on longer. Fire current is measured in amps; a latched gate is a
//     burning bench.
//
#include <Arduino.h>

#include "../config.h"

class Igniter {
 public:
  // Call this as the FIRST statement in setup(), before the LCD, before IR,
  // before Serial. Nothing else matters until the gate is known-low.
  void safeInit() {
    digitalWrite(stand::PIN_GATE, LOW);
    pinMode(stand::PIN_GATE, OUTPUT);
    digitalWrite(stand::PIN_GATE, LOW);
    firing_ = false;
  }

  // Energise the gate. Returns the millis() at which it went high, which the
  // caller logs as EVT:FIRE -- that timestamp is what makes the ignition-delay
  // measurement possible, so it is taken as close to the write as we can get.
  uint32_t fire(uint32_t ms) {
    startMs_ = ms;
    firing_ = true;
    digitalWrite(stand::PIN_GATE, HIGH);
    fireCount_++;
    return startMs_;
  }

  // Unconditional. Safe to call from anywhere, at any time, repeatedly.
  void safe() {
    digitalWrite(stand::PIN_GATE, LOW);
    firing_ = false;
  }

  void tick(uint32_t ms) {
    if (!firing_) {
      digitalWrite(stand::PIN_GATE, LOW);  // defence 2
      return;
    }
    if (ms - startMs_ >= stand::FIRE_PULSE_MS) safe();  // defence 3
  }

  bool firing() const { return firing_; }
  uint32_t startMs() const { return startMs_; }
  uint16_t fireCount() const { return fireCount_; }

 private:
  uint32_t startMs_ = 0;
  uint16_t fireCount_ = 0;
  bool firing_ = false;
};
