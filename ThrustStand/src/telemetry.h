#pragma once
//
// CSV telemetry over USB serial.
//
// The format deliberately matches the flight logger (../../src/hw/logger.h):
// data rows carry every column, event rows carry a timestamp, empty data
// columns, and EVT:<name>:<value> in the last column. That means the host-side
// conventions in ../../tools/plot_flight.py carry over unchanged, and it is
// what lets tools/plot_thrust.py mark ignition on the graph.
//
//   t_ms,state,raw,thrust_n,thrust_filt_n,evt
//   1240,ARMED,8412,0.02,0.01,
//   1250,,,,,EVT:FIRE:0.00
//   1263,FIRING,9903,4.81,3.90,
//
// Streaming starts when the stand ARMS, not when it fires. That is what gives
// the plot its pre-ignition baseline and the full ignition transient for free,
// with no pre-trigger ring buffer eating the Uno's 2 KB of RAM.
//
// Floats are written with Serial.print(v, digits) rather than snprintf: the
// AVR core's reduced printf has no %f and would emit empty fields.
//
#include <Arduino.h>

#include "../config.h"

class Telemetry {
 public:
  void begin() {
    Serial.begin(stand::SERIAL_BAUD);
    // No while(!Serial): on a Uno/Nano that is a no-op, and on a board where
    // it is not, a stand that refuses to boot without a laptop is a trap.
  }

  // Machine-readable preamble. Lines start with '#' so the parsers skip them.
  void header(bool calibrated, float countsPerN, int32_t tare, float rateHz,
              uint16_t runNumber) {
    Serial.println();
    Serial.println(F("#ThrustStand"));
    Serial.print(F("#run="));
    Serial.println(runNumber);
    Serial.print(F("#calibrated="));
    Serial.println(calibrated ? 1 : 0);
    Serial.print(F("#counts_per_n="));
    Serial.println(countsPerN, 2);
    Serial.print(F("#tare="));
    Serial.println(tare);
    Serial.print(F("#sample_hz="));
    Serial.println(rateHz, 1);
    Serial.print(F("#cell_capacity_n="));
    Serial.println(stand::CELL_CAPACITY_N, 1);
    Serial.println(F("t_ms,state,raw,thrust_n,thrust_filt_n,evt"));
  }

  void row(uint32_t ms, const char* state, int32_t raw, float n, float filt) {
    Serial.print(ms);
    Serial.print(',');
    Serial.print(state);
    Serial.print(',');
    Serial.print(raw);
    Serial.print(',');
    Serial.print(n, 3);
    Serial.print(',');
    Serial.print(filt, 3);
    Serial.println(',');
  }

  // Five commas: t_ms + four empty data columns + evt.
  void event(uint32_t ms, const char* name, float value) {
    Serial.print(ms);
    Serial.print(F(",,,,,EVT:"));
    Serial.print(name);
    Serial.print(':');
    Serial.println(value, 2);
  }

  void note(const char* text) {
    Serial.print(F("# "));
    Serial.println(text);
  }
};
