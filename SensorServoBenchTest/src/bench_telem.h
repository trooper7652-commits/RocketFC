#pragma once
//
// Attitude telemetry stream for the PC visualizer.
//
// One line per sample over USB serial, tagged with a "V," prefix so it can
// share the port with the interactive CLI: the host ignores every line that
// does not start with "V," or "#", and the human reading a serial monitor can
// still type commands and see replies.
//
// Units here are chosen for display, not for the flight code's internals:
// gyro in deg/s (the internals are rad/s), angles in degrees, altitude in
// metres, servo pulse widths in microseconds. Converting once here keeps the
// host dumb.
//
// Each line is built in a stack buffer and handed to Serial in a single write,
// so a sample never arrives torn across two reads.
//
#include <Arduino.h>

#include "bench_config.h"
#include "core/quat.h"

// Bumping the schema means bumping the name: the host keys off "#VIZ1".
#define BENCH_VIZ_SCHEMA "VIZ1"

struct VizFrame {
  uint32_t ms = 0;
  Quat q;              // body -> world, Hamilton, w-first
  Vec3 accel;          // body specific force, m/s^2
  Vec3 gyro;           // body rates, rad/s (converted to dps on the wire)
  float baroAlt = 0;   // m, relative to the last `zero`
  float tiltDeg = 0;
  float gimXDeg = 0, gimYDeg = 0;
  float usA = 1500, usB = 1500;
  uint8_t tvcMode = 0;  // bench::TvcMode
  bool imuOk = false, baroOk = false;
};

class BenchTelemetry {
 public:
  void begin(bool on) {
    on_ = on;
    if (on_) printHeader();
  }

  bool enabled() const { return on_; }

  void setEnabled(bool on) {
    if (on && !on_) printHeader();  // re-announce the schema for a fresh reader
    on_ = on;
  }

  // Emit the self-describing column list. Sent once when streaming starts so a
  // host that attaches later can still learn the layout; the host also has a
  // hardcoded fallback for the case where it missed this line.
  void printHeader() const {
    Serial.println(
        "#" BENCH_VIZ_SCHEMA " t_ms,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,baro_alt,"
        "tilt_deg,gimx_deg,gimy_deg,servoA_us,servoB_us,tvc_mode,imu_ok,baro_ok");
  }

  void send(const VizFrame& f) const {
    if (!on_) return;
    char buf[192];
    const int n = snprintf(
        buf, sizeof(buf),
        "V,%lu,%.4f,%.4f,%.4f,%.4f,"
        "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,"
        "%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%u,%u,%u\n",
        (unsigned long)f.ms, f.q.w, f.q.x, f.q.y, f.q.z,
        f.accel.x, f.accel.y, f.accel.z,
        f.gyro.x * cfg::RAD2DEG, f.gyro.y * cfg::RAD2DEG, f.gyro.z * cfg::RAD2DEG,
        f.baroAlt, f.tiltDeg, f.gimXDeg, f.gimYDeg, f.usA, f.usB,
        (unsigned)f.tvcMode, (unsigned)(f.imuOk ? 1 : 0),
        (unsigned)(f.baroOk ? 1 : 0));
    if (n <= 0) return;
    // snprintf returns the length it WANTED; clamp so a truncated line is still
    // written as a whole (the host drops malformed rows rather than hanging).
    const size_t len = (n < (int)sizeof(buf)) ? (size_t)n : sizeof(buf) - 1;
    Serial.write((const uint8_t*)buf, len);
  }

 private:
  bool on_ = false;
};
