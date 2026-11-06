#pragma once
//
// Bench command interface over USB serial, 115200 baud, newline line ending.
//
// Deliberately much smaller than RocketFC's flight CLI: there is no arm, no
// flight mode, no SD logging, no EEPROM, and above all NO pyrotest. This
// project has no code path that can energize a pyro channel.
//
// Long procedures (cal / zero) are only REQUESTED here; the sketch runs them as
// non-blocking sequences so the 500 Hz loop keeps running and the visualizer
// never freezes.
//
#include <Arduino.h>

#include "bench_config.h"
#include "bench_telem.h"
#include "core/ahrs.h"
#include "core/control.h"
#include "hw/actuators.h"
#include "hw/sensors.h"

enum class BenchProc : uint8_t { NONE, CAL, ZERO };

// Compute the IMU_R_SB that would make the axis gravity currently lies along
// become body +Z.
//
// ASSUMES THE VEHICLE IS HELD NOSE UP: the whole method is "whichever sensor
// axis reads gravity right now is the axis that points out the nose".
//
// The result is composed with the mapping already in config.h, so it is correct
// to paste even when IMU_R_SB is not the identity. Free function rather than a
// class member so it can be exercised off-target without an Arduino.
inline void suggestedImuMapping(const Vec3& aBody, float outR[9]) {
  // Corrected body +Z, expressed in the CURRENT body frame.
  Vec3 ez{0, 0, 1};
  const float ax = fabsf(aBody.x), ay = fabsf(aBody.y), az = fabsf(aBody.z);
  if (az >= ax && az >= ay)  ez = Vec3{0, 0, aBody.z >= 0 ? 1.0f : -1.0f};
  else if (ay >= ax)         ez = Vec3{0, aBody.y >= 0 ? 1.0f : -1.0f, 0};
  else                       ez = Vec3{aBody.x >= 0 ? 1.0f : -1.0f, 0, 0};

  // Complete to a right-handed orthonormal frame. Any seed axis not parallel to
  // ez will do; Gram-Schmidt makes ex perpendicular to it, and ey = ez x ex
  // then guarantees ex x ey == ez, i.e. a proper rotation rather than a
  // reflection. A reflection here would silently invert one gimbal axis.
  const Vec3 seed = (fabsf(ez.x) < 0.9f) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  const Vec3 ex = (seed - ez * seed.dot(ez)).normalized();
  const Vec3 ey = ez.cross(ex);

  // P takes a vector's components in the current body frame to its components
  // in the corrected one, so its ROWS are the new basis vectors.
  const float P[9] = {ex.x, ex.y, ex.z,
                      ey.x, ey.y, ey.z,
                      ez.x, ez.y, ez.z};

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      float s = 0;
      for (int k = 0; k < 3; ++k) s += P[r * 3 + k] * cfg::IMU_R_SB[k * 3 + c];
      outR[r * 3 + c] = (fabsf(s) < 1e-4f) ? 0.0f : s;  // snap -0 / rounding dust
    }
  }
}

struct BenchContext {
  Sensors& sensors;
  Actuators& act;
  Ahrs& ahrs;
  TvcControl& ctl;
  BenchTelemetry& telem;
  bench::TvcMode tvcMode = bench::TvcMode::OFF;
  BenchProc procRequest = BenchProc::NONE;
  bool procBusy = false;
};

class BenchCli {
 public:
  explicit BenchCli(BenchContext& ctx) : ctx_(ctx) {}

  bool sweepActive() const { return sweepActive_; }

  void poll() {
    while (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line_[idx_] = 0;
        idx_ = 0;
        handle();
      } else if (idx_ < (int)sizeof(line_) - 1) {
        line_[idx_++] = c;
      }
    }
  }

 private:
  void handle() {
    char* save = nullptr;
    char* cmd = strtok_r(line_, " ", &save);
    if (!cmd) return;
    char* a1 = strtok_r(nullptr, " ", &save);
    char* a2 = strtok_r(nullptr, " ", &save);

    if (!strcmp(cmd, "help")) {
      printHelp();
    } else if (!strcmp(cmd, "status")) {
      printStatus();
    } else if (!strcmp(cmd, "viz")) {
      if (a1 && !strcmp(a1, "off")) {
        ctx_.telem.setEnabled(false);
        Serial.println("viz off");
      } else {
        ctx_.telem.setEnabled(true);
        Serial.println("viz on");
      }
    } else if (!strcmp(cmd, "cal") || !strcmp(cmd, "zero")) {
      if (ctx_.procBusy) { Serial.println("busy with another procedure."); return; }
      ctx_.procRequest = !strcmp(cmd, "cal") ? BenchProc::CAL : BenchProc::ZERO;
    } else if (!strcmp(cmd, "tvc")) {
      setTvcMode(a1);
    } else if (!strcmp(cmd, "trim")) {
      if (!a1 || !a2) { Serial.println("usage: trim a|b <delta_us>"); return; }
      float ta = ctx_.act.trimA(), tb = ctx_.act.trimB();
      const float d = atof(a2);
      if (!strcmp(a1, "a")) ta += d;
      else if (!strcmp(a1, "b")) tb += d;
      else { Serial.println("usage: trim a|b <delta_us>"); return; }
      ctx_.act.setTrims(ta, tb);
      ctx_.act.center();
      // Not persisted: this project has no EEPROM store. Copy any trim you
      // settle on into RocketFC via its own `trim` command.
      Serial.printf("trims: A=%+.0f us  B=%+.0f us (NOT saved - bench only)\n", ta, tb);
    } else if (!strcmp(cmd, "center")) {
      ctx_.tvcMode = bench::TvcMode::OFF;
      ctx_.ctl.reset();
      ctx_.act.center();
      Serial.println("servos centered, tvc off.");
    } else if (!strcmp(cmd, "sweep")) {
      sweep();
    } else if (!strcmp(cmd, "axis")) {
      printAxis();
    } else {
      Serial.printf("unknown command '%s' - type `help`\n", cmd);
    }
  }

  void setTvcMode(const char* arg) {
    if (!arg) { Serial.println("usage: tvc off|calc|drive"); return; }

    bench::TvcMode m;
    if (!strcmp(arg, "off")) m = bench::TvcMode::OFF;
    else if (!strcmp(arg, "calc")) m = bench::TvcMode::CALC;
    else if (!strcmp(arg, "drive")) m = bench::TvcMode::DRIVE;
    else { Serial.println("usage: tvc off|calc|drive"); return; }

    // Always reset the controller across a mode change: a stale integrator or
    // slew state from the previous mode would otherwise show up as a step the
    // moment the servos are enabled.
    ctx_.ctl.reset();
    ctx_.act.center();
    ctx_.tvcMode = m;

    switch (m) {
      case bench::TvcMode::OFF:
        Serial.println("tvc off - controller stopped, servos centered.");
        break;
      case bench::TvcMode::CALC:
        Serial.println("tvc calc - controller running, values streamed.");
        Serial.println("           SERVOS WILL NOT MOVE.");
        break;
      case bench::TvcMode::DRIVE:
        Serial.println("tvc drive - controller running, SERVOS WILL MOVE.");
        Serial.println("           Gimbal must be free. Servos on their own BEC,");
        Serial.println("           never the Teensy 3.3 V rail. No motor installed.");
        Serial.println("           `tvc off` or `center` stops it.");
        break;
    }
  }

  void sweep() {
    // Open-loop gimbal sweep: axis A alone, then both together. Blocking on
    // purpose -- it is a look-at-the-mount test, and telemetry pausing for a
    // few seconds is fine. Ported from RocketFC's Cli::servoTest.
    Serial.println("open-loop sweep: axis A, then axis B. Watch the gimbal.");
    sweepActive_ = true;
    const float maxDeg = cfg::GIMBAL_MAX_RAD * cfg::RAD2DEG;
    for (float s = 0; s <= 4.0f * cfg::PI_F; s += 0.02f) {
      const float x = maxDeg * sinf(s) * cfg::DEG2RAD;
      const bool second = s > 2.0f * cfg::PI_F;
      ctx_.act.writeGimbal(second ? 0 : x, second ? x : 0);
      delay(8);
    }
    ctx_.act.center();
    sweepActive_ = false;
    Serial.println("sweep done, centered.");
  }

  // Which body axis is gravity along right now, and does that agree with a
  // rocket standing on its tail? This is the fastest way to catch a wrong
  // IMU_R_SB or a flipped sign without doing any 3D reasoning.
  void printAxis() {
    const Vec3 a = ctx_.sensors.accel();
    const float ax = fabsf(a.x), ay = fabsf(a.y), az = fabsf(a.z);
    const char* name;
    float mag;
    if (az >= ax && az >= ay)      { name = a.z >= 0 ? "+Z" : "-Z"; mag = az; }
    else if (ay >= ax)             { name = a.y >= 0 ? "+Y" : "-Y"; mag = ay; }
    else                           { name = a.x >= 0 ? "+X" : "-X"; mag = ax; }

    Serial.printf("accel [%.2f %.2f %.2f] m/s^2  |a| = %.2f\n",
                  a.x, a.y, a.z, a.norm());
    Serial.printf("gravity lies along body %s (%.2f m/s^2 of it)\n", name, mag);
    if (!strcmp(name, "+Z")) {
      Serial.println("-> consistent with the vehicle standing NOSE UP.");
      Serial.println("   IMU_R_SB looks right for this orientation.");
    } else {
      Serial.println("-> if the vehicle is standing NOSE UP right now, this is WRONG:");
      Serial.println("   body +Z must point out the nose.");
    }
    Serial.printf("body long axis (+Z) currently %.1f deg from world up.\n",
                  ctx_.ahrs.tiltRad() * cfg::RAD2DEG);
    if (strcmp(name, "+Z") != 0) printSuggestedMapping(a);
  }

  void printSuggestedMapping(const Vec3& aBody) {
    float R[9];
    suggestedImuMapping(aBody, R);

    Serial.println("\nPaste this into RocketFC's src/config.h, then run");
    Serial.println("`python sync_core.py` here and re-upload:");
    Serial.println("constexpr float IMU_R_SB[9] = {");
    for (int r = 0; r < 3; ++r)
      Serial.printf("  %3.0f, %3.0f, %3.0f,\n", R[r * 3], R[r * 3 + 1],
                    R[r * 3 + 2]);
    Serial.println("};");
    Serial.println("This fixes the LONG axis only. Which lateral axis belongs to");
    Serial.println("servo A vs servo B is not observable from gravity -- confirm it");
    Serial.println("with the gimbal direction test (README 5.7). If A and B come out");
    Serial.println("swapped, exchange the first two rows above.");
  }

  void printHelp() {
    Serial.println(
        "commands:\n"
        "  status           sensors, attitude, tvc mode, servo state\n"
        "  viz on|off       50 Hz attitude stream for the PC visualizer\n"
        "  cal              gyro bias + level init (hold still ~4 s)\n"
        "  zero             barometer ground reference\n"
        "  tvc off          controller stopped, servos centered\n"
        "  tvc calc         controller runs, servos DO NOT move\n"
        "  tvc drive        controller runs and DRIVES THE SERVOS\n"
        "  trim a|b <us>    nudge a servo center (not saved)\n"
        "  center           center servos and stop the controller\n"
        "  sweep            open-loop gimbal sweep\n"
        "  axis             which body axis gravity is along (IMU_R_SB check)\n"
        "  help\n"
        "this project cannot fire pyro channels - there is no such command.");
  }

  void printStatus() {
    const Vec3 a = ctx_.sensors.accel(), g = ctx_.sensors.gyro();
    const Quat q = ctx_.ahrs.quat();
    Serial.println("---- SensorServoBenchTest status ----");
    Serial.printf("imu: %s (acc 0x%02X gyro 0x%02X)   baro: %s\n",
                  ctx_.sensors.imuOk() ? "OK" : "FAIL", ctx_.sensors.accelAddr(),
                  ctx_.sensors.gyroAddr(), ctx_.sensors.baroOk() ? "OK" : "FAIL");
    Serial.printf("accel [%.2f %.2f %.2f] m/s^2   gyro [%.2f %.2f %.2f] dps\n",
                  a.x, a.y, a.z, g.x * cfg::RAD2DEG, g.y * cfg::RAD2DEG,
                  g.z * cfg::RAD2DEG);
    Serial.printf("quat  [%.4f %.4f %.4f %.4f] (w,x,y,z)   tilt %.2f deg\n",
                  q.w, q.x, q.y, q.z, ctx_.ahrs.tiltRad() * cfg::RAD2DEG);
    Serial.printf("baro: %.1f Pa  %.2f C  alt %.2f m\n",
                  ctx_.sensors.baroPressure(), ctx_.sensors.baroTemperature(),
                  ctx_.sensors.baroAltitude());
    Serial.printf("tvc mode: %s   servos: A=%.0f B=%.0f us   trims A=%+.0f B=%+.0f\n",
                  modeName(ctx_.tvcMode), ctx_.act.lastUsA(), ctx_.act.lastUsB(),
                  ctx_.act.trimA(), ctx_.act.trimB());
    Serial.printf("viz stream: %s\n", ctx_.telem.enabled() ? "on" : "off");
  }

  static const char* modeName(bench::TvcMode m) {
    switch (m) {
      case bench::TvcMode::CALC: return "calc (servos held)";
      case bench::TvcMode::DRIVE: return "DRIVE (servos live)";
      default: return "off";
    }
  }

  BenchContext& ctx_;
  char line_[96];
  int idx_ = 0;
  bool sweepActive_ = false;
};
