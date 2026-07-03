#pragma once
//
// USB-serial command interface. Type `help` in the Arduino IDE Serial
// Monitor (115200 baud, newline line ending).
//
// Long procedures (cal / zero / arm) are only REQUESTED here — the main
// sketch runs them as non-blocking sequences. Dangerous commands are gated:
// pyrotest needs a two-step typed confirmation and only works disarmed.
//
#include <Arduino.h>

#include "../config.h"
#include "../core/flight_core.h"
#include "actuators.h"
#include "config_store.h"
#include "logger.h"
#include "sensors.h"

enum class ProcRequest : uint8_t { NONE, CAL, ZERO, ARM };

struct CliContext {
  Sensors& sensors;
  Actuators& act;
  Logger& logger;
  ConfigStore& store;
  FlightCore& core;
  ProcRequest procRequest = ProcRequest::NONE;
  bool procBusy = false;
  bool streamOn = false;
  uint8_t faultCode = 0;
  float vbatCached = 0;
  bool contChuteCached = false, contLandCached = false;
  void (*wdogFeed)() = nullptr;
};

class Cli {
 public:
  explicit Cli(CliContext& ctx) : ctx_(ctx) {}

  bool servoTestActive() const { return servoTestActive_; }

  void poll(uint32_t ms) {
    if (pendingTest_ != 0 && (int32_t)(ms - pendingExpireMs_) > 0) {
      pendingTest_ = 0;
      Serial.println("pyrotest confirmation window expired.");
    }
    while (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line_[idx_] = 0;
        idx_ = 0;
        handle(ms);
      } else if (idx_ < (int)sizeof(line_) - 1) {
        line_[idx_++] = c;
      }
    }
  }

  void streamTick(uint32_t ms, const CoreOutput& co) {
    if (!ctx_.streamOn) return;
    Serial.printf(
        "t=%lu %s a=[%.2f %.2f %.2f] g=[%.1f %.1f %.1f]dps baro=%.2fm "
        "kf=[%.2fm %.2fm/s] tilt=%.1f srv=[%.0f %.0f]\n",
        (unsigned long)ms, FlightStateMachine::stateName(co.state),
        ctx_.sensors.accel().x, ctx_.sensors.accel().y, ctx_.sensors.accel().z,
        ctx_.sensors.gyro().x * cfg::RAD2DEG, ctx_.sensors.gyro().y * cfg::RAD2DEG,
        ctx_.sensors.gyro().z * cfg::RAD2DEG, ctx_.sensors.baroAltitude(),
        co.kfAlt, co.kfVel, co.tiltDeg, ctx_.act.lastUsA(), ctx_.act.lastUsB());
  }

 private:
  bool isIdle() const { return ctx_.core.state() == FlightState::IDLE; }

  void requireIdle(const char* what) {
    Serial.printf("'%s' is only allowed while DISARMED (IDLE).\n", what);
  }

  void handle(uint32_t ms) {
    char* save = nullptr;
    char* cmd = strtok_r(line_, " ", &save);
    if (!cmd) return;
    char* a1 = strtok_r(nullptr, " ", &save);
    char* a2 = strtok_r(nullptr, " ", &save);

    if (!strcmp(cmd, "help")) {
      printHelp();
    } else if (!strcmp(cmd, "status")) {
      printStatus();
    } else if (!strcmp(cmd, "stream")) {
      ctx_.streamOn = a1 && !strcmp(a1, "on");
      Serial.printf("stream %s\n", ctx_.streamOn ? "on" : "off");
    } else if (!strcmp(cmd, "cal") || !strcmp(cmd, "zero") ||
               !strcmp(cmd, "arm")) {
      if (!isIdle()) { requireIdle(cmd); return; }
      if (ctx_.procBusy) { Serial.println("busy with another procedure."); return; }
      ctx_.procRequest = !strcmp(cmd, "cal") ? ProcRequest::CAL
                        : !strcmp(cmd, "zero") ? ProcRequest::ZERO
                        : ProcRequest::ARM;
    } else if (!strcmp(cmd, "disarm")) {
      if (ctx_.core.requestDisarm(ms)) {
        ctx_.logger.close();
        Serial.println("DISARMED. Log closed.");
      } else {
        Serial.println("not armed.");
      }
    } else if (!strcmp(cmd, "mode")) {
      if (!isIdle()) { requireIdle("mode"); return; }
      if (a1 && !strcmp(a1, "chute")) setMode(cfg::FlightMode::CHUTE_TEST);
      else if (a1 && !strcmp(a1, "land")) setMode(cfg::FlightMode::FULL_LANDING);
      else Serial.println("usage: mode chute|land");
    } else if (!strcmp(cmd, "trim")) {
      if (!isIdle()) { requireIdle("trim"); return; }
      if (!a1 || !a2) { Serial.println("usage: trim a|b <delta_us>"); return; }
      float ta = ctx_.act.trimA(), tb = ctx_.act.trimB();
      const float d = atof(a2);
      if (!strcmp(a1, "a")) ta += d;
      else if (!strcmp(a1, "b")) tb += d;
      else { Serial.println("usage: trim a|b <delta_us>"); return; }
      ctx_.act.setTrims(ta, tb);
      ctx_.store.setTrims(ta, tb);
      ctx_.act.center();
      Serial.printf("trims: A=%+.0f us  B=%+.0f us (saved)\n", ta, tb);
    } else if (!strcmp(cmd, "center")) {
      if (!isIdle()) { requireIdle("center"); return; }
      ctx_.act.center();
      Serial.println("servos centered.");
    } else if (!strcmp(cmd, "servotest")) {
      if (!isIdle()) { requireIdle("servotest"); return; }
      servoTest();
    } else if (!strcmp(cmd, "pyrotest")) {
      if (!isIdle()) { requireIdle("pyrotest"); return; }
      const int ch = a1 ? atoi(a1) : 0;
      if (ch != 1 && ch != 2) { Serial.println("usage: pyrotest 1|2  (1=chute 2=landing)"); return; }
      pendingTest_ = ch;
      pendingExpireMs_ = ms + 10000;
      Serial.printf(
          "!! WARNING: this WILL energize pyro channel %d (%s) for %.0f ms.\n"
          "!! Remove all e-matches / motors first. Type `confirm %d` within 10 s.\n",
          ch, ch == 1 ? "CHUTE" : "LANDING", cfg::PYRO_FIRE_MS, ch);
    } else if (!strcmp(cmd, "confirm")) {
      const int ch = a1 ? atoi(a1) : 0;
      if (pendingTest_ == 0 || ch != pendingTest_) { Serial.println("nothing pending."); return; }
      pendingTest_ = 0;
      ctx_.act.testFire(ch - 1, ms);
      Serial.printf("pyro channel %d FIRED (test).\n", ch);
    } else {
      Serial.printf("unknown command '%s' — type `help`\n", cmd);
    }
  }

  void setMode(cfg::FlightMode m) {
    ctx_.store.setMode(m);
    ctx_.core.setMode(m);
    Serial.printf("mode = %s (saved). Arm beeps: %d.\n",
                  m == cfg::FlightMode::CHUTE_TEST ? "CHUTE_TEST" : "FULL_LANDING",
                  m == cfg::FlightMode::CHUTE_TEST ? 2 : 3);
  }

  void servoTest() {
    Serial.println("servo sweep: watch the gimbal. Tilt-nose test: tilt the NOSE");
    Serial.println("toward body +X — the motor end must deflect toward -X (thrust");
    Serial.println("pushes back under the vehicle). If reversed, flip SERVO_x_SIGN.");
    servoTestActive_ = true;
    const float maxDeg = cfg::GIMBAL_MAX_RAD * cfg::RAD2DEG;
    for (float sweep = 0; sweep <= 4.0f * cfg::PI_F; sweep += 0.02f) {
      const float x = maxDeg * sinf(sweep) * cfg::DEG2RAD;
      const float y = (sweep > 2.0f * cfg::PI_F) ? x : 0;
      ctx_.act.writeGimbal(sweep > 2.0f * cfg::PI_F ? 0 : x, y);
      if (ctx_.wdogFeed) ctx_.wdogFeed();
      delay(8);
    }
    ctx_.act.center();
    servoTestActive_ = false;
    Serial.println("sweep done (axis A then axis B), centered.");
  }

  void printHelp() {
    Serial.println(
        "commands:\n"
        "  status            snapshot of sensors/state/health\n"
        "  stream on|off     10 Hz live telemetry\n"
        "  cal               gyro bias + level init (keep vehicle still)\n"
        "  zero              baro ground reference + KF reset\n"
        "  mode chute|land   select flight mode (saved to EEPROM)\n"
        "  arm               full pre-flight sequence -> ARMED\n"
        "  disarm            back to IDLE, closes log\n"
        "  servotest         gimbal sweep (disarmed only)\n"
        "  trim a|b <us>     adjust servo center, saved\n"
        "  center            center servos\n"
        "  pyrotest 1|2      test-fire a pyro channel (two-step confirm)\n"
        "  help");
  }

  void printStatus() {
    const CoreOutput dummy{};
    Serial.println("---- RocketFC status ----");
    Serial.printf("state: %s   mode: %s (%d arm beeps)\n",
                  FlightStateMachine::stateName(ctx_.core.state()),
                  ctx_.core.mode() == cfg::FlightMode::CHUTE_TEST ? "CHUTE_TEST"
                                                                  : "FULL_LANDING",
                  ctx_.core.mode() == cfg::FlightMode::CHUTE_TEST ? 2 : 3);
    Serial.printf("imu: %s (acc 0x%02X gyro 0x%02X)  baro: %s (0x%02X)\n",
                  ctx_.sensors.imuOk() ? "OK" : "FAIL", ctx_.sensors.accelAddr(),
                  ctx_.sensors.gyroAddr(), ctx_.sensors.baroOk() ? "OK" : "FAIL",
                  0x77);
    Serial.printf("accel [%.2f %.2f %.2f] m/s^2   gyro [%.2f %.2f %.2f] dps\n",
                  ctx_.sensors.accel().x, ctx_.sensors.accel().y,
                  ctx_.sensors.accel().z, ctx_.sensors.gyro().x * cfg::RAD2DEG,
                  ctx_.sensors.gyro().y * cfg::RAD2DEG,
                  ctx_.sensors.gyro().z * cfg::RAD2DEG);
    Serial.printf("baro: %.1f Pa  %.2f C  alt %.2f m (noise var %.4f m^2)\n",
                  ctx_.sensors.baroPressure(), ctx_.sensors.baroTemperature(),
                  ctx_.sensors.baroAltitude(), ctx_.sensors.baroAltNoiseVar());
    Serial.printf("tilt: %.2f deg\n", ctx_.core.ahrs().tiltRad() * cfg::RAD2DEG);
    Serial.printf("sd: %s  flight #%d %s\n", ctx_.logger.sdOk() ? "OK" : "FAIL",
                  ctx_.logger.flightNumber(),
                  ctx_.logger.isOpen() ? "(log open)" : "");
    Serial.printf("vbat: %.2f V   continuity: chute=%s landing=%s\n",
                  ctx_.vbatCached, ctx_.contChuteCached ? "YES" : "no",
                  ctx_.contLandCached ? "YES" : "no");
    Serial.printf("servo trims: A=%+.0f B=%+.0f us   arm switch: %s\n",
                  ctx_.act.trimA(), ctx_.act.trimB(),
                  ctx_.act.armSwitchOn() ? "ON" : "off");
    Serial.printf("fault code: %d %s\n", ctx_.faultCode,
                  ctx_.faultCode == 0 ? "(healthy)" : "(see README beep table)");
    (void)dummy;
  }

  CliContext& ctx_;
  char line_[96];
  int idx_ = 0;
  int pendingTest_ = 0;
  uint32_t pendingExpireMs_ = 0;
  bool servoTestActive_ = false;
};
