#pragma once
//
// USB-serial command interface. Type `help` in the Arduino IDE Serial
// Monitor (115200 baud, newline line ending).
//
// Long procedures (cal / zero / arm) are only REQUESTED here — the main
// sketch runs them as non-blocking sequences. Dangerous commands are gated:
// everything that moves hardware only works disarmed, pyrotest needs a
// two-step typed confirmation, and `stop` kills every pyro output at once.
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
  float vpyroCached = 0;
  bool contLandCached = false, contLegsCached = false;
  void (*wdogFeed)() = nullptr;
};

class Cli {
 public:
  explicit Cli(CliContext& ctx) : ctx_(ctx) {}

  bool servoTestActive() const { return servoTestActive_; }

  void poll(uint32_t ms) {
    if (pendingTest_ != PendingTest::NONE &&
        (int32_t)(ms - pendingExpireMs_) > 0) {
      pendingTest_ = PendingTest::NONE;
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
    } else if (!strcmp(cmd, "chute")) {
      if (!isIdle()) { requireIdle("chute"); return; }
      chuteCmd(a1, a2);
    } else if (!strcmp(cmd, "pyrotest")) {
      if (!isIdle()) { requireIdle("pyrotest"); return; }
      if (a1 && (!strcmp(a1, "land") || !strcmp(a1, "2"))) {
        pendingTest_ = PendingTest::LAND;
        Serial.printf(
            "!! WARNING: this WILL energize the LANDING pyro channel for %.0f ms.\n"
            "!! Remove the e-match / motor first. Type `confirm` within 10 s.\n",
            cfg::PYRO_FIRE_MS);
      } else if (a1 && !strcmp(a1, "legs")) {
        pendingTest_ = PendingTest::LEGS;
        Serial.printf(
            "!! WARNING: this WILL power the LEGS nichrome for %.0f ms (its\n"
            "!! longest in-flight burn). It glows hot and the legs drop -- keep\n"
            "!! clear. Type `confirm` within 10 s; `stop` cuts it early.\n",
            legsTestMs());
      } else {
        // `pyrotest 1` used to be the chute e-match: never guess a channel.
        Serial.println("usage: pyrotest land|legs  (the chute is a servo: "
                       "see `chute`)");
        return;
      }
      pendingExpireMs_ = ms + 10000;
    } else if (!strcmp(cmd, "confirm")) {
      const PendingTest t = pendingTest_;
      pendingTest_ = PendingTest::NONE;
      if (t == PendingTest::LAND) {
        ctx_.act.testFire(ms);
        Serial.println("landing pyro channel FIRED (test).");
      } else if (t == PendingTest::LEGS) {
        ctx_.act.testLegs(ms, (uint32_t)legsTestMs());
        Serial.println("legs nichrome ON (test) -- time how long until the "
                       "band parts. `stop` cuts it.");
      } else {
        Serial.println("nothing pending.");
      }
    } else if (!strcmp(cmd, "stop")) {
      if (!isIdle()) { requireIdle("stop"); return; }
      ctx_.act.allPyrosOff();
      pendingTest_ = PendingTest::NONE;
      Serial.println("all pyro outputs OFF.");
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

  static const char* chutePosName(Actuators::ChutePos p) {
    switch (p) {
      case Actuators::ChutePos::LOCK: return "LOCKED";
      case Actuators::ChutePos::RELEASE: return "RELEASED";
      case Actuators::ChutePos::JOG: return "JOG (not locked)";
    }
    return "?";
  }

  void chuteCmd(const char* sub, const char* arg) {
    Actuators& act = ctx_.act;
    if (sub && !strcmp(sub, "lock")) {
      act.chuteLock();
      Serial.printf("chute latch LOCKED (%.0f us).\n", act.chuteUs());
    } else if (sub && !strcmp(sub, "open")) {
      act.chuteRelease();
      Serial.printf(
          "chute latch OPEN (%.0f us) -- a loaded spring fires now, stand clear.\n"
          "Load spring + chute, then `chute lock` (arming is refused until "
          "then).\n",
          act.chuteUs());
    } else if (sub && !strcmp(sub, "us") && arg) {
      act.chuteJogUs(atoi(arg));
      Serial.printf(
          "chute servo -> %.0f us (jog). Record the latch-closed / latch-open\n"
          "values as CHUTE_LOCK_US / CHUTE_RELEASE_US in config.h; `chute "
          "lock` before arming.\n",
          act.chuteUs());
    } else if (sub && !strcmp(sub, "cycle")) {
      // The in-flight re-cycle motion, once: back to LOCK, dwell, RELEASE.
      Serial.printf("re-cycle: LOCK for %.0f ms, then RELEASE -- stand clear.\n",
                    cfg::CHUTE_RECYCLE_LOCK_MS);
      act.chuteLock();
      const uint32_t t0 = millis();
      while (millis() - t0 < (uint32_t)cfg::CHUTE_RECYCLE_LOCK_MS)
        if (ctx_.wdogFeed) ctx_.wdogFeed();
      act.chuteRelease();
      Serial.println("released. `chute lock` before arming.");
    } else {
      Serial.printf("chute latch: %s (%.0f us)\n", chutePosName(act.chutePos()),
                    act.chuteUs());
      Serial.println("usage: chute lock|open|cycle|us <n>");
    }
  }

  // Bench legs burn = the longest it can run in flight: from fire +
  // LEGS_DELAY_MS until the LANDING_BURN_MAX_MS touchdown backstop.
  static float legsTestMs() {
    return cfg::LANDING_BURN_MAX_MS - cfg::LEGS_DELAY_MS;
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
        "  chute             show chute latch position\n"
        "  chute open|lock   open the latch to load the spring / lock it\n"
        "  chute us <n>      jog the latch servo to find LOCK/RELEASE pulses\n"
        "  chute cycle       one in-flight re-cycle: LOCK dwell -> RELEASE\n"
        "  pyrotest land     test-fire the landing e-match (two-step confirm)\n"
        "  pyrotest legs     test-burn the legs nichrome (two-step confirm)\n"
        "  stop              all pyro outputs off now\n"
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
    Serial.printf("vbat: %.2f V   pyro: %.2f V   continuity: landing=%s legs=%s\n",
                  ctx_.vbatCached, ctx_.vpyroCached,
                  ctx_.contLandCached ? "YES" : "no",
                  ctx_.contLegsCached ? "YES" : "no");
    Serial.printf("legs nichrome: %s\n", ctx_.act.legsActive() ? "ON" : "off");
    Serial.printf("chute latch: %s (%.0f us)\n",
                  chutePosName(ctx_.act.chutePos()), ctx_.act.chuteUs());
    Serial.printf("servo trims: A=%+.0f B=%+.0f us\n", ctx_.act.trimA(),
                  ctx_.act.trimB());
    Serial.printf("fault code: %d %s\n", ctx_.faultCode,
                  ctx_.faultCode == 0 ? "(healthy)" : "(see README beep table)");
    (void)dummy;
  }

  CliContext& ctx_;
  char line_[96];
  int idx_ = 0;
  enum class PendingTest : uint8_t { NONE, LAND, LEGS };
  PendingTest pendingTest_ = PendingTest::NONE;
  uint32_t pendingExpireMs_ = 0;
  bool servoTestActive_ = false;
};
