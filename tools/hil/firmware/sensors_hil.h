#pragma once
//
// *** HIL BUILD ONLY — NOT FOR FLIGHT ***
//
// Drop-in replacement for src/hw/sensors.h, copied over it by
// tools/hil/make_hil.py into the throwaway build tree
// tools/hil/build/RocketFC_HIL/. The real src/ tree is never touched.
//
// The original Sensors class is compiled unchanged as RealSensors
// (sensors_real.h, a renamed copy). This Sensors class inherits it and only
// overrides what the flight core reads, so a static-fire test runs the flight
// firmware against a simulated flight while the rocket sits clamped to a
// stand:
//
//   PASSTHRU  no trace loaded: behaves exactly like the flight build.
//   LOADING   receiving a trace over the second USB serial port.
//   HOLD      trace loaded: the core sees a still pad (the trace's pad-level
//             accel, zero rates, steady baro) while you arm. Arming uses the
//             trace's pad level + baro noise so the Teensy starts from the
//             same state as the PC reference replay.
//   PLAY      the trace, paced by micros() since `go`: each readImu() takes
//             the frame for "now" (frames a late tick skipped still deliver
//             their baro samples), so playback can't drift behind real time
//             the way one-frame-per-tick would (each tick is >= 2000 us).
//   DONE      trace finished: holds a resting frame.
//   ABORTED   `abort`: still pad + imuOk() false, so a core in flight aborts
//             with IMU_FAIL (chute, no landing-motor fire).
//
// The real IMU is still read every tick. From `go` until 10 s after the
// trace ends it is written to HIL/IMU_NNN.BIN, so the real vibration of the
// static fire can be lined up with the FIRE_LANDING event in the flight log
// (same millis() clock). Real IMU health still feeds imuOk().
//
// Control channel: SerialUSB1 (USB Type "Dual Serial"); tools/hil/hil_ctl.py
// speaks it. The normal CLI on Serial is unchanged.
//
// LIVE (closed-loop bench sim, tools/hil/live/): `live <pad ax ay az> <baro
// var> <legs 0|1>` switches the channel to framed binary packets
//   0xA5 0x5A <type> <len> <payload...> <crc8 over type,len,payload>
// PC -> Teensy:  'F' HilLiveFrame (the PC plant's sensors, streamed live)
//                'C' text command (live ... | live off | status | abort |
//                reboot)
// Teensy -> PC:  'T' HilTelemetry every fast tick, sent right after that
//                tick's core step and actuator update (the flight computer's
//                commands: gimbal, landing fire, legs, chute, state, KF)
//                'M' text reply
// The PC steps its plant to each telemetry packet's millis() (with the
// commands it already holds), takes the new commands, and answers with the
// sensor frame for that moment, which the next tick uses -- so the real
// flight code flies the simulated rocket with one tick (2 ms) of extra lag. No frame for kLiveLinkLossMs, a frame
// flagged IMU-fail, or 0.5 s of identical data (the same frozen-sensor check
// the flight build runs on the real IMU) make imuOk() false, exactly as a
// failed IMU would. Pyro gates: see hil_hooks.h.
//
#include <Arduino.h>
#include <SD.h>
#include <stdarg.h>

#include "../config.h"
#include "../core/flight_core.h"
#include "actuators.h"
#include "hil_hooks.h"
#include "sensors_real.h"

#if !defined(USB_DUAL_SERIAL) && !defined(USB_TRIPLE_SERIAL)
#error "HIL build needs USB Type = Dual Serial (arduino-cli --board-options usb=serial2)"
#endif

#ifndef HIL_SOURCE_DESC
#define HIL_SOURCE_DESC "unknown"
#endif

extern FlightCore core;  // RocketFC.ino's instance; only state()/mode() are read
extern CoreOutput co;    // RocketFC.ino's latest core outputs (LIVE telemetry)
extern Actuators act;    // RocketFC.ino's actuators (LIVE telemetry)

// LIVE packets (little-endian; must match tools/hil/live/link.py).
struct HilLiveFrame {
  uint16_t seq;
  uint8_t flags;     // bit0: new baro sample, bit1: IMU failed (fault injection)
  uint8_t reserved;
  float ax, ay, az;  // body specific force, m/s^2
  float gx, gy, gz;  // body rates, rad/s, bias-corrected
  float baroAlt;     // m AGL
};
static_assert(sizeof(HilLiveFrame) == 32, "HilLiveFrame must match tools/hil/live/link.py");

struct HilTelemetry {
  uint32_t ms;            // millis() at the start of this fast tick
  uint16_t seq;           // last frame used
  uint8_t state;          // FlightState
  uint8_t abortReason;    // AbortReason
  float gimbalX, gimbalY; // core gimbal command, rad
  uint16_t usA, usB;      // gimbal servo pulses
  uint16_t usChute;       // chute servo pulse
  uint8_t chutePos;       // Actuators::ChutePos
  uint8_t bits;           // see LiveBits
  float kfAlt, kfVel, tiltDeg;
  float qw, qx, qy, qz;   // AHRS attitude, body -> world
  float pX, iX, dX, pY, iY, dY;  // TVC PID terms, rad
  uint16_t staleTicks;    // ticks since the last new frame
  uint16_t periodUs;      // this tick's period (the core's dt), us
};
static_assert(sizeof(HilTelemetry) == 80, "HilTelemetry must match tools/hil/live/link.py");

enum LiveBits : uint8_t {
  kBitLandFire = 1,      // landing e-match fire pulse active (pin NOT driven in LIVE)
  kBitLegsCmd = 2,       // leg nichrome commanded on
  kBitLegsPin = 4,       // ... and really powered (live legs)
  kBitChuteRelease = 8,  // core's chute release level
  kBitChuteDetected = 16,
  kBitInFlight = 32,
  kBitTvcActive = 64,
  kBitImuBad = 128,      // imuOk() false (link loss, fault, frozen data)
};

struct HilFrame {
  float ax, ay, az;  // body specific force, m/s^2
  float gx, gy, gz;  // body rates, rad/s, bias-corrected
  float baroAlt;     // m AGL
  uint32_t flags;    // bit0: new baro sample this tick
};
static_assert(sizeof(HilFrame) == 32, "HilFrame must match tools/hil/hil_trace.py");

// Real-IMU record in HIL/IMU_NNN.BIN (little-endian), after a 64-byte header.
struct HilImuRec {
  uint32_t ms;
  int16_t a[3];  // 0.01 m/s^2
  int16_t g[3];  // 0.001 rad/s
};
static_assert(sizeof(HilImuRec) == 16, "HilImuRec must match tools/hil/compare.py");

namespace hil {
constexpr uint32_t kMaxFrames = 13000;  // 26 s at 500 Hz, 416 KB of RAM2
constexpr uint32_t kFrameUs = 2000;     // trace sample period (tools/hil/hil_trace.py FAST_DT)
constexpr int kRecsPerBuf = 32;         // 512-byte SD writes
constexpr uint32_t kLogTailMs = 10000;  // keep logging the real IMU after the trace
constexpr uint32_t kLoadTimeoutMs = 3000;
constexpr uint32_t kLiveLinkLossMs = 100;  // no frame this long -> IMU failed
constexpr int kLiveFrozenTicks = 250;      // 0.5 s identical data -> IMU failed
}  // namespace hil

DMAMEM static HilFrame g_hilFrames[hil::kMaxFrames];

class Sensors : public RealSensors {
 public:
  enum class Mode : uint8_t { PASSTHRU, LOADING, HOLD, PLAY, DONE, ABORTED, LIVE };

  bool begin() {
    const bool ok = RealSensors::begin();
    SerialUSB1.begin(115200);
    banner(Serial);
    return ok;
  }

  // ---- IMU (FAST_HZ) ----
  void readImu() {
    RealSensors::readImu();  // health, staleness, cal: always the real IMU
    tick_++;
    switch (mode_) {
      case Mode::HOLD:
      case Mode::ABORTED:
        stillFrame(padAccel_);
        break;
      case Mode::PLAY: {
        const uint32_t due = (micros() - goUs_) / hil::kFrameUs;  // frame for now
        while (idx_ <= due && idx_ < n_) {
          const HilFrame& f = g_hilFrames[idx_++];
          a_ = Vec3{f.ax, f.ay, f.az};
          g_ = Vec3{f.gx, f.gy, f.gz};
          if (f.flags & 1u) {
            baroAlt_ = f.baroAlt;
            baroPending_ = true;
          }
        }
        if (idx_ >= n_) {
          mode_ = Mode::DONE;
          endMs_ = millis();
          SerialUSB1.printf("DONE %lu\n", (unsigned long)endMs_);
        }
        break;
      }
      case Mode::DONE:
        stillFrame(a_);
        break;
      case Mode::LIVE:
        liveTick(millis());
        break;
      default:
        break;
    }
    if (imuLogOpen_) recordReal();
  }

  Vec3 accel() const { return traced() ? a_ : RealSensors::accel(); }
  Vec3 gyro() const { return traced() ? g_ : RealSensors::gyro(); }
  bool imuOk() const {
    if (!RealSensors::imuOk() || mode_ == Mode::ABORTED) return false;
    if (mode_ == Mode::LIVE && (liveLinkLost_ || liveFail_ || liveAbort_ || !liveFresh_))
      return false;
    return true;
  }
  Vec3 accelLevelAvg() const {
    return traced() ? padAccel_ : RealSensors::accelLevelAvg();
  }

  // ---- barometer (every loop pass; also runs the HIL control channel) ----
  void baroTick(uint32_t ms) {
    RealSensors::baroTick(ms);
    // Called right after the fast tick's core step + actuators in the same
    // loop pass, with the same ms the core saw: report what it just decided.
    if (mode_ == Mode::LIVE && liveTxDue_) {
      liveTxDue_ = false;
      sendTelemetry(ms);
    }
    pollControl(ms);
    serviceImuLog(ms);
    if (mode_ == Mode::PLAY && ms - lastProgressMs_ >= 200) {
      lastProgressMs_ = ms;
      SerialUSB1.printf("P %lu %lu %s\n", (unsigned long)idx_, (unsigned long)n_,
                        FlightStateMachine::stateName(core.state()));
    }
  }
  bool baroNew() {
    const bool realNew = RealSensors::baroNew();  // always drain the real flag
    if (!traced()) return realNew;
    const bool n = baroPending_;
    baroPending_ = false;
    return n;
  }
  float baroAltitude() const {
    return traced() ? baroAlt_ : RealSensors::baroAltitude();
  }
  float baroAltNoiseVar() const {
    return traced() ? baroVar_ : RealSensors::baroAltNoiseVar();
  }

  Mode hilMode() const { return mode_; }

 private:
  bool traced() const {
    return mode_ == Mode::HOLD || mode_ == Mode::PLAY || mode_ == Mode::DONE ||
           mode_ == Mode::ABORTED || mode_ == Mode::LIVE;
  }

  // ---- LIVE: closed-loop frames in, telemetry out ----
  void liveTick(uint32_t ms) {
    pumpLive(ms);  // newest frame(s) the PC has sent
    // Frozen data: the flight build's real-IMU check, applied to the frames.
    if (g_.x == lastG_.x && g_.y == lastG_.y && g_.z == lastG_.z && a_.x == lastA_.x) {
      if (++liveSameTicks_ > hil::kLiveFrozenTicks) liveFresh_ = false;
    } else {
      liveSameTicks_ = 0;
      liveFresh_ = true;
    }
    lastG_ = g_;
    lastA_ = a_;
    liveLinkLost_ = ms - lastFrameMs_ > hil::kLiveLinkLossMs;
    if (liveStaleTicks_ < 0xFFFF) ++liveStaleTicks_;
    liveTxDue_ = true;  // baroTick() sends it once the core has stepped
  }

  void sendTelemetry(uint32_t ms) {
    HilTelemetry t{};
    t.ms = ms;
    t.seq = liveSeq_;
    t.state = (uint8_t)co.state;
    t.abortReason = (uint8_t)co.abortReason;
    t.gimbalX = co.gimbalX;
    t.gimbalY = co.gimbalY;
    t.usA = (uint16_t)act.lastUsA();
    t.usB = (uint16_t)act.lastUsB();
    t.usChute = (uint16_t)act.chuteUs();
    t.chutePos = (uint8_t)act.chutePos();
    uint8_t b = 0;
    if (act.pyroActive()) b |= kBitLandFire;
    if (act.legsActive()) b |= kBitLegsCmd;
    if (act.legsActive() && g_hilLiveLegs) b |= kBitLegsPin;
    if (co.chuteRelease) b |= kBitChuteRelease;
    if (co.chuteDetected) b |= kBitChuteDetected;
    if (co.inFlight) b |= kBitInFlight;
    if (co.tvcActive) b |= kBitTvcActive;
    if (!imuOk()) b |= kBitImuBad;
    t.bits = b;
    t.kfAlt = co.kfAlt;
    t.kfVel = co.kfVel;
    t.tiltDeg = co.tiltDeg;
    t.qw = co.attitude.w;
    t.qx = co.attitude.x;
    t.qy = co.attitude.y;
    t.qz = co.attitude.z;
    t.pX = core.control().pTerm(0);
    t.iX = core.control().iTerm(0);
    t.dX = core.control().dTerm(0);
    t.pY = core.control().pTerm(1);
    t.iY = core.control().iTerm(1);
    t.dY = core.control().dTerm(1);
    t.staleTicks = liveStaleTicks_;
    t.periodUs = (uint16_t)(g_hilPeriodUs > 65535u ? 65535u : g_hilPeriodUs);
    sendPacket('T', reinterpret_cast<const uint8_t*>(&t), sizeof(t));
  }

  // Never block the flight loop on USB: drop a packet the host isn't taking.
  void sendPacket(char type, const uint8_t* p, size_t n) {
    if (SerialUSB1.availableForWrite() < (int)(n + 5)) {
      ++liveTxDropped_;
      return;
    }
    uint8_t hdr[4] = {0xA5, 0x5A, (uint8_t)type, (uint8_t)n};
    uint8_t crc = crc8Update(0, hdr + 2, 2);
    crc = crc8Update(crc, p, n);
    SerialUSB1.write(hdr, 4);
    SerialUSB1.write(p, n);
    SerialUSB1.write(&crc, 1);
  }

  void pumpLive(uint32_t ms) {
    while (SerialUSB1.available()) {
      const uint8_t c = (uint8_t)SerialUSB1.read();
      switch (ps_) {
        case 0: ps_ = (c == 0xA5) ? 1 : 0; break;
        case 1: ps_ = (c == 0x5A) ? 2 : (c == 0xA5 ? 1 : 0); break;
        case 2: pType_ = c; ps_ = 3; break;
        case 3:
          pLen_ = c;
          pGot_ = 0;
          ps_ = (pLen_ <= sizeof(pBuf_)) ? (pLen_ ? 4 : 5) : 0;
          break;
        case 4:
          pBuf_[pGot_++] = c;
          if (pGot_ >= pLen_) ps_ = 5;
          break;
        case 5: {
          uint8_t hdr[2] = {pType_, pLen_};
          const uint8_t crc = crc8Update(crc8Update(0, hdr, 2), pBuf_, pLen_);
          ps_ = 0;
          if (crc != c) {
            ++liveCrcErrors_;
            break;
          }
          livePacket(ms);
          if (mode_ != Mode::LIVE) return;  // `live off`: text from here on
          break;
        }
      }
    }
  }

  void livePacket(uint32_t ms) {
    if (pType_ == 'F' && pLen_ == sizeof(HilLiveFrame)) {
      HilLiveFrame f;
      memcpy(&f, pBuf_, sizeof(f));
      a_ = Vec3{f.ax, f.ay, f.az};
      g_ = Vec3{f.gx, f.gy, f.gz};
      if (f.flags & 1u) {
        baroAlt_ = f.baroAlt;
        baroPending_ = true;
      }
      liveFail_ = (f.flags & 2u) != 0;
      liveSeq_ = f.seq;
      lastFrameMs_ = ms;
      liveStaleTicks_ = 0;
      ++liveFramesRx_;
    } else if (pType_ == 'C' && pLen_ < sizeof(line_)) {
      memcpy(line_, pBuf_, pLen_);
      line_[pLen_] = 0;
      command(line_, ms);
    }
  }

  static uint8_t crc8Update(uint8_t crc, const uint8_t* p, size_t n) {
    while (n--) {
      crc ^= *p++;
      for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
  }

  // A reply on the control channel: framed in LIVE, a text line otherwise.
  void reply(const char* fmt, ...) {
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (mode_ == Mode::LIVE)
      sendPacket('M', reinterpret_cast<const uint8_t*>(buf), strlen(buf));
    else
      SerialUSB1.println(buf);
  }

  // Still vehicle: given accel, zero rates, baro at 100 Hz (every 5th tick).
  void stillFrame(const Vec3& a) {
    a_ = a;
    g_ = Vec3{};
    if (tick_ % 5 == 0) baroPending_ = true;
  }

  static void banner(Print& p) {
    p.println("**************************************************");
    p.println("*** HIL BUILD - NOT FOR FLIGHT                 ***");
    p.println("*** sensors replaced by a trace on port 2      ***");
    p.printf("*** source: %s\n", HIL_SOURCE_DESC);
    p.println("**************************************************");
  }

  // ---- control channel ----
  void pollControl(uint32_t ms) {
    if (mode_ == Mode::LOADING) {
      receiveFrames(ms);
      return;
    }
    if (mode_ == Mode::LIVE) {
      pumpLive(ms);
      return;
    }
    while (SerialUSB1.available()) {
      const char c = (char)SerialUSB1.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line_[len_] = 0;
        if (len_ > 0) command(line_, ms);
        len_ = 0;
        if (mode_ == Mode::LOADING || mode_ == Mode::LIVE) return;  // binary follows
      } else if (len_ < sizeof(line_) - 1) {
        line_[len_++] = c;
      }
    }
  }

  void command(char* line, uint32_t ms) {
    char* save = nullptr;
    char* cmd = strtok_r(line, " ", &save);
    if (!cmd) return;
    const FlightState st = core.state();

    if (!strcmp(cmd, "hello")) {
      if (mode_ != Mode::LIVE) banner(SerialUSB1);
      reply("OK hello");
    } else if (!strcmp(cmd, "status")) {
      if (mode_ == Mode::LIVE)
        reply("OK status mode=LIVE core=%s realImu=%s legs=%s rx=%lu crcErr=%lu txDrop=%lu link=%s",
              FlightStateMachine::stateName(st), RealSensors::imuOk() ? "ok" : "FAIL",
              g_hilLiveLegs ? "LIVE" : "sim", (unsigned long)liveFramesRx_,
              (unsigned long)liveCrcErrors_, (unsigned long)liveTxDropped_,
              liveLinkLost_ ? "LOST" : "ok");
      else
        reply("OK status mode=%s frames=%lu idx=%lu core=%s realImu=%s imuLog=%s",
              modeName(), (unsigned long)n_, (unsigned long)idx_,
              FlightStateMachine::stateName(st), RealSensors::imuOk() ? "ok" : "FAIL",
              imuLogOpen_ ? imuLogName_ : "-");
    } else if (!strcmp(cmd, "live")) {
      // live <pad ax> <pad ay> <pad az> <baro var> <legs 0|1>   (enter LIVE)
      // live off                                                (leave, IDLE only)
      const char* a1 = strtok_r(nullptr, " ", &save);
      if (a1 && !strcmp(a1, "off")) {
        if (mode_ != Mode::LIVE) return fail("live off: not in LIVE");
        if (st != FlightState::IDLE) return fail("live off only when IDLE (disarm first)");
        reply("OK live off");
        g_hilLive = false;
        g_hilLiveLegs = false;
        mode_ = Mode::PASSTHRU;
        return;
      }
      if (st != FlightState::IDLE) return fail("live only when IDLE (disarmed)");
      if (mode_ != Mode::PASSTHRU && mode_ != Mode::HOLD && mode_ != Mode::LIVE)
        return fail("live: reboot first");
      const char* tok[5] = {a1, nullptr, nullptr, nullptr, nullptr};
      for (int i = 1; i < 5; ++i) tok[i] = strtok_r(nullptr, " ", &save);
      for (int i = 0; i < 5; ++i)
        if (!tok[i]) return fail("live: needs <pad ax> <pad ay> <pad az> <baro var> <legs 0|1>");
      // Sent again (framed) once the PC has calibrated on the pad: then only
      // the pad level / baro variance used by `arm` and the legs choice change.
      padAccel_ = Vec3{(float)atof(tok[0]), (float)atof(tok[1]), (float)atof(tok[2])};
      baroVar_ = (float)atof(tok[3]);
      const bool legs = atoi(tok[4]) != 0;
      const bool wasLive = mode_ == Mode::LIVE;
      if (!wasLive) {  // a fresh session: hold a still pad until the first frame
        a_ = padAccel_;
        g_ = Vec3{};
        baroAlt_ = 0;
        lastA_ = a_;
        lastG_ = g_;
        liveSeq_ = 0;
        lastFrameMs_ = ms;  // link-loss grace until the first frame
        liveFail_ = false;
        liveAbort_ = false;
        liveFresh_ = true;
        liveSameTicks_ = 0;
        liveLinkLost_ = false;
        liveTxDue_ = false;
        ps_ = 0;
      }
      g_hilLiveLegs = legs;
      g_hilLive = true;
      if (!wasLive) SerialUSB1.printf("OK live legs=%s\n", legs ? "LIVE" : "sim");
      mode_ = Mode::LIVE;
      if (wasLive) reply("OK live legs=%s", legs ? "LIVE" : "sim");
    } else if (!strcmp(cmd, "load")) {
      // load <frames> <crc32 hex> <mode 0|1> <pad ax> <pad ay> <pad az> <baro var>
      if (st != FlightState::IDLE) return fail("load only when IDLE (disarmed)");
      if (mode_ != Mode::PASSTHRU && mode_ != Mode::HOLD)
        return fail("load: unload / reboot first");
      const char* tok[7];
      for (int i = 0; i < 7; ++i) {
        tok[i] = strtok_r(nullptr, " ", &save);
        if (!tok[i]) return fail("load: needs 7 arguments");
      }
      const uint32_t n = strtoul(tok[0], nullptr, 10);
      if (n == 0 || n > hil::kMaxFrames) return fail("load: frame count out of range");
      n_ = n;
      expectCrc_ = strtoul(tok[1], nullptr, 16);
      traceMode_ = (uint8_t)atoi(tok[2]);
      padAccel_ = Vec3{(float)atof(tok[3]), (float)atof(tok[4]), (float)atof(tok[5])};
      baroVar_ = (float)atof(tok[6]);
      got_ = 0;
      crc_ = 0xFFFFFFFFu;
      lastRxMs_ = ms;
      idx_ = 0;
      mode_ = Mode::LOADING;
      SerialUSB1.printf("OK send %lu bytes\n", (unsigned long)(n_ * sizeof(HilFrame)));
    } else if (!strcmp(cmd, "unload")) {
      if (st != FlightState::IDLE) return fail("unload only when IDLE (disarmed)");
      if (mode_ != Mode::HOLD) return fail("unload: nothing loaded");
      mode_ = Mode::PASSTHRU;
      n_ = 0;
      SerialUSB1.println("OK unloaded: real sensors");
    } else if (!strcmp(cmd, "go")) {
      if (mode_ != Mode::HOLD) return fail("go: no trace loaded (or already played)");
      if (st != FlightState::ARMED) return fail("go: arm first (`arm` on the main console)");
      if ((uint8_t)core.mode() != traceMode_)
        return fail("go: flight mode differs from the trace's mode");
      openImuLog(ms);
      idx_ = 0;
      goUs_ = micros();
      lastProgressMs_ = 0;
      mode_ = Mode::PLAY;
      SerialUSB1.printf("OK go %lu\n", (unsigned long)ms);
    } else if (!strcmp(cmd, "abort")) {
      if (mode_ == Mode::PASSTHRU) return fail("abort: nothing loaded");
      if (mode_ == Mode::LIVE) {
        // Operator abort in a live run: the IMU "fails", so a core in flight
        // aborts to the chute. Stays LIVE so telemetry keeps flowing.
        liveFail_ = true;
        liveAbort_ = true;
        reply("OK aborted %lu core=%s", (unsigned long)ms, FlightStateMachine::stateName(st));
        return;
      }
      mode_ = Mode::ABORTED;
      if (endMs_ == 0) endMs_ = ms;
      SerialUSB1.printf("OK aborted %lu core=%s%s\n", (unsigned long)ms,
                        FlightStateMachine::stateName(st),
                        st == FlightState::ARMED ? " -- type `disarm` on the main console" : "");
    } else if (!strcmp(cmd, "reboot")) {
      if (co.inFlight || st == FlightState::ARMED)
        return fail("reboot: not while armed or in flight");
      reply("OK reboot");
      SerialUSB1.flush();
      delay(50);
      hilReboot();
    } else {
      fail("unknown command (hello status live load unload go abort reboot)");
    }
  }

  void fail(const char* why) { reply("ERR %s", why); }

  void receiveFrames(uint32_t ms) {
    uint8_t* dst = reinterpret_cast<uint8_t*>(g_hilFrames);
    const uint32_t total = n_ * sizeof(HilFrame);
    int avail = SerialUSB1.available();
    while (avail > 0 && got_ < total) {
      uint32_t want = total - got_;
      if ((uint32_t)avail < want) want = (uint32_t)avail;
      if (want > 512) want = 512;
      const size_t k = SerialUSB1.readBytes(reinterpret_cast<char*>(dst + got_), want);
      crc_ = crc32Update(crc_, dst + got_, k);
      got_ += k;
      lastRxMs_ = ms;
      avail = SerialUSB1.available();
    }
    if (got_ >= total) {
      const uint32_t crc = crc_ ^ 0xFFFFFFFFu;
      if (crc != expectCrc_) {
        mode_ = Mode::PASSTHRU;
        n_ = 0;
        SerialUSB1.printf("ERR crc %08lx != %08lx: trace discarded\n",
                          (unsigned long)crc, (unsigned long)expectCrc_);
        return;
      }
      mode_ = Mode::HOLD;
      baroAlt_ = g_hilFrames[0].baroAlt;
      SerialUSB1.printf("OK loaded %lu frames crc %08lx\n", (unsigned long)n_,
                        (unsigned long)crc);
    } else if (ms - lastRxMs_ > hil::kLoadTimeoutMs) {
      mode_ = Mode::PASSTHRU;
      n_ = 0;
      SerialUSB1.printf("ERR load timed out after %lu bytes\n", (unsigned long)got_);
    }
  }

  static uint32_t crc32Update(uint32_t crc, const uint8_t* p, size_t n) {
    while (n--) {
      crc ^= *p++;
      for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc;
  }

  // ---- real-IMU side log ----
  void openImuLog(uint32_t ms) {
    if (!SD.exists("HIL")) SD.mkdir("HIL");
    for (int i = 0; i < 1000; ++i) {
      snprintf(imuLogName_, sizeof(imuLogName_), "HIL/IMU_%03d.BIN", i);
      if (!SD.exists(imuLogName_)) break;
    }
    imuLog_ = SD.open(imuLogName_, FILE_WRITE);
    if (!imuLog_) {
      SerialUSB1.println("WARN could not open the real-IMU log (playing anyway)");
      return;
    }
    uint8_t hdr[64] = {0};
    memcpy(hdr, "RFCHILIMU1", 10);
    memcpy(hdr + 16, &ms, 4);         // playback start, millis()
    memcpy(hdr + 20, &expectCrc_, 4); // trace CRC
    memcpy(hdr + 24, &n_, 4);         // trace frames
    imuLog_.write(hdr, sizeof(hdr));
    imuLogOpen_ = true;
    fill_ = 0;
    active_ = 0;
    pendingBuf_ = -1;
    lastImuFlushMs_ = ms;
  }

  void recordReal() {
    HilImuRec& r = buf_[active_][fill_];
    r.ms = millis();
    const Vec3 a = RealSensors::accel(), g = RealSensors::gyro();
    r.a[0] = sat16(a.x * 100.0f); r.a[1] = sat16(a.y * 100.0f); r.a[2] = sat16(a.z * 100.0f);
    r.g[0] = sat16(g.x * 1000.0f); r.g[1] = sat16(g.y * 1000.0f); r.g[2] = sat16(g.z * 1000.0f);
    if (++fill_ >= hil::kRecsPerBuf) {
      if (pendingBuf_ >= 0) ++droppedBufs_;  // SD fell behind: overwrite
      pendingBuf_ = active_;
      active_ ^= 1;
      fill_ = 0;
    }
  }

  void serviceImuLog(uint32_t ms) {
    if (!imuLogOpen_) return;
    if (pendingBuf_ >= 0) {
      imuLog_.write(reinterpret_cast<const uint8_t*>(buf_[pendingBuf_]), sizeof(buf_[0]));
      pendingBuf_ = -1;
    }
    if (ms - lastImuFlushMs_ >= 500) {
      imuLog_.flush();
      lastImuFlushMs_ = ms;
    }
    if (endMs_ != 0 && ms - endMs_ > hil::kLogTailMs) {
      imuLog_.close();
      imuLogOpen_ = false;
      SerialUSB1.printf("OK imu log closed: %s (%lu buffers dropped)\n", imuLogName_,
                        (unsigned long)droppedBufs_);
    }
  }

  static int16_t sat16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lroundf(v);
  }

  const char* modeName() const {
    switch (mode_) {
      case Mode::PASSTHRU: return "PASSTHRU";
      case Mode::LOADING: return "LOADING";
      case Mode::HOLD: return "HOLD";
      case Mode::PLAY: return "PLAY";
      case Mode::DONE: return "DONE";
      case Mode::ABORTED: return "ABORTED";
      case Mode::LIVE: return "LIVE";
    }
    return "?";
  }

  Mode mode_ = Mode::PASSTHRU;
  uint32_t n_ = 0, idx_ = 0, tick_ = 0;
  uint32_t got_ = 0, crc_ = 0, expectCrc_ = 0, lastRxMs_ = 0;
  uint8_t traceMode_ = 0;
  Vec3 padAccel_{0, 0, cfg::G0};
  float baroVar_ = cfg::BARO_R_FALLBACK_M2;
  Vec3 a_{0, 0, cfg::G0}, g_{};
  float baroAlt_ = 0;
  bool baroPending_ = false;
  uint32_t endMs_ = 0, lastProgressMs_ = 0, goUs_ = 0;

  char line_[160];
  size_t len_ = 0;

  // LIVE
  uint16_t liveSeq_ = 0, liveStaleTicks_ = 0;
  uint32_t lastFrameMs_ = 0, liveFramesRx_ = 0, liveCrcErrors_ = 0, liveTxDropped_ = 0;
  bool liveFail_ = false, liveAbort_ = false, liveFresh_ = true, liveLinkLost_ = false;
  bool liveTxDue_ = false;
  int liveSameTicks_ = 0;
  Vec3 lastA_{}, lastG_{};
  uint8_t ps_ = 0, pType_ = 0, pLen_ = 0, pGot_ = 0;
  uint8_t pBuf_[160];

  File imuLog_;
  char imuLogName_[24] = "";
  bool imuLogOpen_ = false;
  HilImuRec buf_[2][hil::kRecsPerBuf];
  int fill_ = 0, active_ = 0, pendingBuf_ = -1;
  uint32_t droppedBufs_ = 0, lastImuFlushMs_ = 0;
};
