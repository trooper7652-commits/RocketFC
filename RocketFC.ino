//
// RocketFC — thrust-vector-controlled, propulsively-landed model rocket
// flight computer for Teensy 4.1 + BMI088 + MS5611.
//
// All tunables live in src/config.h. All flight LOGIC lives in src/core/
// (hardware-free, tested on PC by tools/replay). This sketch is the hardware
// glue: a polling scheduler that reads sensors, steps FlightCore, and routes
// its outputs to servos, pyros, the SD log, and the status beeper.
//
//   500 Hz  IMU -> AHRS -> KF predict -> FSM -> PID
//   ~100 Hz MS5611 async pipeline -> KF update
//   50 Hz   servo output
//   100 Hz  SD log rows
//   10 Hz   battery/continuity/health, CLI stream, beeper
//
// Serial CLI at 115200 baud: type `help`.
//
#include <Arduino.h>

#include "src/config.h"
#include "src/core/flight_core.h"
#include "src/hw/actuators.h"
#include "src/hw/cli.h"
#include "src/hw/config_store.h"
#include "src/hw/logger.h"
#include "src/hw/sensors.h"
#include "src/hw/status.h"

// ---------------------------------------------------------------------------
// Watchdog (i.MX RT1062 WDOG1): 2 s timeout, fed from loop(). If the firmware
// wedges, the processor resets, boots into IDLE with the pyro inhibited,
// gimbal servos centered and the chute latch at LOCK — a safe state.
// ---------------------------------------------------------------------------
#if defined(ARDUINO_TEENSY41) && defined(WDOG1_WCR)
static void wdogInit() {
  if (!cfg::ENABLE_WATCHDOG) return;
  WDOG1_WCR = WDOG_WCR_WT(3) | WDOG_WCR_WDE | WDOG_WCR_SRS | WDOG_WCR_WDA;
}
static void wdogFeed() {
  if (!cfg::ENABLE_WATCHDOG) return;
  WDOG1_WSR = 0x5555;
  WDOG1_WSR = 0xAAAA;
}
#else
static void wdogInit() {}
static void wdogFeed() {}
#endif

// ---------------------------------------------------------------------------
Sensors sensors;
Actuators act;
Logger logger;
StatusIndicator statusInd;
ConfigStore store;
FlightCore core;
CliContext cliCtx{sensors, act, logger, store, core};
Cli serialCli(cliCtx);  // "cli" is taken: Teensy core defines cli() = __disable_irq()

CoreOutput co;  // latest core outputs (servo task + logging read these)

elapsedMicros usFast, usServo, usLog, usSlow;
uint32_t loopMaxUs = 0;
uint32_t touchdownMs = 0;
bool logClosedAfterTd = false;

// Arming / calibration procedures (non-blocking sequences)
enum class Proc : uint8_t { NONE, CAL, ZERO, ARM_COLLECT };
Proc proc = Proc::NONE;
uint32_t procStartMs = 0;

// ---------------------------------------------------------------------------
void setup() {
  statusInd.begin();
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) {}  // wait briefly for USB

  Serial.println("\n=== RocketFC — TVC self-landing flight computer ===");

  store.load();
  act.begin(store.trimAUs(), store.trimBUs());
  const bool sensorsOk = sensors.begin();
  const bool sdOk = logger.begin();
  core.begin();
  core.setMode(store.mode());

  // Startup self-test report
  Serial.printf("imu:  %s\n", sensors.imuOk() ? "OK" : "** FAIL **");
  Serial.printf("baro: %s\n", sensors.baroOk() ? "OK" : "** FAIL **");
  Serial.printf("sd:   %s\n", sdOk ? "OK" : "** FAIL **");
  Serial.printf("mode: %s   trims A=%+.0f B=%+.0f us\n",
                store.mode() == cfg::FlightMode::CHUTE_TEST ? "CHUTE_TEST"
                                                            : "FULL_LANDING",
                store.trimAUs(), store.trimBUs());
  Serial.printf("chute latch: LOCK (%.0f us, pin %d)\n", act.chuteUs(),
                cfg::PIN_SERVO_CHUTE);
  if (!sensorsOk) Serial.println("!! sensor failure — check wiring, then reboot.");
  Serial.println("type `help` for commands.\n");

  // Servo wiggle: visible + audible proof the outputs are alive. Gimbal
  // only — the chute latch servo is never wiggled.
  act.writeRawUs(1540, 1540);
  delay(150);
  act.writeRawUs(1460, 1460);
  delay(150);
  act.center();

  cliCtx.wdogFeed = &wdogFeed;
  wdogInit();
}

// ---------------------------------------------------------------------------
static void runProcedures(uint32_t ms) {
  // Kick off requested procedure
  if (proc == Proc::NONE && cliCtx.procRequest != ProcRequest::NONE) {
    const ProcRequest req = cliCtx.procRequest;
    cliCtx.procRequest = ProcRequest::NONE;

    if (req == ProcRequest::CAL) {
      Serial.println("calibrating gyro bias + level (keep the vehicle still)...");
      sensors.startImuCal();
      proc = Proc::CAL;
    } else if (req == ProcRequest::ZERO) {
      Serial.println("collecting baro ground reference...");
      sensors.startGroundRef();
      proc = Proc::ZERO;
    } else if (req == ProcRequest::ARM) {
      // ---- pre-arm checks ----
      const bool needLandCont = core.mode() == cfg::FlightMode::FULL_LANDING;
      bool ok = true;
      auto fail = [&](const char* why) {
        Serial.printf("ARM REFUSED: %s\n", why);
        ok = false;
      };
      if (!sensors.imuOk()) fail("IMU not healthy");
      if (!sensors.baroOk()) fail("barometer not healthy");
      if (!logger.sdOk()) fail("SD card missing/failed");
      if (cliCtx.vbatCached < cfg::VBAT_MIN) fail("battery low");
      // Commanded position only -- there's no latch switch, so this can't
      // prove the latch is physically seated. Load it, then `chute lock`.
      if (!act.chuteLocked())
        fail("chute latch not locked (load the spring, then `chute lock`)");
      if (needLandCont && !cliCtx.contLandCached)
        fail("no continuity on LANDING channel");
      if (needLandCont && !cliCtx.contLegsCached)
        fail("no continuity on LEGS (nichrome) channel");
      if (needLandCont && cliCtx.vpyroCached < cfg::PYRO_VBAT_MIN)
        fail("pyro battery low or unplugged");
      if (cfg::REQUIRE_ARM_SWITCH && !act.armSwitchOn())
        fail("arm switch is off");
      if (!ok) return;

      Serial.println("pre-arm OK. Calibrating (keep still, ~4 s)...");
      sensors.startImuCal();
      sensors.startGroundRef();
      proc = Proc::ARM_COLLECT;
    }
    procStartMs = ms;
    cliCtx.procBusy = (proc != Proc::NONE);
    return;
  }

  if (proc == Proc::NONE) return;

  // Timeout: motion keeps restarting the gyro cal
  if (ms - procStartMs > 20000) {
    Serial.println("procedure TIMED OUT (vehicle not still?).");
    proc = Proc::NONE;
    cliCtx.procBusy = false;
    return;
  }

  switch (proc) {
    case Proc::CAL:
      if (sensors.imuCalDone()) {
        core.padLevelInit(sensors.accelLevelAvg());
        const Vec3 b = sensors.gyroBias();
        Serial.printf("cal done. gyro bias [%.3f %.3f %.3f] dps, tilt %.2f deg\n",
                      b.x * cfg::RAD2DEG, b.y * cfg::RAD2DEG, b.z * cfg::RAD2DEG,
                      core.ahrs().tiltRad() * cfg::RAD2DEG);
        proc = Proc::NONE;
      }
      break;

    case Proc::ZERO:
      if (sensors.groundRefDone()) {
        core.setBaroNoiseVar(sensors.baroAltNoiseVar());
        core.zeroAltitude();
        Serial.printf("ground ref set. baro noise std %.3f m\n",
                      sqrtf(sensors.baroAltNoiseVar()));
        proc = Proc::NONE;
      }
      break;

    case Proc::ARM_COLLECT:
      if (sensors.imuCalDone() && sensors.groundRefDone()) {
        // Tilt check straight from the averaged accel (vehicle must be
        // near-vertical on the rail).
        const Vec3 a = sensors.accelLevelAvg();
        const float tilt0 = acosf(a.z / a.norm()) * cfg::RAD2DEG;
        if (tilt0 > cfg::PREARM_TILT_MAX_DEG) {
          Serial.printf("ARM REFUSED: vehicle tilted %.1f deg (max %.0f)\n",
                        tilt0, cfg::PREARM_TILT_MAX_DEG);
          proc = Proc::NONE;
          break;
        }
        core.padLevelInit(a);
        core.setBaroNoiseVar(sensors.baroAltNoiseVar());
        core.zeroAltitude();

        const int fn = store.nextFlightNumber();
        if (!logger.openFlight(fn)) {
          Serial.println("ARM REFUSED: could not open log file.");
          proc = Proc::NONE;
          break;
        }
        core.requestArm(ms);
        touchdownMs = 0;
        logClosedAfterTd = false;
        Serial.printf(
            "*** ARMED — flight #%03d, mode %s. Launch detect live. ***\n", fn,
            core.mode() == cfg::FlightMode::CHUTE_TEST ? "CHUTE_TEST"
                                                       : "FULL_LANDING");
        proc = Proc::NONE;
      }
      break;

    default:
      proc = Proc::NONE;
      break;
  }
  cliCtx.procBusy = (proc != Proc::NONE);
}

// ---------------------------------------------------------------------------
static void logTick(uint32_t ms) {
  // Drain FSM events first (they also go to USB serial for bench visibility).
  FlightStateMachine::Event ev;
  while (core.fsm().popEvent(ev)) {
    const char* name = FlightStateMachine::eventName(ev.code);
    logger.logEvent(ev.ms, name, ev.value);
    Serial.printf("[%lu ms] EVENT %s %.2f\n", (unsigned long)ev.ms, name,
                  ev.value);
  }

  if (!logger.isOpen()) return;

  LogFrame f;
  f.ms = ms;
  f.state = FlightStateMachine::stateName(co.state);
  const Vec3 a = sensors.accel(), g = sensors.gyro();
  f.ax = a.x; f.ay = a.y; f.az = a.z;
  f.gx = g.x; f.gy = g.y; f.gz = g.z;
  f.baroAlt = sensors.baroAltitude();
  f.baroPress = sensors.baroPressure();
  f.qw = co.attitude.w; f.qx = co.attitude.x;
  f.qy = co.attitude.y; f.qz = co.attitude.z;
  f.tiltDeg = co.tiltDeg;
  f.kfAlt = co.kfAlt; f.kfVel = co.kfVel; f.kfBias = co.kfBias;
  f.innov = co.innovation;
  f.pX = core.control().pTerm(0); f.iX = core.control().iTerm(0);
  f.dX = core.control().dTerm(0);
  f.pY = core.control().pTerm(1); f.iY = core.control().iTerm(1);
  f.dY = core.control().dTerm(1);
  f.gimXDeg = co.gimbalX * cfg::RAD2DEG;
  f.gimYDeg = co.gimbalY * cfg::RAD2DEG;
  f.usA = act.lastUsA(); f.usB = act.lastUsB();
  f.pyroFlags = (act.chuteReleased() ? 1 : 0) | (act.pyroActive() ? 2 : 0) |
                (act.legsActive() ? 4 : 0);
  f.cont = (co.chuteDetected ? 1 : 0) | (cliCtx.contLandCached ? 2 : 0) |
           (cliCtx.contLegsCached ? 4 : 0);
  f.vbat = cliCtx.vbatCached;
  f.loopMaxUs = loopMaxUs;
  loopMaxUs = 0;
  logger.logRow(f);
  logger.flushTick(ms);
}

// ---------------------------------------------------------------------------
static void slowTick(uint32_t ms) {
  cliCtx.vbatCached = act.vbat();
  cliCtx.contLandCached = act.continuity();
  cliCtx.contLegsCached = act.legsContinuity();
  cliCtx.vpyroCached = max(act.contVolts(), act.legsContVolts());

  // Fault code for the IDLE beeper (continuity is enforced at arm time
  // instead — a bare bench board shouldn't scream all day).
  uint8_t fault = 0;
  if (!sensors.imuOk()) fault = FAULT_IMU;
  else if (!sensors.baroOk()) fault = FAULT_BARO;
  else if (!logger.sdOk()) fault = FAULT_SD;
  else if (cliCtx.vbatCached > 1.0f && cliCtx.vbatCached < cfg::VBAT_MIN)
    fault = FAULT_VBAT;  // >1 V: ignore when running on USB only
  cliCtx.faultCode = fault;
  statusInd.setFaultCode(fault);

  serialCli.streamTick(ms, co);

  // Close the log a few seconds after touchdown (captures the final rows).
  if (co.state == FlightState::TOUCHDOWN && !logClosedAfterTd) {
    if (touchdownMs == 0) touchdownMs = ms;
    if (ms - touchdownMs > 4000) {
      logger.close();
      logClosedAfterTd = true;
      Serial.println("flight log closed. Safe to power off.");
    }
  }
}

// ---------------------------------------------------------------------------
void loop() {
  const uint32_t ms = millis();

  // --- 500 Hz: IMU -> core step ---
  if (usFast >= (uint32_t)(1e6f / cfg::FAST_HZ)) {
    const uint32_t period = usFast;
    usFast = 0;
    if (period > loopMaxUs) loopMaxUs = period;

    float dt = period * 1e-6f;
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.008f) dt = 0.008f;

    sensors.readImu();

    CoreInput ci;
    ci.ms = ms;
    ci.dt = dt;
    ci.accel = sensors.accel();
    ci.gyro = sensors.gyro();
    ci.baroNew = sensors.baroNew();
    ci.baroAlt = sensors.baroAltitude();
    ci.imuHealthy = sensors.imuOk();
    ci.contLanding = cliCtx.contLandCached;

    core.step(ci, co);

    // Pyro routing: FlightCore only pulses this in flight; the enable adds a
    // second layer so nothing can fire from IDLE/ARMED-on-the-pad states.
    act.enablePyro(co.inFlight);
    if (co.fireLanding) act.firePyro(ms);
    // Leg-release nichrome: a level, on from fire + LEGS_DELAY_MS (confirmed
    // burn only) until touchdown is detected.
    act.applyLegs(co.legsBurn, ms);

    // Chute latch: follows the core's release level (incl. re-cycles of a
    // stuck latch). Only ever released by the core in flight.
    act.applyChute(co.chuteRelease, co.inFlight);
  }

  // --- every pass: cheap state machines ---
  sensors.baroTick(ms);
  act.pyroTick(ms);
  serialCli.poll(ms);
  runProcedures(ms);
  statusInd.tick(ms, co.state, core.mode());

  // --- 50 Hz: servos ---
  if (usServo >= (uint32_t)(1e6f / cfg::SERVO_HZ)) {
    usServo = 0;
    if (!serialCli.servoTestActive()) {
      if (co.tvcActive) act.writeGimbal(co.gimbalX, co.gimbalY);
      else act.center();
    }
  }

  // --- 100 Hz: logging ---
  if (usLog >= (uint32_t)(1e6f / cfg::LOG_FAST_HZ)) {
    usLog = 0;
    logTick(ms);
  }

  // --- 10 Hz: housekeeping ---
  if (usSlow >= (uint32_t)(1e6f / cfg::SLOW_HZ)) {
    usSlow = 0;
    slowTick(ms);
  }

  wdogFeed();
}
