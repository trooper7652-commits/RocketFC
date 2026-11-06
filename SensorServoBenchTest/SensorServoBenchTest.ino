//
// SensorServoBenchTest -- bench rig for the RocketFC flight computer.
//
// Purpose: prove, on a desk, that the BMI088 + MS5611 + AHRS chain reports
// orientation correctly, and that the TVC gimbal deflects in the direction that
// would actually right the vehicle. You hold the rocket, rotate it, and a 3D
// model on the PC rotates with it.
//
// This is NOT flight software. There is no flight state machine, no altitude
// Kalman filter, no SD logging, no EEPROM, no watchdog, and no pyro control of
// any kind -- Actuators::enablePyro(), firePyro(), testFire(), applyLegs(),
// testLegs() and applyChute() are never called from anywhere in this project,
// so no code path here can energize an e-match or the leg-release nichrome,
// or open the parachute latch (begin() parks it at LOCK).
//
// The attitude math, the sensor drivers and the servo mapping are byte-for-byte
// copies of RocketFC's (see sync_core.py), so what you validate here is what
// actually flies.
//
//   500 Hz  IMU -> AHRS -> (optionally) TVC PID
//   ~100 Hz MS5611 async pipeline
//   50 Hz   servo output
//   50 Hz   attitude telemetry line to the PC visualizer
//
// USB serial at 115200: type `help`.
//
#include <Arduino.h>

#include "src/bench_cli.h"
#include "src/bench_config.h"
#include "src/bench_telem.h"
#include "src/config.h"
#include "src/core/ahrs.h"
#include "src/core/control.h"
#include "src/hw/actuators.h"
#include "src/hw/sensors.h"

// ---------------------------------------------------------------------------
Sensors sensors;
Actuators act;
Ahrs ahrs;
TvcControl ctl;
BenchTelemetry telem;
BenchContext ctx{sensors, act, ahrs, ctl, telem};
BenchCli benchCli(ctx);

elapsedMicros usFast, usServo, usViz;

BenchProc proc = BenchProc::NONE;
uint32_t procStartMs = 0;

// Latest controller output, in gimbal radians about body X / Y.
float gimbalX = 0, gimbalY = 0;

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) {}

  Serial.println("\n=== SensorServoBenchTest -- RocketFC sensor & TVC bench rig ===");

  act.begin(0, 0);  // no EEPROM here; trims start at zero
  const bool ok = sensors.begin();

  ctl.configure(cfg::GIMBAL_MAX_RAD, cfg::GIMBAL_SLEW_RADPS, cfg::D_LPF_HZ);
  ctl.setGains(cfg::GAINS_BOOST);
  ctl.reset();
  ahrs.reset();

  Serial.printf("imu:  %s\n", sensors.imuOk() ? "OK" : "** FAIL **");
  Serial.printf("baro: %s\n", sensors.baroOk() ? "OK" : "** FAIL **");
  if (!ok) Serial.println("!! sensor failure -- check I2C wiring (SDA 18, SCL 19, 3.3 V).");
  Serial.println("pyro channels: NOT USED by this project.");
  Serial.println("type `help` for commands. Run `cal` while holding still.\n");

  // Servo wiggle: visible proof the outputs are alive before anything else.
  act.writeRawUs(1540, 1540);
  delay(150);
  act.writeRawUs(1460, 1460);
  delay(150);
  act.center();

  telem.begin(bench::VIZ_ON_AT_BOOT);
}

// ---------------------------------------------------------------------------
// Non-blocking cal / zero, driven by requests from the CLI.
static void runProcedures(uint32_t ms) {
  if (proc == BenchProc::NONE && ctx.procRequest != BenchProc::NONE) {
    proc = ctx.procRequest;
    ctx.procRequest = BenchProc::NONE;
    procStartMs = ms;
    if (proc == BenchProc::CAL) {
      Serial.println("calibrating gyro bias + level -- hold the vehicle STILL (~4 s)...");
      sensors.startImuCal();
    } else {
      Serial.println("collecting barometer ground reference...");
      sensors.startGroundRef();
    }
    ctx.procBusy = true;
    return;
  }

  if (proc == BenchProc::NONE) return;

  if (ms - procStartMs > bench::PROC_TIMEOUT_MS) {
    Serial.println("procedure TIMED OUT (was the vehicle moving?).");
    proc = BenchProc::NONE;
    ctx.procBusy = false;
    return;
  }

  if (proc == BenchProc::CAL && sensors.imuCalDone()) {
    // Same order the flight code uses at arm: bias first (applied inside
    // Sensors::gyro()), then seed attitude from the averaged accel.
    ahrs.initFromAccel(sensors.accelLevelAvg());
    const Vec3 b = sensors.gyroBias();
    Serial.printf("cal done. gyro bias [%.3f %.3f %.3f] dps, tilt now %.2f deg\n",
                  b.x * cfg::RAD2DEG, b.y * cfg::RAD2DEG, b.z * cfg::RAD2DEG,
                  ahrs.tiltRad() * cfg::RAD2DEG);
    proc = BenchProc::NONE;
  } else if (proc == BenchProc::ZERO && sensors.groundRefDone()) {
    Serial.printf("ground ref set. baro noise std %.3f m\n",
                  sqrtf(sensors.baroAltNoiseVar()));
    proc = BenchProc::NONE;
  }
  ctx.procBusy = (proc != BenchProc::NONE);
}

// ---------------------------------------------------------------------------
static void vizTick(uint32_t ms) {
  VizFrame f;
  f.ms = ms;
  f.q = ahrs.quat();
  f.accel = sensors.accel();
  f.gyro = sensors.gyro();
  f.baroAlt = sensors.baroAltitude();
  f.tiltDeg = ahrs.tiltRad() * cfg::RAD2DEG;
  f.gimXDeg = gimbalX * cfg::RAD2DEG;
  f.gimYDeg = gimbalY * cfg::RAD2DEG;
  f.usA = act.lastUsA();
  f.usB = act.lastUsB();
  f.tvcMode = (uint8_t)ctx.tvcMode;
  f.imuOk = sensors.imuOk();
  f.baroOk = sensors.baroOk();
  telem.send(f);
}

// ---------------------------------------------------------------------------
void loop() {
  const uint32_t ms = millis();

  // --- 500 Hz: IMU -> AHRS -> controller ---
  if (usFast >= (uint32_t)(1e6f / cfg::FAST_HZ)) {
    const uint32_t period = usFast;
    usFast = 0;

    float dt = period * 1e-6f;
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.008f) dt = 0.008f;

    sensors.readImu();

    const Vec3 a = sensors.accel();
    const Vec3 g = sensors.gyro();

    if (!ahrs.initialized()) ahrs.initFromAccel(a);
    ahrs.update(g, dt);
    // The bench is permanently "on the pad", so unlike flight we never stop
    // the accel leveling: tilt stays honest indefinitely. Heading still drifts,
    // because nothing observes it without a magnetometer.
    ahrs.accelLevelBlend(a, bench::LEVEL_BLEND_ALPHA);

    if (ctx.tvcMode != bench::TvcMode::OFF) {
      const Vec3 e = ahrs.tiltErrorBody();
      ctl.update(e.x, e.y, g.x, g.y, dt, true);
      gimbalX = ctl.gimbalX();
      gimbalY = ctl.gimbalY();
    } else {
      gimbalX = gimbalY = 0;
    }
  }

  // --- every pass: cheap state machines ---
  sensors.baroTick(ms);
  benchCli.poll();
  runProcedures(ms);

  // --- 50 Hz: servos ---
  if (usServo >= (uint32_t)(1e6f / cfg::SERVO_HZ)) {
    usServo = 0;
    if (!benchCli.sweepActive()) {
      // Only DRIVE reaches the servos. CALC computes and reports the very same
      // numbers with the mount held still, which is what makes it safe to try
      // first.
      if (ctx.tvcMode == bench::TvcMode::DRIVE) act.writeGimbal(gimbalX, gimbalY);
      else act.center();
    }
  }

  // --- 50 Hz: attitude telemetry ---
  if (usViz >= (uint32_t)(1e6f / bench::VIZ_HZ)) {
    usViz = 0;
    vizTick(ms);
  }
}
