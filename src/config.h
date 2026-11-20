#pragma once
//
// RocketFC — master configuration. Single source of truth for pins, rates,
// masses, gains, and thresholds.
//
// This header is PURE C++ (no Arduino includes) because it is shared by the
// Teensy firmware and the PC replay harness (tools/replay).
//
// Items marked [MEASURE] are placeholders that MUST be replaced with values
// measured from the actual vehicle before any flight.
//
#include <cstdint>

namespace cfg {

// ---------------------------------------------------------------------------
// Math constants / units. Internal units are SI (m, m/s, rad). Degrees and
// g's appear only here at the config boundary and in logs.
// ---------------------------------------------------------------------------
constexpr float PI_F    = 3.14159265f;
constexpr float DEG2RAD = PI_F / 180.0f;
constexpr float RAD2DEG = 180.0f / PI_F;
constexpr float G0      = 9.80665f;   // standard gravity, m/s^2

// ---------------------------------------------------------------------------
// Flight modes
// ---------------------------------------------------------------------------
enum class FlightMode : uint8_t {
  CHUTE_TEST   = 0,  // TVC ascent, parachute at apogee. Fly this first.
  FULL_LANDING = 1,  // TVC ascent, powered landing burn; spring chute reserved for aborts.
};
constexpr FlightMode DEFAULT_MODE = FlightMode::CHUTE_TEST;

// ---------------------------------------------------------------------------
// Loop rates (Hz)
// ---------------------------------------------------------------------------
constexpr float FAST_HZ  = 500.0f;  // IMU read, AHRS, KF predict, PID
constexpr float MID_HZ   = 100.0f;  // baro pipeline, FSM, logging
constexpr float SERVO_HZ = 50.0f;   // servo refresh (Servo library standard)
constexpr float SLOW_HZ  = 10.0f;   // battery, health, status patterns
constexpr float FAST_DT  = 1.0f / FAST_HZ;

// ---------------------------------------------------------------------------
// Pins (Teensy 4.1). Adjust to your wiring; verify with `selftest`/`servotest`.
// ---------------------------------------------------------------------------
constexpr int PIN_SERVO_A      = 2;   // gimbal servo producing torque about body +X
constexpr int PIN_SERVO_B      = 3;   // gimbal servo producing torque about body +Y
constexpr int PIN_PYRO_LAND    = 6;   // MOSFET gate, landing-motor e-match
constexpr int PIN_PYRO_LEGS    = 7;   // MOSFET gate, landing-leg release nichrome
constexpr int PIN_CONT_LAND    = 14;  // A0 — continuity sense divider, landing channel
constexpr int PIN_CONT_LEGS    = 15;  // A1 — continuity sense divider, legs nichrome
constexpr int PIN_SERVO_CHUTE  = 23;  // parachute spring-latch release servo
constexpr int PIN_VBAT         = 16;  // A2 — battery voltage divider
constexpr int PIN_BUZZER       = 8;
constexpr int PIN_ARM_SWITCH   = 9;   // optional arm switch -- NOT FITTED, pin unused
constexpr int PIN_LED          = 13;  // built-in LED
// I2C uses Wire defaults on Teensy 4.1: SDA=18, SCL=19.

// No arm switch on this vehicle: plugging in the pyro LiPo is the arm.
constexpr bool REQUIRE_ARM_SWITCH = false;

// ---------------------------------------------------------------------------
// IMU mounting: body = R_SB * sensor. Body frame: +Z = long axis toward the
// NOSE, X/Y lateral aligned with the gimbal axes. Edit this row-major 3x3 to
// match how the BMI088 breakout sits in the vehicle (entries are -1/0/+1 for
// axis-aligned mounting).
//
// Measured on the bench 2026-08-16 with SensorServoBenchTest: standing the
// vehicle nose up put gravity on sensor -X, i.e. the breakout's -X axis points
// out the nose. This matrix maps that onto body +Z and completes a right-handed
// frame (determinant +1).
//
// Gravity only fixes the LONG axis. The lateral assignment below -- sensor +Y
// to body +X, sensor -Z to body +Y -- is the right-handed completion, and which
// of those is the servo A gimbal axis still has to be confirmed by the gimbal
// direction test. If A and B come out swapped, exchange the first two rows.
// ---------------------------------------------------------------------------
constexpr float IMU_R_SB[9] = {
   0,  1,  0,
   0,  0, -1,
  -1,  0,  0,
};

// ---------------------------------------------------------------------------
// Vehicle & motors   [MEASURE] all of these on the real vehicle
// ---------------------------------------------------------------------------
// Ascent AND landing motor: AeroTech E18W (RMS-24/40 reload in a 24->29 mm
// adapter, thrustcurve.org cert curve tools/motors/AeroTech_E18W.eng), both
// flown PLUGGED (no ejection charge). Apogee ~18 m, inside the 100 ft limit
// (motor-chute-study-2026-11-04). The spent ascent casing + adapter (46.3 g)
// is pushed out of the tail at landing ignition; tools/burn_table.py takes
// that into account. Masses: the rocket weighed 1.343 kg without motors
// (owner, 2026-11-20), plus two loaded E18W (57.0 g each, .eng) and their two
// 24->29 mm adapters (10 g each). Re-weigh if the adapters were already in.
constexpr float MASS_PAD_KG     = 1.4770f; // on the pad, everything loaded: 1.343 + 2 x 0.067
constexpr float MASS_DESCENT_KG = 1.4563f; // after ascent burnout: pad - 20.7 g E18W propellant
constexpr float CDA_M2          = 0.0040f; // [MEASURE/estimate] drag Cd*A for descent
constexpr float ASCENT_BURN_MS  = 2140.0f; // AeroTech E18W certified curve burn time
// Landing motor: table generated by tools/burn_table.py into src/core/burn_table.h
constexpr float IGNITION_DELAY_MS = 620.0f; // landing e-match command -> thrust, measured on the static test 2026-10-25

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
constexpr uint32_t I2C_HZ = 400000;
// MS5611: OSR=2048 -> ~4.6 ms conversion; pressure every cycle, temperature
// refreshed every N pressure conversions.
constexpr int   BARO_TEMP_EVERY      = 20;
constexpr int   BARO_GROUND_SAMPLES  = 200;   // ~2 s of pad data for ground ref + noise (R)
constexpr float BARO_R_FALLBACK_M2   = 0.25f; // (0.5 m)^2 if pad measurement unavailable
constexpr float GYRO_CAL_SECONDS     = 3.0f;
constexpr float GYRO_STILL_DPS       = 1.5f;  // max |rate| considered "still" during cal

// ---------------------------------------------------------------------------
// Kalman filter (altitude / vertical velocity / accel bias)
// ---------------------------------------------------------------------------
constexpr float KF_SIGMA_ACCEL      = 0.5f;    // m/s^2 process noise from accel/vibration
constexpr float KF_SIGMA_BIAS_RW    = 0.01f;   // m/s^2 per sqrt(s), bias random walk
constexpr float KF_GATE_SIGMA       = 5.0f;    // reject baro innovations beyond this
constexpr int   KF_GATE_FORCE_ACCEPT = 25;     // accept after N consecutive rejects (recovery)
constexpr float KF_R_BOOST_INFLATE  = 25.0f;   // baro variance multiplier during BOOST

// ---------------------------------------------------------------------------
// TVC control. Two gain sets: vehicle is heavy w/ your ascent motor during
// BOOST, light w/ the landing motor during LANDING_BURN. Units: gimbal rad per rad of
// tilt error (kp), per rad*s (ki), per rad/s of body rate (kd).
// START ON THE TEST STAND — these are bench placeholders, not flight-ready.
// ---------------------------------------------------------------------------
struct PidGains { float kp, ki, kd, iMaxRad; };
constexpr PidGains GAINS_BOOST   = { 0.60f, 0.15f, 0.110f, 2.0f * DEG2RAD };
constexpr PidGains GAINS_LANDING = { 0.80f, 0.20f, 0.130f, 2.0f * DEG2RAD };

constexpr float GIMBAL_MAX_RAD    = 5.0f * DEG2RAD;   // mechanical gimbal limit
constexpr float GIMBAL_SLEW_RADPS = 400.0f * DEG2RAD; // command slew limit
constexpr float D_LPF_HZ          = 25.0f;            // low-pass on gyro rate for D term

// Servo mapping: us = CENTER + SIGN * gimbal_deg * US_PER_GIMBAL_DEG, clamped.
// US_PER_GIMBAL_DEG = (servo us per servo deg, ~10) * (servo deg per gimbal deg
// from linkage geometry). [MEASURE] Verify direction with `servotest` procedure.
constexpr float SERVO_A_CENTER_US = 1500.0f;  // [MEASURE] trim via CLI `trim`
constexpr float SERVO_B_CENTER_US = 1500.0f;  // [MEASURE]
constexpr float SERVO_A_US_PER_GIMBAL_DEG = 20.0f;  // [MEASURE] linkage ratio included
constexpr float SERVO_B_US_PER_GIMBAL_DEG = 20.0f;  // [MEASURE]
constexpr float SERVO_A_SIGN = +1.0f;  // [MEASURE] flip if bench direction test fails
constexpr float SERVO_B_SIGN = +1.0f;  // [MEASURE]
constexpr float SERVO_MIN_US = 1000.0f;
constexpr float SERVO_MAX_US = 2000.0f;

// ---------------------------------------------------------------------------
// Flight state machine thresholds (all debounced)
// ---------------------------------------------------------------------------
constexpr float LAUNCH_ACCEL_G   = 1.5f;   // long-axis specific force to declare liftoff
                                           // (slow-ramp motors like the F15 barely reach 2 g)
constexpr float LAUNCH_MS        = 80.0f;
constexpr float BURNOUT_ACCEL_G  = 0.35f;  // |specific force| below this = motor out
constexpr float BURNOUT_MS       = 150.0f;
constexpr float BOOST_MAX_MS     = ASCENT_BURN_MS + 1500.0f; // burnout backup timer
constexpr float APOGEE_VEL_MS    = 200.0f; // KF vel <= 0 must persist this long
constexpr float APOGEE_DROP_M    = 2.5f;   // backup: altitude fell this far below max
constexpr float COAST_MAX_MS     = 30000.0f; // absolute backup: force apogee
constexpr float TILT_ABORT_DEG   = 30.0f;  // sustained tilt beyond this aborts
constexpr float TILT_ABORT_MS    = 100.0f;
constexpr float TILT_IGNITE_MAX_DEG = 15.0f;  // never light the landing motor beyond this tilt
constexpr float BURN_TABLE_MARGIN_M = 1.0f;   // + = ignite earlier (higher); 1 m with S3:
                                              // sim holdout 2026-10-24 (motor-study-2026-10-24)
constexpr float BURN_MIN_SPEED_MS   = 2.0f;   // table domain lower bound
constexpr float MISSED_WINDOW_ALT_M = 4.0f;   // in DESCENT below this = missed ignition -> abort
constexpr float IGNITE_CONFIRM_G    = 1.3f;   // accel that confirms the landing motor lit
                                              // (only above TOUCHDOWN_ALT_M: lower, a jolt is the impact)
// After the expected delay, no thrust = dud -> abort to the chute. Short on
// purpose: the chute must be released before impact (a dud fired at ~15 m
// hits ~1.2 s after the command). 200 ms = released at ~6 m, and still 0
// false dud aborts at +-15 % ignition-delay scatter (latch-dud-study-2026-11-07).
constexpr float DUD_WINDOW_MS       = 200.0f;
constexpr float LANDING_BURN_MAX_MS = 5000.0f; // E18W 2.14 s + 0.62 s delay + margin
constexpr float TOUCHDOWN_ALT_M  = 3.0f;
constexpr float TOUCHDOWN_VEL_MS = 1.0f;
constexpr float TOUCHDOWN_MS     = 800.0f;
constexpr float PREARM_TILT_MAX_DEG = 5.0f;   // refuse to arm unless near vertical
constexpr bool  ABORT_FIRES_CHUTE_ALWAYS = true; // fire chute on abort even if low
constexpr float KF_UNHEALTHY_ABORT_MS = 500.0f;

// ---------------------------------------------------------------------------
// Landing-burn drift cancelling (S3): during the landing burn, lean the TVC
// target against the sideways velocity dead-reckoned from launch
// (world-frame accelerometer), so the rocket has stopped drifting by
// touchdown and doesn't tip over on its legs. Sim study 2026-10-24
// (motor-study-2026-10-24): sideways contact speed ~1.4 -> ~0.45 m/s.
// ---------------------------------------------------------------------------
constexpr float DRIFT_GAIN = 2.0f;           // 1/s; 0 = off
constexpr float DRIFT_MAX_TILT_DEG = 5.0f;   // cap on the commanded lean

// ---------------------------------------------------------------------------
// Parachute release servo. The chute is pushed out by a spring; this servo
// holds the latch. FlightStateMachine decides WHEN (apogee in CHUTE_TEST, any
// abort in either mode); ChuteDeploy (src/core/chute_deploy.h) then confirms
// the canopy opened from the accelerometer and re-cycles the latch if not.
//
// Three named latch angles:
//   OPEN  -- fully open, releases the chute (flight release, `chute open`)
//   SET   -- ground-only loading position: `chute set`, load the spring and
//            chute, then `chute lock`. Flight never uses it.
//   LOCK  -- holds the loaded spring; arming requires it (`chute lock`)
// Fine-tune them with the CLI `chute deg <n>` jog, then `chute save
// lock|set|open` stores them in EEPROM (no re-upload). The *_DEFAULT values
// below are used on first boot / `chute defaults`.
// Degrees -> pulse: linear between the two US_AT_* points; set them from the
// servo's datasheet (typical: 500..2500 us over 0..180 deg).
//
// Boot: with CHUTE_BOOT_RESTORE the latch powers up at the last position
// commanded on the ground (saved in EEPROM). Exceptions, which boot OPEN as
// before: a watchdog / crash reset, and a power-on while the IMU says the
// vehicle is moving or not at ~1 g (power blip in flight). false = always
// boot OPEN (a reboot with a loaded spring ejects the chute).
// ---------------------------------------------------------------------------
constexpr float CHUTE_SERVO_US_AT_0_DEG   = 500.0f;
constexpr float CHUTE_SERVO_US_AT_180_DEG = 2500.0f;
constexpr float CHUTE_LOCK_DEG_DEFAULT    = 40.0f;   // latch holding the spring (944 us)
constexpr float CHUTE_SET_DEG_DEFAULT     = 120.0f;  // loading position (1833 us)
constexpr float CHUTE_RELEASE_DEG_DEFAULT = 180.0f;  // OPEN: latch fully open (2500 us)
constexpr bool  CHUTE_BOOT_RESTORE        = true;
// In-air guard at boot (only when restoring a LOCK or SET latch).
constexpr float CHUTE_BOOT_STILL_MS       = 200.0f;
constexpr float CHUTE_BOOT_G_MIN          = 0.7f;   // |accel| in g
constexpr float CHUTE_BOOT_G_MAX          = 1.3f;
constexpr float CHUTE_BOOT_RATE_MAX_DPS   = 30.0f;
constexpr float CHUTE_CONFIRM_MS      = 1500.0f; // no chute seen this long after a release -> re-cycle
constexpr float CHUTE_RECYCLE_LOCK_MS = 400.0f;  // [MEASURE] dwell back at LOCK (servo travel time)
constexpr int   CHUTE_MAX_RELEASES    = 4;       // 3 slow re-cycles after the first release (+ the wiggle's), then hold RELEASE
// Fast wiggle for a stuck latch (latch-dud-study-2026-11-07, holdout winner
// 55<->110 at 375 deg/s): the first release goes to WIG_HIGH, then the latch
// strokes WIG_LOW / WIG_HIGH; CHUTE_WIG_CYCLES down-strokes, the last up-stroke
// going to OPEN. Each up-stroke counts as a release. Then the canopy check and
// the slow re-cycles above. Retries come at ~0.1 / 0.4 / 0.7 s instead of
// 1.9 s (from an 18 m apogee the ground comes ~1.9 s after the release).
// LOW and HIGH must straddle the angle where the latch lets the door go
// (owner: 70-90 deg) -- bench-check that HIGH frees the door with the spring
// loaded. Dwells = servo travel time (no feedback): (HIGH - LOCK) / 375 and
// (HIGH - LOW) / 375, rounded up to 5 ms.
constexpr int   CHUTE_WIG_CYCLES      = 2;       // 0 = off
constexpr float CHUTE_WIG_LOW_DEG     = 55.0f;
constexpr float CHUTE_WIG_HIGH_DEG    = 110.0f;
constexpr float CHUTE_WIG_FIRST_MS    = 190.0f;  // first stroke LOCK -> HIGH
constexpr float CHUTE_WIG_DOWN_MS     = 150.0f;  // HIGH -> LOW
constexpr float CHUTE_WIG_UP_MS       = 150.0f;  // LOW -> HIGH
// After any flight release (chute mode, abort, dud...), the latch returns to
// SET at touchdown or this long after the FIRST release, whichever comes
// first, and stays there for the rest of the flight. All re-cycles are over
// by then (the 6 releases end ~6.3 s after the first).
constexpr float CHUTE_STOW_AFTER_MS   = 7000.0f;
// Detection: |specific force| must rise this far above its value at release
// (freefall ~0 g, under canopy ~1 g plus an opening spike) and hold. Relative
// to the release baseline because drag on a fast, tumbling post-abort rocket
// can already read ~0.5 g.
constexpr float CHUTE_DETECT_RISE_G   = 0.4f;
constexpr float CHUTE_DETECT_MS       = 100.0f;

// ---------------------------------------------------------------------------
// Landing-leg release (FULL_LANDING only). A nichrome wire burns through the
// rubber band holding the legs. On LEGS_DELAY_MS after the landing-motor fire
// command (once ignition is confirmed), off when touchdown is detected.
// LEGS_BURN_MAX_MS is an independent actuator-level cutoff in case touchdown
// never comes (LANDING_BURN_MAX_MS forces it, so this should never trip).
// ---------------------------------------------------------------------------
constexpr float LEGS_DELAY_MS    = 0.0f;  // on at ignition confirm: the band must part <= 0.75 s later
constexpr float LEGS_BURN_MAX_MS = 6000.0f;

// ---------------------------------------------------------------------------
// Pyro channels (landing-motor e-match + leg-release nichrome)
// ---------------------------------------------------------------------------
constexpr float PYRO_FIRE_MS        = 1200.0f; // landing e-match gate-high duration
constexpr float CONT_THRESHOLD_V    = 0.4f;    // sense voltage above this = e-match/wire present
constexpr float CONT_DIVIDER_RATIO  = 11.0f;   // 100k over 10k: (100+10)/10; sense_v = drain volts

// ---------------------------------------------------------------------------
// Battery
// ---------------------------------------------------------------------------
constexpr float VBAT_DIVIDER = 6.0f;   // [MEASURE] 10k:2k -> Vbat = Vadc * 6.0
constexpr float VBAT_MIN     = 7.0f;   // refuse to arm below this (2S LiPo)
// Separate 2S pyro LiPo, read through the continuity dividers (a drain sits at
// pyro battery voltage when its load is connected). FULL_LANDING arm only.
constexpr float PYRO_VBAT_MIN = 7.0f;

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
constexpr float LOG_FAST_HZ = 100.0f;  // ARMED and in flight
constexpr float LOG_IDLE_HZ = 5.0f;    // disarmed on the bench
constexpr float LOG_FLUSH_MS = 500.0f;

// ---------------------------------------------------------------------------
// Misc firmware
// ---------------------------------------------------------------------------
constexpr bool ENABLE_WATCHDOG = true;
constexpr int  EEPROM_BASE_ADDR = 0;

} // namespace cfg
