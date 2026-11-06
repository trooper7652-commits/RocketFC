#pragma once
//
// Bench-only settings.
//
// Everything that must match the flight vehicle -- pin assignments, IMU_R_SB,
// servo signs/scales, PID gains -- lives in the vendored config.h and is NOT
// duplicated here. This file holds only knobs that exist because this is a
// bench rig rather than a rocket.
//
#include "config.h"

namespace bench {

// Attitude telemetry rate. 50 Hz is smooth to the eye and costs ~6 kB/s, which
// is nothing over USB CDC (the Teensy ignores the nominal 115200 baud and runs
// at USB speed). Raising this mostly just wastes host CPU on parsing.
constexpr float VIZ_HZ = 50.0f;

// Stream on from boot, so `python rocket_viz.py --port COMx` works without
// typing anything into a serial terminal first. `viz off` stops it.
constexpr bool VIZ_ON_AT_BOOT = true;

// Accel leveling blend applied every fast tick. The bench is always "on the
// pad", so unlike flight we never stop correcting. This is the same alpha the
// flight code uses while IDLE/ARMED (FlightCore::step), giving a ~1 s time
// constant at 500 Hz: tilt self-corrects, heading still drifts freely because
// there is no magnetometer.
constexpr float LEVEL_BLEND_ALPHA = 0.002f;

// TVC bench modes, selected at runtime with the `tvc` command.
enum class TvcMode : uint8_t {
  OFF = 0,    // controller not running; servos held centered
  CALC = 1,   // controller runs, values streamed, servos held centered
  DRIVE = 2,  // controller runs AND drives the gimbal servos
};

// Gyro-cal / ground-reference procedures give up after this long. The usual
// cause of a timeout is that the vehicle was not held still.
constexpr uint32_t PROC_TIMEOUT_MS = 20000;

}  // namespace bench
