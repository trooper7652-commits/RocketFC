#pragma once
//
// *** HIL BUILD ONLY — NOT FOR FLIGHT ***
//
// Hooks that tools/hil/make_hil.py patches into the build copies of
// src/hw/actuators.h and RocketFC.ino. They change nothing unless the HIL
// sensors are in LIVE mode (closed-loop bench sim, tools/hil/live/), and with
// both pyro channels live they change nothing at all: the bench then fires,
// checks continuity and reads the pyro battery exactly like the flight build.
//
//   landing e-match   simulated (default): the fire is timed and reported
//                     (pyroActive()) but the gate pin is NEVER driven.
//                     Live (`live ... 2|3`, the dashboard asks you to type
//                     FIRE): the real gate, exactly as in flight.
//   leg nichrome      simulated (default): never driven. Live (`live ... 1|3`,
//                     type LEGS): the real gate.
//   continuity/volts  a SIMULATED channel reads an intact load on a 7.8 V
//                     pyro battery, so `arm` works with nothing connected.
//                     A live channel reads the real hardware.
//   gimbal servos     every write is recorded (the angle the servos really
//                     got, at 50 Hz) so the PC's simulated nozzle follows the
//                     real servo command, not the 500 Hz core output.
//   reboot            `reboot` (bench, after a run) saves the chute latch
//                     position and marks the reset as planned, so the latch
//                     boots where it was instead of OPEN.
//
// Outside LIVE (normal CLI use, the static-fire PLAY flow) every hook passes
// straight through to the real hardware. `pyrotest` is never hooked.
//
#include <Arduino.h>
#include <EEPROM.h>

#include "../config.h"

// One translation unit (the sketch, or tools/hil/test/host_*.cpp), so plain
// definitions in a header are fine.
bool g_hilLive = false;      // HIL sensors are in LIVE (closed-loop sim)
bool g_hilLiveLegs = false;  // ... and the leg nichrome may really be powered
bool g_hilLiveLand = false;  // ... and the landing e-match may really be fired
uint32_t g_hilPeriodUs = 2000;  // the fast tick's period (the core's dt), set by the sketch

inline bool hilSimLegs() { return g_hilLive && !g_hilLiveLegs; }
inline bool hilSimLand() { return g_hilLive && !g_hilLiveLand; }

// The flight logic's leg-nichrome gate.
inline void hilLegsGate(int level) {
  if (!hilSimLegs() || level == LOW)
    digitalWrite(cfg::PIN_PYRO_LEGS, level);
}

// Simulated pyro-channel sense voltage (both channels read the pyro battery
// through an intact load).
constexpr float kHilSimPyroVolts = 7.8f;

// ---- gimbal servo writes (Actuators::writeGimbal) ----
float g_hilGimWrX = 0, g_hilGimWrY = 0;  // rad, the last angle sent to the servos
uint16_t g_hilServoWrites = 0;           // counts writes (telemetry)

// The angle a pulse really commands: the requested angle unless the pulse
// was clamped at the servo's travel limit, then the clamped pulse's angle.
inline float hilServoAngle(float rad, float us, float centerUs, float usPerDeg, float sign) {
  if (us <= cfg::SERVO_MIN_US || us >= cfg::SERVO_MAX_US)
    return (us - centerUs) / (usPerDeg * sign) * cfg::DEG2RAD;
  return rad;
}
inline void hilGimbalWritten(float gx, float gy, float usA, float usB, float trimA,
                             float trimB) {
  g_hilGimWrX = hilServoAngle(gx, usA, cfg::SERVO_A_CENTER_US + trimA,
                              cfg::SERVO_A_US_PER_GIMBAL_DEG, cfg::SERVO_A_SIGN);
  g_hilGimWrY = hilServoAngle(gy, usB, cfg::SERVO_B_CENTER_US + trimB,
                              cfg::SERVO_B_US_PER_GIMBAL_DEG, cfg::SERVO_B_SIGN);
  ++g_hilServoWrites;
}

// ---- planned reboot (bench) ----
// A marker in the EEPROM's last byte, outside ConfigStore's area: set by
// `reboot`, read and cleared at boot (the patched RocketFC.ino treats a
// planned reboot like a power-on, so the chute latch is restored).
constexpr uint8_t kHilRebootMarker = 0xB5;
inline int hilMarkerAddr() { return (int)EEPROM.length() - 1; }

inline bool hilPlannedReboot() {
  if (EEPROM.read(hilMarkerAddr()) != kHilRebootMarker) return false;
  EEPROM.write(hilMarkerAddr(), 0xFF);
  return true;
}

#if defined(__IMXRT1062__)
inline void hilReboot() {
  EEPROM.write(hilMarkerAddr(), kHilRebootMarker);
  SCB_AIRCR = 0x05FA0004;  // SYSRESETREQ
  while (true) {}
}
#else
void hostReboot();  // tools/hil/test/host_*.cpp
inline void hilReboot() {
  EEPROM.write(hilMarkerAddr(), kHilRebootMarker);
  hostReboot();
}
#endif
