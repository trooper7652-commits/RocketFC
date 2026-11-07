#pragma once
//
// *** HIL BUILD ONLY — NOT FOR FLIGHT ***
//
// Pyro / continuity hooks that tools/hil/make_hil.py patches into the build
// copy of src/hw/actuators.h. They change nothing unless the HIL sensors are
// in LIVE mode (closed-loop bench sim, tools/hil/live/):
//
//   landing e-match   in LIVE the gate pin is NEVER driven; the flight
//                     computer's fire is still timed and reported
//                     (pyroActive()), so the PC can light the simulated motor.
//   leg nichrome      in LIVE driven only with live legs on (`live ... 1`,
//                     which the dashboard sends only after you type LEGS).
//   continuity/volts  in LIVE with simulated pyros: both channels read as an
//                     intact e-match / nichrome on a 7.8 V pyro battery, so
//                     `arm` works with nothing on the landing channel and no
//                     pyro LiPo. With live legs the real readings are used.
//
// Outside LIVE (normal CLI use, the static-fire PLAY flow) every hook passes
// straight through to the real hardware. `pyrotest` is never hooked.
//
#include <Arduino.h>

#include "../config.h"

// One translation unit (the sketch, or tools/hil/test/host_*.cpp), so plain
// definitions in a header are fine.
bool g_hilLive = false;      // HIL sensors are in LIVE (closed-loop sim)
bool g_hilLiveLegs = false;  // ... and the leg nichrome may really be powered
uint32_t g_hilPeriodUs = 2000;  // the fast tick's period (the core's dt), set by the sketch

inline bool hilSimPyros() { return g_hilLive && !g_hilLiveLegs; }

// The flight logic's leg-nichrome gate.
inline void hilLegsGate(int level) {
  if (!g_hilLive || g_hilLiveLegs || level == LOW)
    digitalWrite(cfg::PIN_PYRO_LEGS, level);
}

// Simulated pyro-channel sense voltage (both channels read the pyro battery
// through an intact load).
constexpr float kHilSimPyroVolts = 7.8f;

// `reboot` on the HIL channel (only when not in flight): after a live run the
// core sits in TOUCHDOWN/ABORT until a restart. A software reset is not a
// power-on reset, so the chute latch boots OPEN ("unexpected reset") -- fine
// on the bench, where nothing is loaded; Prepare sends `chute lock` again.
#if defined(__IMXRT1062__)
inline void hilReboot() {
  SCB_AIRCR = 0x05FA0004;  // SYSRESETREQ
  while (true) {}
}
#else
void hostReboot();  // tools/hil/test/host_*.cpp
inline void hilReboot() { hostReboot(); }
#endif
