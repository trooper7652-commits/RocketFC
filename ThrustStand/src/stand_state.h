#pragma once
//
// Stand state machine and run statistics.
//
// Hardware-free on purpose: it consumes (time, thrust) samples and logical
// commands, and emits actions for the sketch to carry out. Nothing in here
// touches a pin, so the arming logic can be reasoned about -- and later
// replayed against recorded data -- without a stand attached.
//
// The arming path deliberately copies the two-step confirmation the flight
// computer uses for its pyro channels (`pyrotest 1` then `confirm 1` with a
// 10 s expiry, ../../src/hw/cli.h). Three separate, deliberate key presses
// stand between an idle stand and current through an igniter:
//
//   IDLE --EQ--> ARM_PENDING --EQ (within 10 s)--> ARMED --PLAY--> COUNTDOWN
//
// and any key at all during COUNTDOWN aborts. ARMED also times out on its own
// after DISARM_TIMEOUT_MS, because the most likely reason an armed stand goes
// quiet is that the operator walked out of IR range.
//
#include <Arduino.h>

#include "../config.h"
#include "ir_input.h"

enum class StandState : uint8_t {
  BOOT = 0,
  IDLE,
  TARING,
  CAL_WAIT,   // waiting for the reference mass + its grams over serial
  CAL_LOAD,   // averaging with the mass in place
  ARM_PENDING,
  ARMED,
  COUNTDOWN,
  FIRING,     // gate energised
  RECORDING,  // gate released, still capturing the burn
  SUMMARY,
  LEARN,
  FAULT,
};

enum class StandAction : uint8_t {
  NONE = 0,
  START_TARE,
  START_CAL,
  FIRE_GATE,
  SAFE_GATE,
  TOGGLE_STREAM,
  NEXT_PAGE,
  ENTER_LEARN,
  EXIT_LEARN,
};

// Run statistics, accumulated live so the LCD can show a summary with no PC
// attached. tools/plot_thrust.py recomputes all of these independently from
// the CSV; if the two ever disagree, trust the PC -- it has every sample.
struct RunStats {
  float peakN = 0;
  float impulseNs = 0;
  float avgN = 0;
  uint32_t fireMs = 0;        // gate high
  uint32_t onsetMs = 0;       // first sample above THRUST_TRIGGER_N
  uint32_t burnEndMs = 0;
  uint32_t burnTimeMs = 0;
  int16_t ignitionDelayMs = -1;  // onset - fire; -1 = never lit
  uint16_t sampleCount = 0;
  bool overload = false;
  bool lit = false;

  void reset() { *this = RunStats{}; }
};

// NAR/TRA impulse classes. Returns '-' below 1.26 N*s (below class A).
inline char narClass(float impulseNs) {
  if (impulseNs < 1.26f) return '-';
  float top = 2.5f;
  for (char c = 'A'; c <= 'O'; ++c) {
    if (impulseNs <= top) return c;
    top *= 2.0f;
  }
  return '?';
}

class StandMachine {
 public:
  StandState state() const { return state_; }
  const RunStats& stats() const { return stats_; }
  uint8_t countdownRemaining() const { return cdRemaining_; }
  const char* stateName() const { return name(state_); }

  static const char* name(StandState s) {
    switch (s) {
      case StandState::BOOT:        return "BOOT";
      case StandState::IDLE:        return "IDLE";
      case StandState::TARING:      return "TARE";
      case StandState::CAL_WAIT:    return "CALWAIT";
      case StandState::CAL_LOAD:    return "CALLOAD";
      case StandState::ARM_PENDING: return "ARMPEND";
      case StandState::ARMED:       return "ARMED";
      case StandState::COUNTDOWN:   return "COUNT";
      case StandState::FIRING:      return "FIRING";
      case StandState::RECORDING:   return "REC";
      case StandState::SUMMARY:     return "SUMMARY";
      case StandState::LEARN:       return "LEARN";
      default:                      return "FAULT";
    }
  }

  bool armed() const {
    return state_ == StandState::ARMED || state_ == StandState::COUNTDOWN ||
           state_ == StandState::FIRING;
  }
  // Whether the PC should be receiving full-rate rows.
  bool recording() const {
    return state_ == StandState::FIRING || state_ == StandState::RECORDING;
  }

  void begin(uint32_t ms) {
    state_ = StandState::IDLE;
    enteredMs_ = ms;
  }

  // Reason for the most recent abort/disarm, for the log and the LCD.
  const char* lastNote() const { return note_; }

  // -------------------------------------------------------------------------
  // Commands
  // -------------------------------------------------------------------------
  StandAction command(IrCmd c, uint32_t ms) {
    if (c == IrCmd::NONE) return StandAction::NONE;

    // Any key aborts a countdown. Checked before everything else so that even
    // an unmapped or misread key stops the sequence rather than being ignored.
    if (state_ == StandState::COUNTDOWN) {
      note_ = "ABORT_KEY";
      go_(StandState::IDLE, ms);
      return StandAction::SAFE_GATE;
    }

    if (state_ == StandState::LEARN) {
      if (c == IrCmd::LEARN || c == IrCmd::ABORT) {
        go_(StandState::IDLE, ms);
        return StandAction::EXIT_LEARN;
      }
      return StandAction::NONE;  // every other key is just a code to display
    }

    if (c == IrCmd::ABORT) {
      note_ = "DISARM_KEY";
      go_(StandState::IDLE, ms);
      return StandAction::SAFE_GATE;
    }
    if (c == IrCmd::PAGE) return StandAction::NEXT_PAGE;
    if (c == IrCmd::STREAM) return StandAction::TOGGLE_STREAM;

    switch (state_) {
      case StandState::IDLE:
      case StandState::SUMMARY:
        if (c == IrCmd::TARE) {
          go_(StandState::TARING, ms);
          return StandAction::START_TARE;
        }
        if (c == IrCmd::CALIBRATE) {
          // Enter TARING, not CAL_WAIT: calibration re-tares first (a scale
          // factor measured against a stale zero is wrong by exactly that
          // drift), and it must actually finish before we prompt for the
          // reference mass. Reaching CAL_WAIT is gated on the tare completing
          // -- see calWaitingForMass() -- so the operator can never type a
          // mass early and race LoadCell::startCalLoad() into cutting off an
          // in-progress tare accumulation.
          go_(StandState::TARING, ms);
          return StandAction::START_CAL;
        }
        if (c == IrCmd::LEARN) {
          go_(StandState::LEARN, ms);
          return StandAction::ENTER_LEARN;
        }
        if (c == IrCmd::ARM) {
          note_ = "ARM_PENDING";
          go_(StandState::ARM_PENDING, ms);
        }
        return StandAction::NONE;

      case StandState::ARM_PENDING:
        // Second EQ within the window completes the arm. Anything else is
        // treated as a change of mind and drops back to idle.
        if (c == IrCmd::ARM) {
          note_ = "ARMED";
          go_(StandState::ARMED, ms);
        } else {
          note_ = "ARM_CANCEL";
          go_(StandState::IDLE, ms);
        }
        return StandAction::SAFE_GATE;

      case StandState::ARMED:
        if (c == IrCmd::FIRE) {
          cdRemaining_ = stand::COUNTDOWN_SECONDS;
          cdTickMs_ = ms;
          go_(StandState::COUNTDOWN, ms);
        }
        return StandAction::NONE;

      default:
        return StandAction::NONE;
    }
  }

  // Serial-side equivalents so the stand is fully operable from a keyboard if
  // the remote dies mid-session.
  StandAction commandFromSerial(IrCmd c, uint32_t ms) { return command(c, ms); }

  void calAccepted(uint32_t ms) { go_(StandState::CAL_LOAD, ms); }
  void calFinished(uint32_t ms) { go_(StandState::IDLE, ms); }
  // The pre-cal tare has actually finished; now it's safe to prompt for and
  // accept the reference mass.
  void calWaitingForMass(uint32_t ms) { go_(StandState::CAL_WAIT, ms); }
  void tareFinished(uint32_t ms) {
    if (state_ == StandState::TARING) go_(StandState::IDLE, ms);
  }
  void fault(const char* why, uint32_t ms) {
    note_ = why;
    go_(StandState::FAULT, ms);
  }
  void clearFault(uint32_t ms) {
    if (state_ == StandState::FAULT) go_(StandState::IDLE, ms);
  }

  // -------------------------------------------------------------------------
  // Time-driven transitions. Returns an action for the sketch to perform.
  // -------------------------------------------------------------------------
  StandAction tick(uint32_t ms) {
    switch (state_) {
      case StandState::ARM_PENDING:
        if (ms - enteredMs_ >= stand::ARM_CONFIRM_MS) {
          note_ = "ARM_EXPIRED";
          go_(StandState::IDLE, ms);
          return StandAction::SAFE_GATE;
        }
        break;

      case StandState::ARMED:
        // The operator has most likely walked out of IR range.
        if (ms - enteredMs_ >= stand::DISARM_TIMEOUT_MS) {
          note_ = "AUTO_DISARM";
          go_(StandState::IDLE, ms);
          return StandAction::SAFE_GATE;
        }
        break;

      case StandState::COUNTDOWN:
        if (ms - cdTickMs_ >= 1000) {
          cdTickMs_ += 1000;
          if (cdRemaining_ > 0) --cdRemaining_;
          if (cdRemaining_ == 0) {
            stats_.reset();
            stats_.fireMs = ms;
            go_(StandState::FIRING, ms);
            return StandAction::FIRE_GATE;
          }
        }
        break;

      case StandState::FIRING:
        // Mirrors the igniter's own hard cap; whichever notices first wins.
        if (ms - enteredMs_ >= stand::FIRE_PULSE_MS) {
          go_(StandState::RECORDING, ms);
          return StandAction::SAFE_GATE;
        }
        break;

      case StandState::RECORDING:
        if (!stats_.lit && ms - stats_.fireMs >= stand::NO_IGNITION_MS) {
          note_ = "NO_IGNITION";
          finish_(ms);
        } else if (ms - stats_.fireMs >= stand::RECORD_MAX_MS) {
          note_ = "REC_TIMEOUT";
          finish_(ms);
        } else if (stats_.lit && belowMs_ != 0 &&
                   ms - belowMs_ >= stand::BURN_END_HOLD_MS) {
          note_ = "BURNOUT";
          stats_.burnEndMs = belowMs_;
          finish_(ms);
        }
        break;

      default:
        break;
    }
    return StandAction::NONE;
  }

  // -------------------------------------------------------------------------
  // Sample intake. Only accumulates while capturing a run.
  // -------------------------------------------------------------------------
  void sample(uint32_t ms, float thrustN, bool overload) {
    if (!recording()) return;

    if (thrustN > stats_.peakN) stats_.peakN = thrustN;
    if (overload) stats_.overload = true;
    stats_.sampleCount++;

    if (!stats_.lit && thrustN >= stand::THRUST_TRIGGER_N) {
      stats_.lit = true;
      stats_.onsetMs = ms;
      const uint32_t d = ms - stats_.fireMs;
      stats_.ignitionDelayMs = d > 32000 ? 32000 : (int16_t)d;
    }

    // Trapezoidal integration, matching Motor.impulse() in
    // ../../tools/burn_table.py. Negative excursions (ringing, recoil) are
    // clamped so they cannot subtract from total impulse.
    if (stats_.lit) {
      const float f = thrustN > 0.0f ? thrustN : 0.0f;
      if (haveLast_) {
        const float dt = (ms - lastMs_) * 0.001f;
        if (dt > 0.0f && dt < 1.0f) {
          stats_.impulseNs += 0.5f * (f + lastF_) * dt;
        }
      }
      // Burn end: below 5% of the running peak, sustained. Using the running
      // peak means the threshold is slightly low early on, which is the safe
      // direction -- it cannot end the burn prematurely.
      const float endThresh = stats_.peakN * stand::BURN_END_FRAC;
      if (f < endThresh || f < stand::THRUST_TRIGGER_N) {
        if (belowMs_ == 0) belowMs_ = ms;
      } else {
        belowMs_ = 0;
      }
    }
    lastF_ = thrustN > 0.0f ? thrustN : 0.0f;
    lastMs_ = ms;
    haveLast_ = true;
  }

 private:
  void go_(StandState s, uint32_t ms) {
    state_ = s;
    enteredMs_ = ms;
    if (s == StandState::FIRING) {
      belowMs_ = 0;
      haveLast_ = false;
    }
  }

  void finish_(uint32_t ms) {
    if (stats_.burnEndMs == 0) stats_.burnEndMs = ms;
    if (stats_.lit && stats_.burnEndMs > stats_.onsetMs) {
      stats_.burnTimeMs = stats_.burnEndMs - stats_.onsetMs;
      const float secs = stats_.burnTimeMs * 0.001f;
      stats_.avgN = secs > 0.0f ? stats_.impulseNs / secs : 0.0f;
    }
    go_(StandState::SUMMARY, ms);
  }

  StandState state_ = StandState::BOOT;
  RunStats stats_;
  const char* note_ = "";
  uint32_t enteredMs_ = 0, cdTickMs_ = 0, belowMs_ = 0, lastMs_ = 0;
  float lastF_ = 0;
  uint8_t cdRemaining_ = 0;
  bool haveLast_ = false;
};
