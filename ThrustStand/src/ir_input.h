#pragma once
//
// IR remote -> logical commands (21-key "car MP3" remote, NEC, address 0x00).
//
// Two things matter here and both are safety-relevant:
//
//  1. NEC REPEAT FRAMES ARE DISCARDED. Holding a key down makes the remote
//     emit a repeat frame every ~110 ms. If those reached the state machine,
//     resting your thumb on the fire key would re-trigger ignition, and a
//     single held key could walk IDLE -> ARM -> ARM -> FIRE in under half a
//     second. Only genuine key-down events are delivered.
//
//  2. THE CODE TABLE IS A GUESS UNTIL YOU CHECK IT. Clone remotes with
//     identical printing ship with different codes. Learn mode prints every
//     received address/command so you can verify config.h before the fire key
//     is ever pointed at an igniter.
//
// Requires IRremote 4.x from the Library Manager. Sketches on the internet
// written for 2.x/3.x will not compile against it -- the decode API changed.
//
#include <Arduino.h>

// These must be defined BEFORE IRremote.hpp -- it is a header-only library
// that compiles itself according to them, and the defaults are expensive on a
// 2 KB / 32 KB part:
//   DECODE_NEC only            -- the stock library decodes every protocol it
//                                 knows, costing several KB of flash we do not
//                                 have. This remote is NEC.
//   RAW_BUFFER_LENGTH 100      -- default 200 entries of uint16_t = 400 bytes
//                                 of RAM. An NEC frame is ~68 entries.
//   NO_LED_FEEDBACK_CODE       -- D13 is our status LED, not an IR tell-tale.
#define DECODE_NEC
#define RAW_BUFFER_LENGTH 100
#define NO_LED_FEEDBACK_CODE
#define EXCLUDE_EXOTIC_PROTOCOLS
#include <IRremote.hpp>

#include "../config.h"

enum class IrCmd : uint8_t {
  NONE = 0,
  ARM,        // EQ
  FIRE,       // >||
  ABORT,      // CH-
  TARE,       // 1
  CALIBRATE,  // 2
  STREAM,     // 3
  PAGE,       // CH+
  LEARN,      // 0
  UNKNOWN,    // a valid NEC frame we have no mapping for
};

class IrInput {
 public:
  void begin() {
    IrReceiver.begin(stand::PIN_IR_RECV, DISABLE_LED_FEEDBACK);
  }

  // Poll every loop. Returns a command on the pass where a key went down.
  IrCmd poll() {
    if (!IrReceiver.decode()) return IrCmd::NONE;

    const uint8_t cmd = IrReceiver.decodedIRData.command;
    const uint16_t addr = IrReceiver.decodedIRData.address;
    const bool repeat =
        (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) != 0;
    const bool junk =
        (IrReceiver.decodedIRData.protocol == UNKNOWN) ||
        (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_PARITY_FAILED) != 0;
    IrReceiver.resume();

    lastAddr_ = addr;
    lastCmd_ = cmd;
    lastWasRepeat_ = repeat;
    if (junk) return IrCmd::NONE;

    // Repeat frames never produce a command. See note 1 above.
    if (repeat) return IrCmd::NONE;

    lastKeyMs_ = millis();
    haveKey_ = true;
    return map_(cmd);
  }

  // Learn mode: poll() keeps returning each key's real mapped command (or
  // UNKNOWN for an unmapped one) rather than collapsing everything to LEARN.
  // That matters: StandMachine::command() only exits LEARN on the LEARN or
  // ABORT command and otherwise ignores commands while state == LEARN, so
  // every other key is displayed by the caller (via learn()) and otherwise
  // has no effect -- letting you press through the whole remote in one pass
  // instead of exiting after the first key.
  void setLearn(bool on) { learn_ = on; }
  bool learn() const { return learn_; }

  uint8_t lastCmd() const { return lastCmd_; }
  uint16_t lastAddr() const { return lastAddr_; }
  bool lastWasRepeat() const { return lastWasRepeat_; }
  bool haveKey() const { return haveKey_; }
  uint32_t lastKeyMs() const { return lastKeyMs_; }

 private:
  static IrCmd map_(uint8_t c) {
    switch (c) {
      case stand::IR_EQ:       return IrCmd::ARM;
      case stand::IR_PLAY:     return IrCmd::FIRE;
      case stand::IR_CH_MINUS: return IrCmd::ABORT;
      case stand::IR_1:        return IrCmd::TARE;
      case stand::IR_2:        return IrCmd::CALIBRATE;
      case stand::IR_3:        return IrCmd::STREAM;
      case stand::IR_CH_PLUS:  return IrCmd::PAGE;
      case stand::IR_0:        return IrCmd::LEARN;
      default:                 return IrCmd::UNKNOWN;
    }
  }

  uint32_t lastKeyMs_ = 0;
  uint16_t lastAddr_ = 0;
  uint8_t lastCmd_ = 0;
  bool lastWasRepeat_ = false, haveKey_ = false, learn_ = false;
};
