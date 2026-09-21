//
// ThrustStand — static thrust test stand for model rocket motors.
//
// Arduino Uno/Nano + HX711 + 5 kg bar load cell + 16x2 HD44780 + logic-level
// MOSFET igniter gate + IR remote. Streams a CSV thrust curve over USB for
// tools/stand_capture.py to plot live and archive.
//
// Read ThrustStand/README.md section 2 (Safety) before wiring anything, and
// WIRING.md before soldering. The short version: the MOSFET gate needs a 10k
// pulldown to ground, and you test the fire channel with a lamp, never with an
// e-match.
//
// Loop structure is the cooperative scheduler from ../RocketFC.ino: no delays,
// no blocking calls, everything polls. At 80 SPS the whole budget between
// samples is 12.5 ms, and the LCD alone would blow it if it ever called
// clear().
//
#include "config.h"
#include "src/cal_store.h"
#include "src/igniter.h"
#include "src/ir_input.h"
#include "src/load_cell.h"
#include "src/stand_state.h"
#include "src/telemetry.h"
#include "src/ui_lcd.h"

static Igniter igniter;
static LoadCell cell;
static CalStore store;
static IrInput ir;
static StandMachine machine;
static Telemetry tlm;
static UiLcd ui;

static bool streamOn_ = false;
static bool calPending_ = false;   // a `cal` is waiting on its tare to finish
static uint32_t lastLcdMs_ = 0, lastIdleRowMs_ = 0, lastStatusMs_ = 0;
static uint32_t loopMaxUs_ = 0, loopStartUs_ = 0;
static uint16_t runNumber_ = 0;
static char line_[24];
static uint8_t lineLen_ = 0;

// ---------------------------------------------------------------------------
// Buzzer / LED — stateless patterns computed from time alone, as in
// ../src/hw/status.h, so the sequencer can never get stuck mid-pattern.
// The buzzer must be an ACTIVE type: tone() would take Timer2 away from
// IRremote and silently kill the abort key. See config.h.
// ---------------------------------------------------------------------------
static bool pattern(uint32_t ms, uint32_t periodMs, uint8_t count,
                    uint16_t onMs, uint16_t gapMs) {
  const uint32_t p = ms % periodMs;
  const uint32_t slot = (uint32_t)onMs + gapMs;
  if (count == 0 || slot == 0) return false;
  if (p >= slot * count) return false;
  return (p % slot) < onMs;
}

static void statusTick(uint32_t ms) {
  bool buzz = false, led = false;
  switch (machine.state()) {
    case StandState::ARM_PENDING:
      buzz = pattern(ms, 400, 1, 60, 0);
      led = (ms % 200) < 100;
      break;
    case StandState::ARMED:
      buzz = pattern(ms, 2500, 2, 90, 180);
      led = (ms % 400) < 200;
      break;
    case StandState::COUNTDOWN:
      // One beep per second, then a solid tone through the final second.
      buzz = machine.countdownRemaining() <= 1 ? true : pattern(ms, 1000, 1, 120, 0);
      led = true;
      break;
    case StandState::FIRING:
    case StandState::RECORDING:
      buzz = false;  // silent: you want to hear the motor, not the stand
      led = (ms % 200) < 100;
      break;
    case StandState::SUMMARY:
      buzz = pattern(ms, 4000, 3, 120, 120);
      led = (ms % 1000) < 500;
      break;
    case StandState::FAULT:
      buzz = pattern(ms, 1000, 3, 80, 80);
      led = (ms % 150) < 75;
      break;
    default:
      buzz = cell.fault() ? pattern(ms, 2000, 2, 100, 150) : false;
      led = (ms % 2000) < 60;
      break;
  }
  digitalWrite(stand::PIN_BUZZER,
               (buzz == stand::BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
  digitalWrite(stand::PIN_LED, led ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// Actions requested by the state machine
// ---------------------------------------------------------------------------
static void runAction(StandAction a, uint32_t ms) {
  switch (a) {
    case StandAction::FIRE_GATE: {
      const uint32_t t = igniter.fire(ms);
      tlm.event(t, "FIRE", 0.0f);
      break;
    }
    case StandAction::SAFE_GATE:
      igniter.safe();
      tlm.event(ms, "SAFE", 0.0f);
      break;
    case StandAction::START_TARE:
      cell.startTare();
      tlm.note("taring, keep the stand still");
      break;
    case StandAction::START_CAL:
      // Calibration always re-tares first: a scale factor measured against a
      // stale zero is wrong by exactly that drift.
      calPending_ = true;
      cell.startTare();
      tlm.note("cal: taring empty cell, do not touch the stand");
      break;
    case StandAction::TOGGLE_STREAM:
      streamOn_ = !streamOn_;
      tlm.note(streamOn_ ? "stream on" : "stream off");
      break;
    case StandAction::NEXT_PAGE:
      ui.nextPage();
      break;
    case StandAction::ENTER_LEARN:
      ir.setLearn(true);
      tlm.note("IR learn mode: press keys, 0 or CH- exits");
      break;
    case StandAction::EXIT_LEARN:
      ir.setLearn(false);
      tlm.note("IR learn mode off");
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Serial CLI — everything the remote can do, so a dead remote mid-session
// does not strand you. Non-blocking accumulate, as in ../src/hw/cli.h.
// ---------------------------------------------------------------------------
static void printHelp() {
  Serial.println(F("# commands: help status tare cal <grams> arm fire abort"));
  Serial.println(F("#           stream on|off learn page save"));
  Serial.println(F("# arm twice to reach ARMED, then fire. abort = any time."));
}

static void printStatus(uint32_t ms) {
  const RunStats& s = machine.stats();
  Serial.print(F("# state="));
  Serial.print(machine.stateName());
  Serial.print(F(" rate="));
  Serial.print(cell.sampleRateHz(), 1);
  Serial.print(F(" cal="));
  Serial.print(cell.calibrated() ? F("yes") : F("NO"));
  Serial.print(F(" cpn="));
  Serial.print(cell.countsPerN(), 1);
  Serial.print(F(" tare="));
  Serial.print(cell.tareOffset());
  Serial.print(F(" thrust="));
  Serial.print(cell.thrustN(), 3);
  Serial.print(F(" peak="));
  Serial.print(s.peakN, 2);
  Serial.print(F(" imp="));
  Serial.print(s.impulseNs, 2);
  Serial.print(F(" loopmax_us="));
  Serial.println(loopMaxUs_);
  (void)ms;
}

static void handleLine(char* s, uint32_t ms) {
  while (*s == ' ') ++s;
  if (*s == '\0') return;

  // A bare number is the reference mass, answering the calibration prompt.
  if ((*s >= '0' && *s <= '9') || *s == '.') {
    const float grams = atof(s);
    if (machine.state() != StandState::CAL_WAIT) {
      tlm.note("not waiting for a mass; type `cal` first");
      return;
    }
    if (!cell.startCalLoad(grams)) {
      tlm.note("mass out of range (20..5000 g)");
      return;
    }
    machine.calAccepted(ms);
    Serial.print(F("# calibrating against "));
    Serial.print(grams, 1);
    Serial.println(F(" g"));
    return;
  }

  if (!strcmp(s, "help")) return printHelp();
  if (!strcmp(s, "status")) return printStatus(ms);
  if (!strcmp(s, "save")) {
    store.setCal(cell.tareOffset(), cell.countsPerN(), cell.calibrated());
    tlm.note("calibration saved to EEPROM");
    return;
  }
  if (!strcmp(s, "stream on")) { streamOn_ = true; return tlm.note("stream on"); }
  if (!strcmp(s, "stream off")) { streamOn_ = false; return tlm.note("stream off"); }

  IrCmd c = IrCmd::NONE;
  if (!strcmp(s, "tare")) c = IrCmd::TARE;
  else if (!strncmp(s, "cal", 3)) c = IrCmd::CALIBRATE;
  else if (!strcmp(s, "arm")) c = IrCmd::ARM;
  else if (!strcmp(s, "fire")) c = IrCmd::FIRE;
  else if (!strcmp(s, "abort") || !strcmp(s, "disarm")) c = IrCmd::ABORT;
  else if (!strcmp(s, "learn")) c = IrCmd::LEARN;
  else if (!strcmp(s, "page")) c = IrCmd::PAGE;
  else if (!strcmp(s, "stream")) c = IrCmd::STREAM;

  if (c == IrCmd::NONE) return tlm.note("unknown command; try `help`");
  runAction(machine.commandFromSerial(c, ms), ms);
}

static void pollSerial(uint32_t ms) {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line_[lineLen_] = '\0';
      handleLine(line_, ms);
      lineLen_ = 0;
      continue;
    }
    if (lineLen_ < sizeof(line_) - 1) line_[lineLen_++] = ch;
  }
}

// ---------------------------------------------------------------------------
static void printSummary() {
  const RunStats& s = machine.stats();
  Serial.println(F("#--- run summary ---"));
  Serial.print(F("# peak_n="));
  Serial.println(s.peakN, 2);
  Serial.print(F("# avg_n="));
  Serial.println(s.avgN, 2);
  Serial.print(F("# impulse_ns="));
  Serial.println(s.impulseNs, 2);
  Serial.print(F("# burn_time_s="));
  Serial.println(s.burnTimeMs * 0.001f, 3);
  Serial.print(F("# nar_class="));
  Serial.println(narClass(s.impulseNs));
  Serial.print(F("# ignition_delay_ms="));
  Serial.println(s.ignitionDelayMs);
  Serial.print(F("# overload="));
  Serial.println(s.overload ? 1 : 0);
  Serial.print(F("# note="));
  Serial.println(machine.lastNote());
  if (!cell.calibrated()) {
    Serial.println(F("# WARNING: stand is UNCALIBRATED, forces are estimates"));
  }
  if (s.overload) {
    Serial.println(F("# WARNING: load cell overload, peak is not trustworthy"));
  }
}

void setup() {
  // FIRST. Before serial, before the LCD, before anything that can take time.
  igniter.safeInit();

  pinMode(stand::PIN_BUZZER, OUTPUT);
  pinMode(stand::PIN_LED, OUTPUT);
  digitalWrite(stand::PIN_BUZZER, stand::BUZZER_ACTIVE_HIGH ? LOW : HIGH);
  if (stand::REQUIRE_ARM_SWITCH) pinMode(stand::PIN_ARM_SWITCH, INPUT_PULLUP);

  tlm.begin();
  store.load();
  cell.begin();
  cell.applyCal(store.tareOffset(), store.countsPerN(), store.calibrated());
  ui.begin();
  ir.begin();

  runNumber_ = store.nextRunNumber();
  const uint32_t ms = millis();
  machine.begin(ms);

  tlm.header(cell.calibrated(), cell.countsPerN(), cell.tareOffset(),
             cell.sampleRateHz(), runNumber_);
  printHelp();
  if (!cell.calibrated()) {
    Serial.println(F("# UNCALIBRATED — run `cal` before trusting any newtons"));
  }
}

void loop() {
  loopStartUs_ = micros();
  const uint32_t ms = millis();

  igniter.tick(ms);  // hard gate cap, first and every pass

  // --- load cell -----------------------------------------------------------
  if (cell.tick(ms)) {
    machine.sample(ms, cell.thrustN(), cell.overloaded());
    const bool full = machine.armed() || machine.recording() || streamOn_;
    if (full) {
      tlm.row(ms, machine.stateName(), cell.raw(), cell.thrustN(),
              cell.thrustFiltN());
    } else if (ms - lastIdleRowMs_ >= (uint32_t)(1000.0f / stand::IDLE_HZ)) {
      lastIdleRowMs_ = ms;
      tlm.row(ms, machine.stateName(), cell.raw(), cell.thrustN(),
              cell.thrustFiltN());
    }
  }

  // Tare / calibration completion
  if (cell.tareComplete()) {
    cell.clearFlags();
    store.setTare(cell.tareOffset());
    tlm.event(ms, "TARE", (float)cell.tareOffset());
    Serial.print(F("# tare="));
    Serial.print(cell.tareOffset());
    Serial.print(F(" noise_n="));
    Serial.println(cell.noiseN(), 4);
    if (calPending_) {
      calPending_ = false;
      cell.awaitCalMass();
      machine.calWaitingForMass(ms);  // only now is a mass entry accepted
      Serial.println(F("# place the reference mass on the cell, then type its"));
      Serial.println(F("# mass in GRAMS and press enter (e.g. 500)"));
    } else {
      machine.tareFinished(ms);
    }
  }
  if (cell.calComplete()) {
    cell.clearFlags();
    store.setCal(cell.tareOffset(), cell.countsPerN(), cell.calibrated());
    machine.calFinished(ms);
    Serial.print(F("# counts_per_n="));
    Serial.print(cell.countsPerN(), 2);
    Serial.println(cell.calibrated() ? F(" OK, saved") : F(" FAILED — no load seen"));
  }

  // --- inputs --------------------------------------------------------------
  const IrCmd c = ir.poll();
  if (c != IrCmd::NONE) {
    if (ir.learn()) {
      Serial.print(F("# IR addr=0x"));
      Serial.print(ir.lastAddr(), HEX);
      Serial.print(F(" command=0x"));
      Serial.println(ir.lastCmd(), HEX);
      ui.showCode(ir.lastAddr(), ir.lastCmd());
    }
    runAction(machine.command(c, ms), ms);
  }
  pollSerial(ms);

  // --- timed transitions ---------------------------------------------------
  const StandState before = machine.state();
  runAction(machine.tick(ms), ms);
  if (before != machine.state()) {
    tlm.event(ms, machine.stateName(), 0.0f);
    if (machine.state() == StandState::SUMMARY) printSummary();
  }

  // Refuse to stay armed if the load cell has stopped talking: with no thrust
  // signal a run is worthless, and a silent HX711 usually means a wiring
  // failure that may involve the same harness as the igniter.
  if (cell.fault() && machine.armed()) {
    machine.fault("HX711_LOST", ms);
    igniter.safe();
    tlm.event(ms, "FAULT_HX711", 0.0f);
  }

  // --- outputs -------------------------------------------------------------
  if (ms - lastLcdMs_ >= (uint32_t)(1000.0f / stand::LCD_HZ)) {
    lastLcdMs_ = ms;
    ui.render(machine, cell, streamOn_ || machine.recording());
  }
  if (ms - lastStatusMs_ >= (uint32_t)(1000.0f / stand::STATUS_HZ)) {
    lastStatusMs_ = ms;
    statusTick(ms);
  }

  const uint32_t dur = micros() - loopStartUs_;
  if (dur > loopMaxUs_) loopMaxUs_ = dur;
}
