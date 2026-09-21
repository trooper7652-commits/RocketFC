#pragma once
//
// 16x2 HD44780 front panel.
//
// Two rules, both about not stalling the sample loop:
//
//  1. NEVER call lcd.clear() or lcd.home() in the update path. Both issue the
//     HD44780 "clear display" command, which the controller takes ~1.6 ms to
//     execute and the library covers with a blocking delay. At 80 SPS the
//     whole sample budget is 12.5 ms. Instead every row is rendered into a
//     fixed 16-character buffer, space-padded, and written over the top of the
//     old one -- constant time, no flicker, no blanking.
//
//  2. Rows are only pushed to the panel when their text actually changed.
//     A full 16-character write is ~0.7 ms; skipping unchanged rows keeps the
//     typical pass near zero.
//
// snprintf here never uses %f. The Arduino AVR core links the reduced printf
// from avr-libc, which silently prints NOTHING for float conversions -- a
// classic way to end up with a blank thrust display. Floats go through
// dtostrf() into a small buffer first.
//
#include <Arduino.h>
#include <LiquidCrystal.h>

#include "../config.h"
#include "load_cell.h"
#include "stand_state.h"

enum class LcdPage : uint8_t { LIVE = 0, STATS, INFO, PAGE_COUNT };

class UiLcd {
 public:
  UiLcd()
      : lcd_(stand::PIN_LCD_RS, stand::PIN_LCD_EN, stand::PIN_LCD_D4,
             stand::PIN_LCD_D5, stand::PIN_LCD_D6, stand::PIN_LCD_D7) {}

  void begin() {
    lcd_.begin(16, 2);
    lcd_.clear();  // the one legal clear: once, at boot
    show(0, "ThrustStand");
    show(1, "booting...");
  }

  void nextPage() {
    page_ = (LcdPage)(((uint8_t)page_ + 1) % (uint8_t)LcdPage::PAGE_COUNT);
  }

  void render(const StandMachine& sm, const LoadCell& cell, bool streaming) {
    char a[17], b[17], num[12];

    switch (sm.state()) {
      case StandState::TARING:
      case StandState::CAL_LOAD:
        snprintf(a, sizeof(a), "%s %u/%u",
                 sm.state() == StandState::TARING ? "TARE" : "CAL",
                 (unsigned)cell.progress(), (unsigned)cell.progressTarget());
        bar(b, cell.progress(), cell.progressTarget());
        break;

      case StandState::CAL_WAIT:
        snprintf(a, sizeof(a), "PUT MASS ON CELL");
        snprintf(b, sizeof(b), "type grams->USB");
        break;

      case StandState::ARM_PENDING:
        snprintf(a, sizeof(a), "ARM? PRESS EQ");
        snprintf(b, sizeof(b), "AGAIN TO CONFIRM");
        break;

      case StandState::ARMED:
        snprintf(a, sizeof(a), "*** ARMED ***");
        snprintf(b, sizeof(b), "PLAY=FIRE CH-=NO");
        break;

      case StandState::COUNTDOWN:
        snprintf(a, sizeof(a), "FIRING IN %u", (unsigned)sm.countdownRemaining());
        snprintf(b, sizeof(b), "ANY KEY = ABORT");
        break;

      case StandState::FIRING:
        snprintf(a, sizeof(a), "!! IGNITION !!");
        fmt(num, sizeof(num), cell.thrustFiltN(), 1);
        snprintf(b, sizeof(b), "%s N", num);
        break;

      case StandState::RECORDING:
        fmt(num, sizeof(num), cell.thrustFiltN(), 1);
        snprintf(a, sizeof(a), "REC %s N", num);
        fmt(num, sizeof(num), sm.stats().peakN, 1);
        snprintf(b, sizeof(b), "peak %s N", num);
        break;

      case StandState::LEARN:
        snprintf(a, sizeof(a), "IR LEARN MODE");
        snprintf(b, sizeof(b), "press keys; 0=x");
        break;

      case StandState::FAULT:
        snprintf(a, sizeof(a), "FAULT");
        snprintf(b, sizeof(b), "%s", sm.lastNote());
        break;

      default:
        renderPage(sm, cell, streaming, a, b, num);
        break;
    }
    show(0, a);
    show(1, b);
  }

  // Learn mode needs to display a raw code the instant it arrives.
  void showCode(uint16_t addr, uint8_t cmd) {
    char b[17];
    snprintf(b, sizeof(b), "a%02X cmd 0x%02X", (unsigned)addr, (unsigned)cmd);
    show(1, b);
  }

 private:
  void renderPage(const StandMachine& sm, const LoadCell& cell, bool streaming,
                  char* a, char* b, char* num) {
    const RunStats& st = sm.stats();
    switch (page_) {
      case LcdPage::STATS: {
        char imp[12];
        fmt(num, 12, st.peakN, 1);
        fmt(imp, sizeof(imp), st.impulseNs, 1);
        snprintf(a, 17, "pk%sN I%s", num, imp);
        fmt(num, 12, st.burnTimeMs * 0.001f, 2);
        snprintf(b, 17, "%ss %c dly%d", num, narClass(st.impulseNs),
                 (int)st.ignitionDelayMs);
        break;
      }

      case LcdPage::INFO:
        snprintf(a, 17, "%d SPS %s", (int)(cell.sampleRateHz() + 0.5f),
                 cell.calibrated() ? "CAL" : "UNCAL");
        fmt(num, 12, cell.noiseN(), 3);
        snprintf(b, 17, "noise %sN", num);
        break;

      default:  // LIVE
        fmt(num, 12, cell.thrustFiltN(), 2);
        snprintf(a, 17, "%s N%s", num, cell.calibrated() ? "" : " UNCAL");
        snprintf(b, 17, "%s%s%s", sm.stateName(), streaming ? " REC" : "",
                 cell.overloaded() ? " OVLD!" : "");
        break;
    }
  }

  // Left-aligned, space-padded to exactly 16 chars, written only if changed.
  void show(uint8_t row, const char* text) {
    char padded[17];
    snprintf(padded, sizeof(padded), "%-16s", text);
    if (strcmp(padded, cache_[row]) == 0) return;
    strcpy(cache_[row], padded);
    lcd_.setCursor(0, row);
    lcd_.print(padded);
  }

  // %f is unavailable in the AVR core's printf -- see the file header.
  static void fmt(char* out, size_t n, float v, uint8_t prec) {
    dtostrf(v, 1, prec, out);
    out[n - 1] = '\0';
  }

  static void bar(char* out, uint16_t done, uint16_t total) {
    const uint8_t filled = total ? (uint8_t)((uint32_t)done * 16 / total) : 0;
    for (uint8_t i = 0; i < 16; ++i) out[i] = i < filled ? '#' : '.';
    out[16] = '\0';
  }

  LiquidCrystal lcd_;
  char cache_[2][17] = {{0}, {0}};
  LcdPage page_ = LcdPage::LIVE;
};
