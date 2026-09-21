#pragma once
//
// ThrustStand — master configuration. Single source of truth for pins, rates,
// load-cell scaling, ignition timing, and IR key codes.
//
// This header is PURE C++ (no Arduino includes) so the numbers can be shared
// with host-side tools without dragging in the core. Note it includes
// <stdint.h> rather than <cstdint>: avr-libc's C++ headers are incomplete on
// the ATmega328P toolchain, unlike the Teensy build used by ../src/config.h.
//
// Items marked [MEASURE] are placeholders that MUST be replaced with values
// measured on the actual stand before any live motor is fired.
//
// SAFETY: nothing in this file can protect you if the MOSFET gate is missing
// its 10k pulldown to ground. Between power-on and the first line of setup()
// the AVR pin is a floating input. See WIRING.md.
//
#include <stdint.h>

namespace stand {

// ---------------------------------------------------------------------------
// Math / units. Internal force unit is the newton. Grams and kilograms appear
// only at the calibration boundary and in logs.
// ---------------------------------------------------------------------------
constexpr float PI_F = 3.14159265f;
constexpr float G0   = 9.80665f;  // standard gravity, m/s^2

// ---------------------------------------------------------------------------
// Pins (Arduino Uno / Nano). D0/D1 are the USB serial port — keep them clear.
// This uses 12 of the 12 available digital pins D2..D13; A0 is the only spare.
// ---------------------------------------------------------------------------
constexpr int PIN_LCD_RS = 12;
constexpr int PIN_LCD_EN = 11;
constexpr int PIN_LCD_D4 = 5;
constexpr int PIN_LCD_D5 = 4;
constexpr int PIN_LCD_D6 = 3;
constexpr int PIN_LCD_D7 = 2;

constexpr int PIN_HX711_DOUT = 6;
constexpr int PIN_HX711_SCK  = 7;

constexpr int PIN_IR_RECV = 8;

constexpr int PIN_GATE = 9;   // MOSFET gate — 10k PULLDOWN TO GND REQUIRED
constexpr int PIN_BUZZER = 10;  // ACTIVE buzzer (see note below)
constexpr int PIN_LED = 13;     // built-in

// Reserved, not yet wired. A0 would take an igniter continuity divider.
constexpr int PIN_ARM_SWITCH = 14;  // A0 as digital, INPUT_PULLUP, LOW = armed

// The buzzer MUST be an ACTIVE (self-oscillating) type driven with plain
// digitalWrite. A passive buzzer needs tone(), and on the ATmega328P tone()
// owns Timer2 — which is exactly the timer IRremote 4.x uses for its 50 us
// receive sampling. Using both silently breaks IR reception.
constexpr bool BUZZER_ACTIVE_HIGH = true;

// ---------------------------------------------------------------------------
// Loop rates (Hz)
// ---------------------------------------------------------------------------
constexpr float LCD_HZ      = 5.0f;   // never call clear(); it blocks ~2 ms
constexpr float STREAM_HZ   = 80.0f;  // CSV rows while armed (== sample rate)
constexpr float IDLE_HZ     = 5.0f;   // CSV rows while idle on the bench
constexpr float STATUS_HZ   = 20.0f;  // buzzer/LED sequencer

// ---------------------------------------------------------------------------
// HX711
//
// 25 SCK pulses selects channel A at gain 128 (+-20 mV full scale). A 5 kg bar
// cell at 2 mV/V on a 5 V excitation gives ~10 mV at capacity, so roughly half
// the converter range is used — plenty of headroom and resolution.
//
// 80 SPS requires the RATE pin tied to VCC. On the common green breakout it is
// hard-tied to GND and needs a solder mod (WIRING.md). Without it you get
// 10 SPS: about ten samples across an Estes burn, which will miss the peak.
// ---------------------------------------------------------------------------
constexpr uint8_t HX711_GAIN_PULSES = 25;
constexpr uint32_t HX711_TIMEOUT_MS = 500;    // no data-ready for this long = fault
constexpr float    RATE_FAST_MAX_MS = 20.0f;  // measured period below this = 80 SPS

// ---------------------------------------------------------------------------
// Load cell / calibration
// ---------------------------------------------------------------------------
constexpr float CELL_CAPACITY_N    = 49.0f;  // 5 kg bar cell = 5 * G0
constexpr float OVERLOAD_WARN_FRAC = 0.80f;  // warn above 80% of capacity
constexpr float LPF_HZ             = 20.0f;  // display/filtered channel only

// [MEASURE] Replaced by the calibration wizard and stored in EEPROM. The
// default is a rough figure for a 5 kg cell at gain 128; until `cal` has been
// run the firmware reports UNCAL and the CSV header records it.
constexpr float DEFAULT_COUNTS_PER_N = 21000.0f;

constexpr uint16_t TARE_SAMPLES = 160;  // ~2 s at 80 SPS
constexpr uint16_t CAL_SAMPLES  = 160;
constexpr float    CAL_MIN_GRAMS = 20.0f;    // refuse a uselessly light reference
constexpr float    CAL_MAX_GRAMS = 5000.0f;  // beyond cell capacity

// ---------------------------------------------------------------------------
// Ignition. Read the safety note at the top of this file first.
// ---------------------------------------------------------------------------
constexpr bool REQUIRE_ARM_SWITCH = false;  // set true once a switch is wired

constexpr uint32_t ARM_CONFIRM_MS    = 10000;  // ARM_PENDING expiry window
constexpr uint32_t DISARM_TIMEOUT_MS = 60000;  // auto-disarm an idle ARMED stand
constexpr uint8_t  COUNTDOWN_SECONDS = 5;
constexpr uint32_t FIRE_PULSE_MS     = 1000;   // HARD CAP on gate-high time

// ---------------------------------------------------------------------------
// Run capture
// ---------------------------------------------------------------------------
constexpr float    THRUST_TRIGGER_N   = 0.50f;  // thrust onset / burn start
constexpr float    BURN_END_FRAC      = 0.05f;  // burn ends below 5% of peak
constexpr uint32_t BURN_END_HOLD_MS   = 800;    // ...sustained this long
constexpr uint32_t RECORD_MAX_MS      = 20000;  // absolute backstop
constexpr uint32_t NO_IGNITION_MS     = 5000;   // no thrust after FIRE = dud

// ---------------------------------------------------------------------------
// IR — 21-key "car MP3" remote, NEC protocol, address 0x00.
//
// These are the codes that remote almost always ships with, but clones vary.
// Run `learn` (IR key 0, or type `learn` on serial) and correct anything that
// does not match BEFORE trusting the fire key.
// ---------------------------------------------------------------------------
constexpr uint8_t IR_CH_MINUS = 0x45;  // ABORT / DISARM
constexpr uint8_t IR_CH       = 0x46;
constexpr uint8_t IR_CH_PLUS  = 0x47;  // next LCD page
constexpr uint8_t IR_PREV     = 0x44;
constexpr uint8_t IR_NEXT     = 0x40;
constexpr uint8_t IR_PLAY     = 0x43;  // FIRE (only from ARMED)
constexpr uint8_t IR_MINUS    = 0x07;
constexpr uint8_t IR_PLUS     = 0x15;
constexpr uint8_t IR_EQ       = 0x09;  // ARM, pressed twice
constexpr uint8_t IR_0        = 0x16;  // learn mode
constexpr uint8_t IR_100PLUS  = 0x19;
constexpr uint8_t IR_200PLUS  = 0x0D;
constexpr uint8_t IR_1        = 0x0C;  // tare
constexpr uint8_t IR_2        = 0x18;  // calibrate
constexpr uint8_t IR_3        = 0x5E;  // toggle stream
constexpr uint8_t IR_4        = 0x08;
constexpr uint8_t IR_5        = 0x1C;
constexpr uint8_t IR_6        = 0x5A;
constexpr uint8_t IR_7        = 0x42;
constexpr uint8_t IR_8        = 0x52;
constexpr uint8_t IR_9        = 0x4A;

// ---------------------------------------------------------------------------
// Serial / EEPROM
// ---------------------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD    = 115200;
constexpr int      EEPROM_BASE_ADDR = 0;

} // namespace stand
