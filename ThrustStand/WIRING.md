# ThrustStand — Wiring

Read this before you connect anything. The mandatory item is the gate
pulldown; everything else is important but recoverable if you get it wrong.

## Parts list

| Part | Spec | Notes |
|---|---|---|
| Arduino Uno or Nano | ATmega328P | Nano clones sometimes need the "old bootloader" board variant in Arduino IDE — see README §6. |
| Load cell | 5 kg straight bar, 4-wire | Rated ~49 N. See the motor-sizing table below before you commit to this capacity. |
| HX711 breakout | any common green board | The RATE pin needs a solder-bridge mod for 80 SPS — see below. |
| LCD | 16x2 HD44780, parallel (16-pin) | Not an I2C backpack — this design wires it directly. |
| Contrast pot | 10 kΩ trim pot | Across LCD V0; without it the display is usually all-black or blank. |
| Backlight resistor | 220 Ω | In series with LCD pin 15 (A/LED+), unless your panel has one built in. |
| MOSFET | Logic-level N-channel: IRLZ44N, IRL540N, or FQP30N06L | **Must** be a logic-level part (fully on at 5 V gate drive). A standard IRF-series FET will barely turn on from an Arduino pin and can overheat and fail. |
| Gate pulldown | 10 kΩ resistor, gate → GND | **Mandatory. See the safety note below.** |
| Gate series resistor | 100–220 Ω, Arduino pin → gate | Limits inrush into the gate capacitance; protects the pin if the FET is ever swapped for one with a lower gate charge. |
| Flyback diode | 1N4001 or similar | Only needed if you ever drive an inductive load (a relay coil) instead of a resistive e-match — include it anyway, it's cheap insurance. |
| Igniter battery | separate from the Arduino's supply | See "why a separate battery" below. |
| IR receiver | TSOP38238 (38 kHz) or the kit-bundled VS1838B | 3-pin: OUT, GND, VCC. |
| IR remote | 21-key "car MP3" style, NEC protocol | Confirm the exact codes with `learn` mode — clones vary. |
| Buzzer | **Active** (self-oscillating) type | Passive buzzers need `tone()`, which on the ATmega328P uses Timer2 — the same timer IRremote 4.x uses for receiving. Using both breaks the abort key. See config.h. |
| Wire, igniter run | 22 AWG or heavier, short as practical | E-matches pull several amps for a brief pulse; thin, long leads add resistance that can prevent ignition. |

## Safety-critical: the gate pulldown

Between power-on and the first executed line of `setup()`, every AVR digital
pin is a **floating input** — undefined, easily pulled high by noise or
capacitive coupling from a neighboring trace. If pin D9 floats high during
that window, the MOSFET can turn on and energize the igniter **before any of
your code has run, and before there is any way to abort it.**

A 10 kΩ resistor from the gate to ground holds the gate near 0 V through that
entire window, and through every reset, brown-out, and firmware upload. It
costs a few cents and is not something you can code your way around. Wire it
before you ever connect an igniter — check it with a meter (gate-to-ground
resistance ≈ 10 kΩ) before the first test.

## Why the igniter needs a separate battery

The igniter and the Arduino/load-cell circuit must share **ground only**, not
a supply rail:

- An e-match's ignition pulse draws several amps for a fraction of a second.
  Pulled from the same battery powering the Arduino, that current spike can
  sag the 5 V rail enough to brown out the MCU mid-log — the exact moment you
  most need the log.
- The same current, if it shares a ground path with the load-cell signal
  wires, induces a voltage transient on the HX711's differential input right
  at the ignition instant — corrupting the one sample you care about most.

Star-ground both supplies at a single point at the Arduino's GND pin, and run
the igniter's return wire directly to that point rather than daisy-chaining it
through the load-cell wiring.

## Pinout

| Arduino pin | Connects to |
|---|---|
| D2 | LCD D7 |
| D3 | LCD D6 |
| D4 | LCD D5 |
| D5 | LCD D4 |
| D6 | HX711 DOUT |
| D7 | HX711 SCK |
| D8 | IR receiver OUT |
| D9 | MOSFET gate (through 100–220 Ω, **with the 10k pulldown to GND**) |
| D10 | Active buzzer + |
| D11 | LCD EN |
| D12 | LCD RS |
| D13 | Status LED (built-in) |
| A0 | Reserved — igniter continuity sense, not used yet |
| 5V | HX711 VCC, LCD VDD, IR receiver VCC |
| GND | Common ground — HX711, LCD, IR receiver, MOSFET source, igniter battery return |

LCD R/W ties to GND (write-only, as in every standard LiquidCrystal wiring).

## MOSFET low-side switch

```
                      +---- igniter battery (+)
                      |
                  [ igniter / e-match ]
                      |
   Arduino D9         |
       |               \  drain
       +--[220R]--+     |
                  |   +-+-+
                  +---| G |  logic-level N-MOSFET
                  |   +-+-+
               [10k]    | source
                  |      |
                 GND ----+---- igniter battery (-) ---- Arduino GND (star point)
```

The load is on the high side (battery+ → igniter → drain), the FET switches
the low side (source → ground) — this is the standard, forgiving arrangement
because the gate-drive reference (Arduino GND) and the source are the same
node.

## HX711 80 SPS solder mod

Stock green HX711 breakouts tie the **RATE** pin to GND, which selects 10 SPS.
At 10 SPS you get roughly ten samples across a one-second motor burn — enough
to see that something happened, not enough to resolve the peak or the curve
shape.

To get 80 SPS: find the RATE pin pad on the board (often labeled `RATE` near
the HX711 chip, sometimes only a via). Cut the trace to GND if there is one,
then bridge the pad to VCC (a short blob of solder to the adjacent VCC/5V pin
usually works, or a short jumper wire). The exact pad layout differs between
board revisions — check with a continuity meter against the datasheet pinout
for your specific board before cutting anything.

The firmware measures the actual sample period and reports it on the LCD
("INFO" page) and in the CSV header (`sample_hz`), so a missed or failed mod
is visible immediately rather than silently producing a blocky curve.

## Mechanical mounting

A straight bar load cell is only accurate loaded **in its rated axis** — one
end fixed to the stand base, the other end carrying the motor mount, both
using the manufacturer's spacer/washer stack so the load path runs straight
through the sensing element. A motor mounted off-axis, or thrust transmitted
through a bracket that also bears a side load, applies a bending moment the
cell was never calibrated for — this is what actually damages or invalidates
a 5 kg cell well below its rated 49 N, not straight-line overthrust from a
small motor.

Bolt or clamp the motor mount so thrust is purely axial, and make sure the
whole stand is anchored (weighted or fastened down) — an unrestrained stand
will try to accelerate backward under thrust, which both endangers anyone
nearby and adds spurious load-cell readings from the stand's own motion.

## Motor sizing vs. the 5 kg cell (49 N capacity)

| Motor | Approx. peak thrust | Margin on a 5 kg cell |
|---|---|---|
| Estes A8 | ~10 N | ~5x |
| Estes B6 | ~12 N | ~4x |
| Estes C6 | ~14 N | ~3.5x |
| Estes D12 | ~30 N | ~1.6x |
| RocketFC's F15 | ~25 N peak, sharp spike | ~2x, and spike timing matters |

A/B/C motors have comfortable margin. D-class and up erode it quickly,
especially motors with a sharp ignition spike rather than a smooth ramp. If
you plan to characterize the F15 (or anything D-class or larger) for RocketFC,
budget for a 10–20 kg cell rather than relying on the 5 kg one's margin.

## Test order

1. Bench-test load cell + LCD + calibration wizard with **no igniter wired at
   all**.
2. Wire the fire channel and prove it with a 12 V lamp or an LED + resistor in
   place of the igniter — never with a live e-match. Confirm the gate is low
   at power-on, through reset, and through a brown-out, and that the pulse
   never exceeds `FIRE_PULSE_MS`.
3. Only once both are proven, connect a real igniter and follow README §8.
