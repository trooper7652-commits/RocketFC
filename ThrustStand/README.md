# ThrustStand

A static thrust test stand for model rocket motors: an Arduino Uno/Nano reads
an HX711 + load cell at up to 80 samples/second, arms and fires an igniter
over a 21-key IR remote, shows live status on a 16x2 LCD, and streams the
thrust curve over USB to a Python script that plots it live and saves a CSV
and a PNG. It also computes peak thrust, total impulse, burn time, NAR motor
class, and — the number this project exists to measure for the companion
[RocketFC](../README.md) flight computer — the delay between commanding
ignition and the motor actually producing thrust.

```
   [ motor + igniter ]
          |
    [ load cell ]---[ HX711 ]---+
                                 |
[ IR remote ] -->[ IR recv ]--[ Arduino Uno/Nano ]--[ 16x2 LCD ]
                                 |         |
                          [ MOSFET gate ]  +--[ USB / serial ]--> laptop
                                 |                         |
                        [ igniter battery ]     stand_capture.py (live plot)
```

---

## 1. Safety

Read this section before you connect anything, not after.

- **The MOSFET gate needs a 10 kΩ pulldown resistor to ground.** Between
  power-on and the first line of firmware, the Arduino pin driving the gate is
  a floating input — no code can protect you during that window, only the
  resistor can. See [WIRING.md](WIRING.md). Check it with a meter before the
  first test.
- **Never test the fire channel with a live e-match.** Prove the whole arm →
  countdown → fire → abort sequence with a 12 V lamp or an LED + resistor in
  its place first. Only connect a real igniter once that has passed.
- **The arm sequence takes three deliberate actions**: press EQ, press EQ
  again within 10 seconds, then press the fire key. Any other key at any time
  during the countdown aborts. An armed-but-idle stand disarms itself after 60
  seconds — if you've stepped away, it stands down on its own.
- **Minimum safe distance**: follow your national model rocketry
  organization's range safety code for the motor class you're testing (e.g.
  NAR/NFPA 1122 in the US). Static-fire distances are generally similar to
  launch distances — you are standing next to an ignition source and an
  unrestrained thrust output, not behind a shield.
- **Restrain the stand.** It must be anchored or heavy enough that motor
  thrust cannot move it. An unrestrained stand is both a projectile hazard and
  a source of bad data (the load cell will read the stand's own acceleration,
  not just the motor's thrust).
- **Blast deflection**: motor exhaust is hot and can ignite debris or dry
  grass. Test over a non-flammable surface with a clear area behind the
  nozzle.
- **Keep a fire extinguisher at the stand**, appropriate for the surface
  you're testing on.
- **This project has no physical arm switch** (a deliberate build choice) —
  the stand is "live" (able to reach ARMED) any time it is powered and an IR
  remote is in range. If that is not the safety posture you want, wire a
  switch to A0 and set `REQUIRE_ARM_SWITCH = true` in `config.h`; the firmware
  already has the hook.
- **Legal**: check local and national regulations on model rocket motor
  possession, testing, and ignition before you do any of this.

---

## 2. What you get

- Live thrust-vs-time plot while the motor burns (`stand_capture.py`)
- A timestamped CSV of every run, saved automatically (`runs/`)
- An annotated PNG per run with peak thrust, average thrust, burn time, total
  impulse, NAR class, and ignition delay (`plot_thrust.py`)
- A RASP `.eng` export of the measured curve that feeds directly into
  `tools/burn_table.py` back in the main RocketFC repo (`eng_export.py`)
- The one number RocketFC's `src/config.h` currently has as a guess:
  `IGNITION_DELAY_MS`, the real e-match-command-to-thrust delay of your motor

---

## 3. Bill of materials

See [WIRING.md](WIRING.md) for the full parts table with specific part
numbers, the MOSFET circuit, the HX711 80 SPS solder mod, and the mechanical
mounting notes. Summary:

- Arduino Uno or Nano
- 5 kg straight-bar load cell + HX711 breakout
- 16x2 HD44780 LCD (parallel, 16-pin) + 10k contrast pot + 220Ω backlight
  resistor
- Logic-level N-channel MOSFET (IRLZ44N or equivalent) + 10kΩ gate pulldown +
  100–220Ω gate series resistor
- Separate igniter battery
- IR receiver (TSOP38238/VS1838B) + 21-key NEC remote
- Active buzzer

---

## 4. Wiring

Full pinout, the MOSFET schematic, the star-ground rule, and the HX711 solder
mod are in [WIRING.md](WIRING.md). **Read the safety note in there about the
gate pulldown before wiring the fire channel.**

---

## 5. Mechanical assembly

Covered in [WIRING.md](WIRING.md#mechanical-mounting): how a bar cell must be
loaded (fixed end, load end, straight through the sensing axis), why off-axis
loading — not straight-line overthrust — is what actually damages a 5 kg cell,
and motor retention.

---

## 6. Software install

**Arduino IDE** (2.x recommended):

1. Board: **Arduino Uno**, or **Arduino Nano**. If you have a Nano and it
   fails to upload with a `programmer is not responding` error, it's likely a
   clone with an older bootloader — under Tools → Processor, pick **ATmega328P
   (Old Bootloader)**.
2. Libraries, via Library Manager:
   - **LiquidCrystal** (bundled with the IDE — install if it's missing)
   - **IRremote** by Armin Joachimsmeyer, **version 4.x**. Sketches or
     tutorials you find online written for IRremote 2.x/3.x will not compile
     against 4.x — the decode API changed. This project already targets 4.x.
3. Open `ThrustStand.ino` and hit Upload.

Command line, if you use `arduino-cli`:

```sh
arduino-cli lib install "LiquidCrystal" "IRremote"
arduino-cli compile --fqbn arduino:avr:uno ThrustStand
arduino-cli upload --fqbn arduino:avr:uno -p <PORT> ThrustStand
```

**Python tools** (on your laptop, not the Arduino):

```sh
pip install pyserial matplotlib
```

---

## 7. First-time setup

Do this with **no igniter connected**. Power the stand and open a serial
terminal (or just run `stand_capture.py`, which prints everything it
receives) at 115200 baud.

1. **Confirm your remote's codes.** Press IR key `0` (or type `learn` over
   serial) to enter learn mode. Press each button on your remote; the LCD and
   serial both print the address and command byte received. Compare against
   the table in [§8](#8-running-a-test) and the defaults in `config.h`. If
   any differ, edit the `IR_*` constants and re-upload. Press `0` again (or
   `CH-`) to exit learn mode.
2. **Check the sample rate.** The LCD's INFO page (cycle pages with `CH+`)
   shows the measured rate. It should read 80 SPS. If it reads 10 SPS, the
   HX711 RATE-pin solder mod either wasn't done or didn't take — see
   [WIRING.md](WIRING.md#hx711-80-sps-solder-mod). You can still use the
   stand at 10 SPS, but you will under-resolve the ignition spike and the
   ignition-delay measurement will be coarse (±~50-100 ms instead of ~12 ms).
3. **Tare.** With nothing on the cell, press `1` (or type `tare`). The LCD
   shows a progress bar (~2 seconds at 80 SPS). Don't touch the stand while it
   runs.
4. **Calibrate.** Press `2` (or type `cal`). The stand re-tares first (do not
   touch it), then prompts `PUT MASS ON CELL`. Weigh any convenient object on
   a kitchen scale — a full water bottle, a bag of something — place it on the
   load cell in the same orientation the motor thrust will apply, then type
   its mass in grams over serial and press enter (e.g. `500`). The stand
   averages ~2 seconds of data and computes the scale factor. It's saved to
   EEPROM automatically (`# ... OK, saved`), so this survives power cycles —
   redo it only if you suspect the cell's mounting has changed. To sanity
   check, weigh a *different* known object next and confirm the reading
   agrees within a few percent.

---

## 8. Running a test

1. Connect USB, then start the capture tool:

   ```sh
   python tools/stand_capture.py --motor C6-5
   ```

   (`--port COMx` if auto-detect picks the wrong port; list ports with `python -m serial.tools.list_ports`.)

2. Install the motor and igniter, following the igniter manufacturer's
   instructions. Insert the e-match leads into your igniter clip, wired to the
   MOSFET-switched circuit.
3. **Retreat to a safe distance** per your range safety code.
4. **Arm**: press `EQ`, then `EQ` again within 10 seconds. The LCD shows
   `*** ARMED ***`.
5. **Fire**: press the play key (▶||). The LCD counts down 5…4…3…2…1 with a
   beep each second; any key press during the countdown aborts and disarms.
6. At zero, the gate energizes for up to 1 second (`FIRE_PULSE_MS`), an
   `EVT:FIRE` marker is logged, and the stand starts watching for thrust. If
   the motor doesn't light within 5 seconds it's flagged as a dud
   (`EVT:NO_IGNITION`) and the run ends safely with the gate already off.
7. Once the motor lights, the stand records until thrust drops below 5% of
   peak and stays there for 0.8 s (burnout), or 20 seconds pass (backstop).
8. The LCD shows a summary (peak, impulse, burn time, NAR class, ignition
   delay); `stand_capture.py` finalizes the CSV and produces the annotated PNG
   automatically.

### State diagram

```
IDLE --EQ--> ARM_PENDING --EQ (<=10s)--> ARMED --FIRE--> COUNTDOWN(5s)
  ^                |                       |                 |
  |            (timeout/                (60s idle          (any key
  |             any other key)            timeout)           = abort)
  +----------------+-----------------------+-----------------+
                                            |
                                        (0 reached)
                                            v
                                         FIRING --(gate off)--> RECORDING
                                                                    |
                                                          (burnout/no-ignition/
                                                              timeout)
                                                                    v
                                                                SUMMARY --> IDLE
```

### IR key map

| Key | Command | Notes |
|---|---|---|
| EQ | Arm (press twice) | First press → `ARM_PENDING`, expires after 10 s |
| ▶\|\| (play) | Fire | Only accepted from `ARMED` |
| CH- | Abort / disarm | Works from any state |
| CH+ | Next LCD page | Live → Stats → Info |
| 1 | Tare | Idle/Summary only |
| 2 | Calibrate | Idle/Summary only; then enter grams over serial |
| 3 | Toggle serial stream | Forces full-rate CSV rows even when idle |
| 0 | IR learn mode | Prints every key's raw code |
| all others | unmapped | shown as `UNKNOWN` in learn mode |

Every one of these also has a serial-typed equivalent (`tare`, `cal`, `arm`,
`fire`, `abort`, `learn`, `page`, `stream on`/`stream off`, `status`, `help`,
`save`) — type `help` at any time. This means a dead remote mid-session
doesn't strand you; the stand is fully operable from a keyboard.

---

## 9. Reading your results

Each run produces `runs/<timestamp>_<motor>.csv` and, after running
`plot_thrust.py` (which `stand_capture.py` does automatically), a matching
`_plots.png` with two panels: thrust vs. time (raw and filtered, with a
dotted line at the cell's rated capacity) and cumulative impulse vs. time.
Red dashed lines mark the `FIRE` and `BURNOUT` events.

**Stats, printed to the terminal and drawn on the chart:**

- **Peak thrust (N)** — from raw samples, not the filtered trace, so a real
  spike is never under-reported by smoothing.
- **Average thrust (N)** — total impulse divided by burn time.
- **Burn time (s)** — from thrust onset (first sample ≥ 0.5 N) to burnout
  (drops below 5% of peak and stays there).
- **Total impulse (N·s)** — trapezoidal integration of thrust over the burn,
  the same method `tools/burn_table.py` uses for `.eng` files, so the number
  is directly comparable to a published motor's total impulse.
- **NAR class** — the letter class (A, B, C, …) derived from total impulse;
  each class covers double the impulse of the one before it.
- **Ignition delay (ms)** — time from the `FIRE` event (gate energized) to
  thrust onset. This is the number to paste into RocketFC's
  `src/config.h:IGNITION_DELAY_MS`; the script prints the exact line for you.

**Telling a good run from a bad one:**

- A curve that pins at the dotted capacity line is **clipped** — the cell
  maxed out and the true peak is unknown. Don't trust the peak number; if this
  happens routinely, move to a higher-capacity cell.
- A run flagged `overload=1` in the CSV header/comments means the same thing
  happened and was caught in real time by the firmware.
- Heavy oscillation ("ringing") right at ignition is usually mechanical —
  the mount flexing or the stand itself moving — not something the motor is
  doing. Check that the stand is rigidly anchored.
- A non-zero, non-flat baseline *before* `FIRE` (drift) usually means the tare
  is stale (something touched the stand after taring) or temperature is
  moving the zero point — retare immediately before a run rather than at the
  start of a long session.

---

## 10. Feeding RocketFC

Two exports close the loop back to the main flight computer:

**1. The e-match ignition delay.** `plot_thrust.py` prints a ready-to-paste
line:

```
constexpr float IGNITION_DELAY_MS = 362.0f;
```

Copy that value into `../src/config.h` (the flight computer's own config,
one directory up from this one), replacing the placeholder that's marked
`[MEASURE]`.

**2. The measured thrust curve**, as a RASP `.eng` file:

```sh
python tools/eng_export.py runs/20260823_193000_F15.csv \
    --name MyF15 --diam 29 --len 114 --delay 0 \
    --prop-g 60.0 --total-g 102.0 --mfr YourMfr \
    --out ../tools/motors/MyF15.eng
```

`--prop-g`/`--total-g` come from weighing the motor before and after the burn
on the same kitchen scale you used for calibration (propellant mass = before
− after). Then regenerate the landing-burn table from your real motor instead
of the published `Estes_F15.eng`:

```sh
cd ..   # back to the RocketFC repo root
python tools/burn_table.py --eng tools/motors/MyF15.eng --mass 0.95 \
    --delay-ms 362
```

This overwrites `src/core/burn_table.h` with a table built from data you
actually measured.

---

## 11. Configuration reference

Everything lives in `config.h`, `namespace stand`. Pin assignments and IR key
codes are covered in [§8](#8-running-a-test) and [WIRING.md](WIRING.md); the
rest:

| Constant | Default | Unit | What it does |
|---|---|---|---|
| `LCD_HZ` | 5 | Hz | LCD refresh rate. Rows are cached and only re-written on change, and the code never calls `clear()`, so this can be raised without risking a stall. |
| `STREAM_HZ` | 80 | Hz | Target CSV row rate while armed — matches the HX711 sample rate, so every sample gets a row. |
| `IDLE_HZ` | 5 | Hz | CSV row rate while idle on the bench (keeps a heartbeat in the log without flooding it). |
| `STATUS_HZ` | 20 | Hz | Buzzer/LED pattern update rate. |
| `HX711_GAIN_PULSES` | 25 | pulses | 25 = channel A, gain 128 (±20 mV FS). Don't change unless you rewire for channel B. |
| `HX711_TIMEOUT_MS` | 500 | ms | No data-ready pulse for this long → fault flagged, and an armed stand auto-disarms. |
| `RATE_FAST_MAX_MS` | 20 | ms | Measured sample period at or below this reports "80 SPS" on the LCD; above it reports "10 SPS". |
| `CELL_CAPACITY_N` | 49.0 | N | Load cell's rated capacity (5 kg × 9.80665). Change if you fit a different cell. |
| `OVERLOAD_WARN_FRAC` | 0.80 | — | Overload warning fires above this fraction of `CELL_CAPACITY_N`. |
| `LPF_HZ` | 20.0 | Hz | Cutoff of the one-pole filter used for the displayed/filtered channel only — raw samples (used for peak/impulse) are never filtered. |
| `DEFAULT_COUNTS_PER_N` | 21000.0 | counts/N | Placeholder scale factor before `cal` has been run; overwritten by the calibration wizard and stored in EEPROM. |
| `TARE_SAMPLES` / `CAL_SAMPLES` | 160 | samples | ~2 s at 80 SPS. Raise for less noisy tare/cal at the cost of a longer wait. |
| `CAL_MIN_GRAMS` / `CAL_MAX_GRAMS` | 20 / 5000 | g | Sanity bounds on the reference mass you enter; rejects a typo or a mass that would overload the cell. |
| `REQUIRE_ARM_SWITCH` | false | — | Set `true` after wiring a physical arm switch to `PIN_ARM_SWITCH` (A0) to require it in addition to the remote sequence. |
| `ARM_CONFIRM_MS` | 10000 | ms | Window for the second `EQ` press to complete arming. |
| `DISARM_TIMEOUT_MS` | 60000 | ms | Auto-disarm if `ARMED` sits idle this long (e.g. you walked out of IR range). |
| `COUNTDOWN_SECONDS` | 5 | s | Countdown length before firing. |
| `FIRE_PULSE_MS` | 1000 | ms | **Hard cap** on how long the gate can be energized — enforced independently in both the igniter driver and the state machine. |
| `THRUST_TRIGGER_N` | 0.50 | N | Threshold that counts as "the motor has lit" / burn onset. |
| `BURN_END_FRAC` | 0.05 | — | Burn considered over once thrust falls below this fraction of the run's peak. |
| `BURN_END_HOLD_MS` | 800 | ms | How long thrust must stay below that threshold before burnout is declared (avoids ending the run on a brief dip). |
| `RECORD_MAX_MS` | 20000 | ms | Absolute backstop — recording ends here no matter what. |
| `NO_IGNITION_MS` | 5000 | ms | No thrust above `THRUST_TRIGGER_N` this long after `FIRE` → flagged as a dud, run ends. |
| `SERIAL_BAUD` | 115200 | baud | Must match `--baud` on the Python side (default already matches). |

---

## 12. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| LCD shows nothing, or a row of solid black boxes | Contrast pot not adjusted | Turn the 10k pot on V0 until text appears; it's very sensitive. |
| LCD text garbled/random characters | Loose data pin, or wiring doesn't match `config.h` | Reseat D2–D5, D11, D12; verify against the pinout table. |
| HX711 reading stuck at a fixed value, or reads `-1`/`8388607` | DOUT/SCK swapped, or the HX711 isn't powered | Check wiring against `config.h` (`PIN_HX711_DOUT`/`SCK`); confirm 5V/GND reach the breakout. |
| Readings drift slowly over minutes | Temperature affecting the load cell's zero point | Retare right before each run rather than once per session. |
| Readings get noisy the moment the igniter battery is connected | Ground loop / shared return path | Star-ground both supplies at one point on the Arduino GND — see [WIRING.md](WIRING.md#why-the-igniter-needs-a-separate-battery). |
| IR remote does nothing | Wrong code table for your remote, wrong IRremote version, or receiver pins reversed | Run `learn` mode and compare codes to `config.h`; confirm IRremote is v4.x; check OUT/GND/VCC orientation on the receiver. |
| `stand_capture.py` says "no serial port found" | Port not detected, or in use | Pass `--port COMx` explicitly; close the Arduino IDE's Serial Monitor first — only one program can hold the port. |
| Python "port busy" / `PermissionError` | Serial Monitor or another instance of the script is already open | Close it before running `stand_capture.py`. |
| LCD/serial keeps showing `UNCAL` after calibrating | The reference mass never registered a large enough delta (out of range or wasn't actually resting on the cell) | Re-run `cal`; make sure the mass sits directly on the load surface, not on the frame. |
| Rows appear to drop / plot has gaps | Serial buffer overrun at high baud on a slow USB-serial adapter | Confirm `SERIAL_BAUD` matches on both ends; try a shorter/better USB cable; avoid running other serial-heavy programs at the same time. |

---

## 13. File map

```
ThrustStand/
  ThrustStand.ino        Scheduler + hardware glue; setup() calls igniter
                          safeInit() first, before anything else.
  config.h                All pins, rates, calibration/ignition tunables, IR
                          codes. namespace stand.
  src/load_cell.h         Non-blocking HX711 driver: sampling, tare,
                          calibration, one-pole filter, overload check.
  src/cal_store.h         EEPROM persistence for tare/scale factor/run count.
  src/igniter.h            MOSFET gate control with the hard-capped pulse.
  src/stand_state.h       Arm/countdown/fire/record state machine + run
                          statistics (peak, impulse, NAR class, ign. delay).
  src/ir_input.h           NEC decode -> logical commands, repeat filtering,
                          learn mode.
  src/ui_lcd.h             Non-blocking 16x2 rendering (no clear() in the
                          hot path), three pages (Live/Stats/Info).
  src/telemetry.h         CSV + event rows over serial, matching the format
                          convention in ../src/hw/logger.h.
  tools/stand_capture.py  Serial -> live matplotlib plot, saves runs/*.csv,
                          auto-finalizes with plot_thrust.py.
  tools/plot_thrust.py    CSV -> annotated PNG + printed stats; can be run
                          standalone on any saved CSV.
  tools/eng_export.py     CSV -> RASP .eng file for ../tools/burn_table.py.
  WIRING.md               Parts list, pinout, MOSFET schematic, HX711 mod,
                          mechanical mounting, motor-sizing table.
  runs/                   Saved CSVs and PNGs land here (gitignored).
```
