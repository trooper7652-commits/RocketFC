# SensorServoBenchTest

**A bench rig for the RocketFC flight computer.** You hold the rocket in your
hands, rotate it, and a 3D model on your laptop rotates the same way.

---

## 1. What this is, and why you'd use it

A thrust-vector-controlled rocket decides which way to point the motor based
entirely on what its sensors say about which way is up. If a sensor axis is
swapped, or a sign is inverted, the flight computer will confidently steer the
rocket into the ground — and you will not find that out from a bench multimeter,
because every individual reading looks perfectly reasonable. The failure only
shows up when you look at the *orientation as a whole*.

So that is what this program shows you. It reads the attitude the flight
computer has actually computed, 50 times a second, and draws the vehicle in 3D
with the world's up direction marked. Tilt the real rocket north; the model
should tilt north. If it doesn't, something in the sensor chain is wrong, and
you have found it on a desk instead of on a launch rail.

It also lets you run the real TVC control loop on the bench, so you can watch
the gimbal deflect in response to your hand-tilting — and confirm it deflects in
the direction that would *right* the vehicle rather than tip it over further.

This is a separate project from RocketFC. It does not modify the flight
software in any way. The attitude math, sensor drivers, and servo mapping are
copies of RocketFC's own, so what you validate here is what actually flies.

---

## 2. What you need

### Hardware

| Item | Notes |
|---|---|
| Teensy 4.1 | the flight computer board |
| BMI088 accelerometer + gyro | on I2C |
| MS5611 barometer (e.g. GY-63) | on the same I2C bus |
| USB cable | Teensy to PC, must be a *data* cable, not charge-only |
| 2 gimbal servos + their own BEC | **optional** — only for the last step |

Wiring (the same as RocketFC's, so you can bench-test the assembled vehicle):

| Signal | Teensy 4.1 pin |
|---|---|
| BMI088 + MS5611 SDA | 18 |
| BMI088 + MS5611 SCL | 19 |
| Sensor power | 3.3 V and GND |
| Gimbal servo A (body X torque) | 2 |
| Gimbal servo B (body Y torque) | 3 |

> **The Teensy 4.1 is a 3.3 V board and is NOT 5 V tolerant.** Do not feed it
> 5 V sensor boards with 5 V pull-ups, and do not power servos from its 3.3 V
> rail. Servos get their own BEC or battery, with a common ground.

You do **not** need: an SD card, pyro channels, e-matches, motors, or a battery.
This project never touches any of them.

### Software

- **Python 3** (3.8 or newer) — runs the visualizer
- **Arduino IDE 2.x** with Teensy support — uploads the firmware
- One Arduino library: **Bolder Flight Systems BMI088**

---

## 3. One-time setup

Do these in order. Steps 3.1–3.3 need no hardware at all, so you can get the
graphics half working before you plug anything in.

> **On the machine this was built on (checked 2026-08-16), §3.1–3.4 are already
> done** — Python 3.12.10 with pyserial/pygame/PyOpenGL/numpy installed, the
> demo verified at 60 FPS, Arduino IDE 2.x with Teensy board package 1.62.0, and
> `Bolder_Flight_Systems_BMI088` already in the sketchbook. Skip to **3.5**.
> Steps 3.5 (wiring), 3.6 (upload — this sketch has never been flashed) and 3.7
> (find the COM port) still have to be done. The instructions below remain for a
> fresh machine.

### 3.1 Install Python

1. Download Python 3 from <https://www.python.org/downloads/>.
2. Run the installer. **Tick "Add python.exe to PATH"** on the first screen —
   this is easy to miss and everything below depends on it.
3. Open a new terminal (Windows: press Start, type `cmd`, Enter) and check:

   ```
   python --version
   ```

   You should see something like `Python 3.12.10`. If you instead get
   "'python' is not recognized", the PATH checkbox was missed — re-run the
   installer, choose *Modify*, and enable it.

### 3.2 Install the Python packages

In that same terminal:

```
cd "path\to\SensorServoBenchTest\viz"
pip install -r requirements.txt
```

That installs four packages: `pyserial` (talks to the Teensy), `pygame` (opens
the window), `PyOpenGL` (draws the 3D), and `numpy` (the math). It should end
with a line like `Successfully installed PyOpenGL-3.1.10 pygame-2.6.1 ...`.

If `pip` is not recognized, use `python -m pip install -r requirements.txt`
instead.

### 3.3 Check the graphics before touching hardware

```
python rocket_viz.py --demo
```

A window should open showing a rocket slowly tumbling above a grid, with panels
of numbers around the edges. Press **Esc** to close it.

This is the single most useful troubleshooting step in the whole document: if
the demo works, your Python and graphics setup is fine, and any later problem is
in the wiring, the firmware, or the serial port.

| If you see | It means |
|---|---|
| `ModuleNotFoundError: No module named 'OpenGL'` | step 3.2 didn't complete — re-run it and read the output |
| `could not create an OpenGL window` | graphics driver too old or missing; update it via Windows Update or the GPU vendor's site |
| a window that opens and instantly closes | run it from a terminal, not by double-clicking, so you can read the error |

### 3.4 Install the Arduino toolchain

1. Install **Arduino IDE 2.x** from <https://www.arduino.cc/en/software>.
2. Open it. Go to **File → Preferences**.
3. In *Additional boards manager URLs*, paste:
   `https://www.pjrc.com/teensy/package_teensy_index.json`
   Click OK.
4. Open **Tools → Board → Boards Manager**, search for `Teensy`, click Install.
   (This takes a few minutes.)
5. Open **Tools → Manage Libraries**, search for `BMI088`, and install
   **Bolder Flight Systems BMI088**.

### 3.5 Wire the sensors

Connect SDA to pin 18, SCL to pin 19, and power the sensor boards from **3.3 V**
and GND. Re-read the voltage warning in §2 before powering anything on.

Leave the servos disconnected for now. You will connect them only at step 5.7.

### 3.6 Upload the firmware

1. In Arduino IDE: **File → Open**, and open
   `SensorServoBenchTest/SensorServoBenchTest.ino`.
2. **Tools → Board → Teensy → Teensy 4.1**.
3. **Tools → Port** — pick the port the Teensy appears on.
4. Click the **→** (Upload) arrow.

A successful upload ends with the Teensy Loader window flashing and the IDE
reporting the sketch size. The board immediately starts streaming attitude data.

### 3.7 Find your COM port

```
cd "C:\Users\troop\OneDrive\Documents\Inventor\SpaceX Rocket\SensorServoBenchTest\viz"
python rocket_viz.py --list-ports
```

This lists every serial port with a description; the Teensy usually shows as
`USB Serial Device`. Note the name — something like `COM7` on Windows, or
`/dev/ttyACM0` on Linux.

(You can also find it in Windows Device Manager, under *Ports (COM & LPT)*.)

---

## 4. Running it

From the `viz` folder:

```
python rocket_viz.py --port COM3
```

replacing `COM7` with your port from step 3.7.

> **Only one program can hold a serial port at a time.** If the Arduino IDE's
> Serial Monitor is open, this will fail with an access error. Close the Serial
> Monitor first. This is the most common stumbling block by far.

### What's on screen

- **The rocket** — drawn at the orientation the flight computer currently
  believes it has. Its nose is body **+Z**; the four fins lie on the body ±X and
  ±Y axes. Short red/green/blue rods mark body +X, +Y and +Z.
- **The long thin red/green/blue arrows** — the *world* axes, drawn from the
  same point as the rocket's own. With the vehicle upright and level, each body
  rod lies along its matching world arrow. That is the comparison this whole
  program exists to make easy.
- **The grid** — the ground plane. World +Z points up out of it.
- **The pale translucent second rocket ("the ghost")** — the attitude implied by
  the accelerometer alone, computed from gravity, with no gyro involved. See
  §4.1.
- **SENSORS / AHRS** — live accelerometer, gyro, quaternion, tilt, altitude.
- **AXIS CHECK** — which body axis gravity currently lies along.
- **GIMBAL / SERVOS** — what the TVC controller is commanding.
- **BOARD OUTPUT** — text the Teensy printed. Because this program owns the COM
  port, this panel is where the board's replies appear.
- **Top right** — the source, and the *measured* packet rate. It should sit near
  50 Hz.

### 4.1 What the ghost is for

The solid rocket comes from the flight computer's AHRS: gyro integration, gently
corrected by the accelerometer. The ghost comes from gravity alone. They are
computed from different sensors by different routes, so **when they disagree,
something is wrong** — a bad gyro bias calibration, real gyro drift, or a wrong
axis mapping.

The panel reports the disagreement in **tilt only**, and that is deliberate. The
accelerometer can see which way is down but has no idea which way is *north*, so
the ghost's heading is arbitrary and meaningless. You will often see the ghost
pointing a different compass direction than the solid model. That is expected
and is not a fault. Only the lean matters.

### 4.2 Controls

| Key / action | Does |
|---|---|
| drag with left mouse | orbit the camera |
| scroll wheel | zoom |
| `G` | show/hide the ghost |
| `C` | run `cal` — gyro bias + level (hold the rocket still ~4 s) |
| `Z` | run `zero` — barometer ground reference |
| `X` | run `axis` — print the axis check to the board output |
| `S` | run `status` |
| `1` / `2` | `tvc off` / `tvc calc` |
| `/` | type any command, Enter to send, Esc to cancel |
| `Esc` | quit |

`tvc drive` — the one command that physically moves the servos — has no
shortcut on purpose. Type it in full at the `/` prompt.

---

## 5. The bench test

Work through these in order. Each step adds exactly one new thing, so when
something fails you know what caused it.

### 5.1 Calibrate

Stand the rocket upright and hold it still. Press **`C`**.

*Expect:* after about four seconds, `cal done.` in the board output, with a
small gyro bias and a tilt near 0°.

*If it times out:* the vehicle was moving. Rest it on a table and try again.

### 5.2 Nose up

Hold the rocket vertical, nose up.

*Expect:* the model is vertical. `tilt` reads under about 2°. AXIS CHECK says
**gravity along body +Z**, "consistent with the rocket standing nose up".

*If AXIS CHECK names a different axis:* `IMU_R_SB` in `../RocketFC/src/config.h`
does not match how the BMI088 is physically mounted. **While still holding the
vehicle nose up, press `X`.** The board prints the corrected matrix, ready to
paste:

```
gravity lies along body -X (9.62 m/s^2 of it)
-> if the vehicle is standing NOSE UP right now, this is WRONG:
   body +Z must point out the nose.
body long axis (+Z) currently 87.4 deg from world up.

Paste this into RocketFC's src/config.h, then run
`python sync_core.py` here and re-upload:
constexpr float IMU_R_SB[9] = {
    0,   1,   0,
    0,   0,  -1,
   -1,   0,   0,
};
```

Paste it over the `IMU_R_SB` block in `../RocketFC/src/config.h`, run
`python sync_core.py`, re-upload, and repeat §5.1–5.2.

Gravity only pins down the *long* axis. Which lateral axis belongs to servo A
versus servo B is not observable from a static reading, so confirm that with the
gimbal direction test in §5.7 — if the two channels come out swapped, exchange
the first two rows of the matrix.

**Do not "fix" this by relabeling the axes in the visualizer.** The display is
reporting what the flight computer actually believes. With a wrong `IMU_R_SB`,
`Ahrs::tiltRad()` reads about 90° while the rocket stands upright, so the
pre-arm check (5° limit) refuses to arm, the abort logic (30° limit) triggers,
and TVC would gimbal hard over trying to right a vehicle it thinks is lying
down. Relabeling would not even stay correct: with no magnetometer the heading
is unobservable and drifts, so the compass direction the nose appears to point
changes on its own over a minute or two.

### 5.3 Tilt toward body +X

Find body +X on the real vehicle — it's the gimbal axis that servo A drives, and
on screen it's the red rod. Lean the nose that way by roughly 20°.

*Expect:* the model leans the same direction on screen, by roughly the same
amount, and `tilt` climbs to about 20°.

*If it leans the wrong way, or leans about the wrong axis:* `IMU_R_SB` has a
wrong sign or a swapped row. This is the single most valuable thing this test
catches.

### 5.4 Roll

Hold the rocket vertical again and rotate it about its own long axis.

*Expect:* the model spins about its long axis. `tilt` stays near 0° throughout —
rolling is not tilting.

*If `tilt` swings while you roll:* the body long axis is not mapped to +Z.

### 5.5 Drift check

Set the rocket down, upright, and leave it alone for a minute.

*Expect:* `tilt` stays near 0 and the ghost stays on top of the solid model. The
model's *heading* may slowly rotate. **That is correct and expected** — there is
no magnetometer, so nothing observes heading, and it drifts. Only tilt matters
for TVC.

*If tilt itself creeps upward:* re-run `cal` while genuinely still; a gyro bias
captured during movement will drift.

### 5.6 TVC with the servos held still

Press **`2`** (`tvc calc`). The controller now runs, but the servos do not move.

Tilt the nose toward body +X.

*Expect:* `gimbal X` in the GIMBAL panel swings **opposite** to your tilt, and
the crosshair marker moves off center. The controller is commanding thrust that
would push the tail back under the nose.

*If gimbal X stays at 0:* the controller isn't running — check the mode line.

### 5.7 TVC driving the real servos

Now, and only now, connect the gimbal servos. Before you do:

- servos powered from **their own BEC**, never the Teensy's 3.3 V rail
- common ground between the BEC, the battery, and the Teensy
- the gimbal free to move through its full travel
- **no motor installed**

Press **`/`**, type `tvc drive`, press Enter.

*Expect:* as you tilt the nose toward +X, the gimbal physically deflects so the
motor mount points the thrust to push the tail back under the nose. It fights
your tilt. Release, and it returns toward center.

*If a channel deflects the wrong way:* `SERVO_A_SIGN` or `SERVO_B_SIGN` in
`../RocketFC/src/config.h` is inverted. Fix it there, re-run `sync_core.py`,
re-upload, retest.

*If the board resets or the servos judder:* the servos are browning out the
Teensy. They need their own supply.

Type `tvc off` or press **`1`** when you're done.

---

## 6. Command reference

Commands go to the board either through this program (press `/` and type, or use
the shortcut keys in §4.2), or through the Arduino IDE's Serial Monitor at
115200 baud **when the visualizer is not running**.

| Command | Action |
|---|---|
| `help` | list commands |
| `status` | sensors, attitude, TVC mode, servo state |
| `viz on` / `viz off` | start/stop the 50 Hz attitude stream (on at boot) |
| `cal` | gyro bias + level initialization — hold still ~4 s |
| `zero` | barometer ground reference |
| `tvc off` | controller stopped, servos centered |
| `tvc calc` | controller runs, **servos do not move** |
| `tvc drive` | controller runs and **drives the servos** |
| `trim a +20` | nudge a servo center by +20 µs (not saved) |
| `center` | center the servos and stop the controller |
| `sweep` | open-loop gimbal sweep, to check the servos are alive |
| `axis` | which body axis gravity is along; if it isn't +Z, also prints a corrected `IMU_R_SB` to paste |

There is deliberately **no pyro command of any kind**. See §9.

---

## 7. Recording and replay

Record a bench session while you work:

```
python rocket_viz.py --port COM7 --record bench.csv
```

Play it back later, with no hardware attached:

```
python rocket_viz.py --csv bench.csv
```

Replay also reads RocketFC's own SD card flight logs, which already contain the
attitude quaternion:

```
python rocket_viz.py --csv ../../RocketFC/FLIGHTS/flight_007.csv
```

During replay: **Space** pauses, **`,`** and **`.`** seek 5 s, **`-`** and
**`+`** change speed. Playback loops.

> RocketFC's synthetic replay cases (`tools/replay/cases/*.csv`) will *not* work
> here: they contain raw sensor data only, with no attitude columns.

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `could not open COM7: access is denied` | another program holds the port | close the Arduino Serial Monitor |
| `could not open COM7: file not found` | wrong port, or board unplugged | `--list-ports` |
| Window opens, rocket never moves, rate reads **NO DATA** | board isn't streaming | send `viz on`; check the board output panel for a boot banner |
| Rate sits far below 50 Hz | dropped serial or a stalled loop | unplug/replug; check the USB cable is a data cable |
| `imu OK` shows **FAIL** at startup | I2C wiring or address | check SDA=18, SCL=19, 3.3 V, GND; confirm both boards share the bus |
| `baro` shows **FAIL** | MS5611 not responding | same checks; the MS5611 sits at 0x77 or 0x76 |
| Model's heading slowly rotates on its own | no magnetometer — expected | not a fault; see §5.5 |
| Rocket shows **horizontal** while you hold it upright | `IMU_R_SB` wrong: the long axis isn't mapped to body +Z | hold it nose up, press `X`, paste the printed matrix — §5.2 |
| Model tilts the wrong way | `IMU_R_SB` wrong | §5.3 |
| Tilt creeps up over time | gyro bias captured while moving | re-run `cal` while genuinely still |
| Gimbal deflects the wrong way | `SERVO_x_SIGN` inverted | §5.7 |
| Board resets when servos move | servos powered from the Teensy | give them their own BEC |
| `ModuleNotFoundError` | packages not installed | §3.2 |
| Rocket looks fine but the ghost is far off in *heading* | expected | the accelerometer cannot see heading; §4.1 |

---

## 9. Safety

**This project cannot fire a pyro channel or open the parachute latch.**
`enablePyro()`, `firePyro()`, `testFire()`, `applyLegs()`, `testLegs()` and
`applyChute()` are never called from anywhere in this codebase, and the bench
CLI has no `pyrotest` or `chute` command. Both pyro gates (pin 6 landing
e-match, pin 7 leg-release nichrome) are driven LOW as outputs at startup and left
there, and the chute latch servo (pin 23) is parked at LOCK — if one is
plugged in, it will move to its LOCK position at power-up and stay there.

For the `tvc drive` step:

- servos on their own BEC or battery, **never** the Teensy's 3.3 V rail, common
  ground
- gimbal free to move through its full travel
- **no rocket motor installed** while the mount is moving
- keep fingers clear; metal-gear micro servos can pinch

---

## 10. How this relates to RocketFC

These files under `src/` are **copies** taken from RocketFC:

```
src/config.h
src/core/quat.h        src/core/ahrs.h      src/core/control.h
src/hw/sensors.h       src/hw/actuators.h
```

Each one carries a banner naming the RocketFC commit it came from. **Do not edit
them here.** Edit the original in RocketFC, then re-copy:

```
python sync_core.py            # re-copy from ../RocketFC
python sync_core.py --check    # exit non-zero if the copies are stale
```

If RocketFC lives somewhere else, pass `--rocketfc PATH`.

Copying rather than including keeps this an ordinary self-contained Arduino
sketch — the IDE copies a sketch folder to a temporary build directory, so
relative includes reaching outside it are unreliable. The cost is that the copies
can go stale, which is what `--check` is for.

Anything you *fix* as a result of this bench test — `IMU_R_SB`, `SERVO_A_SIGN`,
`SERVO_B_SIGN`, PID gains — belongs in RocketFC's `src/config.h`, because that
is the copy that flies. Change it there, re-sync, re-upload, and re-run the
test.
