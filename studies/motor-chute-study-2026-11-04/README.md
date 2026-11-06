# Motor choice at 1.4 kg + chute size for a 3 m/s landing (2026-11-04, report-only)

Plan: `C:\Users\troop\.claude\plans\i-want-to-be-wise-haven.md`. This is closed-loop simulator evidence only: nothing has flown and RocketFC is unchanged.

**Workspaces**
- Scratch copies of the RocketFC working tree, built by `setup_ws.py`.
- `ws/base` adds three things to that copy:
  - the casing-eject model from `landing-velocity-study-2026-10-25/casing-eject`;
  - a dud-igniter switch;
  - a sized-canopy model.
- Each motor pair gets its own workspace:
  - 1.4 kg liftoff;
  - casing pushed out at landing thrust onset;
  - burn table rebuilt (`--eject-kg`);
  - legs on at ignition confirm (`LEGS_DELAY_MS 0`), 0.75 s band cut;
  - margin 1.0 m, S3 on, 620 ms ignition delay.

**Checks**
- With every new knob off, `ws/base` reproduces RocketFC exactly: 50 / 50 identical flights.
- With the October E12-fix inputs, it reproduces that study's dev result: 504 / 600 = 84.0 %.

## Part A: motors (1.4 kg at liftoff, everything aboard)

**Filter** (`motors.py`, OpenRocket motor database, 70 motors pass):
- ≤ 29 mm; ≤ G, ≤ 62.5 g propellant (still a model rocket, no L1);
- **the ascent motor must be plugged**, because a `-0` booster vents forward at burnout and would light the landing motor above it;
- the landing motor must be `P` or `-0`;
- reloads with no `P` listed are flagged "plug?".

**Point-mass pre-screen:**
- The **E12 can't lift 1.4 kg** (11 N average vs 13.7 N weight).
- F-class ascent motors are either > 30 m apogee (F24W 35 m, F39T 44 m, F22J 52 m) or only come with ejection delays (F20W, F21W, F27R).
- For comparison, the earlier E12-fix winner's soft fire window was ~0.4 m of fire height. At 1.4 kg the best landing motors have ~0.33 m.

**Closed loop:**
- Stage 1: 88 pairs, 200 flights each. Stage 2: top 10, dev 600. Stage 3: holdout 701-710, 2,000 flights.
- Measured spreads: mass ±1 %, delay ±5 %. Success = powered, upright, intact, 3 m/s crash limit.

| Ascent / landing (holdout, 2,000) | Success | Apogee med / p99 | Contact vz med / p90 | Both motors in production? |
|---|---|---|---|---|
| **AeroTech E18W / E18W** | **79.5 %** | 18.2 / 18.8 m | 2.00 / 3.37 m/s | **yes** (RMS-24/40 reloads) |
| AeroTech E18W / F12J | 78.5 % (same, p = 0.19) | 18.2 / 18.8 m | 2.11 / 3.23 m/s | yes |
| AeroTech E15W / E15W | 85.3 % | 13.2 / 13.7 m | 2.20 / 3.20 m/s | **no**: OOP on ThrustCurve, P version no longer listed |
| Cesaroni 26E31-15A / AeroTech E15W | 85.8 % | 10.4 / 10.7 m | 1.49 / 2.96 m/s | no (E15W) |

The E15W landing pairs are about 6 points better (p ≈ 1e-13), but the motor is out of production. **Recommendation: E18W / E18W.**

**Robustness, E18W/E18W** (dev 600 per row; `robust.log`):

| Condition | Success |
|---|---|
| Measured spreads | 81.2 % |
| **Today's guessed spreads** (mass ±5 %, delay ±15 %) | **28.0 %**: weigh the rocket and measure the E18W's ignition delay |
| Steady wind 0 / 2 / 4 / 5 / 6 / 7 m/s | 74.8 / 79.5 / 87.7 / **55.8 / 12.0 / 0** %. Above ~4 m/s, descent tilt aborts take over, so the wind limit is ~4 m/s. |
| Landing thrust ±3 % (motor-to-motor spread) | 72.7 % |
| Ascent impulse ±5 % | 77.2 %. Apogee p99 21.1 m, max 21.5 m: **inside 100 ft** |
| **Today's leg timing** (nichrome 1 s after fire, 1.5 s band) | **0 %**: legs late on every flight. Needs nichrome at ignition confirm + band ≤ 0.75 s, a firmware change that needs your OK. |

E15W/E15W, by comparison, holds up in wind (93–95 % at 5–6 m/s) because the 13 m flight is short. It is 39.5 % with today's guessed spreads.

**Buying questions** (not answerable from the sim):
1. **Plugging the E18W ascent motor.** No plugged E18W is listed, and no plugged forward closure for RMS-24/40 hardware could be confirmed. The ascent motor's delay/ejection gas vents forward into the landing motor's nozzle, so it must be plugged.
2. **Landing E18W ejection.** Fly it without the ejection charge. Its delay grain still vents forward into the airframe after burnout.
3. **Two RMS-24/40 cases.** The ascent case (with the 24→29 adapter, 46 g) leaves the tail at landing ignition and falls free.
4. **Stack length.** E18W reload/case 70 mm each, plus two 24→29 adapters in the 29 mm tube.
5. **The E18W's own ignition delay and scatter.** The sim assumed the measured 620 ms ±5 %.

## Part B: chute size (E18W/E18W; 1.379 kg under the chute)

**Model** (`ws/base`, checked by `chute_model_check.py`):
- CdA = Cd·πD²/4.
- Line stretch 0.3 s after the latch releases.
- The canopy then fills over **n·D of fall**, with drag area ∝ (distance / n·D)². Nominal n = 8 (Knacke, solid canopy, conservative); n = 4 = fast-fill check.
- With instant fill, the steady descent rate matches √(2mg/ρCdA) to 0.00 %.
- Sea-level air (ρ 1.225). Every run is 600 flights, dev 101-103, measured spreads, wind −4…+4 m/s (`chute.log`, `s4_wind_aborts.log`).

### S1: chute released at apogee (chute-test mode; release at 17.9 m)
Vertical impact speed, p99 ≈ median:

| Canopy type | 2.9–3.0 m/s (just meets 3) | With margin |
|---|---|---|
| Dome / elliptical, Cd 1.5 | 1.5 m (59 in): 2.89 m/s | 1.6 m (63 in): 2.71; 1.7 m: 2.55 |
| **Annular (Iris-type), Cd 2.2** | 1.2 m (47 in): 2.98 m/s | **1.3 m (51 in): 2.75; 1.4 m (55 in): 2.55** |
| Flat / hex sheet, Cd 0.8 | 2.0 m (79 in): 2.97 m/s | 2.2 m only fills 95 % by impact (2.83 m/s). Bigger doesn't help from 18 m. |

- n = 4 gives the same numbers: from apogee, every dome/annular size fills fully.
- Opening load is 1.8–1.9 g.
- Sideways drift at impact is the wind's speed: median 2.1, p90 3.6 m/s. No chute size changes it.

### Emergency releases in landing mode (vertical impact)

| Event | Release height | Best result (any size) |
|---|---|---|
| Tilt abort in the coast (gust) | ~13 m | n = 8: annular 1.3 m 51 % ≤ 3 m/s, dome 1.5 m 30 %, flat 2.0 m 0 %. n = 4: dome 1.5 m 73 %. |
| Tilt abort at apogee (gust) | ~11 m | n = 8: annular 1.3 m 16 %, dome 1.5 m 5 %, flat 0 %. n = 4: dome 1.5 m 59 %. |
| Gust in boost → missed fire window | 3.7 m | 0 %, ~18.7 m/s |
| Natural aborts in 5–6 m/s wind (missed fire window at 5 m/s; tilt + missed window at 6 m/s) | 4–6 m | 0 %, ~18.5 m/s |
| **Dud landing igniter** (`DUD_WINDOW_MS` 700) | after impact | 0 %, 18.6 m/s |
| Dud, `DUD_WINDOW_MS` 300 | 4.4 m | 0 %, 18.6 m/s |

**Why:**
- While a canopy is still filling, its drag area grows with the distance fallen, *not* with its final size. So a bigger chute only helps once it is full.
- From ≤ 6 m nothing fills.
- From 11–13 m, a canopy that reaches the needed drag at a **smaller diameter** (annular) fills sooner and wins.
- A dud can't be recognized until ~1 s after the 15 m fire. The motor's own 620 ms ignition delay uses most of that.

### How often the chute opens, and how high
Counted from the saved runs (no new sims).

| Situation | Chute opens | Below 10 m |
|---|---|---|
| Landing mode, wind ≤ 4 m/s. Holdout 2,000, plus dev, guessed spreads, ±3 % thrust and ±5 % impulse runs. | never (0 %) | 0 % |
| Landing mode, steady 5 m/s wind | 25.8 % | 25.8 % (all missed fire window, median 3.7 m) |
| Landing mode, steady 6 m/s wind | 82.2 % | 78.2 % (tilt + missed window, median 5.7 m) |
| Landing mode, steady 7 m/s wind | 100 % | 46.2 % (median release 10.3 m) |
| Dud landing igniter, `DUD_WINDOW_MS` 700 (today) | never: hits at 18.6 m/s before the dud is declared | — |
| Dud landing igniter, 300 ms | 100 % | 100 % (4.4 m) |
| Chute-test mode | 100 % | 0 % (17.9 m) |

In normal conditions the ~20 % of failed landings are hard landings or lift-offs with the chute never used. Real-world frequency depends on how often you fly above 4 m/s wind and how often the landing igniter fails; the igniter failure rate isn't in the sim.

### Clusters vs one big chute (`cluster.py`, `cluster.log`)
- **Setup:** every row has the same full drag as one 60 in Cd 2.2 annular (cluster drag efficiency 0.9). Ideal = all canopies fill together (n = 8). Out of step = sluggish, staggered cluster inflation (n = 12).
- **Data:** 600 apogee releases + 600 gust flights per row; only aborted flights are counted in the abort columns.

| Chutes | Apogee release | Coast abort (~13 m) ≤ 3 m/s | Apogee abort (~11 m) ≤ 3 m/s | All gust aborts ≤ 3 m/s |
|---|---|---|---|---|
| 1 × 60 in | 2.35 m/s | 51 % | 16 % | 33 % |
| 2 × 45 in, ideal | 2.35 m/s | 68 % | 50 % | 56 % |
| 2 × 45 in, out of step | 2.35 m/s | 39 % | 7 % | 24 % |
| 3 × 37 in, ideal | 2.35 m/s | 73 % | 62 % | 63 % |
| 3 × 37 in, out of step | 2.35 m/s | 63 % | 29 % | 45 % |

- From apogee it makes no difference.
- In 11–13 m aborts, smaller canopies fill sooner and help, but only if they open together. An out-of-step pair is worse than one chute.
- Releases below ~6 m (wind aborts, dud igniter) are still unrecoverable.

### Recommendation
- **An annular (Iris-type) canopy, about 1.35–1.4 m (54–55 in), Cd ≈ 2.2.** Nominal 2.55–2.6 m/s from apogee.
  - It stays ≤ 3 m/s with +10 % mass and hot or high-altitude air (ρ 1.12 → ×1.10 speed).
  - It gives the best chance of all the shapes in a coast or apogee abort.
- **Second choice:** a 1.6–1.7 m (63–67 in) elliptical/dome, Cd 1.5. Same apogee result, but worse in aborts.
- **Avoid a flat sheet:** it needs > 2 m and can't fill from 18 m.
- **Check the canopy's own Cd and packed size** on the product page. Its weight is part of the 1.4 kg.

**What no chute fixes:**
- Aborts below ~10 m and dud landing igniters hit at ~18 m/s. Keep landing-mode flights to **wind ≤ 4 m/s**, where the sim shows 0 % aborts.
- Sideways drift: "3 m/s" total impact speed only holds in calm wind.

## Caveats
- 1.4 kg is predicted, not weighed. The success rates assume you weigh it and measure the delay before flying.
- The fill constant is a textbook estimate (n 4–8 brackets it). The E18W ignition delay and the RMS casing masses (db values) are unmeasured.
- Plugging the RMS ascent motor is unconfirmed.
- The sim's wind is steady. Gusts were tested only as one 12 m/s pulse.

## Files
- **Scripts:**
  - `motors.py`: filter + pre-screen, `prescreen.json`, `motors/*.eng`;
  - `setup_ws.py`: workspaces;
  - `screen.py`: stages 1-3;
  - `robust.py`;
  - `chute_model_check.py`;
  - `chute_study.py`;
  - `mclib.py`.
- **Logs:** `stage1-3.log`, `robust.log`, `chute.log`, `s4_wind_aborts.log`.
- **Raw results:** `results.zip`.
