# Stuck chute latch and dud landing motor (2026-11-07)

The closed-loop bench sim (PR #1) found two weak spots with the E18W/E18W 1.4 kg setup. This study tests fixes in a scratch copy; RocketFC was not changed.

**Setup:**
- the Nov 4 study's E18W/E18W workspace: 1.4 kg, casing eject, legs on at ignition confirm, 0.75 s band cut;
- 1.4 m annular canopy, Cd 2.2, with the fill-by-distance model (n = 8, i.e. it fills over 8 diameters of fall);
- dev seeds 101–103 × 200 flights, measured spreads, wind −4..+4 m/s;
- holdout seeds 701–710 (2,000 flights).

Every knob defaults to off, and the patched copy reproduced the unpatched one 50/50 in both modes. Files: `setup_ws.py`, `latch.py`, `dud.py`, logs `*.log`.

## 1. Stuck latch
**How late can the canopy be freed?** (Step A, chute mode, the delay counted from the first release)

| freed after | n = 8: impact median | ≤ 3 m/s | n = 4 |
|---|---|---|---|
| 0–0.5 s | 2.55 m/s | 100 % | 2.55 m/s |
| 0.75 s | 2.85 m/s | 87 % | 2.55 m/s |
| 1.0 s | 6.0 m/s | 0 % | 2.56 m/s |
| 1.25 s | 16.6 m/s | 0 % | 11.8 m/s |
| ≥ 1.5 s | 18.6 m/s | 0 % | 18.6 m/s |

So a retry must free the canopy within ~0.6 s. Today's retry takes ~1.9 s: 1.5 s waiting for the canopy to show in the accelerometer, then 0.4 s back at LOCK, then the stroke back up.

**Retry schemes** (Step B, chute mode)

How the sim models the latch:
- The latch servo is simulated at 375°/s (a DS3218-type 20 kg servo at 5 V).
- The first stroke frees the canopy as it passes 110°.
- With a stuck latch, a later stroke counts as a retry once it gets back to OPEN, after swinging back either ≥ 30° (a sticky pin that any shake frees) or all the way to LOCK (needs a full cycle).

| scheme | healthy latch | 1 stuck release, shake frees it | 2 stuck, shake | needs full cycle |
|---|---|---|---|---|
| today | 2.55 m/s | 18.6 m/s, 0 % ≤ 3 | 18.6 m/s | 18.6 m/s |
| shorter canopy wait (600/900 ms) | same | 18.5 m/s | 18.6 m/s | 18.5 m/s |
| fast full cycles OPEN→LOCK→OPEN (1–3) | same | 13.3 m/s | 18.6 m/s | 13.3 m/s |
| **fast wiggles** (OPEN → 100 ms toward LOCK, ~37° → OPEN; 2–4 of them, the first after 0.4 s) | **same** | **2.56 m/s, 100 % ≤ 3** (600 of 600 rescued) | **3.16 m/s, 17 % ≤ 3** | 18.6 m/s |

- A full cycle takes ~1.1 s (140° each way at 0.37 s), which is past the 0.6 s deadline.
- A wiggle retries every ~0.22 s.
- No scheme changed a healthy latch: 0 % false "unconfirmed".
- In landing mode, the wiggle changed nothing (487/600 either way). The chute only opens on an abort, and none happened.
- **The wiggle only helps a sticky pin.** If the latch needs a full swing back to LOCK, nothing is fast enough at this servo speed and throw. A shorter LOCK→OPEN throw (about 80°) would bring a full cycle to about 0.65 s.

## 2. Dud landing motor
**No dud window rescues the rocket** (DUD_WINDOW_MS sweep):

| window | dud: impact median | canopy freed at | good motors with ±15 % ignition-delay scatter: false dud aborts |
|---|---|---|---|
| 0 ms | 13.3 m/s | 8.9 m | 415 of 600 |
| 100 ms | 17.1 m/s | 7.5 m | 96 |
| 200–500 ms | 18.5–18.6 m/s | 6.0–1.0 m | 0 |
| 700 ms (today) | 18.6 m/s | after impact | 0 |

Even aborting the instant thrust was due still crashes, and it would abort most good motors. Keep 700 ms.

**What does help is making duds rarer:** igniter quality, handling and continuity checks.

**The impact jolt passes as "ignition confirmed":** today, 600 of 600 dud flights turn the legs nichrome on at impact.

| confirm rule | dud: false confirms | good motors: success (paired) | legs later |
|---|---|---|---|
| today (one tick > 1.3 g) | 600/600 | 487/600 | 0 |
| hold > 1.3 g for 20 ms | 566/600 | 483 (−4) | 18 ms |
| hold 40 ms / 60 ms | 0 | 480 (−7) / 472 (−15) | 38 / 58 ms |
| **KF altitude > 3 m (or > 1 m) at the confirm** | **0** | **487 (+0/−0)** | **0** |

- **Altitude-gate checks:**
  - With ±15 % ignition-delay scatter: 0 false dud aborts, and no change in success (+0/−0).
  - **Holdout:** 1591/2000 = 79.5 % (95 % CI 77.7–81.3), identical to today, flight for flight. 0/600 false confirms on dud flights.
  - The gate uses the same "near the ground, a jolt is the impact" rule as the chute detector (`TOUCHDOWN_ALT_M`, 3 m). 1 m performed identically.
- Holding the thrust for 40–60 ms also works, but it delays the legs and costs landings.

## 3. Angle-target wiggle (the owner's 70°↔150° idea), `wiggle.py`
**Owner's inputs:** the latch lets the door go at R ≈ 70–90°, and a wiggle can't re-catch the door.

**How it works:** the latch is commanded to explicit angles, LOCK → HIGH → LOW → HIGH → … → OPEN, with dwells equal to the servo travel time. A stuck latch frees on a later stroke that comes back up through R after having been ≥ 5° below it (the "re-pass" rule).

**The scripts:**
- The workspace is `ld_base2`: the same knobs plus the wiggle, still 50/50 identical with everything off.
- The simulated retry times match hand estimates: 70↔150 at R = 80 gives 0.11 / 0.54 / 0.97 s.

| cycle (servo 375°/s) | 1 stuck, R 70 / 80 / 90 | 2 stuck, R 70 / 80 / 90 | tries at (s) |
|---|---|---|---|
| today | 18.6 m/s for all | 18.6 m/s | 0.1 / 2.0 |
| 143↔180 (the earlier wiggle) | 18.6 (never back below R) | 18.6 | — |
| **70↔150** | 18.6 / **2.55** / **2.55** | 18.6 / 5.2 / 5.9 | 0.1 / 0.54 / 0.97 |
| 55↔150 | 2.56 / 2.56 / 2.57 | 10–13 | 0.1 / 0.6 / 1.1 |
| **55↔110** | **2.55 for all** | **2.61 / 2.69 / 2.78, 100 % ≤ 3** | 0.1 / 0.4 / 0.7 |

**What the table shows:**
- With a healthy latch every cycle gives 2.55 m/s, 100 % ≤ 3, identical to today.
- K = 3 down-strokes gives the same results as K = 2.
- **70↔150** fixes one stuck release, but only if R is ≥ ~75°, so that 70° gets back below it.
- **55↔110** fixes two stuck releases over the whole 70–90° range. That assumes 110° opens the door far enough to clear.

**Servo speed** (R = 80):
- 55↔110 still rescues two stuck releases at 430°/s (6.8 V) and at 300°/s (a spring-loaded servo slower than its rating, with the 5 V dwells): 2.56–2.75 m/s.
- 70↔150 with two stuck: 3.4 m/s at 430°/s, 5.3 m/s at 300°/s.

**Holdout** (701–710, 2,000 flights, R = 80):
- Healthy latch: unchanged for both cycles.
- 1 stuck: 0 → 100 % ≤ 3 m/s for both.
- 2 stuck: 0 → 100 % for 55↔110 (2.68 m/s median); 70↔150 stays at 0 % (5.1 m/s).

## Proposed ports (need the owner's OK)
1. **`chute_deploy.h`: the fast wiggle.**
   - **What:** after the first release, wait 400 ms, then up to 3 wiggles: 100 ms toward LOCK, then back to OPEN with a 120 ms hold. After that, today's canopy check and slow re-cycles as before.
   - **Bench first:**
     - with the spring loaded, a 100 ms LOCK command really swings the real servo back ≥ 30°;
     - a wiggle can't re-catch the door or the shock cord.
2. **`flight_state.h`: the ignition-confirm altitude gate** (`kfAlt > TOUCHDOWN_ALT_M`).
   - **Effect:** a dud then logs DUD_IGNITER, and the legs stay off at the crash.
   - **Also update:** PR #1's `test_live.py` dud case, which asserts today's behaviour.
