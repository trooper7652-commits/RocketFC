# 00 — The current landing-burn system (as of git 239d096 + your uncommitted edits)

Workspace copies were taken 2026-10-12 18:14 (hashes in `hashes_at_copy.txt`).

## 1. Exact firing logic (`src/core/flight_state.h`, `updateDescent()`)
Every flight-computer tick while in DESCENT (FULL_LANDING mode):
1. `checkAborts()` — tilt > 30° for 100 ms → TILT abort; IMU unhealthy 200 ms; KF unhealthy 500 ms.
2. `kfAlt < MISSED_WINDOW_ALT_M (4 m)` → MISSED_WINDOW abort (chute).
3. `vDown = -kfVel`; do nothing while `vDown < 2 m/s`.
4. `hIgn = burntable::hIgnite(vDown) + BURN_TABLE_MARGIN_M (0)` — linear interpolation in the
   47-row table (2..25 m/s, 0.5 m/s steps, clamped at both ends).
5. Fire when `kfAlt <= hIgn` **and** tilt < 15° **and** KF healthy **and** landing-pyro continuity.
   One-shot: `fireLanding`, enter LANDING_BURN (TVC on, landing gains), no aborts from then on.
6. LANDING_BURN: ignition confirmed if |accel| > 1.3 g after 50 ms; no confirmation within
   500 + 700 ms → DUD abort. Legs nichrome at fire + 1 s (confirmed burns only).

## 2. Loop rate
`FlightCore::step()` (AHRS → KF predict/update → FSM → PID) runs in the **500 Hz** FAST loop
(`RocketFC.ino`), so the decision is checked every 2 ms. (The `config.h` comment that the FSM
runs at `MID_HZ` 100 Hz is stale.) Barometer samples arrive ~100 Hz in the sim (MS5611 OSR 2048
converts in ~4.1 ms on hardware). FAST frame budget: 2000 µs.

## 3. Estimator lag — mechanism and size
- The KF (`altitude_kf.h`: altitude, velocity, accel bias) predicts with world-vertical accel and
  corrects with every baro sample, with R measured from pad noise (small) → it follows the baro
  closely.
- The sim's baro has a **first-order 25 ms lag** (`baro_lag_s`; stands for bay venting + filtering;
  the chip itself is ~4 ms). A lagged baro reads where the rocket *was*: during descent that is
  **higher** by ≈ |v|·τ and its rate is **slower** by ≈ |a|·τ.
- Measured (clean nominal flight, seed 42; `plots/p0_kf_lag_and_tilt.png`): KF altitude error grows
  ≈ linearly with descent speed, +0.12 m at 5 m/s → +0.30 m at the fire point (15 m/s);
  velocity error ≈ +0.2 m/s (KF too slow) throughout freefall. Least-squares effective lag:
  τ_h ≈ 20 ms, τ_v ≈ 22 ms (clean); over the 228 dev-seed powered landings τ_h = 16.5 ± 2.1 ms,
  τ_v = 19.3 ± 2.3 ms; error at fire = **+0.27 ± 0.06 m, +0.18 ± 0.04 m/s**.
- Effect on timing: the table is entered with a too-high altitude and a too-low speed, so it fires
  late by ≈ 0.27 m + (dh/dv ≈ 3.1 m per m/s) × 0.18 ≈ **0.8–0.9 m** at 15 m/s — this is the
  "3.5 m/s floor". It also acts as an *accidental safety margin* against the hop cliff.

## 4. Ignition delay and its dispersion (sim)
- Plant ignites at `fire_t + vcfg.ignition_delay_s` (`run.py`), nominal 0.5 s taken from
  `IGNITION_DELAY_MS` via the bridge.
- Monte Carlo: `ignition_delay_s` × U(0.85, 1.15) per flight (±75 ms, uniform). The table and any
  solver assume 500 ms. Real spread is **unknown** (see 01_research.md) — must be measured.

## 5. Thrust curve
`tools/motors/Estes_F15.eng` (header: "approximate curve from public cert data… replace with the
official file"): 50.4 N·s over 3.45 s, peak 25.3 N at 0.42 s. It is the Coker certification curve
with a simplified tail (+1.6 % impulse). **The plant (`plant.py`) and the table generator
(`burn_table.py`) read the same file** → the sim and the solver share the exact thrust model.

## 6. What the sim models / doesn't
Models: 6-DOF rigid body; thrust along the body axis through the (servo-lagged, slew-limited)
gimbal, so tilt really costs cos(tilt) of vertical thrust; axial drag 0.5ρ·CdA·v²; crossflow normal
force at the CP (CN_α = 2/rad) — a fin-less airframe that is **unstable nose-first (coast)** and
**weathervanes tail-first (descent)**; rotational damping; disturbance torques ∝ thrust; steady wind
+ optional gust; propellant mass loss; IMU noise (thrust-dependent) and constant biases; baro lag +
2 Pa noise; ground contact as a ~12 g arrest.
Monte Carlo dispersions (`sweep.py`): mass ±5 %, inertia ±15 %, CdA ±20 %, CG ±5 %, gimbal pivot
±3 %, servo slew ±10 %, servo lag ±25 %, ignition delay ±15 %, wind U(−4, 4) m/s.

**Not modelled** (before this study's scaffolding): motor-to-motor impulse / thrust-shape
variation; any ignition-delay shape other than uniform ±15 %; baro static-pressure error from
airflow; temperature effects; accel scale-factor error and IMU misalignment; CG shift during the
burn; leg dynamics (ground simply holds the vehicle after first contact, so a bounce/hop after
touchdown is not simulated). The scaffolding adds off-by-default knobs for landing thrust scale,
burn-time stretch, alternative curve, baro q-error, and uses the existing baro-lag and accel-bias
fields.

## 7. Baseline reproduction (must match 2026-10-12 exactly — it does)
| Check | 2026-10-12 | Now | Match |
|---|---|---|---|
| Seed-7 Monte Carlo, all 200 summaries | median 6.63, mean 8.51, 2 soft, 130 aborts | identical (byte-for-byte summaries) | ✅ |
| Same after adding scaffolding | — | 0 differing fields | ✅ |
| Driver `mc.py` vs `sweep.py` | — | 200/200 identical | ✅ |
| Oracle DLL with hook unused | — | 200/200 identical | ✅ |
| Clean nominal flight (seed 42) | 3.55 m/s | 3.551 m/s | ✅ |
| Clean ascent ×1.10 (65 m apogee) | 8.82 m/s, tilt-blocked | 8.818 m/s, fired on the tick tilt fell below 15° | ✅ |

## 8. Baseline characterization (dev seeds 7, 11, 13 × 200 = 600 flights)
- **Outcomes**: 228 powered landings (38 %); aborts: TILT during COAST 228 (38 %, before the solver
  ever acts — the fin-less airframe tumbles in coast), TILT during DESCENT 124 (21 %),
  MISSED_WINDOW 20 (3 %).
- Powered landings: median **7.10 m/s**, mean 9.96, p90 28.2; soft (< 2.5) 11; < 1 m/s 2.
- **Hops: 58 / 228 (25 %)** — fired too early *for that flight's* dispersed mass/delay/CdA; they
  stop in the air, climb, burn out and fall back at 3.9–11.5 m/s (53 above 5 m/s).
- **Tilt-limited fires: 62 / 228 (27 %)** — the window opened while tilt ≥ 15°; median touchdown
  **22.9 m/s**. They are the whole p90 tail.
- Descent speed at fire 13.7 / 15.3 / 26.5 m/s (5th/50th/95th pct); tilt at fire 8.3 / 12.3 / 15.0°.
- Clean apogee sweep (3 noise seeds): 2.2 m/s at 20 m apogee rising to 3.6 m/s at 48–52 m;
  tilt-limited from 56 m (5.2) → 8–9.4 m/s at 60–73 m; TILT aborts at ≥ 73–77 m.

## 9. Headroom ("oracle") analysis — the best ANY timing rule could do
Method: the 228 powered landings and 20 missed windows of the dev seeds were re-flown with the
fire command forced at a 20 ms grid (−1.2 s … +0.3 s around the natural fire time), then a 2 ms
grid around each flight's best point and hop boundary — 25 881 flights, 0 errors (sim-only test
hook, never in a solver). Everything before the forced fire is bit-identical to the original flight.

| powered landings (228) | median | mean | p90 |
|---|---|---|---|
| natural (current table on Kalman state) | 7.10 | 9.96 | 28.2 |
| current table on the TRUE state (perfect lag fix) | 6.60 | 6.65 | 10.4 |
| current table on Kalman state, tilt interlock ignored | 6.28 | 6.42 | 9.9 |
| best fire time while respecting tilt < 15° | 0.81 | 5.87 | 28.2 |
| best fire time, no constraint (knows the flight's unknowns) | 0.74 | 1.35 | 1.0 |

- Not tilt-limited (166): natural 6.01 median; a perfect lag fix *worsens* it to 6.45 (the lag is an
  accidental margin); the best constant margin on the true state gives 5.99 — i.e. the current table
  is already at the timing optimum for these flights.
- Tilt-limited (62): natural 22.9; best possible while respecting the 15° rule 20.2 (!); ignoring
  the interlock 7.6. Several need firing > 1.2 s before the tilt clears.
- The soft (< 2.5 m/s) fire-time window is ~10 ms wide (median), while the per-flight best fire time
  scatters ±100 ms (5–95 %) around the truth-fed table — that scatter is the unobservable delay,
  mass and drag of each flight.
- The 1-D physics model needs fires ~0.3 m *higher* than the 6-DOF sim would (r = 0.76 with |wind|).
