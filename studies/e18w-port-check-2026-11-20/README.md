# E18W port check (2026-11-20)

Before porting, this check proved the owner's 2026-11-19 values and the "chute opens ASAP" fixes in scratch copies of RocketFC. It used the closed-loop sim per the prove-before-changing rule.

**Copies**
- **A:** E18W/E18W, 1.4 kg (pad 1.4000 / descent 1.3793 kg), `ASCENT_BURN_MS 2140`, `LEGS_DELAY_MS 0`, burn table rebuilt. This is the motor-chute-study-2026-11-04 setup.
- **B:** A, plus:
  - `DUD_WINDOW_MS` 700 → 200;
  - ignition confirmed only above `TOUCHDOWN_ALT_M`;
  - the 55°↔110° latch wiggle (K = 2; latch-dud-study-2026-11-07).
- **C:** B, plus that study's servo-travel latch model ("repass", latch lets go at 80°, 375 °/s). C is used only to check the port reproduces the study.

**Settings**
- Measured spreads: mass ±1 %, delay ±5 %.
- Band cut 0.75 s.
- Wind −4…+4 m/s.

**Files**
- `run_proof.sh`, `proof_patch.py`: proof-only sim summary fields, the latch model, and a `PROOF_DUD` switch for `mc_batch.py`.
- `analyze.py` → `results.txt`.
- Raw results: `SpaceX Rocket\e18w-port-check-2026-11-20\` (`results_v1` = the first B/C build, see below).

## Results (final build)

| Check | A (setup only) | B (ported) |
|---|---|---|
| Holdout 701–710, 2,000 flights: success | 79.5 % | **79.5 %**, identical flight for flight (+0/−0) |
| Same, ignition delay ±15 % | 30.2 % | 30.2 % (+0/−0) |
| False dud aborts (both runs above) | 0 | **0** |
| Apogee median / max | 18.2 / 18.8 m | 18.2 / 18.8 m (limit 30.48 m) |
| Dud landing igniter (600): DUD_IGNITER abort | 0 % | **100 %** |
| Dud: chute released before impact | 0 % | **100 %** (median 0.43 s before impact) |
| Chute mode, latch stuck 1× / 2× (count model): canopy out before impact | 0 / 0 % | **100 / 100 %** (freed 0.34 / 0.64 s after the first release) |
| Servo-travel model (C), stuck 0 / 1 / 2 | — | 100 % each, freed 0.11 / 0.41 / 0.71 s (study: 0.1 / 0.4 / 0.7) |
| Healthy latch: impact speed, canopy confirmed | 6.97 m/s, 600/600 | 6.97 m/s, 600/600 |

## Bug found and fixed before the port
**Bug:** the first B build re-took the canopy detector's baseline at every wiggle stroke, as the study's code did. As a result, a healthy latch was **never confirmed** in chute mode (0/600; A: 600/600).

**Why:** with no opening spike, the g builds up slowly under the filling canopy (0.4 g at 0.34 s, 0.75 g at 0.62 s). Each stroke reset the baseline just below it. The chute still came out (same impact speed), but the latch kept re-cycling and the log said "no canopy".

**Fix:** keep the first release's baseline through the wiggle. Re-take it only at the slow re-cycles. A new replay unit check (slow g build-up after an apogee release) fails on the old code and passes on the fix. After the fix: 600/600 confirmed in every chute run, with every other number unchanged.
