# Motor / leg / software-fix study (2026-10-24): COMPLETE

Plan: `C:\Users\troop\.claude\plans\what-is-the-single-wobbly-sifakis.md`. Everything here is closed-loop simulator evidence; nothing has flown.

## Result
- **Ascent motor: AeroTech F10.** It's 29 mm single-use, 76 N·s, with a 7 s burn and a certified curve.
  - It keeps pushing nearly to apogee, so the fin-less body never coasts long enough to tip over: 0 % aborts, even in 4 m/s wind.
  - The short-burn legal motors (F44W, E28T, 41F36-11A) tip over during the coast: 94–100 % aborts.
  - Caveats:
    - T/W ≈ 1.1;
    - delays 4/6/8 s, so it has an ejection charge and needs a plugged build;
    - may be out of production;
    - no current price found (similar 29 mm 2-packs are ~$44–50).
- **Ported into RocketFC, switched on (uncommitted):**
  - S3 drift cancelling (`DRIFT_GAIN 2.0`, `DRIFT_MAX_TILT_DEG 5`);
  - `BURN_TABLE_MARGIN_M 1.0`;
  - a NaN guard before `burntable::hIgnite()` in `flight_state.h`.
  - Edits are in `port_fixes.py --margin`: `config.h`, `ahrs.h`, `flight_core.h`, `flight_state.h`.
- **Holdout** (fresh seeds 701-710, 2,000 flights, measured spreads), RocketFC before → after:

  | Ascent motor | Success | Paired | Aborts | Crashes |
  |---|---|---|---|---|
  | F10 | 6.05 % → **18.15 %** | 361 rescued / 119 lost, p ≈ 2e-29 | 0 % | −5.5 pt |
  | F15 (today) | 0 % → 0 % | | unchanged | −1.8 pt |

  - The ported tree reproduces the study copy on 2,000 / 2,000 flights for each motor.
  - With today's (unmeasured) spreads the F10 goes 2.2 % → 6.5 %.
- **Not ported:**
  - **S3 alone** narrowly failed the crash bar (+1.3 pt, limit +1.0); the S3 + margin pair passes cleanly.
  - **D2** had no effect on the F10, so your 15° rule stays.
  - **Wind-matched ascent:**
    - 45 rescued vs 35 lost on the F10, p = 0.31;
    - +9 pt crashes with the F15;
    - worse at ±1° gimbal error.
  - **LEGS_DELAY_MS 0** made no success difference at a ≤ 2 s band cut.
- **Legs** (F10, S3 + margin; 600 flights; 0.3 s swing estimate):

  | Nichrome on | Band cut 1.5 s | 2.0 s | 2.5 s | 3.0 s |
  |---|---|---|---|---|
  | 1 s after fire (today) | 100 % | **97.8 %** | 55.5 % | 18.8 % |
  | at ignition confirm (fire + 0.69 s) | 100 % | 100 % | 81.8 % | 46.3 % |

  So the band must part within **~2.0 s** of the nichrome turning on.
- **Tests on the ported RocketFC: all pass.**
  - replay `make test`;
  - `test_bridge.py`, `test_landing_outcome.py`;
  - `tools/hil/test_hil.py`, with the HIL traces regenerated (the old ones are in `hil_traces_pre_S3_2026-10-21/`);
  - `arduino-cli compile` for teensy41.

## To switch the firmware to the F10 (after buying and weighing it; all [MEASURE])
1. `MASS_PAD_KG 0.991`, `MASS_DESCENT_KG 0.951`, `ASCENT_BURN_MS 7050`.
2. `python tools/burn_table.py --mass 0.951`.
3. Add `motors/F10.eng` to `tools/motors/`.

`BOOST_MAX_MS` is `ASCENT_BURN_MS + 1500`, so flying an F10 on today's F15 settings would end boost steering at 5 s, mid-burn.

## Rebuilding the study
Run `python prep_ws.py <dir>` from Git Bash, then unzip `results.zip` there.
