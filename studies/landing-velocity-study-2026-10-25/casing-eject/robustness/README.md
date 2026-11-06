# Robustness study of the casing-eject fix (2026-10-26, report-only)

- **Fix:** E12/E12, casing pushed out, 80 g ballast, margin 1.0 m, legs at confirm + 0.75 s band.
- **Base:** dev 101-103, 84.0 % (holdout 86.5 %).
- **Method:** each row changes one thing and is paired against the base (600 flights). All runs had 0 errors. The knob copy with default settings reproduces the base 600/600.

## Ranked: what costs the most if it turns out worse than assumed

| Unknown | Tested | Success | Note |
|---|---|---|---|
| **E12 ignition-delay scatter** | ±10 % / ±15 % (base ±5 %) | 45.2 % / 29.3 % | Delay known exactly → **100 %** (median 1.36 m/s) |
| **E12 landing thrust spread** | ±3 % / ±5 % | 67.3 % / 51.0 % | |
| **Crash speed the legs survive** | 2.0 / 2.5 / 3.5 / 4.0 / 5.0 m/s (base 3.0) | 65.8 / 76.8 / 92.0 / 94.3 / 95.0 % | Holdout, re-classified |
| Casing stuck (never leaves) | | 42.8 % | |
| Eject kick | +0.2 / −0.2 N·s | 71.5 % / 76.8 % | |
| Casing leaves late | 0.1 s / 0.3 s | 85.3 % / 77.8 % | |
| Leg band cut | 0.5–1.25 s / 1.5 s | 84.0 % / 68.8 % | |
| E12 delay mean, table built for it | 400 / 800 / 1000 ms | 84.8 / 82.7 / 80.5 % | |
| Random 1 s gusts | up to 4 / 6 m/s | 83.3 % / 81.7 % | |
| Ascent thrust spread | ±3 % | 84.0 % | No effect |

**Error budget (one source removed):**
- delay scatter → 100 %;
- mass → 86.3 %;
- sensors → 86.3 %;
- drag, airframe, wind → about 0 change;
- perfect knowledge → 100 % (median 1.48 m/s).

**Remaining holdout failures (13.4 %):** legs broken (> 3 m/s) 8.6 %; lifted off 4.9 %. Nothing else.

## Dud landing igniter (200 flights each)

| Setup | Fire height | Canopy open before ground | Impact median |
|---|---|---|---|
| Fix, `DUD_WINDOW_MS` 700 (today) | 17.3 m | 0 % | 18.9 m/s |
| Fix, `DUD_WINDOW_MS` 300 | 17.3 m | **100 %** | **7.5 m/s** |
| Today's F10 + F15, 700 ms | 54.3 m | 100 % | 5.8 m/s |

A 300 ms window causes 0 false dud aborts on good flights, even with ±15 % delay scatter. It doesn't change success (paired 0 / 0).
