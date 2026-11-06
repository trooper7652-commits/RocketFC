# Fix study: win back landing success with the casing pushed out (2026-10-25, report-only)

Winner: **E12/E12 + 80 g of fixed ballast**, with the burn table rebuilt for the
casing leaving at landing thrust onset. The fire margin stays at 1.0 m; legs on
at ignition confirm, band cut ≤ 0.75 s. Files: `config_winner.h` and
`burn_table_winner.h`.

## Holdout 701-710 (2,000 flights, measured spreads, paired)

| Setup | Success | Contact vz median / p90 | Lifted off | Apogee |
|---|---|---|---|---|
| Casing aboard (earlier best) | 86.8 % | 2.22 / 2.96 m/s | 11 % | 26 m |
| Casing out, no fix | 69.7 % | 1.75 / 3.12 | 23 % | 26 m |
| **Casing out + 80 g ballast** | **86.5 %** | **1.67 / 2.92** | **5 %** | 19.5 m |

- vs casing aboard: 140 rescued / 146 lost, p = 0.77 (the same).
- vs no fix: 390 rescued / 53 lost.

## Dev sweeps (101-103)

**E12/E12 ballast** (margin 1.0 m): 0 g 63.7 %, 20 g 71.2 %, 39 g 78.3 %, 60 g 82.5 %, 80 g 84.0 %, 100 g 83.8 %.
- The curve is flat from 60 to 100 g, so the exact grams aren't critical.
- Margins 0.5 m and 1.5 m are worse at every ballast. 0.75 m and 1.25 m at 80 g are no better.

**Ballast location** (80 g): balance point 84.2 %, nose 83.2 %, low by the motor mount 84.8 %. No difference is significant.

**Steady wind:**

| Wind | 0 m/s | 3 m/s | 5 m/s | 6 m/s | 7 m/s |
|---|---|---|---|---|---|
| Casing aboard | 92.2 % | 83.0 % | 27.3 % | 2.2 % | 0 % |
| Fix | 81.8 % | 86.3 % | 82.7 % | 70.5 % | 51.5 % |

The descent tilt aborts in wind disappear with the heavier, lower flight.

**F10 + F15:** ballast 0 / 20 / 43 g × margin 0.5 / 1.0 / 2.0 m. The best result is 9.3 %, against 22.3 % with the casing aboard. It can't be recovered, because the weak F10 loses apogee as soon as weight is added.

## Caveats
- Pad mass is 1.025 kg, so the E12's average thrust-to-weight off the pad is ~1.12 (initial peak ~1.8).
- The ejected mass (39.1 g) and the E12's ignition delay are unmeasured.
- An eject kick of −0.2 N·s cost about 7 points (measured without ballast).
