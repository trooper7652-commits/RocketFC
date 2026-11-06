# Casing-eject study (2026-10-25, report-only, scratch copies; RocketFC unchanged)

Question: the landing motor's exhaust pushes the spent ascent casing out of the
tail at landing thrust onset. How does that change landing success?

Model (`eject_model.diff`, built by `eject_setup.py`):
- `VehicleConfig.ascent_casing_eject_kg`: mass drops at landing thrust onset.
  The CG moves forward, inertia loses the casing's share, and the standing CG
  rises. A knob of 0 reproduces the old holdout 2000/2000.
- `burn_table.py --eject-kg`: the table is rebuilt for the lighter burn.

Casing mass: F10 43 g; E12 39.1 g (spent reload + R/C case + adapter, using the
earlier study's guessed mass).

## Holdout 701-710 (2,000 flights, measured spreads, paired)

| Setup | Casing stays | Pushed out | Pushed out + ballast = casing mass |
|---|---|---|---|
| Today F10 + F15 | 26.0 % | **8.6 %** (4 rescued / 352 lost) | 10.4 % (apogee 73 → 60 m) |
| E12/E12, legs as today | 37.0 % | **45.3 %** (285 / 120) | not run |
| E12/E12, legs at confirm + 0.75 s band | 86.8 % | **69.7 %** (0 / 343) | **80.2 %** (apogee 26 → 23 m) |
| Today, burn table NOT rebuilt | | 0.1 % | |

## Why
- **Mass only:** the dev result is the same whether or not the casing CG
  shift is modelled.
- **A lighter rocket makes the same motor relatively stronger.** Lift-offs go
  38 → 47 % (F10) and 11 → 23 % (E12 best), so the soft window narrows.
- **Ballast helps E12** (the landing mass is restored), **but not today's
  setup:** the weak F10 carries the extra 43 g only to 60 m, the fall is too slow
  for the F15, and lift-offs stay at 47 %.

## Sensitivity (dev 101-103)
- **Re-tuning `BURN_TABLE_MARGIN_M`** (−0.5 … 2.0 m) doesn't recover it.
  The best was 8.0 % at 2.0 m (today) and 65.8 % at 1.5 m (E12), neither
  significant.
- **Eject kick ±0.2 N·s:** no significant effect on today's setup; E12 −0.2 N·s
  costs 7 points (p = 0.001).
