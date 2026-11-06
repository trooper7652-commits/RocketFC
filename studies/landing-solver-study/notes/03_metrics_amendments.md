# 03 — Amendments / observations (the pre-registered file `03_metrics.md` is unchanged;
# its SHA-256 `2d5f3a8c…` still matches)

2026-10-12, after the holdout verdicts (no criterion was changed or loosened):

1. **Blind spot in criterion 4.** It stops a solver from looking better by turning bad landings
   into chute aborts. It does not cover the reverse: D2 turns MISSED_WINDOW aborts into powered
   landings. On the Track-2 holdout those 65 rescued flights hit at a median 28.6 m/s, versus
   24.4 m/s as aborts (the chute is released at 4 m, too low to open). They are excluded from the
   paired speed statistics because they aren't powered landings in both runs. Reported separately
   in REPORT.md as an "every flight that reached the landing decision" comparison.
2. **Hop count and denominators.** Hops are counted over all powered landings, so a candidate
   that rescues missed windows can add hops simply by having more landings. Kept as
   pre-registered (strict total). Noted in the report.
3. **Robustness gate run after failure.** Both finalists had already failed criteria 1 (and others)
   on the holdout. The robustness matrices were still run (Track 2, Track 1, and post-hoc D2-only)
   because they answer the brief's separate question: is the new code overfitted to the simulator?
