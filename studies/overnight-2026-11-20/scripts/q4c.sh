#!/bin/bash
# task 4 confirm: fresh seeds 801-810 (baseline + margin 0.75), and wind robustness screens
cd "$(dirname "$0")"
M='{"BURN_TABLE_MARGIN_M": "0.75f"}'
./job.sh T0_fresh base fresh
./job.sh T4_m0.75_fresh base fresh --fw-overrides "$M"
for W in 0 2 4 5 6 7; do ./job.sh T4_m0.75_w$W base screen --wind $W --fw-overrides "$M"; done
echo Q4C DONE
