#!/bin/bash
# task 7 combo: wind-matched ascent + burn-table margin 0.75 (both off-by-default study/fw knobs)
cd "$(dirname "$0")"
O='{"STUDY_WMA": "1", "BURN_TABLE_MARGIN_M": "0.75f"}'
for W in 0 2 4 5 6 7 8; do ./job.sh T7c_wma_m0.75_w$W s57 screen --wind $W --fw-overrides "$O"; done
./job.sh T7c_wma_m0.75_hold s57 hold --fw-overrides "$O"
./job.sh T7c_wma_m0.75_fresh s57 fresh --fw-overrides "$O"
echo Q7C DONE
