#!/bin/bash
# task 7 finalist: wind-matched ascent (STUDY_WMA=1): holdout, fresh, dev at winds 4 and 7, screen at wind 8
cd "$(dirname "$0")"
O='{"STUDY_WMA": "1"}'
./job.sh T7_wma_hold s57 hold --fw-overrides "$O"
./job.sh T7_wma_fresh s57 fresh --fw-overrides "$O"
for W in 4 7; do ./job.sh T7_base_w${W}dev s57 dev --wind $W; ./job.sh T7_wma_w${W}dev s57 dev --wind $W --fw-overrides "$O"; done
./job.sh T7_base_w8 s57 screen --wind 8; ./job.sh T7_wma_w8 s57 screen --wind 8 --fw-overrides "$O"
echo Q7F DONE
