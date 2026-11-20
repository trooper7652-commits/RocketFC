#!/bin/bash
# all recommended patches together (margin 0.75, window-passed 0.5, wind-matched ascent on)
cd "$(dirname "$0")"
O='{"WIND_MATCH_ASCENT": "1", "WINDOW_PASSED_D_M": "0.5f"}'
./job.sh ALL_hold all hold --fw-overrides "$O"
for W in 0 5 7; do ./job.sh ALL_w$W all screen --wind $W --fw-overrides "$O"; done
echo QALL DONE
