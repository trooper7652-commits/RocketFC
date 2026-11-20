#!/bin/bash
# task 4 fine grid around the dev optimum (0.75), then holdout for 0.75 and the best fine value
cd "$(dirname "$0")"
while pgrep -f q48.sh >/dev/null; do sleep 10; done
for M in 0.625 0.7 0.8 0.875; do ./job.sh T4_m$M base dev --fw-overrides "{\"BURN_TABLE_MARGIN_M\": \"${M}f\"}"; done
./job.sh T4_m0.75_hold base hold --fw-overrides '{"BURN_TABLE_MARGIN_M": "0.75f"}'
echo Q4B DONE
