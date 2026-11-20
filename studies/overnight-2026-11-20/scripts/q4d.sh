#!/bin/bash
# task 4: compromise margins in steady wind
cd "$(dirname "$0")"
for M in 0.875 0.8; do for W in 0 4 5 6 7; do
  ./job.sh T4_m${M}_w$W base screen --wind $W --fw-overrides "{\"BURN_TABLE_MARGIN_M\": \"${M}f\"}"
done; done
echo Q4D DONE
