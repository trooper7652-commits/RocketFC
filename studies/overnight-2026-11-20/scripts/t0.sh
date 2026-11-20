#!/bin/bash
cd "$(dirname "$0")"
./job.sh T0_dev base dev
for W in 0 2 4 5 6 7; do ./job.sh T0_w$W base screen --wind $W; done
./job.sh T0_hold base hold
echo T0 DONE
