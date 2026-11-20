#!/bin/bash
# task 4 margin sweep (dev) + task 8 one-at-a-time gain screens; waits for t0.sh
cd "$(dirname "$0")"
while pgrep -f t0.sh >/dev/null; do sleep 10; done
for M in 0 0.25 0.5 0.75 1.25 1.5; do ./job.sh T4_m$M base dev --fw-overrides "{\"BURN_TABLE_MARGIN_M\": \"${M}f\"}"; done
g() { ./job.sh T8_$1 base screen --fw-overrides "{\"$2\": {\"$3\": \"$4f\"}}"; }
for S in B L; do
  [ $S = B ] && { G=GAINS_BOOST; KP=0.60; KI=0.15; KD=0.110; } || { G=GAINS_LANDING; KP=0.80; KI=0.20; KD=0.130; }
  for x in 0.5 0.75 1.5 2; do
    g ${S}_kp$x $G kp $(python -c "print(round($KP*$x,5))")
    g ${S}_kd$x $G kd $(python -c "print(round($KD*$x,5))")
  done
  g ${S}_ki0 $G ki 0.0; g ${S}_ki2 $G ki $(python -c "print(round($KI*2,5))")
done
echo Q48 DONE
