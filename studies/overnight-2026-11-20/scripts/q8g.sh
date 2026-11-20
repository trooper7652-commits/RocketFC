#!/bin/bash
# task 8: 3x3 landing kp x kd grid around the best landing set (kd x0.5), screen 300
cd "$(dirname "$0")"
while pgrep -f "q6.sh|q6g.sh|qx1.sh|q4b.sh" >/dev/null; do sleep 10; done
for P in 0.75 1.0 1.5; do for D in 0.35 0.5 0.65; do
  kp=$(python -c "print(round(0.80*$P,5))"); kd=$(python -c "print(round(0.130*$D,5))")
  ./job.sh T8g_kp${P}_kd$D base screen --fw-overrides "{\"GAINS_LANDING\": {\"kp\": \"${kp}f\", \"kd\": \"${kd}f\"}}"
done; done
echo Q8G DONE
