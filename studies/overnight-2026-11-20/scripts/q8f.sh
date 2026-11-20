#!/bin/bash
# task 8 finalists: F1 landing kd x0.5; F2 landing kp x0.75 kd x0.35. holdout, fresh, wind 5, inertia +-30 %, slow servo
cd "$(dirname "$0")"
F1='{"GAINS_LANDING": {"kd": "0.065f"}}'
F2='{"GAINS_LANDING": {"kp": "0.6f", "kd": "0.0455f"}}'
for F in F1 F2; do eval O=\$$F
  ./job.sh T8${F}_hold base hold --fw-overrides "$O"
  ./job.sh T8${F}_fresh base fresh --fw-overrides "$O"
  ./job.sh T8${F}_w5 base dev --wind 5 --fw-overrides "$O"
done
./job.sh T8base_w5dev base dev --wind 5
for P in "inertia_kgm2=0.12407" "inertia_kgm2=0.23041" "servo_slew_dps=240 servo_lag_tau_s=0.016"; do
  tag=$(echo $P | tr ' =' '__')
  PLANT_EXTRA="$P" ./job.sh T8base_$tag base dev
  for F in F1 F2; do eval O=\$$F; PLANT_EXTRA="$P" ./job.sh T8${F}_$tag base dev --fw-overrides "$O"; done
done
echo Q8F DONE
