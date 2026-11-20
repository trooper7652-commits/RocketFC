#!/bin/bash
# task 7 combo robustness: dev at w4/w5, gimbal offset +-0.5 deg at w0/w5, slow servo, inertia +-30 %
cd "$(dirname "$0")"
O='{"STUDY_WMA": "1", "BURN_TABLE_MARGIN_M": "0.75f"}'
./job.sh T7c_w4dev s57 dev --wind 4 --fw-overrides "$O"
./job.sh T7c_w5dev s57 dev --wind 5 --fw-overrides "$O"
for Of in 0.5 -0.5; do for W in 0 5; do PLANT_EXTRA="gimbal_offset_deg=$Of" ./job.sh T7c_off${Of}_w$W s57 screen --wind $W --fw-overrides "$O"; done; done
for P in "servo_slew_dps=240 servo_lag_tau_s=0.016" "inertia_kgm2=0.12407" "inertia_kgm2=0.23041"; do
  tag=$(echo $P | tr ' =' '__'); PLANT_EXTRA="$P" ./job.sh T7c_$tag s57 dev --fw-overrides "$O"; done
echo Q7R DONE
