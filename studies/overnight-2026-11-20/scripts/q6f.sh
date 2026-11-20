#!/bin/bash
# task 6 extra: success vs exact true ignition delay near the 620 ms nominal (today vs margin 0.75)
cd "$(dirname "$0")"
while pgrep -f "q4c.sh" >/dev/null; do sleep 10; done
export DISP="mass_descent_kg=0.01 ignition_delay_s=0.0"
for X in 0.56 0.58 0.60 0.64 0.66 0.68; do
  PLANT_EXTRA="ignition_delay_s=$X" ./job.sh T6_d${X}_w200 base screen
  PLANT_EXTRA="ignition_delay_s=$X" ./job.sh T6m_d$X base screen --fw-overrides '{"BURN_TABLE_MARGIN_M": "0.75f"}'
done
PLANT_EXTRA="ignition_delay_s=0.62" ./job.sh T6m_d0.62 base screen --fw-overrides '{"BURN_TABLE_MARGIN_M": "0.75f"}'
echo Q6F DONE
