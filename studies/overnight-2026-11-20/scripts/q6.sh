#!/bin/bash
# task 6: true (plant) ignition delay x DUD_WINDOW_MS, screen 300; and real duds per window. Waits for q48.sh
cd "$(dirname "$0")"
while pgrep -f q48.sh >/dev/null; do sleep 10; done
export DISP="mass_descent_kg=0.01 ignition_delay_s=0.0"
for W in 200 300 400; do
  for X in 0.62 0.70 0.76 0.80 0.84 0.90 1.00; do
    PLANT_EXTRA="ignition_delay_s=$X" ./job.sh T6_d${X}_w$W base screen --fw-overrides "{\"DUD_WINDOW_MS\": \"$W.0f\"}"
  done
  STUDY_DUD=1 ./job.sh T6_dud_w$W base screen --fw-overrides "{\"DUD_WINDOW_MS\": \"$W.0f\"}"
done
echo Q6 DONE
