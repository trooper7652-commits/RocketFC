#!/bin/bash
# task 6 guard variant (scratch s6, STUDY_DUD_GUARD=1), paired against T6_d*_w200 (base)
cd "$(dirname "$0")"
while pgrep -f "q6.sh" >/dev/null; do sleep 10; done
export DISP="mass_descent_kg=0.01 ignition_delay_s=0.0"
for X in 0.62 0.80 0.84 0.90 1.00; do
  PLANT_EXTRA="ignition_delay_s=$X" ./job.sh T6g_d$X s6 screen --fw-overrides '{"STUDY_DUD_GUARD": "1"}'
done
STUDY_DUD=1 ./job.sh T6g_dud s6 screen --fw-overrides '{"STUDY_DUD_GUARD": "1"}'
echo Q6G DONE
