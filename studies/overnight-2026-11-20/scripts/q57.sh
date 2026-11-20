#!/bin/bash
# tasks 5 and 7 on the study copy s57 (switches off by default); waits for earlier queues
cd "$(dirname "$0")"
while pgrep -f "q48.sh|q6.sh|qx1.sh" >/dev/null; do sleep 10; done
f() { echo "{\"$1\": \"$2\"}"; }
# task 5
for W in 5 6; do ./job.sh T5_base_w$W s57 dev --wind $W; done
for A in 5 6 8; do for W in 5 6; do ./job.sh T5_mw${A}_w$W s57 dev --wind $W --fw-overrides "$(f MISSED_WINDOW_ALT_M $A.0f)"; done; done
for D in 0.5 1.0 2.0; do for W in 5 6; do ./job.sh T5_wp${D}_w$W s57 dev --wind $W --fw-overrides "$(f STUDY_WP_D_M ${D}f)"; done; done
for A in 5 6 8; do ./job.sh T5_mw${A}_hold s57 hold --fw-overrides "$(f MISSED_WINDOW_ALT_M $A.0f)"; done
for D in 0.5 1.0 2.0; do ./job.sh T5_wp${D}_hold s57 hold --fw-overrides "$(f STUDY_WP_D_M ${D}f)"; done
# task 7 wind sweep
for W in 0 2 4 5 6 7; do
  ./job.sh T7_d2_w$W s57 screen --wind $W --fw-overrides "$(f STUDY_D2 1)"
  ./job.sh T7_wma_w$W s57 screen --wind $W --fw-overrides "$(f STUDY_WMA 1)"
  for G in 1.0 3.0 4.0; do ./job.sh T7_dg${G}_w$W s57 screen --wind $W --fw-overrides "$(f DRIFT_GAIN ${G}f)"; done
  for T in 3.0 8.0; do ./job.sh T7_dt${T}_w$W s57 screen --wind $W --fw-overrides "$(f DRIFT_MAX_TILT_DEG ${T}f)"; done
done
for O in 0.5 -0.5; do for W in 0 5; do
  PLANT_EXTRA="gimbal_offset_deg=$O" ./job.sh T7_off${O}_w$W s57 screen --wind $W
  PLANT_EXTRA="gimbal_offset_deg=$O" ./job.sh T7_wma_off${O}_w$W s57 screen --wind $W --fw-overrides "$(f STUDY_WMA 1)"
done; done
echo Q57 DONE
