#!/bin/bash
# port check: the unapplied task-5 patch at 0.5 m must reproduce the study switch flight for flight
cd "$(dirname "$0")"
while pgrep -f "q5f.sh" >/dev/null; do sleep 10; done
./job.sh P5_wp0.5_w6 p5 dev --wind 6 --fw-overrides '{"WINDOW_PASSED_D_M": "0.5f"}'
./job.sh P5_off_w6 p5 dev --wind 6
echo QP5 DONE
