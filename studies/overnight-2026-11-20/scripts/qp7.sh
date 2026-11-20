#!/bin/bash
cd "$(dirname "$0")"
while pgrep -f "q7r.sh" >/dev/null; do sleep 10; done
./job.sh P7c_w6 p7 screen --wind 6 --fw-overrides '{"WIND_MATCH_ASCENT": "1"}'
echo QP7 DONE
