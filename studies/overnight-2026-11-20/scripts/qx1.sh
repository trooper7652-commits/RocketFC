#!/bin/bash
cd "$(dirname "$0")"
while pgrep -f "q48.sh|q6.sh" >/dev/null; do sleep 10; done
./job.sh T4_m0 base dev --fw-overrides '{"BURN_TABLE_MARGIN_M": "0.0f"}'
echo QX1 DONE
