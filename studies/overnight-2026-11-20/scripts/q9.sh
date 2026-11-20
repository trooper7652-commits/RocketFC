#!/bin/bash
# never-used seeds 901-910: baseline vs WMA + margin 0.75 (the combination was designed after seeing holdout/fresh)
cd "$(dirname "$0")"
./job.sh T0_s900 s57 untouched
./job.sh T7c_s900 s57 untouched --fw-overrides '{"STUDY_WMA": "1", "BURN_TABLE_MARGIN_M": "0.75f"}'
echo Q9 DONE
