#!/bin/bash
# task 5 confirm: window-passed D=0.5 on fresh seeds (default wind, wind 6) and dev wind 7
cd "$(dirname "$0")"
O='{"STUDY_WP_D_M": "0.5f"}'
./job.sh T5_wp0.5_fresh s57 fresh --fw-overrides "$O"
./job.sh T5_base_w6_fresh s57 fresh --wind 6
./job.sh T5_wp0.5_w6_fresh s57 fresh --wind 6 --fw-overrides "$O"
./job.sh T5_base_w7 s57 dev --wind 7
./job.sh T5_wp0.5_w7 s57 dev --wind 7 --fw-overrides "$O"
echo Q5F DONE
