#!/bin/bash
cd "$(dirname "$0")"
while pgrep -f "q57.sh" >/dev/null; do sleep 10; done
./job.sh T1_dev t1 dev
echo T1 DONE
