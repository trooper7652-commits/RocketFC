#!/bin/bash
# mkws.sh <name> [git-ref]: scratch copy of RocketFC at /tmp/ws/<name> with the study summary fields.
set -e
N=$1; REF=${2:-origin/main}; D=/tmp/ws/$N
R=$(cd "$(dirname "$0")/../../.." && pwd)
rm -rf $D; mkdir -p $D
(cd $R && git archive $REF) | tar -x -C $D
python "$R/studies/overnight-2026-11-20/scripts/sim_fields.py" $D
(cd $D/tools/sim && python -c "import core; print(\"built\", core.default_dll())")
