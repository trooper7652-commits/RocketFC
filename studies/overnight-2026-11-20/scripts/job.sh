#!/bin/bash
# job.sh <name> <ws> <set: screen|dev|hold|fresh> [mc_batch args...]  (defaults: land, measured disp, legs_cut 0.75)
# extra --plant k=v pairs: pass PLANT_EXTRA="k=v ..." in the env. STUDY_DUD=1 for a dud landing igniter.
N=$1; WS=$2; SET=$3; shift 3
R=$(cd "$(dirname "$0")/../../.." && pwd); OUT=$R/studies/overnight-2026-11-20/results
mkdir -p /tmp/res $OUT
[ -f $OUT/$N.csv.gz ] && { echo "skip $N (CSV committed)"; exit 0; }
case $SET in screen) S="101 102 103"; n=100;; dev) S="101 102 103"; n=200;;
  untouched) S="901 902 903 904 905 906 907 908 909 910"; n=200;;
  hold) S="701 702 703 704 705 706 707 708 709 710"; n=200;; fresh) S="801 802 803 804 805 806 807 808 809 810"; n=200;; esac
MODE=${MODE:-land}; DISP=${DISP:-"mass_descent_kg=0.01 ignition_delay_s=0.05"}
(cd /tmp/ws/$WS/tools/sim && python mc_batch.py --seeds $S --n $n --workers ${WORKERS:-4} --mode $MODE \
   --disp $DISP --plant legs_cut_s=0.75 $PLANT_EXTRA "$@" --out /tmp/res/$N.json 2>&1 | tail -1) \
 && python $R/studies/overnight-2026-11-20/scripts/to_csv.py /tmp/res/$N.json $OUT/$N.csv.gz $N
