#!/bin/bash
# Proof batch: copies A (motor/mass/legs only), B (+ dud 200 ms, alt gate, wiggle), C (B + latch travel model)
P=/c/Users/troop/AppData/Local/Temp/claude/3e9e20f0e564c4e3/f180f36a-d407-44ff-b013-b2ec3e4fd03d/scratchpad/proof
R=$P/results
HOLD="701 702 703 704 705 706 707 708 709 710"
DEV="101 102 103"
MEAS="mass_descent_kg=0.01 ignition_delay_s=0.05"
S15="mass_descent_kg=0.01 ignition_delay_s=0.15"
job() {  # name copy mode seeds n disp plant... ; env PROOF_DUD optional
  local name=$1 w=$2 mode=$3 seeds=$4 n=$5 disp=$6; shift 6
  [ -f $R/$name.json ] && { echo "skip $name"; return; }
  (cd $P/$w/tools/sim && python mc_batch.py --seeds $seeds --n $n --workers 19 --mode $mode \
     --disp $disp --plant legs_cut_s=0.75 "$@" --out $R/$name.json 2>&1 | tail -1)
}
job A_hold   A land  "$HOLD" 200 "$MEAS"
job B_hold   B land  "$HOLD" 200 "$MEAS"
job A_hold15 A land  "$HOLD" 200 "$S15"
job B_hold15 B land  "$HOLD" 200 "$S15"
export PROOF_DUD=1; job A_dud A land "$DEV" 200 "$MEAS"
job B_dud B land "$DEV" 200 "$MEAS"; unset PROOF_DUD
for s in 0 1 2; do
  job A_chute_s$s A chute "$DEV" 200 "$MEAS" chute_stuck_releases=$s
  job B_chute_s$s B chute "$DEV" 200 "$MEAS" chute_stuck_releases=$s
  job C_latch_s$s C chute "$DEV" 200 "$MEAS" chute_stuck_releases=$s latch_rate_dps=375 latch_release_deg=80
done
echo PROOF DONE
