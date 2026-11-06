import json, sys, os, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lvlib as L
if __name__ == "__main__":
    c = json.load(open("combos.json"))["c1_e12_e12"]
    P75 = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75}
    R = {}
    for w in (0, 2, 3, 4, 5, 6):
        R["E12/E12 wind %d m/s" % w] = L.run("w_c1_%d" % w, ws=c["ws"], fw={"LEGS_DELAY_MS": "0.0f"}, plant=P75, wind=w)
    L.table(R, "E12/E12 wind 0 m/s")
    for lab, rows in R.items():
        ab = collections.Counter(r["summary"].get("abort_state") for r in rows if r["summary"].get("abort_reason"))
        print("   %-22s aborts by phase: %s" % (lab, dict(ab)))
