import json, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lvlib as L
if __name__ == "__main__":
    c = json.load(open("combos.json"))["c1_e12_e12"]
    P = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"]}
    R = {"today (F10 + F15)": L.run("b_base")}
    R["E12/E12, legs as today"] = L.run("c1_dev", ws=c["ws"], plant=P)
    R["E12/E12, nichrome at confirm"] = L.run("c1_dev_conf", ws=c["ws"], plant=P, fw={"LEGS_DELAY_MS": "0.0f"})
    R["E12/E12, at confirm + 0.75 s band"] = L.run("c1_dev_conf_cut075", ws=c["ws"], fw={"LEGS_DELAY_MS": "0.0f"},
                                                    plant=dict(P, legs_cut_s=0.75))
    R["E12/E12, at confirm + 0.5 s band"] = L.run("c1_dev_conf_cut05", ws=c["ws"], fw={"LEGS_DELAY_MS": "0.0f"},
                                                   plant=dict(P, legs_cut_s=0.5))
    L.table(R, "today (F10 + F15)")
    for lab, rows in R.items():
        late = sum(1 for r in rows if "LEGS_NOT_DOWN" in r["summary"]["landing_outcome"]["factors"]) / len(rows)
        ap = sorted(r["summary"]["apogee_h"] for r in rows)
        print("   %-36s legs late %3.0f %% | apogee p10/median/p90 %.0f/%.0f/%.0f m" % (lab, 100 * late, ap[60], ap[300], ap[540]))
