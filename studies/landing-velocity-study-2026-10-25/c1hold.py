import json, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lvlib as L
if __name__ == "__main__":
    c = json.load(open("combos.json"))["c1_e12_e12"]
    P = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"]}
    LEGS = {"LEGS_DELAY_MS": "0.0f"}
    P75 = dict(P, legs_cut_s=0.75)
    print("== HOLDOUT seeds 701-710, mass weighed + delay measured")
    H = {"today (F10 + F15)": L.run("h_base", seeds=L.HOLD)}
    H["E12/E12, legs as today"] = L.run("h_c1", ws=c["ws"], seeds=L.HOLD, plant=P)
    H["E12/E12, confirm + 0.75 s band"] = L.run("h_c1_best", ws=c["ws"], seeds=L.HOLD, fw=LEGS, plant=P75)
    L.table(H, "today (F10 + F15)")
    print("\n== STRESS (dev seeds 101-103), best E12/E12 leg setup")
    S = {"E12/E12 best (dev)": L.run("c1_dev_conf_cut075", ws=c["ws"], fw=LEGS, plant=P75)}
    S["+ today's guesses (mass 5 %, delay 15 %)"] = L.run("s_c1_guess", ws=c["ws"], disp={}, fw=LEGS, plant=P75)
    S["+ E12 thrust spread +-3 % (both motors)"] = L.run("s_c1_spread", ws=c["ws"], fw=LEGS, plant=P75,
                                                       ext=["landing_thrust_scale:0.97:1.03"])
    S["+ steady wind 6 m/s"] = L.run("s_c1_wind6", ws=c["ws"], fw=LEGS, plant=P75, wind=6)
    L.table(S, "E12/E12 best (dev)")
    T = {"today (dev)": L.run("b_base"), "today + today's guesses": L.run("s_base_guess", disp={}),
         "today + wind 6": L.run("s_base_wind6", wind=6)}
    print()
    L.table(T, "today (dev)")
