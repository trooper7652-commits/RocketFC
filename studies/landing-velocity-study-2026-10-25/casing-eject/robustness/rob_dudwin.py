"""Robustness study: does a 300 ms dud window cause false dud aborts on good flights?"""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lvlib as L
if __name__ == "__main__":
    c = json.load(open(os.path.join(HERE, "combos.json")))["c1_e12_e12"]
    P = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75,
         "ascent_casing_eject_kg": 0.0391}
    FW = {"LEGS_DELAY_MS": "0.0f", "BURN_TABLE_MARGIN_M": "1.0f"}
    R = {"fix, dud window 700 ms": L.run("f_c1_b80_m1.0", ws="c1_ej_b80", plant=P, fw=FW),
         "fix, dud window 300 ms": L.run("r_dudwin300", ws="c1_ej_b80", plant=P, fw=dict(FW, DUD_WINDOW_MS="300.0f")),
         "fix, dud window 300 ms, delay scatter +-15 %": L.run("r_dudwin300_dsc15", ws="c1_ej_b80", plant=P,
                                                               fw=dict(FW, DUD_WINDOW_MS="300.0f"),
                                                               disp={"mass_descent_kg": 0.01, "ignition_delay_s": 0.15})}
    L.table(R, "fix, dud window 700 ms")
    for lab, rows in R.items():
        n = sum(1 for r in rows if r["summary"].get("abort_reason") == "DUD_IGNITER")
        print("   %-46s false dud aborts: %d" % (lab, n))
