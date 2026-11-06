"""Fix study, step 5: holdout 701-710 for the winner (E12/E12, casing out,
80 g ballast at the balance point, margin 1.0 m, legs at confirm + 0.75 s band),
plus steady wind 6-7 m/s on dev."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lvlib as L
import fix_sweep as F
import fix_location as FL
if __name__ == "__main__":
    c = json.load(open(os.path.join(HERE, "combos.json")))["c1_e12_e12"]
    P75 = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75}
    LEGS = {"LEGS_DELAY_MS": "0.0f"}
    ej = dict(P75, ascent_casing_eject_kg=F.E12_EJ)
    fix = dict(ej, **FL.point_mass_overrides(0.080, None, c["landing_eng"]))
    FW = dict(LEGS, BURN_TABLE_MARGIN_M="1.0f")
    H = {"casing aboard (earlier best)": L.run("h_c1_best", ws=c["ws"], seeds=L.HOLD, fw=LEGS, plant=P75),
         "casing out, no fix": L.run("h_c1_best_ej", ws="c1_eject", seeds=L.HOLD, fw=LEGS, plant=ej),
         "casing out + 80 g ballast (fix)": L.run("h_fix_b80", ws="c1_ej_b80", seeds=L.HOLD, fw=FW, plant=fix)}
    L.table(H, "casing aboard (earlier best)")
    print()
    L.table({"casing out, no fix": H["casing out, no fix"], "casing out + 80 g ballast (fix)": H["casing out + 80 g ballast (fix)"]},
            "casing out, no fix")
    for lab, rows in H.items():
        print("   %-34s apogee median %.1f m" % (lab, F.apogee_med(rows)))
    print()
    W = {}
    for w in (6, 7):
        W["casing aboard, wind %d" % w] = L.run("w_c1_%d" % w, ws=c["ws"], fw=LEGS, plant=P75, wind=w)
        W["fix, wind %d" % w] = L.run("f_wind_b80_m1.0_w%d" % w, ws="c1_ej_b80", plant=dict(ej), fw=FW, wind=w)
    L.table(W, "casing aboard, wind 6")
