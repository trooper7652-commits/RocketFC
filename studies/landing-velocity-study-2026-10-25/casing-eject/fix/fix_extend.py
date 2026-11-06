"""Fix study: extend the E12 sweep past its edge (100 g) and fine-tune the margin at 80 g."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lvlib as L
import fix_sweep as F
if __name__ == "__main__":
    F.E12_WS[100] = "c1_ej_b100"
    F.make_ws("c1_ej_b100", "c1_eject", 0.100, F.E12_EJ, os.path.join("tools", "motors", "landing.eng"))
    c = json.load(open(os.path.join(HERE, "combos.json")))["c1_e12_e12"]
    P75 = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75,
           "ascent_casing_eject_kg": F.E12_EJ}
    R = {}
    for g, mg in ((60, 1.0), (80, 1.0), (80, 0.75), (80, 1.25), (100, 0.75), (100, 1.0), (100, 1.25)):
        R["%d g, margin %.2f m" % (g, mg)] = L.run("f_c1_b%d_m%s" % (g, ("%.2f" % mg).rstrip("0").rstrip(".") if mg not in (0.5, 1.0, 1.5) else "%.1f" % mg),
                                                ws=F.E12_WS[g], plant=P75,
                                                fw={"LEGS_DELAY_MS": "0.0f", "BURN_TABLE_MARGIN_M": "%.2ff" % mg})
    L.table(R, "80 g, margin 1.00 m")
    for lab, rows in R.items():
        print("   %-20s apogee median %.1f m" % (lab, F.apogee_med(rows)))
