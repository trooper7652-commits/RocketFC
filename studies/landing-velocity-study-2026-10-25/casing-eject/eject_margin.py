"""Casing-eject study: re-tune BURN_TABLE_MARGIN_M for the lighter landing-burn
mass on dev seeds (the 1.0 m default was tuned with the casing aboard)."""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lvlib as L
MARGINS = (-0.5, 0.0, 0.5, 1.5, 2.0)
if __name__ == "__main__":
    c = json.load(open("combos.json"))["c1_e12_e12"]
    P75 = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75}
    ej = {"ascent_casing_eject_kg": 0.0430}
    ej12 = dict(P75, ascent_casing_eject_kg=0.0391)
    A = {"today + eject, margin 1.0 m": L.run("d_ej", ws="base_eject", plant=ej)}
    for mg in MARGINS:
        A["today + eject, margin %.1f m" % mg] = L.run("d_ej_m%.1f" % mg, ws="base_eject", plant=ej,
                                                       fw={"BURN_TABLE_MARGIN_M": "%.1ff" % mg})
    A["(today, casing stays, margin 1.0 m)"] = L.run("b_base")
    L.table(A, "today + eject, margin 1.0 m")
    print()
    B = {"E12 best + eject, margin 1.0 m": L.run("d_c1_best_ej", ws="c1_eject", fw={"LEGS_DELAY_MS": "0.0f"}, plant=ej12)}
    for mg in MARGINS:
        B["E12 best + eject, margin %.1f m" % mg] = L.run(
            "d_c1_best_ej_m%.1f" % mg, ws="c1_eject", plant=ej12,
            fw={"LEGS_DELAY_MS": "0.0f", "BURN_TABLE_MARGIN_M": "%.1ff" % mg})
    B["(E12 best, casing stays, margin 1.0 m)"] = L.run("c1_dev_conf_cut075", ws=c["ws"],
                                                       fw={"LEGS_DELAY_MS": "0.0f"}, plant=P75)
    L.table(B, "E12 best + eject, margin 1.0 m")
