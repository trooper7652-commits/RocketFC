"""Casing-eject study: isolate the pure mass effect (casing CG put at the rocket
CG = no CG shift, no inertia cut, no standing-geometry change) on dev seeds."""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lvlib as L
if __name__ == "__main__":
    c = json.load(open("combos.json"))["c1_e12_e12"]
    P75 = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75}
    LEGS = {"LEGS_DELAY_MS": "0.0f"}
    ej = {"ascent_casing_eject_kg": 0.0430}
    ej12 = {"ascent_casing_eject_kg": 0.0391}
    mass_only = {"ascent_casing_cg_from_nose_m": 0.66}
    A = {"today, casing stays (dev)": L.run("b_base"),
         "today + eject (full model)": L.run("d_ej", ws="base_eject", plant=ej),
         "today + eject, mass only": L.run("d_ej_massonly", ws="base_eject", plant=dict(ej, **mass_only))}
    L.table(A, "today, casing stays (dev)")
    print()
    B = {"E12/E12 best, casing stays (dev)": L.run("c1_dev_conf_cut075", ws=c["ws"], fw=LEGS, plant=P75),
         "E12/E12 best + eject (full model)": L.run("d_c1_best_ej", ws="c1_eject", fw=LEGS, plant=dict(P75, **ej12)),
         "E12/E12 best + eject, mass only": L.run("d_c1_best_ej_massonly", ws="c1_eject", fw=LEGS,
                                                 plant=dict(P75, **ej12, **mass_only))}
    L.table(B, "E12/E12 best, casing stays (dev)")
