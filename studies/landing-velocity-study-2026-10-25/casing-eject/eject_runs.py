"""Casing-eject study runs: holdout 701-710 paired against the earlier results,
then a dev-seed kick bracket."""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lvlib as L
F10_EJ = 0.0430
E12_EJ = 0.0391
if __name__ == "__main__":
    c = json.load(open("combos.json"))["c1_e12_e12"]
    P = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"]}
    P75 = dict(P, legs_cut_s=0.75)
    LEGS = {"LEGS_DELAY_MS": "0.0f"}
    ej = {"ascent_casing_eject_kg": F10_EJ}
    ej12 = {"ascent_casing_eject_kg": E12_EJ}
    print("== HOLDOUT 701-710: today's F10 + F15")
    H = {"casing stays (today)": L.run("h_base", seeds=L.HOLD)}
    H["model check: eject knob 0"] = L.run("h_ej_zero", ws="base_eject_old", seeds=L.HOLD)
    H["casing pushed out, burn table NOT rebuilt"] = L.run("h_ej_old", ws="base_eject_old", seeds=L.HOLD, plant=ej)
    H["casing pushed out, burn table rebuilt"] = L.run("h_ej", ws="base_eject", seeds=L.HOLD, plant=ej)
    L.table(H, "casing stays (today)")
    print("\n== HOLDOUT 701-710: E12/E12, legs as today")
    E = {"casing stays": L.run("h_c1", ws=c["ws"], seeds=L.HOLD, plant=P)}
    E["casing pushed out, table rebuilt"] = L.run("h_c1_ej", ws="c1_eject", seeds=L.HOLD, plant=dict(P, **ej12))
    L.table(E, "casing stays")
    print("\n== HOLDOUT 701-710: E12/E12, nichrome at confirm + 0.75 s band")
    B = {"casing stays": L.run("h_c1_best", ws=c["ws"], seeds=L.HOLD, fw=LEGS, plant=P75)}
    B["casing pushed out, table rebuilt"] = L.run("h_c1_best_ej", ws="c1_eject", seeds=L.HOLD, fw=LEGS,
                                                  plant=dict(P75, **ej12))
    L.table(B, "casing stays")
    print("\n== KICK BRACKET (dev 101-103): net impulse change at eject")
    K = {"today + eject": L.run("d_ej", ws="base_eject", plant=ej)}
    for k in (0.2, -0.2):
        K["today + eject, kick %+.1f N*s" % k] = L.run("d_ej_k%+.1f" % k, ws="base_eject",
                                                       plant=dict(ej, ascent_casing_kick_ns=k))
    L.table(K, "today + eject")
    K2 = {"E12/E12 best + eject": L.run("d_c1_best_ej", ws="c1_eject", fw=LEGS, plant=dict(P75, **ej12))}
    for k in (0.2, -0.2):
        K2["E12/E12 best + eject, kick %+.1f N*s" % k] = L.run(
            "d_c1_best_ej_k%+.1f" % k, ws="c1_eject", fw=LEGS, plant=dict(P75, ascent_casing_kick_ns=k, **ej12))
    L.table(K2, "E12/E12 best + eject")
