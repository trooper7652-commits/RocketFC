import json, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lvlib as L
if __name__ == "__main__":
    C = json.load(open("combos.json"))
    for name, c in C.items():
        rows = L.run("calm_" + name, ws=c["ws"], seeds=(1, 2, 3), n=1, disp=None,
                     plant={"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"]})
        print(name)
        for r in rows:
            s = r["summary"]; g = (s.get("ground") or {}).get("contact") or {}
            ev = s["landing_outcome"]
            print("   apogee %.1f m | fire at fall %s m/s | abort %s | contact vz %s | legs margin %s s | %s" % (
                s["apogee_h"], None if s.get("fire_v_true") is None else round(-s["fire_v_true"], 1),
                s["abort_reason"], None if not g else round(abs(g["vel"][2]), 2),
                None if s.get("legs_margin_s") is None else round(s["legs_margin_s"], 2), ev["label"][:70]))
