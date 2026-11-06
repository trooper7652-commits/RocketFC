"""Casing-eject study: ballast equal to the casing mass, fixed at the CG, so the
landing-burn mass is back to the casing-aboard value (the ascent is heavier)."""
import json, os, re, shutil, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lvlib as L
PY = L.PY
# name: (source workspace, landing .eng or None, casing kg)
WS = {"base_eject_ballast": ("base_eject", None, 0.0430),
      "c1_eject_ballast": ("c1_eject", os.path.join("tools", "motors", "landing.eng"), 0.0391)}


def bump(text, name, add):
    m = re.search(r"(constexpr\s+float\s+%s\s*=\s*)([-0-9.eE]+)f" % name, text)
    return text[:m.start(2)] + "%.4f" % (float(m.group(2)) + add) + text[m.end(2):]


if __name__ == "__main__":
    for name, (src, eng, kg) in WS.items():
        ws = os.path.join(HERE, "ws", name)
        if not os.path.isdir(ws):
            shutil.copytree(os.path.join(HERE, "ws", src), ws,
                            ignore=shutil.ignore_patterns("build", "*.dll", "__pycache__"))
            cfg = os.path.join(ws, "src", "config.h")
            t = open(cfg, encoding="utf-8").read()
            t = bump(bump(t, "MASS_PAD_KG", kg), "MASS_DESCENT_KG", kg)
            open(cfg, "w", encoding="utf-8", newline="\n").write(t)
            cmd = [PY, os.path.join("tools", "burn_table.py"), "--eject-kg", "%.4f" % kg]
            if eng:
                cmd += ["--eng", eng]
            r = subprocess.run(cmd, cwd=ws, capture_output=True, text=True)
            if r.returncode:
                raise SystemExit(r.stderr)
            print(name, "|", " | ".join(l for l in r.stdout.splitlines() if "delta-v" in l or "Target" in l))
    c = json.load(open(os.path.join(HERE, "combos.json")))["c1_e12_e12"]
    P = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"]}
    P75 = dict(P, legs_cut_s=0.75)
    LEGS = {"LEGS_DELAY_MS": "0.0f"}
    print("== HOLDOUT 701-710: today's F10 + F15")
    A = {"casing stays (today)": L.run("h_base", seeds=L.HOLD),
         "casing pushed out": L.run("h_ej", ws="base_eject", seeds=L.HOLD, plant={"ascent_casing_eject_kg": 0.043}),
         "casing pushed out + 43 g ballast": L.run("h_ej_ballast", ws="base_eject_ballast", seeds=L.HOLD,
                                                  plant={"ascent_casing_eject_kg": 0.043})}
    L.table(A, "casing stays (today)")
    print("\n== HOLDOUT 701-710: E12/E12, nichrome at confirm + 0.75 s band")
    ej12 = {"ascent_casing_eject_kg": 0.0391}
    B = {"casing stays": L.run("h_c1_best", ws=c["ws"], seeds=L.HOLD, fw=LEGS, plant=P75),
         "casing pushed out": L.run("h_c1_best_ej", ws="c1_eject", seeds=L.HOLD, fw=LEGS, plant=dict(P75, **ej12)),
         "casing pushed out + 39 g ballast": L.run("h_c1_best_ej_ballast", ws="c1_eject_ballast", seeds=L.HOLD,
                                                  fw=LEGS, plant=dict(P75, **ej12))}
    L.table(B, "casing stays")
    for lab, rows in list(A.items()) + list(B.items()):
        ap = sorted(r["summary"]["apogee_h"] for r in rows)
        print("   %-34s apogee p10/median/p90 %.0f/%.0f/%.0f m" % (lab, ap[len(ap) // 10], ap[len(ap) // 2], ap[9 * len(ap) // 10]))
