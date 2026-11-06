"""Fix study, steps 1-2: ballast amount x BURN_TABLE_MARGIN_M with the casing
pushed out, on dev seeds. Ballast sits at the CG (no CG / inertia change); the
firmware's MASS_* include it and the burn table is rebuilt with --eject-kg."""
import json, os, re, shutil, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lvlib as L
E12_EJ, F10_EJ = 0.0391, 0.0430
# (setup, ballast g) -> workspace; the 0 g and casing-mass ones already exist
E12_WS = {0: "c1_eject", 20: "c1_ej_b20", 39: "c1_eject_ballast", 60: "c1_ej_b60", 80: "c1_ej_b80"}
F10_WS = {0: "base_eject", 20: "base_ej_b20", 43: "base_eject_ballast"}
E12_MARGINS = (0.5, 1.0, 1.5)
F10_MARGINS = (0.5, 1.0, 2.0)


def bump(text, name, add):
    m = re.search(r"(constexpr\s+float\s+%s\s*=\s*)([-0-9.eE]+)f" % name, text)
    return text[:m.start(2)] + "%.4f" % (float(m.group(2)) + add) + text[m.end(2):]


def make_ws(name, src, ballast_kg, eject_kg, eng=None):
    ws = os.path.join(HERE, "ws", name)
    if os.path.isdir(ws):
        return
    shutil.copytree(os.path.join(HERE, "ws", src), ws,
                    ignore=shutil.ignore_patterns("build", "*.dll", "__pycache__"))
    cfg = os.path.join(ws, "src", "config.h")
    t = open(cfg, encoding="utf-8").read()
    t = bump(bump(t, "MASS_PAD_KG", ballast_kg), "MASS_DESCENT_KG", ballast_kg)
    open(cfg, "w", encoding="utf-8", newline="\n").write(t)
    cmd = [L.PY, os.path.join("tools", "burn_table.py"), "--eject-kg", "%.4f" % eject_kg]
    if eng:
        cmd += ["--eng", eng]
    r = subprocess.run(cmd, cwd=ws, capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(r.stderr)
    print("  built", name, "|", " | ".join(l for l in r.stdout.splitlines() if "Target" in l), flush=True)


def apogee_med(rows):
    a = sorted(r["summary"]["apogee_h"] for r in rows)
    return a[len(a) // 2]


def grid(title, results, ballasts, margins):
    print("\n" + title)
    print("| Ballast | apogee | " + " | ".join("margin %.1f m" % m for m in margins) + " |")
    print("|---|---|" + "---|" * len(margins))
    for g in ballasts:
        cells = []
        for mg in margins:
            mt = L.metrics(results[g, mg])
            cells.append("%.1f %% (lift %.0f %%, vz %.2f)" % (100.0 * mt["k"] / mt["n"], 100 * mt["lifted"], mt["vz_med"]))
        print("| %d g | %.0f m | %s |" % (g, apogee_med(results[g, margins[0]]), " | ".join(cells)), flush=True)


if __name__ == "__main__":
    c = json.load(open(os.path.join(HERE, "combos.json")))["c1_e12_e12"]
    P75 = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75,
           "ascent_casing_eject_kg": E12_EJ}
    for g, name in E12_WS.items():
        make_ws(name, "c1_eject", g / 1000.0, E12_EJ, os.path.join("tools", "motors", "landing.eng"))
    for g, name in F10_WS.items():
        make_ws(name, "base_eject", g / 1000.0, F10_EJ)
    R = {}
    for g, name in E12_WS.items():
        for mg in E12_MARGINS:
            R[g, mg] = L.run("f_c1_b%d_m%.1f" % (g, mg), ws=name, plant=P75,
                             fw={"LEGS_DELAY_MS": "0.0f", "BURN_TABLE_MARGIN_M": "%.1ff" % mg})
    ref = L.run("d_c1_best_ej", ws="c1_eject", fw={"LEGS_DELAY_MS": "0.0f"}, plant=P75)
    same = sum(1 for a, b in zip(ref, R[0, 1.0]) if a["summary"]["landing_outcome"]["success"] == b["summary"]["landing_outcome"]["success"])
    print("check: 0 g / margin 1.0 vs d_c1_best_ej: %d/%d identical outcomes" % (same, len(ref)))
    grid("== E12/E12 best legs, casing out (dev 101-103; casing aboard = 86.5 %)", R, list(E12_WS), E12_MARGINS)
    F = {}
    for g, name in F10_WS.items():
        for mg in F10_MARGINS:
            F[g, mg] = L.run("f_f10_b%d_m%.1f" % (g, mg), ws=name, plant={"ascent_casing_eject_kg": F10_EJ},
                             fw={"BURN_TABLE_MARGIN_M": "%.1ff" % mg})
    grid("== F10 + F15, casing out (dev 101-103; casing aboard = 22.3 %)", F, list(F10_WS), F10_MARGINS)
