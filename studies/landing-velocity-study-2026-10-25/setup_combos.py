"""Build one scratch workspace per ascent/landing combo: config.h masses, burn
times and LANDING_BURN_MAX_MS set, burn_table.h regenerated for the landing
motor at that descent mass (620 ms delay), motor files copied in."""
import json
import os
import re
import shutil
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
PY = r"C:\Users\troop\AppData\Local\Programs\Python\Python312\python.exe"
AIRFRAME = 0.951 - 0.102 - 0.043    # today's descent mass minus the F15 (landing) and F10 casing
ADAPTER = 0.010
MOT = os.path.join(HERE, "ws", "base", "tools", "motors")
E12 = os.path.join(HERE, "motors", "20.eng")          # AeroTech E12-RCJ (24 mm RMS reload, P)
F10 = os.path.join(MOT, "AeroTech_F10.eng")
F15 = os.path.join(MOT, "Estes_F15.eng")
# name: (ascent eng, ascent total kg, ascent prop kg, ascent dia, ascent burn s,
#        landing eng, landing total kg, landing dia, landing burn s)
COMBOS = {
    "c1_e12_e12": (E12, 0.0594, 0.0303, 24, 3.05, E12, 0.0594, 24, 3.05),
    "c2_f15_f10": (F15, 0.1020, 0.0600, 29, 3.45, F10, 0.0830, 29, 7.05),
}


def set_const(text, name, value):
    new, n = re.subn(r"(constexpr\s+float\s+%s\s*=\s*)[-0-9.eE]+f" % name, r"\g<1>%s" % value, text)
    assert n == 1, name
    return new


info = {}
for name, (a_eng, a_tot, a_prop, a_dia, a_burn, l_eng, l_tot, l_dia, l_burn) in COMBOS.items():
    m_desc = (AIRFRAME + l_tot + (ADAPTER if l_dia < 26 else 0.0)
              + (a_tot - a_prop) + (ADAPTER if a_dia < 26 else 0.0))
    m_pad = m_desc + a_prop
    ws = os.path.join(HERE, "ws", name)
    if not os.path.isdir(ws):
        shutil.copytree(os.path.join(HERE, "ws", "base"), ws,
                        ignore=shutil.ignore_patterns("build", "*.dll", "__pycache__"))
    for src, dst in ((a_eng, "ascent.eng"), (l_eng, "landing.eng")):
        shutil.copy(src, os.path.join(ws, "tools", "motors", dst))
    cfg = os.path.join(ws, "src", "config.h")
    t = open(cfg, encoding="utf-8").read()
    t = set_const(t, "MASS_PAD_KG", "%.4ff" % m_pad)
    t = set_const(t, "MASS_DESCENT_KG", "%.4ff" % m_desc)
    t = set_const(t, "ASCENT_BURN_MS", "%.1ff" % (1000 * a_burn))
    t = set_const(t, "LANDING_BURN_MAX_MS", "%.1ff" % max(5000.0, 1000 * (0.62 + l_burn + 1.0)))
    open(cfg, "w", encoding="utf-8", newline="\n").write(t)
    r = subprocess.run([PY, os.path.join("tools", "burn_table.py"), "--eng",
                        os.path.join("tools", "motors", "landing.eng")], cwd=ws, capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(r.stderr)
    info[name] = {"ws": name, "m_desc": round(m_desc, 4), "m_pad": round(m_pad, 4),
                  "ascent_eng": os.path.join(ws, "tools", "motors", "ascent.eng"),
                  "landing_eng": os.path.join(ws, "tools", "motors", "landing.eng")}
    print(name, info[name]["m_desc"], info[name]["m_pad"], "|", r.stdout.strip().splitlines()[1])
json.dump(info, open(os.path.join(HERE, "combos.json"), "w"), indent=1)
