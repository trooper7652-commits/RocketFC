"""Export candidate ascent motors from the OpenRocket motor database to RASP .eng
files and build one workspace per motor (copy of ws/fix with config.h masses /
ASCENT_BURN_MS set and burn_table.h regenerated for that motor's descent mass)."""
import json
import os
import re
import shutil
import sqlite3
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
DB = os.path.join(HERE, "mdb", "motors.db")   # OpenRocket motor-database build 20260920070751
PY = r"C:\Users\troop\AppData\Local\Programs\Python\Python312\python.exe"
MOT = os.path.join(HERE, "motors")
os.makedirs(MOT, exist_ok=True)
F15_DRY = 0.1020 - 0.0600          # RocketFC Estes_F15.eng header (total - propellant)
BASE = 0.95 - F15_DRY              # descent mass without an ascent casing
ADAPTER_24_29 = 0.010              # [ESTIMATE] 24 -> 29 mm motor adapter

CANDS = [("F44W", "AeroTech"), ("E28T", "AeroTech"), ("41F36-11A", "Cesaroni Technology"), ("F10", "AeroTech")]
RANK = {"cert": 0, "mfr": 1, "user": 2, None: 3}

con = sqlite3.connect("file:" + DB + "?mode=ro", uri=True)
info = {}
for des, mfr in CANDS:
    (mid, dia, ln, tw, pw, ti, bt), = con.execute(
        "select m.id, m.diameter, m.length, m.total_weight, m.propellant_weight, m.total_impulse, m.burn_time "
        "from motors m join manufacturers f on f.id=m.manufacturer_id where m.designation=? and f.name=?",
        (des, mfr)).fetchall()
    cid, src, sim = min(con.execute("select id, source, tc_simfile_id from thrust_curves where motor_id=?", (mid,)),
                        key=lambda r: RANK.get(r[1], 3))
    pts = con.execute("select time_seconds, force_newtons from thrust_data where curve_id=? order by time_seconds",
                      (cid,)).fetchall()
    pts = [p for p in pts if p[0] > 0 or p[1] > 0]
    if pts[-1][1] != 0.0:
        pts.append((pts[-1][0] + 0.001, 0.0))
    tw, pw = tw / 1000.0, pw / 1000.0
    name = re.sub(r"[^A-Za-z0-9]", "", des)
    path = os.path.join(MOT, name + ".eng")
    with open(path, "w", newline="\n") as f:
        f.write("; %s %s exported from the OpenRocket motor database (ThrustCurve.org simfile %s, source %s)\n"
                % (mfr, des, sim, src))
        f.write("; masses from the database listing, [MEASURE] the real motor before trusting them\n")
        f.write("%s %d %d P %.4f %.4f %s\n" % (name, round(dia), round(ln), pw, tw, mfr.split()[0]))
        for t, F in pts:
            f.write("   %.3f %.3f\n" % (t, F))
    burn = pts[-1][0]
    m_desc = BASE + (tw - pw) + (ADAPTER_24_29 if dia < 26 else 0.0)
    info[name] = {"des": des, "mfr": mfr, "eng": path, "src": src, "dia": dia, "total_kg": tw, "prop_kg": pw,
                  "impulse": ti, "burn_s": burn, "m_desc": round(m_desc, 4), "m_pad": round(m_desc + pw, 4)}

# F15 x1.10 reference (the earlier study's stand-in for "a slightly bigger motor")
f15 = os.path.join(HERE, "ws", "fix", "tools", "motors", "Estes_F15.eng")
x110 = os.path.join(MOT, "F15_x1.10.eng")
with open(f15, encoding="utf-8", errors="replace") as fi, open(x110, "w", newline="\n") as fo:
    for line in fi:
        p = line.split()
        if len(p) == 2 and not line.lstrip().startswith(";"):
            fo.write("   %s %.3f\n" % (p[0], float(p[1]) * 1.10))
        else:
            fo.write(line)
info["F15x110"] = {"eng": x110, "m_desc": 0.95, "burn_s": 3.45}
json.dump(info, open(os.path.join(MOT, "motors.json"), "w"), indent=1)


def set_const(text, name, value):
    new, n = re.subn(r"(constexpr\s+float\s+%s\s*=\s*)[-0-9.eE]+f" % name, r"\g<1>%s" % value, text)
    assert n == 1, name
    return new


for name, m in info.items():
    if name == "F15x110":
        continue
    ws = os.path.join(HERE, "ws", "m_" + name)
    if not os.path.isdir(ws):
        shutil.copytree(os.path.join(HERE, "ws", "fix"), ws,
                        ignore=shutil.ignore_patterns("build", "*.dll", "__pycache__"))
    cfgp = os.path.join(ws, "src", "config.h")
    t = open(cfgp, encoding="utf-8").read()
    t = set_const(t, "MASS_PAD_KG", "%.4ff" % m["m_pad"])
    t = set_const(t, "MASS_DESCENT_KG", "%.4ff" % m["m_desc"])
    t = set_const(t, "ASCENT_BURN_MS", "%.1ff" % (1000.0 * m["burn_s"]))
    open(cfgp, "w", encoding="utf-8", newline="\n").write(t)
    shutil.copy(m["eng"], os.path.join(ws, "tools", "motors"))
    r = subprocess.run([PY, os.path.join("tools", "burn_table.py"), "--mass", "%.4f" % m["m_desc"]],
                       cwd=ws, capture_output=True, text=True)
    print(name, "burn table:", (r.stdout or r.stderr).strip().splitlines()[-1])
    m["ws"] = "m_" + name
json.dump(info, open(os.path.join(MOT, "motors.json"), "w"), indent=1)
for k, v in info.items():
    print(k, {kk: v[kk] for kk in v if kk in ("src", "impulse", "burn_s", "m_desc", "m_pad", "prop_kg")})
