"""Build the study workspaces (scratch copies; RocketFC itself is never touched).

  ws/orig   plain copy of the RocketFC working tree (src/, tools/), for the
            bit-identical check;
  ws/base   orig + the casing-eject model (reused from
            landing-velocity-study-2026-10-25/casing-eject/eject_setup.py),
            a dud-igniter switch, a sized-canopy fill model and extra chute
            summary fields. Every new knob defaults to "off" = orig;
  ws/<combo> base + ascent.eng / landing.eng, config.h at 1.4 kg liftoff,
            legs on at ignition confirm (LEGS_DELAY_MS 0) with a 0.75 s band
            cut, and burn_table.h rebuilt for the landing motor with the
            casing pushed out at thrust onset.

usage: python setup_ws.py base
       python setup_ws.py combo ASCENT_KEY LANDING_KEY [ASCENT_KEY LANDING_KEY ...]
"""
import json
import multiprocessing
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SPACEX = os.path.dirname(HERE)
RFC = os.path.join(SPACEX, "RocketFC")
SCR = r"C:\Users\troop\AppData\Local\Temp\claude\3e9e20f0e564c4e3\2e110106-3a4f-48f3-9a7d-a66c508094e2\scratchpad\mc"
WS = os.path.join(SCR, "ws")
PY = r"C:\Users\troop\AppData\Local\Programs\Python\Python312\python.exe"
sys.path.insert(0, os.path.join(SPACEX, "landing-velocity-study-2026-10-25", "casing-eject"))
import eject_setup  # noqa: E402  (patch_vehicle, patch_plant, patch_burn_table)

M_PAD = 1.4
ADAPTER_KG = 0.010
IGNORE = shutil.ignore_patterns("build", "*.dll", "__pycache__", "*.exe", "traces", "*.json")


def sub(path, old, new):
    t = open(path, encoding="utf-8").read()
    assert t.count(old) == 1, (path, old[:70])
    open(path, "w", encoding="utf-8", newline="\n").write(t.replace(old, new))


def copy_rfc(dst):
    os.makedirs(dst)
    shutil.copytree(os.path.join(RFC, "src"), os.path.join(dst, "src"), ignore=IGNORE)
    shutil.copytree(os.path.join(RFC, "tools"), os.path.join(dst, "tools"), ignore=IGNORE)


def patch_chute_and_dud(ws):
    veh = os.path.join(ws, "tools", "sim", "vehicle.py")
    sub(veh, "    chute_stuck_releases: int = 0\n", """    chute_stuck_releases: int = 0
    # Chute size study (motor-chute-study-2026-11-04). chute_diameter_m > 0
    # switches to a sized canopy: CdA = chute_cd * pi * D^2 / 4. It opens
    # chute_line_stretch_s after the latch releases, then fills over
    # chute_fill_n * D of travel through the air (Knacke fill constant), its
    # drag area growing as (distance / fill distance) ** chute_fill_exp.
    # 0 = the old cda_chute_m2 + chute_inflation_s time ramp (bit-identical).
    chute_diameter_m: float = 0.0
    chute_cd: float = 1.5
    chute_fill_n: float = 8.0
    chute_fill_exp: float = 2.0
    chute_line_stretch_s: float = 0.3
    # The landing e-match fires but the motor never lights (DUD_IGNITER path).
    dud_landing: bool = False
""")
    pl = os.path.join(ws, "tools", "sim", "plant.py")
    sub(pl, "        self.chute_open_t = None\n        self._area_ref", """        self.chute_open_t = None
        self.chute_frac = 0.0
        self._chute_s = 0.0
        _d = getattr(vcfg, "chute_diameter_m", 0.0)
        self._chute_cda0 = vcfg.chute_cd * math.pi * 0.25 * _d * _d if _d > 0 else 0.0
        self._chute_fill_m = vcfg.chute_fill_n * _d if _d > 0 else 0.0
        self._area_ref""")
    sub(pl, """            k = min(1.0, (s.t - self.chute_open_t) / max(v.chute_inflation_s, 1e-6))
            chute_cda = k * v.cda_chute_m2
""", """            if self._chute_cda0 > 0.0:
                # sized canopy: line stretch, then fill by distance travelled
                if s.t - self.chute_open_t >= v.chute_line_stretch_s:
                    self._chute_s += speed * dt
                    k = (min(1.0, self._chute_s / max(self._chute_fill_m, 1e-9))
                         ** v.chute_fill_exp)
                else:
                    k = 0.0
                chute_cda = k * self._chute_cda0
            else:
                k = min(1.0, (s.t - self.chute_open_t) / max(v.chute_inflation_s, 1e-6))
                chute_cda = k * v.cda_chute_m2
            self.chute_frac = k
""")
    run = os.path.join(ws, "tools", "sim", "run.py")
    sub(run, """if ev["code"] == core.FlightEvent.FIRE_LANDING and not dud_landing_igniter:""",
        """if ev["code"] == core.FlightEvent.FIRE_LANDING and not (
                        dud_landing_igniter or getattr(vcfg, "dud_landing", False)):""")
    sub(run, "    chute_releases = 0\n", """    chute_releases = 0
    chute_dep = None
    chute_peak_g = 0.0
    chute_frac_td = None
""")
    sub(run, """                    if chute_releases > vcfg.chute_stuck_releases:
                        pl.deploy_chute(t)
""", """                    if chute_releases > vcfg.chute_stuck_releases:
                        pl.deploy_chute(t)
                        if chute_dep is None:
                            chute_dep = {"t": t, "h": s.pos.z, "vz": s.vel.z,
                                         "speed": s.vel.norm()}
""")
    sub(run, """            pl.legs_frac = legs_frac
            pl.step(dt, act_x, act_y)
""", """            pl.legs_frac = legs_frac
            pl.step(dt, act_x, act_y)
            if pl.chute_deployed and pl.state.touchdown_t is None:
                _sf = pl.state.specific_force_body.norm() / 9.80665
                if _sf > chute_peak_g:
                    chute_peak_g = _sf
""")
    sub(run, """            if pl.state.touchdown_t is not None and touchdown_speed is None:
""", """            if pl.state.touchdown_t is not None and touchdown_speed is None:
                chute_frac_td = getattr(pl, "chute_frac", None) if pl.chute_deployed else 0.0
""")
    sub(run, """        "chute_unconfirmed": chute_unconfirmed,
""", """        "chute_unconfirmed": chute_unconfirmed,
        "chute_deploy_t": chute_dep["t"] if chute_dep else None,
        "chute_deploy_h": chute_dep["h"] if chute_dep else None,
        "chute_deploy_vz": chute_dep["vz"] if chute_dep else None,
        "chute_deploy_speed": chute_dep["speed"] if chute_dep else None,
        "chute_peak_g": chute_peak_g,
        "chute_frac_td": chute_frac_td,
""")


def build_base():
    orig = os.path.join(WS, "orig")
    if not os.path.isdir(orig):
        copy_rfc(orig)
        print("orig copied")
    base = os.path.join(WS, "base")
    if not os.path.isdir(base):
        copy_rfc(base)
        eject_setup.patch_vehicle(os.path.join(base, "tools", "sim", "vehicle.py"))
        eject_setup.patch_plant(os.path.join(base, "tools", "sim", "plant.py"))
        eject_setup.patch_burn_table(os.path.join(base, "tools", "burn_table.py"))
        patch_chute_and_dud(base)
        print("base built")


def set_const(text, name, value):
    new, n = re.subn(r"(constexpr\s+float\s+%s\s*=\s*)[-0-9.eE]+f" % name, r"\g<1>%s" % value, text)
    assert n == 1, name
    return new


def motor_info(key):
    for m in json.load(open(os.path.join(HERE, "prescreen.json"))):
        if m["key"] == key:
            return m
    raise KeyError(key)


def combo_name(a, l):
    return "c_%s__%s" % (a, l)


def build_combo(args):
    a_key, l_key = args
    name = combo_name(a_key, l_key)
    ws = os.path.join(WS, name)
    done = os.path.join(ws, "combo.json")
    if os.path.exists(done):
        return json.load(open(done))
    if os.path.isdir(ws):
        shutil.rmtree(ws)
    shutil.copytree(os.path.join(WS, "base"), ws, ignore=IGNORE)
    a, l = motor_info(a_key), motor_info(l_key)
    for m, dst in ((a, "ascent.eng"), (l, "landing.eng")):
        shutil.copy(m["eng"], os.path.join(ws, "tools", "motors", dst))
    a_prop = a["prop_g"] / 1000.0
    casing = (a["total_g"] - a["prop_g"]) / 1000.0 + (ADAPTER_KG if a["dia"] < 26 else 0.0)
    m_desc = M_PAD - a_prop
    veh = os.path.join(ws, "tools", "sim", "vehicle.py")
    t = open(veh, encoding="utf-8").read()
    t = t.replace('DEFAULT_ENG = os.path.join(MOTORS_DIR, "Estes_F15.eng")',
                  'DEFAULT_ENG = os.path.join(MOTORS_DIR, "landing.eng")')
    t = t.replace('ASCENT_ENG = os.path.join(MOTORS_DIR, "AeroTech_F10.eng")',
                  'ASCENT_ENG = os.path.join(MOTORS_DIR, "ascent.eng")')
    t = t.replace("    legs_cut_s: float = 1.5 ", "    legs_cut_s: float = 0.75")
    t = t.replace("    ascent_casing_eject_kg: float = 0.0\n",
                  "    ascent_casing_eject_kg: float = %.4f\n" % casing)
    assert "landing.eng" in t and "ascent.eng" in t and "legs_cut_s: float = 0.75" in t
    assert "ascent_casing_eject_kg: float = %.4f" % casing in t
    open(veh, "w", encoding="utf-8", newline="\n").write(t)
    cfg = os.path.join(ws, "src", "config.h")
    t = open(cfg, encoding="utf-8").read()
    t = set_const(t, "MASS_PAD_KG", "%.4ff" % M_PAD)
    t = set_const(t, "MASS_DESCENT_KG", "%.4ff" % m_desc)
    t = set_const(t, "ASCENT_BURN_MS", "%.1ff" % (1000.0 * a["burn"]))
    t = set_const(t, "LANDING_BURN_MAX_MS", "%.1ff" % max(5000.0, 1000.0 * (0.62 + l["burn"] + 1.0)))
    t = set_const(t, "LEGS_DELAY_MS", "0.0f")
    open(cfg, "w", encoding="utf-8", newline="\n").write(t)
    r = subprocess.run([PY, os.path.join("tools", "burn_table.py"), "--eng",
                        os.path.join("tools", "motors", "landing.eng"), "--eject-kg", "%.4f" % casing],
                       cwd=ws, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(name + r.stderr[-2000:])
    window = [ln for ln in r.stdout.splitlines() if "Target apogee" in ln or "NO apogee" in ln]
    r = subprocess.run([PY, "-c", "import core; print(core.default_dll())"],
                       cwd=os.path.join(ws, "tools", "sim"), capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(name + r.stderr[-2000:])
    info = {"name": name, "ascent": a_key, "landing": l_key, "m_pad": M_PAD,
            "m_desc": round(m_desc, 4), "casing_kg": round(casing, 4),
            "ascent_burn_s": a["burn"], "landing_burn_s": l["burn"],
            "burn_table_window": window[0].strip() if window else ""}
    json.dump(info, open(done, "w"), indent=1)
    return info


if __name__ == "__main__":
    if sys.argv[1] == "base":
        build_base()
    elif sys.argv[1] == "combo":
        pairs = list(zip(sys.argv[2::2], sys.argv[3::2]))
        with multiprocessing.Pool(min(10, len(pairs))) as pool:
            for info in pool.imap_unordered(build_combo, pairs):
                print(info["name"], "| desc %.4f casing %.4f |" % (info["m_desc"], info["casing_kg"]),
                      info["burn_table_window"], flush=True)
