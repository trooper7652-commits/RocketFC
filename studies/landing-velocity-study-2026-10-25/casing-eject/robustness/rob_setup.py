"""Robustness study setup: knob workspace (ascent thrust scale, late casing eject)
and E12 ignition-delay workspaces (table rebuilt for each delay)."""
import os, re, shutil, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import eject_setup as ES
import lvlib as L
SRC = "c1_ej_b80"
DELAYS = (400, 800, 1000)


def copy(name):
    ws = os.path.join(HERE, "ws", name)
    if os.path.isdir(ws):
        return None
    shutil.copytree(os.path.join(HERE, "ws", SRC), ws,
                    ignore=shutil.ignore_patterns("build", "*.dll", "__pycache__"))
    return ws


if __name__ == "__main__":
    ws = copy("c1_fix_knobs")
    if ws:
        sim = os.path.join(ws, "tools", "sim")
        ES.sub(os.path.join(sim, "vehicle.py"), """    ascent_casing_kick_ns: float = 0.0   # net impulse change from the eject, over the first 0.15 s
""", """    ascent_casing_kick_ns: float = 0.0   # net impulse change from the eject, over the first 0.15 s
    ascent_casing_eject_delay_s: float = 0.0  # casing leaves this long after landing thrust onset
    ascent_thrust_scale: float = 1.0     # multiplies ascent thrust (impulse scales too)
""")
        p = os.path.join(sim, "plant.py")
        ES.sub(p, """                              else max(avg_thrust, 1e-3))
""", """                              else max(avg_thrust, 1e-3))
        # Robustness knob: ascent motor-to-motor spread. The disturbance
        # reference thrust above stays at the nominal curve's.
        _as = getattr(vcfg, "ascent_thrust_scale", 1.0)
        if _as != 1.0:
            self.ascent_motor = bt.Motor(vcfg.ascent_eng_path, scale=_as)
""")
        ES.sub(p, """        self._eject_kg = getattr(vcfg, "ascent_casing_eject_kg", 0.0)
""", """        self._eject_kg = getattr(vcfg, "ascent_casing_eject_kg", 0.0)
        self._eject_delay = getattr(vcfg, "ascent_casing_eject_delay_s", 0.0)
""")
        ES.sub(p, """            if self._eject_kg > 0 and tb >= 0.0:""",
               """            if self._eject_kg > 0 and tb >= self._eject_delay:""")
        ES.sub(p, """                s.landing_ignite_t is not None and s.t >= s.landing_ignite_t):""",
               """                s.landing_ignite_t is not None and
                s.t >= s.landing_ignite_t + self._eject_delay):""")
        print("built c1_fix_knobs")
    for d in DELAYS:
        ws = copy("c1_fix_d%d" % d)
        if not ws:
            continue
        cfg = os.path.join(ws, "src", "config.h")
        t = open(cfg, encoding="utf-8").read()
        t, n = re.subn(r"(constexpr\s+float\s+IGNITION_DELAY_MS\s*=\s*)[-0-9.eE]+f", r"\g<1>%d.0f" % d, t)
        assert n == 1
        open(cfg, "w", encoding="utf-8", newline="\n").write(t)
        r = subprocess.run([L.PY, os.path.join("tools", "burn_table.py"), "--eng",
                            os.path.join("tools", "motors", "landing.eng"), "--eject-kg", "0.0391"],
                           cwd=ws, capture_output=True, text=True)
        if r.returncode:
            raise SystemExit(r.stderr)
        print("built c1_fix_d%d |" % d, " | ".join(l for l in r.stdout.splitlines() if "Target" in l))
