"""Part A, step 5: robustness of the finalist motor pairs (dev seeds 101-103,
600 flights per run, paired against the measured-spreads dev run).

  guessed   today's unmeasured spreads (mass +-5 %, delay +-15 %, ...);
  wind W    steady wind of W m/s on every flight;
  land3     landing-motor thrust +-3 % (motor-to-motor spread, uniform);
  asc5      ascent-motor impulse +-5 % -> the apogee high tail (100 ft rule);
  legs1s    today's leg timing: nichrome 1 s after fire, 1.5 s band cut.

The ascent spread needs one plant knob (ascent_thrust_scale, default 1.0 =
unchanged), patched into the finalists' workspaces here.
usage: python robust.py COMBO [COMBO ...]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mclib as L  # noqa: E402
from screen import short  # noqa: E402


def add_ascent_scale(ws):
    sim = os.path.join(L.WS, ws, "tools", "sim")
    veh, pl = os.path.join(sim, "vehicle.py"), os.path.join(sim, "plant.py")
    t = open(veh, encoding="utf-8").read()
    if "ascent_thrust_scale" not in t:
        old = "    landing_time_scale: float = 1.0"
        assert t.count(old) == 1
        t = t.replace(old, "    ascent_thrust_scale: float = 1.0     # robustness: ascent motor impulse x this\n" + old)
        open(veh, "w", encoding="utf-8", newline="\n").write(t)
    t = open(pl, encoding="utf-8").read()
    if "ascent_thrust_scale" not in t:
        old = "        self.ascent_motor = bt.Motor(vcfg.ascent_eng_path)\n"
        assert t.count(old) == 1
        t = t.replace(old, "        self.ascent_motor = bt.Motor(vcfg.ascent_eng_path,\n"
                           "                                     scale=getattr(vcfg, \"ascent_thrust_scale\", 1.0))\n")
        open(pl, "w", encoding="utf-8", newline="\n").write(t)


def line(lab, rows, base):
    m = L.metrics(rows)
    lo, hi = L.wilson(m["k"], m["n"])
    pr = "" if rows is base else "%d / %d, p=%.2g" % L.paired(base, rows)
    print("| %s | %.1f %% (%.0f-%.0f) | %.1f / %.1f / %.1f m | %.2f / %.2f | %.0f %% | %.0f %% | %.1f %% | %s |" % (
        lab, 100.0 * m["k"] / m["n"], lo, hi, m["ap_med"], m["ap_p99"], m["ap_max"], m["vz_med"], m["vz_p90"],
        100 * m["lifted"], 100 * m["legs_late"], 100 * m["abort"], pr), flush=True)


if __name__ == "__main__":
    for combo in sys.argv[1:]:
        add_ascent_scale(combo)
        base = L.run("dev_" + combo, ws=combo)
        print("\n== Robustness: %s (dev 101-103, 600 flights each)" % short(combo))
        print("| Run | Success (95 % CI) | Apogee med / p99 / max | Contact vz med / p90 | Lifted off | "
              "Legs late | Aborts | vs measured: rescued / lost |")
        print("|---|---|---|---|---|---|---|---|")
        line("measured spreads (reference)", base, base)
        line("today's guessed spreads", L.run("rb_guess_" + combo, ws=combo, disp=None), base)
        for w in (0, 2, 4, 5, 6, 7):
            line("steady wind %d m/s" % w, L.run("rb_wind%d_%s" % (w, combo), ws=combo, wind=w), base)
        line("landing thrust +-3 %", L.run("rb_land3_" + combo, ws=combo, ext=["landing_thrust_scale:0.97:1.03"]), base)
        line("ascent impulse +-5 %", L.run("rb_asc5_" + combo, ws=combo, ext=["ascent_thrust_scale:0.95:1.05"]), base)
        line("today's legs (1 s delay, 1.5 s band)",
             L.run("rb_legs1s_" + combo, ws=combo, fw={"LEGS_DELAY_MS": "1000.0f"}, plant={"legs_cut_s": 1.5}), base)
