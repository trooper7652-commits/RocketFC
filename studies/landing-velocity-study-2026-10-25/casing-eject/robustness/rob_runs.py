"""Robustness study runs (dev 101-103, paired vs the fix base f_c1_b80_m1.0)."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lvlib as L
import budget as B
BASE_WS, KNOBS = "c1_ej_b80", "c1_fix_knobs"
FW = {"LEGS_DELAY_MS": "0.0f", "BURN_TABLE_MARGIN_M": "1.0f"}
BASE_LABEL = "fix (80 g, casing out)"
if __name__ == "__main__":
    c = json.load(open(os.path.join(HERE, "combos.json")))["c1_e12_e12"]
    P = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75,
         "ascent_casing_eject_kg": 0.0391}
    base = L.run("f_c1_b80_m1.0", ws=BASE_WS, plant=P, fw=FW)

    def run(label, ws=BASE_WS, plant=None, **kw):
        return L.run("r_" + label, ws=ws, plant=dict(P, **(plant or {})), fw=kw.pop("fw", FW), **kw)

    G = {"knob workspace, defaults": run("knobs_default", ws=KNOBS)}
    same = sum(1 for a, b in zip(base, G["knob workspace, defaults"])
               if a["summary"]["landing_outcome"]["success"] == b["summary"]["landing_outcome"]["success"])
    print("check: knob workspace defaults vs base: %d/%d identical" % (same, len(base)), flush=True)
    sections = []
    S = {"E12 landing thrust spread +-3 %": run("land3", ext=["landing_thrust_scale:0.97:1.03"]),
         "E12 landing thrust spread +-5 %": run("land5", ext=["landing_thrust_scale:0.95:1.05"]),
         "E12 ascent thrust spread +-3 %": run("asc3", ws=KNOBS, ext=["ascent_thrust_scale:0.97:1.03"]),
         "both spread +-3 %": run("both3", ws=KNOBS, ext=["landing_thrust_scale:0.97:1.03", "ascent_thrust_scale:0.97:1.03"])}
    sections.append(("Motor unknowns", S))
    D = {}
    for d in (400, 800, 1000):   # same absolute scatter (+-31 ms) as the 620 ms base
        D["E12 delay %d ms (table built for it)" % d] = run("d%d" % d, ws="c1_fix_d%d" % d,
                                                            disp={"mass_descent_kg": 0.01, "ignition_delay_s": 31.0 / d})
    D["delay scatter +-10 %"] = run("dsc10", disp={"mass_descent_kg": 0.01, "ignition_delay_s": 0.10})
    D["delay scatter +-15 %"] = run("dsc15", disp={"mass_descent_kg": 0.01, "ignition_delay_s": 0.15})
    sections.append(("E12 ignition delay", D))
    E = {"casing leaves 0.1 s late": run("ej_late01", ws=KNOBS, plant={"ascent_casing_eject_delay_s": 0.1}),
         "casing leaves 0.3 s late": run("ej_late03", ws=KNOBS, plant={"ascent_casing_eject_delay_s": 0.3}),
         "eject kick +0.2 N*s": run("kick_p02", plant={"ascent_casing_kick_ns": 0.2}),
         "eject kick -0.2 N*s": run("kick_m02", plant={"ascent_casing_kick_ns": -0.2}),
         "casing stuck (never leaves)": run("stuck", plant={"ascent_casing_eject_kg": 0.0})}
    sections.append(("Casing eject", E))
    Bd = {"band cut %.2f s" % s: run("band%03d" % round(100 * s), plant={"legs_cut_s": s}) for s in (0.5, 1.0, 1.25, 1.5)}
    sections.append(("Leg band cut time (base 0.75 s)", Bd))
    Gu = {"random 1 s gust up to %d m/s" % g: run("gust%d" % g, plant={"gust_mps": float(g), "gust_time_s": 7.5, "gust_dur_s": 1.0},
                                                  ext=["gust_mps:-1:1", "gust_time_s:0.5:1.6"]) for g in (4, 6)}
    sections.append(("Gusts", Gu))
    Bu = {"- mass spread": run("b_nomass", disp={"mass_descent_kg": 0, "ignition_delay_s": 0.05}),
          "- delay spread": run("b_nodelay", disp={"mass_descent_kg": 0.01, "ignition_delay_s": 0}),
          "- drag spread": run("b_nodrag", disp=dict(L.MEAS, cda_m2=0)),
          "- airframe guesses": run("b_noairframe", disp=dict(L.MEAS, **B.AIRFRAME)),
          "- wind": run("b_nowind", wind=0),
          "- sensor noise/bias": run("b_nosensors", plant=B.QUIET),
          "perfect knowledge": run("b_perfect", disp=B.NONE, plant=B.QUIET, wind=0)}
    sections.append(("Error budget (one source removed)", Bu))
    for title, sec in sections:
        print("\n== " + title)
        L.table(dict({BASE_LABEL: base}, **sec), BASE_LABEL)
