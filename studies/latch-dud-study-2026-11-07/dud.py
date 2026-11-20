"""Dud landing motor (Step C), landing mode, E18W/E18W 1.4 kg, 1.4 m annular
canopy (Cd 2.2, sized fill model), dev seeds 101-103 x 200, measured spreads.

  window   C(a): can ANY dud window rescue the rocket? DUD_WINDOW_MS sweep with
           a dud igniter (impact speed, canopy height), and the false DUD
           aborts it costs on good motors whose ignition delay scatters +-15 %.
  confirm  C(b): stop the impact jolt passing as "ignition confirmed" (which
           turns the leg nichrome on at the crash). Dud flights: false
           confirms; good flights: landing success (paired), legs-on delay.
  hold     the confirm candidates that pass, on the fresh holdout (701-710).

usage: python dud.py window | confirm | hold NAME [NAME ...]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "motor-chute-study-2026-11-04"))
import mclib as L  # noqa: E402

WS = "ld_base2"
CH = {"chute_diameter_m": 1.4, "chute_cd": 2.2, "chute_fill_n": 8}
SCATTER = ["ignition_delay_s:0.85:1.15"]
CONFIRM = {
    "current": {},
    "hold20": {"IGNITE_CONFIRM_MS": "20.0f"},
    "hold40": {"IGNITE_CONFIRM_MS": "40.0f"},
    "hold60": {"IGNITE_CONFIRM_MS": "60.0f"},
    "alt3": {"IGNITE_CONFIRM_MIN_ALT_M": "3.0f"},
    "alt1": {"IGNITE_CONFIRM_MIN_ALT_M": "1.0f"},
    "hold40+alt1": {"IGNITE_CONFIRM_MS": "40.0f", "IGNITE_CONFIRM_MIN_ALT_M": "1.0f"},
}


def succ(rows):
    return sum(r["summary"]["landing_outcome"]["success"] for r in rows)


def dud_aborts(rows):
    return sum(1 for r in rows if r["summary"].get("abort_reason") == "DUD_IGNITER")


def window():
    print("\n== C(a) dud window: dud igniter (impact) | good motors, ignition delay +-15 % (cost)")
    print("| DUD_WINDOW_MS | dud: impact med / p90 | canopy freed before impact | freed at (med) | "
          "good motors: success | false DUD aborts |")
    print("|---|---|---|---|---|---|")
    for w in (0, 100, 200, 300, 500, 700):
        fw = {"DUD_WINDOW_MS": "%.1ff" % w}
        dud = L.run("ld_Ca_dud_w%d" % w, WS, plant=dict(CH, dud_landing=True), fw=fw)
        good = L.run("ld_Ca_good_w%d" % w, WS, plant=CH, fw=fw, ext=SCATTER)
        m = L.chute_metrics(dud)
        print("| %d | %.1f / %.1f m/s | %.0f %% | %.1f m | %d/%d | %d |" % (
            w, m["vz_med"], m["vz_p90"], 100 * m["opened"], m["dh_med"],
            succ(good), len(good), dud_aborts(good)), flush=True)


def legs_shift(base, rows):
    a = {(r["seed"], r["i"]): r["summary"] for r in base}
    d = []
    for r in rows:
        s0, s1 = a.get((r["seed"], r["i"])), r["summary"]
        if s0 and s0.get("legs_on_t") is not None and s1.get("legs_on_t") is not None:
            d.append(1000.0 * (s1["legs_on_t"] - s0["legs_on_t"]))
    return L.q(d, 0.5), L.q(d, 0.99), (max(d) if d else float("nan"))


def confirm():
    base_d = L.run("ld_Cb_dud_current", WS, plant=dict(CH, dud_landing=True))
    base_g = L.run("ld_Cb_good_current", WS, plant=CH)
    print("\n== C(b) ignition confirm vs the impact jolt (dev, 600 flights each)")
    print("| rule | dud: false confirms (legs on at impact) | dud: DUD_IGNITER aborts | "
          "good: success (paired vs current) | good: legs-on later by med / p99 / max | "
          "good: false DUD aborts |")
    print("|---|---|---|---|---|---|")
    for name, fw in CONFIRM.items():
        dud = base_d if name == "current" else L.run("ld_Cb_dud_" + name, WS,
                                                      plant=dict(CH, dud_landing=True), fw=fw)
        good = base_g if name == "current" else L.run("ld_Cb_good_" + name, WS, plant=CH, fw=fw)
        fc = sum(1 for r in dud if r["summary"].get("legs_on_t") is not None)
        resc, lost, p = L.paired(base_g, good)
        med, p99, mx = legs_shift(base_g, good)
        print("| %s | %d/%d | %d | %d/%d (+%d/−%d, p=%.2f) | %.0f / %.0f / %.0f ms | %d |" % (
            name, fc, len(dud), dud_aborts(dud), succ(good), len(good), resc, lost, p,
            med, p99, mx, dud_aborts(good)), flush=True)


def hold(names):
    base = L.run("ld_Ch_good_current", WS, seeds=L.HOLD, plant=CH)
    lo, hi = L.wilson(succ(base), len(base))
    print("\n== holdout 701-710 (2000 flights), good motors")
    print("current: %d/%d = %.1f %% (95 %% CI %.1f-%.1f)" % (succ(base), len(base),
                                                         100 * succ(base) / len(base), lo, hi))
    for name in names:
        rows = L.run("ld_Ch_good_" + name, WS, seeds=L.HOLD, plant=CH, fw=CONFIRM[name])
        resc, lost, p = L.paired(base, rows)
        lo, hi = L.wilson(succ(rows), len(rows))
        med, p99, mx = legs_shift(base, rows)
        print("%s: %d/%d = %.1f %% (CI %.1f-%.1f), paired +%d/−%d p=%.2f, legs later med %.0f / max %.0f ms, "
              "false DUD aborts %d" % (name, succ(rows), len(rows), 100 * succ(rows) / len(rows), lo, hi,
                                       resc, lost, p, med, mx, dud_aborts(rows)), flush=True)
        dud = L.run("ld_Ch_dud_" + name, WS, seeds=L.HOLD, n=60, plant=dict(CH, dud_landing=True),
                    fw=CONFIRM[name])
        print("   dud holdout (600): false confirms %d" % sum(
            1 for r in dud if r["summary"].get("legs_on_t") is not None), flush=True)


def scatter(names=("alt1", "alt3")):
    """A late-lighting motor could light below the altitude gate: good motors
    with ignition delay +-15 % (dev), paired vs current."""
    base = L.run("ld_Cs_good_current", WS, plant=CH, ext=SCATTER)
    print("\n== altitude gate vs ignition-delay scatter +-15 % (good motors, dev 600)")
    print("current: %d/600 success, %d false DUD aborts" % (succ(base), dud_aborts(base)))
    for name in names:
        rows = L.run("ld_Cs_good_" + name, WS, plant=CH, ext=SCATTER, fw=CONFIRM[name])
        resc, lost, p = L.paired(base, rows)
        print("%s: %d/600 (+%d/−%d, p=%.2f), %d false DUD aborts" % (
            name, succ(rows), resc, lost, p, dud_aborts(rows)), flush=True)


if __name__ == "__main__":
    what = sys.argv[1]
    if what == "window":
        window()
    elif what == "confirm":
        confirm()
    elif what == "scatter":
        scatter()
    elif what == "hold":
        hold(sys.argv[2:])
