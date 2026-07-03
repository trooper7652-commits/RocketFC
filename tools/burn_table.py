#!/usr/bin/env python3
"""Generate the landing-burn ignition table (src/core/burn_table.h).

A solid motor can't throttle or restart, so landing is a pure TIMING problem:
for each descent speed v, find the altitude h_ignite(v) at which the fire
command must be issued (including the e-match ignition delay) so the F15
kills the descent as close to the ground as possible.

The tool also answers the question timing alone can't fix: because the F15's
total impulse is fixed, only descents arriving with roughly the right energy
can be stopped AT the ground (too little energy -> the motor stops you high
and pushes you back up; too much -> you hit hard). The apogee report at the
end tells you which apogee band your ascent must hit — adjust ballast or the
ascent motor until you're inside it.

Usage:
    python tools/burn_table.py                      # defaults from config
    python tools/burn_table.py --mass 0.95 --delay-ms 500 --cda 0.004

Pure stdlib. Regenerate whenever mass, motor, ignition delay, or drag change.
"""
import argparse
import datetime
import math
import os

G = 9.80665
RHO = 1.225


class Motor:
    def __init__(self, path, scale=1.0):
        self.path = path
        self.points = [(0.0, 0.0)]
        header = None
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith(";"):
                    continue
                if header is None:
                    p = line.split()
                    header = p
                    self.name = p[0]
                    self.prop_kg = float(p[4])
                    self.total_kg = float(p[5])
                    continue
                t, F = line.split()[:2]
                self.points.append((float(t), float(F) * scale))
        self.points.sort()
        self.burn_time = self.points[-1][0]
        # cumulative impulse (trapezoid) for mass-burned interpolation
        self.cum = [0.0]
        for i in range(1, len(self.points)):
            t0, f0 = self.points[i - 1]
            t1, f1 = self.points[i]
            self.cum.append(self.cum[-1] + 0.5 * (f0 + f1) * (t1 - t0))
        self.total_impulse = self.cum[-1]

    def thrust(self, t):
        if t <= 0 or t >= self.burn_time:
            return 0.0
        for i in range(1, len(self.points)):
            if self.points[i][0] >= t:
                t0, f0 = self.points[i - 1]
                t1, f1 = self.points[i]
                return f0 + (f1 - f0) * (t - t0) / (t1 - t0)
        return 0.0

    def impulse(self, t):
        if t <= 0:
            return 0.0
        if t >= self.burn_time:
            return self.total_impulse
        for i in range(1, len(self.points)):
            if self.points[i][0] >= t:
                t0 = self.points[i - 1][0]
                t1 = self.points[i][0]
                f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return self.cum[i - 1] + f * (self.cum[i] - self.cum[i - 1])
        return self.total_impulse

    def mass_burned(self, t):
        return self.prop_kg * self.impulse(t) / self.total_impulse


def sim_burn(motor, m_descent, cda, delay_s, h0, v_down, dt=0.002):
    """Fire command at altitude h0 (m) falling at v_down (m/s).
    Returns (impact_velocity[m/s, +down], time_to_impact, peak_reascend[m])."""
    h, v, t = h0, -abs(v_down), 0.0  # v up-positive
    peak_reascend = 0.0
    while t < delay_s + motor.burn_time + 60.0:
        tb = t - delay_s
        T = motor.thrust(tb)
        m = m_descent - motor.mass_burned(tb)
        drag_a = 0.5 * RHO * cda * v * abs(v) / m
        a = T / m - G - drag_a
        v += a * dt
        h += v * dt
        t += dt
        if v > 0 and h > peak_reascend:
            peak_reascend = h
        if h <= 0.0:
            return -v, t, peak_reascend
    return -v, t, peak_reascend


def find_h_ignite(motor, m, cda, delay_s, v_down):
    """Scan + refine for the command altitude minimizing impact speed."""
    best_h, best_vi = 0.5, 1e9
    h = 0.5
    while h <= 300.0:
        vi, _, _ = sim_burn(motor, m, cda, delay_s, h, v_down, dt=0.004)
        if abs(vi) < abs(best_vi):
            best_h, best_vi = h, vi
        h += 0.5
    h = max(0.25, best_h - 0.5)
    while h <= best_h + 0.5:
        vi, _, _ = sim_burn(motor, m, cda, delay_s, h, v_down, dt=0.002)
        if abs(vi) < abs(best_vi):
            best_h, best_vi = h, vi
        h += 0.05
    return best_h, best_vi


def descent_ignition_point(apogee, m, cda, hi_of_v, margin=0.0, dt=0.002):
    """Freefall from apogee; return (h, v_down, t) when the FSM's ignition
    condition h <= h_ignite(v)+margin first trips, or None."""
    h, v, t = apogee, 0.0, 0.0
    while h > 0:
        drag_a = 0.5 * RHO * cda * v * abs(v) / m
        v += (-G - drag_a) * dt
        h += v * dt
        t += dt
        vd = -v
        if vd > 2.0 and h <= hi_of_v(vd) + margin:
            return h, vd, t
    return None


def interp_factory(v_min, v_step, table):
    def hi(v):
        if v <= v_min:
            return table[0]
        idx = (v - v_min) / v_step
        i = int(idx)
        if i >= len(table) - 1:
            return table[-1]
        f = idx - i
        return table[i] * (1 - f) + table[i + 1] * f
    return hi


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--eng", default=os.path.join(here, "motors", "Estes_F15.eng"))
    ap.add_argument("--mass", type=float, default=0.95,
                    help="vehicle mass at descent, landing motor loaded (kg)")
    ap.add_argument("--cda", type=float, default=0.004, help="drag Cd*A (m^2)")
    ap.add_argument("--delay-ms", type=float, default=500.0,
                    help="measured ignition delay, command->thrust (ms)")
    ap.add_argument("--vmin", type=float, default=2.0)
    ap.add_argument("--vmax", type=float, default=25.0)
    ap.add_argument("--vstep", type=float, default=0.5)
    ap.add_argument("--out", default=os.path.join(here, "..", "src", "core",
                                                  "burn_table.h"))
    args = ap.parse_args()

    motor = Motor(args.eng)
    delay_s = args.delay_ms / 1000.0
    print(f"motor {motor.name}: {motor.total_impulse:.1f} N*s over "
          f"{motor.burn_time:.2f} s, prop {motor.prop_kg*1000:.0f} g")
    dv = motor.total_impulse / args.mass - G * motor.burn_time
    print(f"vehicle {args.mass:.3f} kg -> net delta-v capacity ~ {dv:.1f} m/s "
          f"(energy-matched arrival speed)")

    n = int(round((args.vmax - args.vmin) / args.vstep)) + 1
    vs, table, residuals = [], [], []
    print("\n  v_down   h_ignite   |v_impact| (ideal timing)")
    for i in range(n):
        v = args.vmin + i * args.vstep
        h, vi = find_h_ignite(motor, args.mass, args.cda, delay_s, v)
        vs.append(v)
        table.append(h)
        residuals.append(vi)
        flag = "" if abs(vi) < 2.5 else "   <-- can't land softly from this speed"
        print(f"  {v:6.1f}   {h:8.2f}   {abs(vi):8.2f}{flag}")

    # ---- emit header ----
    out = os.path.abspath(args.out)
    stamp = datetime.date.today().isoformat()
    rows = []
    for i in range(0, n, 10):
        rows.append("  " + " ".join(f"{h:7.2f}f," for h in table[i:i + 10]))
    with open(out, "w", newline="\n") as f:
        f.write(f"""#pragma once
// GENERATED by tools/burn_table.py on {stamp} — DO NOT EDIT BY HAND.
//   motor: {motor.name} ({os.path.basename(args.eng)}), {motor.total_impulse:.1f} N*s / {motor.burn_time:.2f} s
//   mass at descent: {args.mass:.3f} kg   CdA: {args.cda:.4f} m^2
//   ignition delay: {args.delay_ms:.0f} ms (command -> thrust)
// Maps descent speed (m/s, downward-positive) to the altitude (m AGL) at
// which the landing-motor fire command must be issued.
namespace burntable {{

constexpr float V_MIN = {args.vmin}f;
constexpr float V_STEP = {args.vstep}f;
constexpr int N = {n};

constexpr float H_IGNITE[N] = {{
{chr(10).join(rows)}
}};

// Linear interpolation, clamped to the table domain.
inline float hIgnite(float vDown) {{
  if (vDown <= V_MIN) return H_IGNITE[0];
  const float idx = (vDown - V_MIN) / V_STEP;
  const int i = (int)idx;
  if (i >= N - 1) return H_IGNITE[N - 1];
  const float f = idx - (float)i;
  return H_IGNITE[i] * (1.0f - f) + H_IGNITE[i + 1] * f;
}}

}}  // namespace burntable
""")
    print(f"\nwrote {out}")

    # ---- apogee guidance report ----
    hi = interp_factory(args.vmin, args.vstep, table)
    print("\n==== APOGEE WINDOW REPORT ====")
    print("(freefall from apogee, FSM fires when h <= h_ignite(v); impact speed")
    print(" tells you which apogees your ascent should target)")
    print("  apogee   ignite@h   v_at_ignite   |v_impact|")
    good = []
    a = 15.0
    while a <= 220.0:
        pt = descent_ignition_point(a, args.mass, args.cda, hi)
        if pt:
            h0, vd, _ = pt
            vi, _, peak = sim_burn(motor, args.mass, args.cda, delay_s, h0, vd)
            note = ""
            if abs(vi) <= 2.5:
                good.append(a)
                note = "  GOOD"
            if peak > h0 + 1:
                note += f"  (re-ascends to {peak:.0f} m!)"
            print(f"  {a:6.0f}   {h0:8.2f}   {vd:11.2f}   {abs(vi):9.2f}{note}")
        a += 5.0
    if good:
        print(f"\n>> Target apogee window: {min(good):.0f}-{max(good):.0f} m "
              f"(touchdown under 2.5 m/s with ideal timing).")
        mid = good[len(good) // 2]
        print(f">> Sensitivity at {mid:.0f} m apogee:")
        for label, scale, dly in [("thrust -10%", 0.9, delay_s),
                                  ("thrust +10%", 1.1, delay_s),
                                  ("delay +150 ms", 1.0, delay_s + 0.15),
                                  ("delay -150 ms", 1.0, max(0.0, delay_s - 0.15))]:
            m2 = Motor(args.eng, scale=scale)
            pt = descent_ignition_point(mid, args.mass, args.cda, hi)
            if pt:
                h0, vd, _ = pt
                vi, _, _ = sim_burn(m2, args.mass, args.cda, dly, h0, vd)
                print(f"     {label:14s} -> |v_impact| {abs(vi):5.2f} m/s")
    else:
        print("\n>> NO apogee lands softly with this mass/motor combination!")
        print(f">> Net delta-v capacity is {dv:.1f} m/s — adjust vehicle mass")
        print(">> (ballast) so the energy-matched arrival speed is reachable.")


if __name__ == "__main__":
    main()
