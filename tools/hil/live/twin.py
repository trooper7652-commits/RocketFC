"""The pure-sim twin of a live bench flight, and the comparison.

The twin flies the SAME scenario through the SAME flight model (flight.py),
with the flight code compiled for the PC from the HIL build tree (the config.h
and burn table the Teensy was built with). It replays the live run's own tick
timeline -- every telemetry packet's millis(), tick period and the sequence
number of the sensor frame the Teensy actually used -- and emulates the HIL
sensor shim (tools/hil/firmware/sensors_hil.h LIVE: held frames, baro flags,
100 ms link loss, frozen-data check, operator abort). Sensor noise comes from
the same seeded streams in the same order, so a healthy bench run and its twin
see identical inputs; whatever differs afterwards is the Teensy (or the USB
link) behaving differently from the PC build.
"""
import math
import struct

import flight
from flight import core

LINK_LOSS_MS = 100
FROZEN_TICKS = 250

# Tolerances for compare().
TOL_EVENT_S = 0.050
TOL_HEIGHT_M = 0.5
TOL_SPEED_MPS = 0.3


_US = struct.unpack("<f", struct.pack("<f", 1e-6))[0]


def _f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def fly(session, scen, dll=None, profile=None):
    """session: runner.Runner.session_data() (ticks + timeline). Returns a
    result like run.simulate()'s, in plant time (rebase() it like the run)."""
    profile = profile or flight.load_profile()
    dll = dll or flight.flight_dll()
    snap = core.FlightCore.config_snapshot(dll)
    vcfg = flight.vehicle_for(dll, profile)
    fl = flight.LiveFlight(scen, vcfg, snap)
    tl = session["timeline"]
    ticks = session["ticks"]
    fc = core.FlightCore(dll)
    fc.begin()
    fc.set_mode(core.FlightMode.FULL_LANDING if scen.mode == "land"
                else core.FlightMode.CHUTE_TEST)
    pad0 = session.get("pad0", (0.0, 0.0, flight.G0))
    # sensors_hil.h LIVE state
    a, g = tuple(pad0), (0.0, 0.0, 0.0)
    baro_alt = 0.0
    baro_pending = False
    fail = False
    last_a, last_g = a, g   # `live` primes the frozen check with the pad frame
    same = 0
    fresh = True
    last_frame_ms = ticks[0][0] if ticks else 0
    prev_seq = 0
    frames = {}
    rows, events = [], []
    imu_bad_ticks = 0
    ms0 = ticks[0][0] if ticks else 0
    cal_tick = tl.get("cal_tick")
    arm_tick = tl.get("arm_tick")
    launch_tick = tl.get("launch_tick")
    abort_ms = tl.get("abort_ms")
    record_from = cal_tick if cal_tick is not None else 0
    try:
        for k, (ms, period_us, seq, _bits, _state) in enumerate(ticks):
            t = (ms - ms0) / 1000.0
            if k == cal_tick:
                fl.finish_cal()
            if k == launch_tick:
                fl.launch()
            fl.advance(t)
            # -- the frames the Teensy received since its last tick --
            n_new = (seq - prev_seq) & 0xFFFF
            if n_new and n_new < 1000:
                for i in range(1, n_new + 1):
                    f = frames.get((prev_seq + i) & 0xFFFF)
                    if f is None:
                        continue
                    a, g = f[2], f[3]
                    if f[1] & 1:
                        baro_alt = f[4]
                        baro_pending = True
                    fail = bool(f[1] & 2)
                last_frame_ms = ms
                prev_seq = seq
            if last_g is not None and g == last_g and a[0] == last_a[0]:
                same += 1
                if same > FROZEN_TICKS:
                    fresh = False
            else:
                same = 0
                fresh = True
            last_a, last_g = a, g
            link_lost = ms - last_frame_ms > LINK_LOSS_MS
            aborted = abort_ms is not None and ms >= abort_ms
            healthy = not (link_lost or fail or aborted or not fresh)
            imu_bad_ticks += 0 if healthy else 1
            if k == arm_tick and fl.cal is not None:
                fc.pad_level_init(fl.cal["pad_accel"])
                fc.set_baro_noise_var(fl.cal["baro_var"])
                fc.zero_altitude()
                fc.request_arm(int(tl.get("arm_ms", ms)))
            dt = min(max(_f32(period_us * _US), 0.0005), 0.008)  # RocketFC.ino, in float
            out = fc.step(ms, dt, a, g, baro_new=baro_pending, baro_alt=baro_alt,
                          imu_healthy=healthy)
            baro_pending = False
            for ev in fc.pop_events():
                events.append({"t": (ev["ms"] - ms0) / 1000.0, "ms": ev["ms"],
                               "code": ev["code"], "value": ev["value"]})
            fl.command(out["gimbal_x"], out["gimbal_y"], out["fire_landing"],
                       out["chute_release"], out["legs_burn"], t)
            if k >= record_from:
                o = dict(out, state=out["state"].name, quat=out["quat"])
                rows.append(fl.row(o))
            f = fl.frame()
            if f is not None:
                frames[f[0]] = f
                frames.pop((f[0] - 2000) & 0xFFFF, None)
    finally:
        fc.close()
    summary = flight.summarize(fl, rows, events, scen.mode)
    summary["imu_bad_ticks"] = imu_bad_ticks
    return {"rows": rows, "events": events, "summary": summary}


# ------------------------------------------------------------------ compare
def _ev_name(e):
    return getattr(e["code"], "name", str(e["code"]))


def compare(live, twin):
    """[(item, live, twin, diff, tol, ok)] -- events by occurrence, then the
    headline numbers. Both results on the same time base."""
    out = []

    def by_name(evs):
        d = {}
        for e in evs:
            d.setdefault(_ev_name(e), []).append(e["t"])
        return d

    le, te = by_name(live["events"]), by_name(twin["events"])
    order = []
    for e in sorted(live["events"] + twin["events"], key=lambda e: e["t"]):
        n = _ev_name(e)
        if n not in order and n not in ("ARM", "DISARM"):
            order.append(n)
    for n in order:
        lt, tt = le.get(n, []), te.get(n, [])
        for i in range(max(len(lt), len(tt))):
            a = lt[i] if i < len(lt) else None
            b = tt[i] if i < len(tt) else None
            label = n if max(len(lt), len(tt)) == 1 else "{} #{}".format(n, i + 1)
            if a is None or b is None:
                out.append((label + " (s)", a, b, None, TOL_EVENT_S, False))
            else:
                out.append((label + " (s)", a, b, a - b, TOL_EVENT_S, abs(a - b) <= TOL_EVENT_S))

    ls, ts = live["summary"], twin["summary"]
    for key, label, tol in (("apogee_h", "apogee (m)", TOL_HEIGHT_M),
                            ("fire_h_true", "fire height (m)", TOL_HEIGHT_M),
                            ("fire_v_true", "fire speed (m/s)", TOL_SPEED_MPS),
                            ("touchdown_speed_mps", "touchdown speed (m/s)", TOL_SPEED_MPS),
                            ("touchdown_t", "touchdown (s)", 4 * TOL_EVENT_S)):
        a, b = ls.get(key), ts.get(key)
        if a is None and b is None:
            continue
        if a is None or b is None:
            out.append((label, a, b, None, tol, False))
        else:
            out.append((label, a, b, a - b, tol, abs(a - b) <= tol))
    for key, label in (("abort_reason", "abort reason"), ("final_state", "final state"),
                       ("chute_releases", "chute releases")):
        a, b = ls.get(key), ts.get(key)
        out.append((label, a, b, None, None, a == b))
    lo, to = ls.get("landing_outcome"), ts.get("landing_outcome")
    if lo or to:
        a = lo["label"] if lo else None
        b = to["label"] if to else None
        out.append(("outcome", a, b, None, None, a == b))
    return out


def format_table(rows):
    def f(v):
        if v is None:
            return "-"
        if isinstance(v, float):
            return "nan" if math.isnan(v) else "{:.3f}".format(v)
        return str(v)
    lines = ["{:<26} {:>12} {:>12} {:>9}  {}".format("", "bench", "twin", "diff", "")]
    for item, a, b, d, tol, ok in rows:
        lines.append("{:<26} {:>12} {:>12} {:>9}  {}".format(
            item, f(a), f(b), f(d) if d is not None else "", "ok" if ok else "** DIFFERS **"))
    n_bad = sum(1 for r in rows if not r[5])
    lines.append("{} of {} checks agree".format(len(rows) - n_bad, len(rows)))
    return "\n".join(lines)
