#!/usr/bin/env python3
""""What-if" re-run: feed a real (or sim-written) flight log's raw sensor
readings back through a freshly built -- possibly config-patched -- flight
core, and compare what it WOULD have decided against what the original
flight computer actually did.

Two real limitations, both surfaced here rather than hidden:

  - OPEN LOOP. The vehicle's true motion is fixed history (whatever actually
    happened). This can validly test estimator tuning, thresholds, and
    state-machine changes (KF_SIGMA_*, TILT_ABORT_DEG, BURN_TABLE_MARGIN_M,
    APOGEE_*, ...) but CANNOT evaluate different PID gains -- different
    gains would have flown a physically different trajectory, which this
    re-run has no way to produce. Gain changes belong in run.simulate()'s
    closed loop instead.
  - COARSER RATE. The log only has LOG_FAST_HZ (100 Hz) rows, not the
    FAST_HZ (500 Hz) rate FlightCore actually steps at in flight -- so this
    re-run steps at ~100 Hz, giving debounce/threshold timing ~10 ms
    resolution here vs ~2 ms on the original flight. Fine for "would this
    threshold have tripped"; imprecise for exact millisecond timing.

What does NOT need recalibrating: src/hw/sensors.h already applies
IMU_R_SB and gyro bias removal before RocketFC.ino logs ax/ay/az/gx/gy/gz
(see sensors.h's accel()/gyro()/mapToBody()), so the logged values are used
exactly as logged. baro_alt is likewise used as logged (already zeroed to
the ORIGINAL flight's own pad reference) rather than re-derived -- the new
core's zero_altitude() re-anchors to whatever that column reads at the ARM
moment, which is correct as long as the vehicle launched and landed at the
same physical spot (the normal case).
"""
import sys

import core
import logfile


def reflight(log_path, dll_path=None, mode=None):
    """Returns {"rows": [...new core outputs...], "events": [...new
    FlightEvents...], "original_events": [...from the log...],
    "summary": {...}}."""
    log = logfile.read_log(log_path)
    rows = log["rows"]
    orig_events = log["events"]
    dll_path = dll_path or core.default_dll()

    if mode is None:
        mode = ("land" if any(r["state"] == "LANDING_BURN" for r in rows)
               else "chute")

    arm_t = next((e["t"] for e in orig_events if e["name"] == "ARM"), None)
    if arm_t is None:
        raise ValueError("log has no ARM event -- cannot re-arm for reflight")

    cal_end_t = arm_t - 0.2
    aax = aay = aaz = 0.0
    n_cal = 0
    baro_pre_arm = []
    for r in rows:
        if r["t"] >= cal_end_t:
            break
        aax += r["ax"]; aay += r["ay"]; aaz += r["az"]
        n_cal += 1
        baro_pre_arm.append(r["baro_alt"])
    if n_cal == 0:
        raise ValueError("log has no data before ARM to calibrate pad-level from")
    aax /= n_cal; aay /= n_cal; aaz /= n_cal
    if len(baro_pre_arm) > 1:
        mean_b = sum(baro_pre_arm) / len(baro_pre_arm)
        baro_var = sum((b - mean_b) ** 2 for b in baro_pre_arm) / (len(baro_pre_arm) - 1)
    else:
        baro_var = 0.25

    fc = core.FlightCore(dll_path)
    fc.begin()
    fc.set_mode(core.FlightMode.FULL_LANDING if mode == "land"
               else core.FlightMode.CHUTE_TEST)

    new_rows = []
    new_events = []
    armed = False
    prev_t = None
    try:
        for r in rows:
            dt = (r["t"] - prev_t) if prev_t is not None else 0.01
            prev_t = r["t"]
            ms = int(r["t"] * 1000.0 + 0.5)

            if not armed and r["t"] >= arm_t:
                fc.pad_level_init((aax, aay, aaz))
                fc.set_baro_noise_var(max(baro_var, 0.0025))
                fc.zero_altitude()
                fc.request_arm(ms)
                armed = True

            out = fc.step(ms, dt, (r["ax"], r["ay"], r["az"]),
                         (r["gx"], r["gy"], r["gz"]), baro_new=True,
                         baro_alt=r["baro_alt"])
            for ev in fc.pop_events():
                ev = dict(ev)
                ev["t"] = r["t"]
                new_events.append(ev)
            new_rows.append(dict(out, t=r["t"]))
    finally:
        fc.close()

    orig_seq = [e["name"] for e in orig_events]
    new_seq = [core.log_event_name(ev["code"]) for ev in new_events]
    orig_abort = next((e for e in orig_events if e["name"] == "ABORT"), None)
    new_abort = next((e for e in new_events
                      if e["code"] == core.FlightEvent.ABORT_DET), None)

    summary = {
        "mode": mode,
        "orig_final_state": rows[-1]["state"] if rows else None,
        "new_final_state": new_rows[-1]["state"].name if new_rows else None,
        "orig_event_sequence": orig_seq,
        "new_event_sequence": new_seq,
        "diverged": orig_seq != new_seq,
        "orig_abort": orig_abort["name"] if orig_abort else None,
        "new_abort": core.log_event_name(core.FlightEvent.ABORT_DET) if new_abort else None,
        "new_abort_reason": (core.AbortReason(int(new_abort["value"])).name
                             if new_abort else None),
    }
    return {"rows": new_rows, "events": new_events,
            "original_events": orig_events, "summary": summary}


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: python reflight.py FLIGHTS/flight_NNN.csv [config_overrides.json]")
    log_path = sys.argv[1]
    dll = core.default_dll()
    if len(sys.argv) > 2:
        import json
        with open(sys.argv[2]) as f:
            overrides = json.load(f)
        dll = core.build_with_overrides(overrides)
    result = reflight(log_path, dll_path=dll)
    for k, v in result["summary"].items():
        print("{}: {}".format(k, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
