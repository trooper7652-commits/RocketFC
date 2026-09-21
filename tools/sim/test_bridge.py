#!/usr/bin/env python3
"""Phase 1 verification gate: prove the ctypes bridge (core.py + bridge.cpp)
reproduces tools/replay/main.exe EXACTLY on all five existing replay cases.

This ports main.cpp's runCase() (pad calibration, arming, stepping, event
handling, assertions) line-for-line into Python driving FlightCore through
the DLL instead of a native binary. If this ever disagrees with main.exe,
the bridge is wrong -- fix it before building anything else in tools/sim/ on
top of it.

Run:  python tools/sim/test_bridge.py
(regenerates tools/replay/cases/*.csv via tools/synth_flight.py first if
missing, same as `make test` does for the C++ harness)
"""
import json
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(HERE)
CASES_DIR = os.path.join(TOOLS_DIR, "replay", "cases")

failures = 0


def check(cond, msg):
    global failures
    status = "PASS" if cond else "FAIL"
    print("  {}: {}".format(status, msg))
    if not cond:
        failures += 1


def parse_csv(path):
    with open(path) as f:
        meta = json.loads(f.readline()[len("#META "):])
        f.readline()  # header
        rows = []
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            t, ax, ay, az, gx, gy, gz = (float(x) for x in parts[0:7])
            baro_new = int(parts[7])
            p_pa = float(parts[8]) if parts[8] else 0.0
            h_true, v_true, tilt_true, thrust = (float(x) for x in parts[9:13])
            rows.append((t, ax, ay, az, gx, gy, gz, baro_new, p_pa, h_true,
                        v_true, tilt_true, thrust))
    return meta, rows


def run_case(dll, path):
    meta, rows = parse_csv(path)
    scenario = meta["scenario"]
    mode = meta["mode"]
    arm_t = meta.get("arm_t", 2.6)
    td_true = meta.get("touchdown_t") or 1e9
    max_kf_err = meta.get("max_kf_alt_err", 3.0)
    expect_abort = meta.get("expect_abort")
    expect_states = meta.get("expect_states", [])
    fire_window = meta.get("fire_window")

    print("\n== case {} ({} rows, mode {}) ==".format(scenario, len(rows), mode))

    # --- pad calibration, exactly like main.cpp's runCase() ---
    gbx = gby = gbz = 0.0
    aax = aay = aaz = 0.0
    n_cal = 0
    p_mean = 0.0
    p_m2 = 0.0
    n_p = 0
    for row in rows:
        t = row[0]
        if t >= arm_t - 0.2:
            break
        gbx += row[4]; gby += row[5]; gbz += row[6]
        aax += row[1]; aay += row[2]; aaz += row[3]
        n_cal += 1
        if row[7] and row[8] > 0:
            n_p += 1
            d = row[8] - p_mean
            p_mean += d / n_p
            p_m2 += d * (row[8] - p_mean)
    gbx /= n_cal; gby /= n_cal; gbz /= n_cal
    aax /= n_cal; aay /= n_cal; aaz /= n_cal
    p0 = p_mean
    m_per_pa = 8434.6 / p0
    alt_var = (p_m2 / (n_p - 1)) * m_per_pa * m_per_pa

    with core.FlightCore(dll) as fc:
        fc.begin()
        fc.set_mode(core.FlightMode.FULL_LANDING if mode == "land"
                   else core.FlightMode.CHUTE_TEST)

        seq = ["IDLE"]
        fire_t = -1.0
        abort_t = -1.0
        td_t = -1.0
        launch_t = -1.0
        abort_reason = core.AbortReason.NONE
        max_err = 0.0
        armed = False

        for row in rows:
            (t, ax, ay, az, gx, gy, gz, baro_new_raw, p_pa, h_true, v_true,
             tilt_true, thrust) = row
            ms = int(t * 1000.0 + 0.5)
            if not armed and t >= arm_t:
                fc.pad_level_init((aax, aay, aaz))
                fc.set_baro_noise_var(alt_var if alt_var > 0.0025 else 0.0025)
                fc.zero_altitude()
                fc.request_arm(ms)
                armed = True

            baro_new = bool(baro_new_raw) and p_pa > 0
            baro_alt = 0.0
            if baro_new:
                baro_alt = 44330.0 * (1.0 - (p_pa / p0) ** 0.190295)

            out = fc.step(ms, 0.002, (ax, ay, az), (gx - gbx, gy - gby, gz - gbz),
                         baro_new=baro_new, baro_alt=baro_alt)

            sn = out["state"].name
            if not seq or seq[-1] != sn:
                seq.append(sn)

            for ev in fc.pop_events():
                if ev["code"] == core.FlightEvent.LAUNCH:
                    launch_t = t
                elif ev["code"] == core.FlightEvent.FIRE_LANDING:
                    fire_t = t
                elif ev["code"] == core.FlightEvent.ABORT_DET:
                    abort_t = t
                    abort_reason = core.AbortReason(int(ev["value"]))
                elif ev["code"] == core.FlightEvent.TOUCHDOWN_DET and td_t < 0:
                    td_t = t

            if launch_t > 0 and td_t < 0:
                err = abs(out["kf_alt"] - h_true)
                if err > max_err:
                    max_err = err

    got = " ".join(seq)
    want = " ".join(expect_states)
    check(seq == expect_states, "state sequence [{}] (want [{}])".format(got, want))

    if fire_window:
        fw0, fw1 = fire_window
        check(fw0 <= fire_t <= fw1,
              "landing fire cmd at {:.2f} s inside window [{:.2f}, {:.2f}]".format(
                  fire_t, fw0, fw1))

    if expect_abort:
        check(abort_t > 0 and abort_reason.name == expect_abort,
              "abort {} at {:.2f} s (want {})".format(abort_reason.name, abort_t,
                                                       expect_abort))
    else:
        check(abort_t < 0, "no abort (got {} at {:.2f} s)".format(
            abort_reason.name, abort_t))

    check(td_t > 0 and td_t < td_true + 8.0,
          "touchdown detected at {:.2f} s (true contact {:.2f} s)".format(
              td_t, td_true))

    check(max_err < max_kf_err,
          "KF altitude max error {:.2f} m < {:.2f} m".format(max_err, max_kf_err))

    return {
        "scenario": scenario, "seq": seq, "fire_t": fire_t, "abort_t": abort_t,
        "abort_reason": abort_reason.name, "td_t": td_t, "max_err": max_err,
    }


def main():
    if not os.path.isdir(CASES_DIR) or not os.listdir(CASES_DIR):
        print("regenerating replay cases via tools/synth_flight.py ...")
        subprocess.run([sys.executable, "synth_flight.py"], cwd=TOOLS_DIR,
                       check=True)

    dll = core.default_dll()
    print("bridge DLL:", dll)

    results = []
    for name in sorted(os.listdir(CASES_DIR)):
        if name.endswith(".csv"):
            results.append(run_case(dll, os.path.join(CASES_DIR, name)))

    print("\n{} ({} failure{})".format(
        "ALL PASS" if failures == 0 else "FAILURES", failures,
        "" if failures == 1 else "s"))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
