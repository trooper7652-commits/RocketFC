#!/usr/bin/env python3
"""Check a HIL ground-test run against the trace's PC reference.

    python tools/hil/compare.py --trace tools/hil/traces/nominal \\
        --flight E:/FLIGHTS/flight_012.csv --imu E:/HIL/IMU_003.BIN [--plot]

Everything is lined up on the moment playback started (`go`): taken from the
real-IMU file's header, else from --session (hil_ctl.py's session json), else
-- with a warning -- from the LAUNCH event.

Reports:
  1. events, Teensy vs reference (same order, within --tol ms)
  2. gimbal command difference (RMS / max, degrees)
  3. worst fast-loop period (the 500 Hz loop should stay near 2000 us)
  4. pyro gate timing from the log's pyro bits: landing gate on/off, legs on/off
  5. with --imu: the real ignition delay, FIRE_LANDING -> vibration onset of
     the real accelerometer. This is the measurement IGNITION_DELAY_MS
     (src/config.h) wants. Check it on the --plot figure before trusting it.

Exit code 0 only if 1-3 pass.
"""
import argparse
import bisect
import json
import math
import os
import struct
import sys

import hil_trace  # puts tools/sim on sys.path

import logfile  # noqa: E402

IMU_HDR = 64
IMU_REC = struct.Struct("<I3h3h")
LOOP_WARN_US = 4000


def read_imu_bin(path):
    with open(path, "rb") as f:
        blob = f.read()
    if blob[:10] != b"RFCHILIMU1":
        raise ValueError(path + ": not a HIL real-IMU file")
    start_ms, crc, frames = struct.unpack_from("<III", blob, 16)
    recs = []
    for off in range(IMU_HDR, len(blob) - IMU_REC.size + 1, IMU_REC.size):
        ms, ax, ay, az, gx, gy, gz = IMU_REC.unpack_from(blob, off)
        recs.append((ms, ax / 100.0, ay / 100.0, az / 100.0,
                     gx / 1000.0, gy / 1000.0, gz / 1000.0))
    return {"start_ms": start_ms, "crc32": "{:08x}".format(crc), "frames": frames,
            "recs": recs}


def edges(rows, bit, t0):
    """[(t_rel_s, on)] transitions of one pyro bit in the log rows."""
    out, prev = [], False
    for r in rows:
        on = bool(r["pyro"] & bit)
        if on != prev:
            out.append((r["t"] - t0, on))
            prev = on
    return out


def ignition_onset(recs, fire_ms, floor, k_sigma=6.0, win_ms=10, need=3):
    """First 10 ms window after fire_ms whose high-passed |accel| RMS stands
    out from the pre-fire baseline. Returns (onset_ms, baseline, threshold,
    series) or (None, ...)."""
    if not recs:
        return None, None, None, []
    ts = [r[0] for r in recs]
    mag = [math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2) for r in recs]
    # High-pass: subtract a trailing 100 ms mean, leaving the vibration.
    hp, acc, q = [], 0.0, []
    for m in mag:
        q.append(m)
        acc += m
        if len(q) > 50:
            acc -= q.pop(0)
        hp.append(m - acc / len(q))
    series = []  # (window start ms, RMS)
    i = 0
    while i < len(ts):
        j = i
        s2 = 0.0
        while j < len(ts) and ts[j] < ts[i] + win_ms:
            s2 += hp[j] ** 2
            j += 1
        series.append((ts[i], math.sqrt(s2 / max(1, j - i))))
        i = j
    base = [v for t, v in series if fire_ms - 1000 <= t < fire_ms - 100]
    if len(base) < 10:
        return None, None, None, series
    mu = sum(base) / len(base)
    sd = math.sqrt(sum((v - mu) ** 2 for v in base) / len(base))
    thr = max(mu + k_sigma * sd, mu + floor)
    run = 0
    for n, (t, v) in enumerate(series):
        if t < fire_ms:
            continue
        run = run + 1 if v > thr else 0
        if run >= need:
            return series[n - need + 1][0], mu, thr, series
    return None, mu, thr, series


def interp(ts, vs, t):
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
    return vs[i - 1] + f * (vs[i] - vs[i - 1])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--trace", required=True, help="trace base path (with _ref.csv beside it)")
    ap.add_argument("--flight", required=True, help="the Teensy's FLIGHTS/flight_NNN.csv")
    ap.add_argument("--imu", help="the Teensy's HIL/IMU_NNN.BIN")
    ap.add_argument("--session", help="hil_ctl.py session json (for go_ms)")
    ap.add_argument("--tol", type=float, default=20.0, help="event tolerance, ms")
    ap.add_argument("--onset-floor", type=float, default=1.5,
                    help="minimum vibration rise counted as ignition, m/s^2 RMS")
    ap.add_argument("--plot", action="store_true", help="save <flight>_hil.png")
    args = ap.parse_args()

    _, meta = hil_trace.load(args.trace)
    base = os.path.splitext(args.trace)[0] if args.trace.endswith((".bin", ".json")) else args.trace
    ref = logfile.read_log(base + "_ref.csv")
    fl = logfile.read_log(args.flight)
    ref_go = meta["ref_go_ms"] / 1000.0

    imu = read_imu_bin(args.imu) if args.imu else None
    if imu and imu["crc32"] != meta["crc32"]:
        print("WARNING: the IMU file was recorded with a different trace (crc {} vs {})".format(
            imu["crc32"], meta["crc32"]))
    go = None
    if imu:
        go, src = imu["start_ms"] / 1000.0, "real-IMU file header"
    elif args.session:
        with open(args.session) as f:
            g = json.load(f).get("go_ms")
        if g is not None:
            go, src = g / 1000.0, "session file"
    if go is None:
        lt = [e["t"] for e in fl["events"] if e["name"] == "LAUNCH"]
        rl = [e["t"] for e in ref["events"] if e["name"] == "LAUNCH"]
        if not lt or not rl:
            raise SystemExit("can't line the logs up: no go time and no LAUNCH event")
        go, src = lt[0] - (rl[0] - ref_go), "LAUNCH event (WARNING: hides a launch-time error)"
    print("aligned on playback start ({}): Teensy t={:.3f} s".format(src, go))

    ok = True
    # ---- 1. events ----
    te = [(e["name"], e["t"] - go) for e in fl["events"] if e["t"] - go >= -0.001]
    re_ = [(e["name"], e["t"] - ref_go) for e in ref["events"] if e["t"] - ref_go >= -0.001]
    print("\n1. events (s after go)\n{:<22}{:>10}{:>10}{:>9}".format(
        "", "expected", "Teensy", "diff ms"))
    for i in range(max(len(te), len(re_))):
        r = re_[i] if i < len(re_) else ("-", None)
        t = te[i] if i < len(te) else ("-", None)
        d = (t[1] - r[1]) * 1000.0 if r[1] is not None and t[1] is not None else None
        bad = r[0] != t[0] or d is None or abs(d) > args.tol
        ok &= not bad
        print("{:<22}{:>10}{:>10}{:>9}  {}".format(
            r[0], "" if r[1] is None else "{:.3f}".format(r[1]),
            "" if t[1] is None else "{:.3f}".format(t[1]),
            "" if d is None else "{:+.0f}".format(d),
            ("FAIL" + ("" if r[0] == t[0] else ": Teensy logged " + t[0])) if bad else "ok"))

    # ---- 2. gimbal ----
    rr = [r for r in ref["rows"] if r["t"] >= ref_go]
    rts = [r["t"] - ref_go for r in rr]
    want_of = {k: [x[k] for x in rr] for k in ("gimbal_x_deg", "gimbal_y_deg")}
    worst = 0.0
    s2 = 0.0
    n = 0
    for r in fl["rows"]:
        t = r["t"] - go
        if t < 0 or t > rts[-1]:
            continue
        for key, want in want_of.items():
            d = r[key] - interp(rts, want, t)
            s2 += d * d
            n += 1
            worst = max(worst, abs(d))
    rms = math.sqrt(s2 / n) if n else float("nan")
    g_ok = n > 0 and rms < 0.5 and worst < 3.0
    ok &= g_ok
    print("\n2. gimbal command vs expected: RMS {:.2f} deg, max {:.2f} deg  {}".format(
        rms, worst, "ok" if g_ok else "FAIL (limits: RMS 0.5, max 3.0)"))

    # ---- 3. loop timing ----
    lm = [r["loop_max_us"] for r in fl["rows"] if r["t"] >= go]
    over = sum(1 for v in lm if v > LOOP_WARN_US)
    l_ok = bool(lm) and over == 0
    ok &= l_ok
    print("3. worst fast-loop period {:.0f} us; {} log rows over {} us  {}".format(
        max(lm) if lm else float("nan"), over, LOOP_WARN_US, "ok" if l_ok else "FAIL"))

    # ---- 4. pyro gates ----
    fires = [t for nm, t in te if nm == "FIRE_LANDING"]
    land = edges(fl["rows"], 2, go)
    legs = edges(fl["rows"], 4, go)
    chute = edges(fl["rows"], 1, go)
    print("\n4. outputs from the log's pyro bits (s after go; 10 ms resolution):")
    print("   landing gate: " + (", ".join("{} {:.2f}".format("on" if on else "off", t)
                                         for t, on in land) or "never on"))
    print("   legs nichrome: " + (", ".join("{} {:.2f}".format("on" if on else "off", t)
                                          for t, on in legs) or "never on"))
    print("   chute latch:  " + (", ".join("{} {:.2f}".format("release" if on else "lock", t)
                                         for t, on in chute) or "never released"))
    if meta.get("fire_s") is None and land:
        print("   FAIL: this trace must NOT fire the landing motor")
        ok = False
    vb = [r["vbat"] for r in fl["rows"] if r["t"] >= go and r["vbat"] > 1.0]
    if vb:
        print("   flight battery: min {:.2f} V, max {:.2f} V".format(min(vb), max(vb)))

    # ---- 5. real ignition delay ----
    if imu and fires:
        fire_ms = (go + fires[0]) * 1000.0
        onset, mu, thr, series = ignition_onset(imu["recs"], fire_ms, args.onset_floor)
        print("\n5. real IMU: {} samples, {:.1f} s".format(
            len(imu["recs"]), (imu["recs"][-1][0] - imu["recs"][0][0]) / 1000.0
            if imu["recs"] else 0.0))
        if onset is None:
            print("   no vibration onset found after FIRE_LANDING (baseline {}, threshold {})".format(
                "-" if mu is None else "{:.2f}".format(mu), "-" if thr is None else "{:.2f}".format(thr)))
        else:
            print("   vibration onset {:.0f} ms after FIRE_LANDING (baseline {:.2f}, threshold "
                  "{:.2f} m/s^2 RMS)".format(onset - fire_ms, mu, thr))
            print("   => measured ignition delay ~{:.0f} ms (src/config.h IGNITION_DELAY_MS "
                  "is a placeholder). Confirm on the plot.".format(onset - fire_ms))
        if args.plot:
            plot(args.flight, imu, fire_ms, onset, thr, series, go, land, legs)
    elif imu:
        print("\n5. no FIRE_LANDING in this run: no ignition delay to measure")

    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def plot(flight_path, imu, fire_ms, onset, thr, series, go, land, legs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t0 = fire_ms
    recs = [r for r in imu["recs"] if t0 - 1500 <= r[0] <= t0 + 3000]
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax[0].plot([(r[0] - t0) for r in recs],
               [math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2) for r in recs], lw=0.6)
    ax[0].set_ylabel("|accel| m/s²")
    ax[1].plot([t - t0 for t, _ in series if t0 - 1500 <= t <= t0 + 3000],
               [v for t, v in series if t0 - 1500 <= t <= t0 + 3000], lw=0.8)
    if thr is not None:
        ax[1].axhline(thr, color="0.5", ls="--", lw=0.8, label="threshold")
    ax[1].set_ylabel("vibration RMS m/s²")
    ax[1].set_xlabel("ms after FIRE_LANDING")
    for a in ax:
        a.axvline(0, color="C3", lw=1, label="FIRE_LANDING")
        if onset is not None:
            a.axvline(onset - t0, color="C2", lw=1, label="onset {:.0f} ms".format(onset - t0))
        for t, on in legs:
            if on:
                a.axvline((go + t) * 1000 - t0, color="C1", lw=0.8, ls=":", label="legs on")
    ax[0].legend(loc="upper right", fontsize=8)
    out = os.path.splitext(flight_path)[0] + "_hil.png"
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print("   plot:", out)


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
