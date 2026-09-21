"""Reader/writer for the SD-card flight log format src/hw/logger.h writes
(FLIGHTS/flight_NNN.csv): 100 Hz data rows plus interleaved EVT: event rows,
event rows carrying empty data columns. Parsing follows the same approach as
tools/plot_flight.py's load() (which already handles the short/empty-column
event rows correctly).

This module is the ONLY place that knows both naming conventions: the
on-disk logger.h column names, and tools/sim's own internal row-dict schema
(the one run.py's simulate() produces). read_log() translates disk -> sim
schema so the dashboard can treat a real flight and a simulated one
identically; write_log() translates a simulate() result back to disk in the
EXACT on-disk format, so:
  - the dashboard/log-viewer can be developed and tested against a synthetic
    log before any real flight has been flown, and
  - a sim run and a real flight are byte-format-compatible (open either one
    the same way).

Real hardware has no ground truth (h_true, v_true, quat_true, x_true,
y_true) -- read_log() simply omits those keys; callers must handle their
absence (the dashboard's "dead-reckoned trajectory" mode exists precisely
because a real log has no true position to fall back on).
"""
import csv
import math
import os

import core

LOG_HEADER = ("t_ms,state,ax,ay,az,gx,gy,gz,baro_alt,baro_pa,qw,qx,qy,qz,"
             "tilt_deg,kf_alt,kf_vel,kf_bias,innov,pX,iX,dX,pY,iY,dY,"
             "gimx_deg,gimy_deg,servoA_us,servoB_us,pyro,cont,vbat,"
             "loop_max_us,evt")
_COLUMNS = LOG_HEADER.split(",")


def read_log(path):
    """Returns {"rows": [...], "events": [...], "flight_number": int|None}.

    Row dicts use tools/sim's schema (see run.py's row dict for the sim
    side): t (s), state, ax/ay/az, gx/gy/gz, baro_alt, baro_pa, quat_est,
    tilt_deg, kf_alt, kf_vel, kf_bias, innovation, p_x/i_x/d_x/p_y/i_y/d_y,
    gimbal_x_deg/gimbal_y_deg (real hardware only logs the commanded angle,
    not a separately-modeled actual deflection -- unlike sim rows, which
    have both gimbal_x_cmd and gimbal_x_act), us_a, us_b, pyro, cont, vbat,
    loop_max_us. No truth fields.

    Event dicts: {"t": seconds, "name": str, "value": float}.
    """
    rows = []
    events = []
    flight_number = None
    base = os.path.basename(path)
    if base.startswith("flight_") and base.endswith(".csv"):
        try:
            flight_number = int(base[len("flight_"):-len(".csv")])
        except ValueError:
            pass

    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = None
        for row in reader:
            if not row or (row[0].startswith("#")):
                continue
            if header is None:
                header = row
                continue
            if len(row) < len(header):
                row += [""] * (len(header) - len(row))
            rec = dict(zip(header, row))
            evt = rec.get("evt", "")
            t_ms = float(rec["t_ms"]) if rec.get("t_ms") not in (None, "") else None
            if t_ms is None:
                continue
            t = t_ms / 1000.0
            if evt.startswith("EVT:"):
                parts = evt[4:].rsplit(":", 1)
                name = parts[0]
                value = float(parts[1]) if len(parts) > 1 else 0.0
                events.append({"t": t, "name": name, "value": value})
                continue

            def f(key, default=0.0):
                v = rec.get(key, "")
                return float(v) if v not in (None, "") else default

            rows.append({
                "t": t, "state": rec.get("state", ""),
                "ax": f("ax"), "ay": f("ay"), "az": f("az"),
                "gx": f("gx"), "gy": f("gy"), "gz": f("gz"),
                "baro_alt": f("baro_alt"), "baro_pa": f("baro_pa"),
                "quat_est": (f("qw", 1.0), f("qx"), f("qy"), f("qz")),
                "tilt_deg": f("tilt_deg"),
                "kf_alt": f("kf_alt"), "kf_vel": f("kf_vel"),
                "kf_bias": f("kf_bias"), "innovation": f("innov"),
                "p_x": f("pX"), "i_x": f("iX"), "d_x": f("dX"),
                "p_y": f("pY"), "i_y": f("iY"), "d_y": f("dY"),
                "gimbal_x_deg": f("gimx_deg"), "gimbal_y_deg": f("gimy_deg"),
                "us_a": f("servoA_us", 1500.0), "us_b": f("servoB_us", 1500.0),
                "pyro": int(f("pyro", 0)), "cont": int(f("cont", 0)),
                "vbat": f("vbat"), "loop_max_us": f("loop_max_us"),
            })

    return {"rows": rows, "events": events, "flight_number": flight_number}


def write_log(path, sim_result, flight_number=999):
    """Write a run.simulate() result to disk in the EXACT logger.h format.

    Known simplifications vs. real hardware (documented, not hidden):
      - pyro/cont/vbat/loop_max_us aren't modeled in detail: pyro is set from
        the one-shot fire_chute/fire_landing pulse rather than a sustained
        PYRO_FIRE_MS-wide gate bit, cont is always "both good" (3), vbat is a
        fixed nominal placeholder, loop_max_us is always 0.
    """
    rows = sim_result["rows"]
    events = sim_result["events"]
    # Merge rows and events into one time-ordered stream, matching how the
    # real logger interleaves them (events are pushed as soon as they occur,
    # data rows at the fixed LOG_FAST_HZ cadence).
    merged = [(r["t"], "row", r) for r in rows]
    merged += [(e["t"], "evt", e) for e in events]
    merged.sort(key=lambda x: x[0])

    with open(path, "w", newline="\n") as f:
        f.write(LOG_HEADER + "\n")
        for t, kind, obj in merged:
            t_ms = int(round(t * 1000.0))
            if kind == "evt":
                code = obj["code"]
                name = core.log_event_name(code) if hasattr(code, "name") else code
                f.write("{},,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,EVT:{}:{:.2f}\n".format(
                    t_ms, name, obj["value"]))
                continue
            r = obj
            qw, qx, qy, qz = r.get("quat_est", (1.0, 0.0, 0.0, 0.0))
            pyro = (1 if r.get("fire_chute") else 0) | (2 if r.get("fire_landing") else 0)
            f.write(",".join(str(v) for v in [
                t_ms, r["state"],
                "{:.3f}".format(r["ax"]), "{:.3f}".format(r["ay"]), "{:.3f}".format(r["az"]),
                "{:.4f}".format(r["gx"]), "{:.4f}".format(r["gy"]), "{:.4f}".format(r["gz"]),
                "{:.2f}".format(r.get("baro_alt", 0.0)), "{:.1f}".format(r.get("baro_pa", 0.0)),
                "{:.4f}".format(qw), "{:.4f}".format(qx), "{:.4f}".format(qy), "{:.4f}".format(qz),
                "{:.2f}".format(r["tilt_deg"]),
                "{:.2f}".format(r["kf_alt"]), "{:.2f}".format(r["kf_vel"]),
                "{:.3f}".format(r["kf_bias"]), "{:.2f}".format(r["innovation"]),
                "{:.4f}".format(r["p_x"]), "{:.4f}".format(r["i_x"]), "{:.4f}".format(r["d_x"]),
                "{:.4f}".format(r["p_y"]), "{:.4f}".format(r["i_y"]), "{:.4f}".format(r["d_y"]),
                "{:.2f}".format(math.degrees(r["gimbal_x_cmd"])),
                "{:.2f}".format(math.degrees(r["gimbal_y_cmd"])),
                "{:.0f}".format(r["us_a"]), "{:.0f}".format(r["us_b"]),
                pyro, 3, "7.60", 0, "",
            ]) + "\n")

    return path
