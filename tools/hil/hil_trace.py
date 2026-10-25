"""HIL trace format + PC reference replay (shared by make_trace, hil_ctl, compare).

A trace is two files:
    <name>.bin   N frames of 32 bytes, little-endian, exactly
                 firmware/sensors_hil.h's HilFrame:
                     float ax, ay, az, gx, gy, gz, baroAlt; uint32 flags
                 (flags bit0 = new baro sample this tick), one per 500 Hz tick
                 from the sim's arm tick onward.
    <name>.json  metadata: frame count, CRC32 of the .bin (zlib.crc32, the
                 same CRC the Teensy computes), flight mode, the pad-level
                 accel + baro noise variance the sim armed with, where launch
                 and the landing-motor fire fall in the trace, and the
                 reference events.

replay() drives the real flight core (tools/sim's DLL) through the SAME
sequence the HIL firmware does -- still pad, arm with the trace's pad level,
still pad, trace frames, resting frame -- so its output is the expected
answer for the Teensy, row for row.
"""
import json
import os
import struct
import sys
import zlib

HIL_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(os.path.dirname(HIL_DIR), "sim")
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

import core  # noqa: E402  (tools/sim)

FRAME = struct.Struct("<7fI")
assert FRAME.size == 32
MAX_FRAMES = 13000  # sensors_hil.h hil::kMaxFrames
FAST_DT = 0.002
FORMAT_VERSION = 1


def frames_from_inputs(inputs):
    """run.simulate(record_inputs=True)["inputs"] -> packed .bin bytes."""
    out = bytearray()
    for _ms, _dt, a, g, baro_new, baro_alt in inputs:
        out += FRAME.pack(a[0], a[1], a[2], g[0], g[1], g[2], baro_alt,
                          1 if baro_new else 0)
    return bytes(out)


def unpack_frames(blob):
    return [FRAME.unpack_from(blob, i) for i in range(0, len(blob), FRAME.size)]


def save(base, blob, meta):
    meta = dict(meta)
    meta["version"] = FORMAT_VERSION
    meta["frames"] = len(blob) // FRAME.size
    meta["crc32"] = "{:08x}".format(zlib.crc32(blob) & 0xFFFFFFFF)
    with open(base + ".bin", "wb") as f:
        f.write(blob)
    with open(base + ".json", "w") as f:
        json.dump(meta, f, indent=2)
    return meta


def load(path):
    """path: <name>.bin, <name>.json or <name>. Returns (blob, meta)."""
    base = os.path.splitext(path)[0] if path.endswith((".bin", ".json")) else path
    with open(base + ".bin", "rb") as f:
        blob = f.read()
    with open(base + ".json") as f:
        meta = json.load(f)
    crc = "{:08x}".format(zlib.crc32(blob) & 0xFFFFFFFF)
    if crc != meta["crc32"] or len(blob) != meta["frames"] * FRAME.size:
        raise ValueError("{}: .bin does not match its .json (crc {} vs {})".format(
            base, crc, meta["crc32"]))
    return blob, meta


def replay(dll, blob, meta, hold_idle_s=1.0, hold_armed_s=1.0, rest_s=6.0):
    """Expected Teensy behaviour for this trace. Returns a dict shaped like
    run.simulate()'s result (rows at the logger's 100 Hz, events with "t" in
    s) plus "go_ms": the replay clock at the first trace frame."""
    frames = unpack_frames(blob)
    pad = tuple(meta["pad_accel"])
    rows, events = [], []
    st = {"ms": 0, "tick": 0}

    fc = core.FlightCore(dll)
    try:
        fc.begin()
        fc.set_mode(core.FlightMode(meta["mode"]))

        def step(accel, gyro, baro_new, baro_alt):
            out = fc.step(st["ms"], FAST_DT, accel, gyro, baro_new=baro_new,
                          baro_alt=baro_alt)
            for ev in fc.pop_events():
                ev["t"] = ev["ms"] / 1000.0
                events.append(ev)
            if st["tick"] % 5 == 0:  # logger.h rows at LOG_FAST_HZ = 100 Hz
                rows.append({
                    "t": st["ms"] / 1000.0, "state": out["state"].name,
                    "ax": accel[0], "ay": accel[1], "az": accel[2],
                    "gx": gyro[0], "gy": gyro[1], "gz": gyro[2],
                    "baro_alt": baro_alt, "baro_pa": 0.0,
                    "quat_est": out["quat"], "tilt_deg": out["tilt_deg"],
                    "kf_alt": out["kf_alt"], "kf_vel": out["kf_vel"],
                    "kf_bias": out["kf_bias"], "innovation": out["innovation"],
                    "p_x": out["p_x"], "i_x": out["i_x"], "d_x": out["d_x"],
                    "p_y": out["p_y"], "i_y": out["i_y"], "d_y": out["d_y"],
                    "gimbal_x_cmd": out["gimbal_x"], "gimbal_y_cmd": out["gimbal_y"],
                    "us_a": out["us_a"], "us_b": out["us_b"],
                    "chute_release": out["chute_release"],
                    "chute_detected": out["chute_detected"],
                    "fire_landing": out["fire_landing"],
                    "legs_burn": out["legs_burn"],
                })
            st["ms"] += 2
            st["tick"] += 1

        def still(accel, seconds, baro_alt):
            for _ in range(int(round(seconds / FAST_DT))):
                step(accel, (0.0, 0.0, 0.0), st["tick"] % 5 == 0, baro_alt)

        baro0 = frames[0][6]
        still(pad, hold_idle_s, baro0)          # HOLD, disarmed
        fc.pad_level_init(pad)                  # `arm`: same calls, same order
        fc.set_baro_noise_var(meta["baro_var"])  # as RocketFC.ino's ARM_COLLECT
        fc.zero_altitude()
        fc.request_arm(st["ms"])
        still(pad, hold_armed_s, baro0)         # HOLD, armed, waiting for `go`
        go_ms = st["ms"]
        baro_alt = baro0
        for f in frames:                        # PLAY
            if f[7] & 1:
                baro_alt = f[6]
            step(f[0:3], f[3:6], bool(f[7] & 1), baro_alt)
        last = frames[-1][0:3]
        still(last, rest_s, baro_alt)           # DONE
    finally:
        fc.close()
    return {"rows": rows, "events": events, "go_ms": go_ms}


def event_list(events, zero_ms):
    """[(name, t_rel_s)] with RocketFC.ino's logged event names."""
    out = []
    for e in events:
        code = e["code"]
        name = core.log_event_name(code) if hasattr(code, "name") else str(code)
        ms = e["ms"] if "ms" in e else e["t"] * 1000.0
        out.append((name, (ms - zero_ms) / 1000.0))
    return out
