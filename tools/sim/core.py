#!/usr/bin/env python3
"""ctypes wrapper + build manager for RocketFC's C++ flight core.

Drives the EXACT code the Teensy flies (src/core/flight_core.h) from Python,
via the flat C ABI in bridge.cpp, compiled to rocketfc_core.dll. This is the
foundation the rest of tools/sim/ builds on: the plant, sensors and dashboard
never re-implement any flight-computer logic, they only call step().

Two ways to get a usable DLL:
  - `default_dll()`   -- builds (or reuses a cached build of) the bridge
                          against the repo's real src/ tree, unmodified.
  - `build_with_overrides({...})` -- for sweeping firmware constexpr values
    (PID gains, thresholds, KF sigmas, ...) without ever touching the real
    src/ tree: copies src/ into build/<hash>/src/, regex-patches the requested
    constants in the copy, compiles a private DLL from there, and caches it by
    a hash of the override set. ~2 s the first time a given override set is
    seen, then instant.

Pure stdlib (ctypes, hashlib, re, shutil, subprocess) + no third-party deps.
"""
import ctypes
import enum
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(SIM_DIR)
REPO_ROOT = os.path.dirname(TOOLS_DIR)
SRC_DIR = os.path.join(REPO_ROOT, "src")
BUILD_DIR = os.path.join(SIM_DIR, "build")

GXX_CANDIDATES = [
    r"C:\msys64\ucrt64\bin\g++.exe",
    "g++",
]


def _find_gxx():
    for c in GXX_CANDIDATES:
        if os.path.isabs(c):
            if os.path.isfile(c):
                return c
        else:
            found = shutil.which(c)
            if found:
                return found
    raise RuntimeError(
        "no g++ found; tried: " + ", ".join(GXX_CANDIDATES) +
        ". Install MSYS2 ucrt64 (matches tools/replay's toolchain) or put "
        "g++ on PATH.")


# ---------------------------------------------------------------------------
# Enums mirroring src/core/flight_state.h. Keep in sync by hand -- there are
# only three of these and they change rarely; a mismatch here is caught
# immediately by test_bridge.py comparing state NAMES against tools/replay.
# ---------------------------------------------------------------------------
class FlightState(enum.IntEnum):
    IDLE = 0
    ARMED = 1
    BOOST = 2
    COAST = 3
    APOGEE = 4
    DESCENT = 5
    LANDING_BURN = 6
    DESCENT_CHUTE = 7
    TOUCHDOWN = 8
    ABORT = 9


class AbortReason(enum.IntEnum):
    NONE = 0
    TILT = 1
    IMU_FAIL = 2
    KF_UNHEALTHY = 3
    MISSED_WINDOW = 4
    DUD_IGNITER = 5


class FlightEvent(enum.IntEnum):
    NONE = 0
    ARM = 1
    DISARM = 2
    LAUNCH = 3
    BURNOUT = 4
    APOGEE_DET = 5
    FIRE_CHUTE = 6
    FIRE_LANDING = 7
    IGNITION_CONFIRMED = 8
    LANDING_BURNOUT = 9
    TOUCHDOWN_DET = 10
    ABORT_DET = 11


class FlightMode(enum.IntEnum):
    CHUTE_TEST = 0
    FULL_LANDING = 1


# FlightEvent.name doesn't match the on-disk log string for three events --
# src/core/flight_state.h's eventName() (what RocketFC.ino actually logs)
# shortens APOGEE_DET/ABORT_DET/TOUCHDOWN_DET to APOGEE/ABORT/TOUCHDOWN. Every
# other event's log string matches its Python enum name exactly. Used by
# logfile.write_log() (must match real hardware's on-disk format) and
# reflight.py (comparing a re-run's events against a real log's).
LOG_EVENT_NAME = {
    FlightEvent.APOGEE_DET: "APOGEE",
    FlightEvent.ABORT_DET: "ABORT",
    FlightEvent.TOUCHDOWN_DET: "TOUCHDOWN",
}


def log_event_name(code):
    """FlightEvent -> the exact string RocketFC.ino logs for it."""
    return LOG_EVENT_NAME.get(code, code.name)


# ---------------------------------------------------------------------------
# ctypes structs -- field order/types MUST match bridge.cpp exactly.
# ---------------------------------------------------------------------------
class CInput(ctypes.Structure):
    _fields_ = [
        ("ms", ctypes.c_uint32),
        ("dt", ctypes.c_float),
        ("accel", ctypes.c_float * 3),
        ("gyro", ctypes.c_float * 3),
        ("baroNew", ctypes.c_int32),
        ("baroAlt", ctypes.c_float),
        ("imuHealthy", ctypes.c_int32),
        ("contChute", ctypes.c_int32),
        ("contLanding", ctypes.c_int32),
    ]


class COutput(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_int32),
        ("abortReason", ctypes.c_int32),
        ("tvcActive", ctypes.c_int32),
        ("gimbalX", ctypes.c_float),
        ("gimbalY", ctypes.c_float),
        ("fireChute", ctypes.c_int32),
        ("fireLanding", ctypes.c_int32),
        ("kfAlt", ctypes.c_float),
        ("kfVel", ctypes.c_float),
        ("kfBias", ctypes.c_float),
        ("innovation", ctypes.c_float),
        ("tiltDeg", ctypes.c_float),
        ("quat", ctypes.c_float * 4),
        ("inFlight", ctypes.c_int32),
        ("logFast", ctypes.c_int32),
        ("pTermX", ctypes.c_float),
        ("iTermX", ctypes.c_float),
        ("dTermX", ctypes.c_float),
        ("pTermY", ctypes.c_float),
        ("iTermY", ctypes.c_float),
        ("dTermY", ctypes.c_float),
        ("usA", ctypes.c_float),
        ("usB", ctypes.c_float),
    ]


class CEvent(ctypes.Structure):
    _fields_ = [
        ("ms", ctypes.c_uint32),
        ("code", ctypes.c_int32),
        ("value", ctypes.c_float),
    ]


class CConfigSnapshot(ctypes.Structure):
    _fields_ = [
        ("imuRSb", ctypes.c_float * 9),
        ("massPadKg", ctypes.c_float),
        ("massDescentKg", ctypes.c_float),
        ("cdaM2", ctypes.c_float),
        ("ascentBurnMs", ctypes.c_float),
        ("ignitionDelayMs", ctypes.c_float),
        ("gainsBoostKp", ctypes.c_float),
        ("gainsBoostKi", ctypes.c_float),
        ("gainsBoostKd", ctypes.c_float),
        ("gainsBoostIMax", ctypes.c_float),
        ("gainsLandKp", ctypes.c_float),
        ("gainsLandKi", ctypes.c_float),
        ("gainsLandKd", ctypes.c_float),
        ("gainsLandIMax", ctypes.c_float),
        ("gimbalMaxRad", ctypes.c_float),
        ("gimbalSlewRadps", ctypes.c_float),
        ("dLpfHz", ctypes.c_float),
        ("servoACenterUs", ctypes.c_float),
        ("servoBCenterUs", ctypes.c_float),
        ("servoAUsPerDeg", ctypes.c_float),
        ("servoBUsPerDeg", ctypes.c_float),
        ("servoASign", ctypes.c_float),
        ("servoBSign", ctypes.c_float),
        ("servoMinUs", ctypes.c_float),
        ("servoMaxUs", ctypes.c_float),
        ("launchAccelG", ctypes.c_float),
        ("launchMs", ctypes.c_float),
        ("burnoutAccelG", ctypes.c_float),
        ("burnoutMs", ctypes.c_float),
        ("boostMaxMs", ctypes.c_float),
        ("apogeeVelMs", ctypes.c_float),
        ("apogeeDropM", ctypes.c_float),
        ("coastMaxMs", ctypes.c_float),
        ("tiltAbortDeg", ctypes.c_float),
        ("tiltAbortMs", ctypes.c_float),
        ("tiltIgniteMaxDeg", ctypes.c_float),
        ("burnTableMarginM", ctypes.c_float),
        ("burnMinSpeedMs", ctypes.c_float),
        ("missedWindowAltM", ctypes.c_float),
        ("igniteConfirmG", ctypes.c_float),
        ("dudWindowMs", ctypes.c_float),
        ("landingBurnMaxMs", ctypes.c_float),
        ("touchdownAltM", ctypes.c_float),
        ("touchdownVelMs", ctypes.c_float),
        ("touchdownMs", ctypes.c_float),
        ("prearmTiltMaxDeg", ctypes.c_float),
        ("abortFiresChuteAlways", ctypes.c_int32),
        ("kfUnhealthyAbortMs", ctypes.c_float),
        ("kfSigmaAccel", ctypes.c_float),
        ("kfSigmaBiasRw", ctypes.c_float),
        ("kfGateSigma", ctypes.c_float),
        ("kfRBoostInflate", ctypes.c_float),
        ("kfGateForceAccept", ctypes.c_int32),
        ("baroRFallbackM2", ctypes.c_float),
        ("g0", ctypes.c_float),
        ("fastDt", ctypes.c_float),
    ]


def _bind(lib):
    lib.rfc_create.restype = ctypes.c_void_p
    lib.rfc_create.argtypes = []
    lib.rfc_destroy.argtypes = [ctypes.c_void_p]
    lib.rfc_begin.argtypes = [ctypes.c_void_p]
    lib.rfc_set_mode.argtypes = [ctypes.c_void_p, ctypes.c_int32]
    lib.rfc_pad_level_init.argtypes = [ctypes.c_void_p, ctypes.c_float,
                                       ctypes.c_float, ctypes.c_float]
    lib.rfc_set_baro_noise_var.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.rfc_zero_altitude.argtypes = [ctypes.c_void_p]
    lib.rfc_request_arm.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.rfc_request_arm.restype = ctypes.c_int32
    lib.rfc_request_disarm.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.rfc_request_disarm.restype = ctypes.c_int32
    lib.rfc_state.argtypes = [ctypes.c_void_p]
    lib.rfc_state.restype = ctypes.c_int32
    lib.rfc_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(CInput),
                             ctypes.POINTER(COutput)]
    lib.rfc_pop_event.argtypes = [ctypes.c_void_p, ctypes.POINTER(CEvent)]
    lib.rfc_pop_event.restype = ctypes.c_int32
    lib.rfc_config_snapshot.argtypes = [ctypes.POINTER(CConfigSnapshot)]
    return lib


class FlightCore:
    """One flight computer instance, backed by a loaded DLL.

    Usage:
        core = FlightCore(core.default_dll())
        core.begin()
        core.set_mode(FlightMode.FULL_LANDING)
        ...
        out = core.step(ms, dt, accel=(ax,ay,az), gyro=(gx,gy,gz), ...)
        for ev in core.pop_events():
            ...
        core.close()

    Or as a context manager: `with FlightCore(dll_path) as core: ...`
    """

    def __init__(self, dll_path):
        self.dll_path = dll_path
        self._lib = _bind(ctypes.CDLL(dll_path))
        self._h = self._lib.rfc_create()
        self._in = CInput()
        self._out = COutput()

    def close(self):
        if self._h:
            self._lib.rfc_destroy(self._h)
            self._h = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def begin(self):
        self._lib.rfc_begin(self._h)

    def set_mode(self, mode):
        self._lib.rfc_set_mode(self._h, int(mode))

    def pad_level_init(self, accel_avg):
        ax, ay, az = accel_avg
        self._lib.rfc_pad_level_init(self._h, ax, ay, az)

    def set_baro_noise_var(self, var):
        self._lib.rfc_set_baro_noise_var(self._h, var)

    def zero_altitude(self):
        self._lib.rfc_zero_altitude(self._h)

    def request_arm(self, ms):
        return bool(self._lib.rfc_request_arm(self._h, ms))

    def request_disarm(self, ms):
        return bool(self._lib.rfc_request_disarm(self._h, ms))

    def state(self):
        return FlightState(self._lib.rfc_state(self._h))

    def step(self, ms, dt, accel, gyro, baro_new=False, baro_alt=0.0,
             imu_healthy=True, cont_chute=True, cont_landing=True):
        ci = self._in
        ci.ms = int(ms)
        ci.dt = dt
        ci.accel[0], ci.accel[1], ci.accel[2] = accel
        ci.gyro[0], ci.gyro[1], ci.gyro[2] = gyro
        ci.baroNew = 1 if baro_new else 0
        ci.baroAlt = baro_alt
        ci.imuHealthy = 1 if imu_healthy else 0
        ci.contChute = 1 if cont_chute else 0
        ci.contLanding = 1 if cont_landing else 0
        self._lib.rfc_step(self._h, ctypes.byref(ci), ctypes.byref(self._out))
        o = self._out
        return {
            "state": FlightState(o.state),
            "abort_reason": AbortReason(o.abortReason),
            "tvc_active": bool(o.tvcActive),
            "gimbal_x": o.gimbalX, "gimbal_y": o.gimbalY,
            "fire_chute": bool(o.fireChute), "fire_landing": bool(o.fireLanding),
            "kf_alt": o.kfAlt, "kf_vel": o.kfVel, "kf_bias": o.kfBias,
            "innovation": o.innovation, "tilt_deg": o.tiltDeg,
            "quat": (o.quat[0], o.quat[1], o.quat[2], o.quat[3]),
            "in_flight": bool(o.inFlight), "log_fast": bool(o.logFast),
            "p_x": o.pTermX, "i_x": o.iTermX, "d_x": o.dTermX,
            "p_y": o.pTermY, "i_y": o.iTermY, "d_y": o.dTermY,
            "us_a": o.usA, "us_b": o.usB,
        }

    def pop_events(self):
        ev = CEvent()
        while self._lib.rfc_pop_event(self._h, ctypes.byref(ev)):
            yield {"ms": ev.ms, "code": FlightEvent(ev.code), "value": ev.value}

    @staticmethod
    def config_snapshot(dll_path):
        lib = _bind(ctypes.CDLL(dll_path))
        snap = CConfigSnapshot()
        lib.rfc_config_snapshot(ctypes.byref(snap))
        return {name: (list(getattr(snap, name)) if hasattr(getattr(snap, name), "__len__")
                       else getattr(snap, name))
                for name, _ in CConfigSnapshot._fields_}


# ---------------------------------------------------------------------------
# Build management.
# ---------------------------------------------------------------------------
# -static (not just -static-libgcc/-libstdc++): the MSYS2 g++ toolchain also
# dynamically links libwinpthread-1.dll by default. That's fine when the DLL
# is loaded by a Python that itself lives next to libwinpthread-1.dll (e.g.
# MSYS2's own python3.exe, which finds it via ordinary same-directory DLL
# search) but ctypes.CDLL from any OTHER Python install -- including a
# normal python.org/Microsoft Store Python, likely what actually runs this
# GUI -- fails with a misleading "Could not find module ... (or one of its
# dependencies)" since Python 3.8+ no longer searches PATH/CWD for a loaded
# DLL's own dependencies. Fully static linking removes every non-Windows-API
# dependency (verify with `objdump -p rocketfc_core.dll | grep "DLL Name"` --
# only KERNEL32 and api-ms-win-crt-* should remain), so the DLL loads
# identically no matter which Python calls ctypes.CDLL on it.
_CXXFLAGS = ["-O2", "-std=c++17", "-Wall", "-Wextra", "-shared", "-static"]


def _compile(work_dir, bridge_rel, out_dll):
    """Invoke g++ with the same env workaround tools/replay/Makefile uses:
    MSYS2 make/g++ can fall back to C:\\WINDOWS for temp files and fail with
    "Permission denied" if TMP/TEMP are unset. Point them at work_dir."""
    gxx = _find_gxx()
    env = dict(os.environ)
    env["TMPDIR"] = work_dir
    env["TMP"] = work_dir
    env["TEMP"] = work_dir
    cmd = [gxx] + _CXXFLAGS + [bridge_rel, "-o", out_dll]
    proc = subprocess.run(cmd, cwd=work_dir, env=env,
                          capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            "g++ failed building {}:\n{}\n{}".format(out_dll, proc.stdout,
                                                      proc.stderr))
    return os.path.join(work_dir, out_dll)


def default_dll(force=False):
    """Build (or reuse) rocketfc_core.dll from the repo's own src/, unmodified.

    Rebuilds when missing or older than bridge.cpp / any src header, mirroring
    the Makefile's dependency list.
    """
    out = os.path.join(SIM_DIR, "rocketfc_core.dll")
    bridge = os.path.join(SIM_DIR, "bridge.cpp")
    deps = [bridge, os.path.join(SRC_DIR, "config.h")]
    for root, _, files in os.walk(os.path.join(SRC_DIR, "core")):
        for f in files:
            if f.endswith(".h"):
                deps.append(os.path.join(root, f))
    stale = force or not os.path.isfile(out) or \
        any(os.path.getmtime(d) > os.path.getmtime(out) for d in deps)
    if stale:
        _compile(SIM_DIR, "bridge.cpp", "rocketfc_core.dll")
    return out


# Constants that are simple `constexpr <type> NAME = value;` lines in
# config.h and safe to patch with a single-line regex substitution. This is
# most of the tunable thresholds, gains scalars and KF sigmas; PID gain
# STRUCTS (GAINS_BOOST / GAINS_LANDING) need the dedicated struct-literal
# patcher below since they're `constexpr PidGains X = { a, b, c, d };`.
def _patch_simple_constant(text, name, value):
    """Replace `constexpr <type> NAME = <old>;` with `... NAME = value;`.

    `value` must already be a valid C++ literal (e.g. "12.5f", "true", "3").
    Raises if NAME isn't found, so a typo'd override fails loudly instead of
    silently no-op'ing.
    """
    pattern = re.compile(
        r"(constexpr\s+[\w:]+\s+" + re.escape(name) + r"\s*=\s*)([^;]+)(;)")
    new_text, n = pattern.subn(lambda m: m.group(1) + value + m.group(3), text)
    if n == 0:
        raise ValueError("config constant not found for patching: {}".format(name))
    if n > 1:
        raise ValueError("ambiguous match ({} hits) patching: {}".format(n, name))
    return new_text


def _patch_pid_gains(text, name, kp=None, ki=None, kd=None, iMaxRad=None):
    """Replace one or more fields of `constexpr PidGains NAME = { kp, ki, kd,
    iMaxRad };` (field names match the C++ struct exactly). Fields left as
    None keep their current value."""
    pattern = re.compile(
        r"(constexpr\s+PidGains\s+" + re.escape(name) + r"\s*=\s*\{)"
        r"\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),\s*([^,}]+)\s*(\}\s*;)")
    m = pattern.search(text)
    if not m:
        raise ValueError("PidGains constant not found for patching: {}".format(name))
    cur = [m.group(2).strip(), m.group(3).strip(), m.group(4).strip(),
           m.group(5).strip()]
    override = [kp, ki, kd, iMaxRad]
    new_fields = [str(o) if o is not None else c for o, c in zip(override, cur)]
    replacement = "{}{}, {}, {}, {}{}".format(
        m.group(1), new_fields[0], new_fields[1], new_fields[2], new_fields[3],
        m.group(6))
    return text[:m.start()] + replacement + text[m.end():]


def _patch_float_array(text, name, values):
    """Replace `constexpr float NAME[N] = { ... };` wholesale with the given
    list of N C++ float literals (e.g. for IMU_R_SB's 9-element mount
    matrix)."""
    pattern = re.compile(
        r"(constexpr\s+float\s+" + re.escape(name) + r"\s*\[\s*\d+\s*\]\s*=\s*\{)"
        r".*?(\}\s*;)", re.S)
    m = pattern.search(text)
    if not m:
        raise ValueError("float array constant not found for patching: {}".format(name))
    replacement = "{}\n  {},\n  {},\n  {},\n{}".format(
        m.group(1), ", ".join(str(v) for v in values[0:3]),
        ", ".join(str(v) for v in values[3:6]),
        ", ".join(str(v) for v in values[6:9]), m.group(2))
    return text[:m.start()] + replacement + text[m.end():]


def _config_hash(overrides):
    blob = json.dumps(overrides, sort_keys=True).encode("utf-8")
    return hashlib.sha1(blob).hexdigest()[:16]


def build_with_overrides(overrides):
    """Build a DLL from a patched copy of config.h, cached by override hash.

    `overrides`: dict of simple-constant NAME -> C++ literal string, e.g.
        {"TILT_ABORT_DEG": "20.0f", "KF_SIGMA_ACCEL": "0.8f"}
    Plus two optional special keys for the PID gain structs, each a dict of
    field -> literal:
        {"GAINS_BOOST": {"kp": "0.5f", "kd": "0.1f"},
         "GAINS_LANDING": {"ki": "0.25f"}}
    And one for the 9-element IMU mount matrix, a flat list of 9 literals
    (row-major, replacing the array wholesale -- there's no meaningful
    "leave this entry unchanged" for a mount matrix):
        {"IMU_R_SB": ["0", "0", "-1", "0", "1", "0", "-1", "0", "0"]}

    Returns the path to the cached DLL. Never modifies the real src/ tree:
    everything happens in build/<hash>/, mirroring the repo layout (src/ and
    tools/sim/bridge.cpp at the same relative depth) so bridge.cpp's
    "../../src/..." include resolves into the COPY.
    """
    if not overrides:
        return default_dll()

    h = _config_hash(overrides)
    cache_dir = os.path.join(BUILD_DIR, h)
    work_dir = os.path.join(cache_dir, "tools", "sim")
    out = os.path.join(work_dir, "rocketfc_core.dll")
    if os.path.isfile(out):
        return out

    # Mirror the repo layout under cache_dir: cache_dir/src, cache_dir/tools/sim.
    dst_src = os.path.join(cache_dir, "src")
    if os.path.isdir(dst_src):
        shutil.rmtree(dst_src)
    shutil.copytree(SRC_DIR, dst_src)
    os.makedirs(work_dir, exist_ok=True)
    shutil.copy2(os.path.join(SIM_DIR, "bridge.cpp"),
                os.path.join(work_dir, "bridge.cpp"))

    config_path = os.path.join(dst_src, "config.h")
    with open(config_path, "r", encoding="utf-8") as f:
        text = f.read()

    for name, val in overrides.items():
        if name in ("GAINS_BOOST", "GAINS_LANDING"):
            text = _patch_pid_gains(text, name, **val)
        elif name == "IMU_R_SB":
            text = _patch_float_array(text, name, val)
        else:
            text = _patch_simple_constant(text, name, val)

    with open(config_path, "w", encoding="utf-8") as f:
        f.write(text)

    with open(os.path.join(cache_dir, "overrides.json"), "w") as f:
        json.dump(overrides, f, indent=2, sort_keys=True)

    return _compile(work_dir, "bridge.cpp", "rocketfc_core.dll")


if __name__ == "__main__":
    dll = default_dll()
    print("built:", dll)
    snap = FlightCore.config_snapshot(dll)
    print("TILT_ABORT_DEG =", snap["tiltAbortDeg"])
    print("GAINS_BOOST kp/ki/kd =", snap["gainsBoostKp"], snap["gainsBoostKi"],
          snap["gainsBoostKd"])
    print("IMU_R_SB =", snap["imuRSb"])
    sys.exit(0)


def mount_matrix_misalignment_deg(dll_path, reference_dll_path=None):
    """Algebraic check for cfg::IMU_R_SB: how far the DLL-under-test's mount
    matrix deviates from a reference (default: the repo's own default_dll(),
    i.e. the currently bench-measured value).

    This is NOT a full closed-loop simulation of a wrong sensor mounting --
    run.py deliberately generates sensor truth directly in the flight-core's
    body frame (see run.py's module docstring) rather than modeling a
    separate physical sensor frame and threading IMU_R_SB through it, the
    same scope decision tools/synth_flight.py and tools/replay already made
    (R_SB correctness is validated by the real-hardware
    SensorServoBenchTest rig, not by software simulation). What this DOES
    verify: that a swapped-rows / wrong-sign edit to IMU_R_SB is not
    accidentally still a valid rotation equal to the reference -- i.e. that
    such an edit would actually change flight behavior, by how much.

    Returns the rotation angle (degrees) of R_active @ R_reference^T relative
    to identity: 0 means the two matrices describe the same mounting.
    """
    ref_dll = reference_dll_path or default_dll()
    r_ref = FlightCore.config_snapshot(ref_dll)["imuRSb"]
    r_act = FlightCore.config_snapshot(dll_path)["imuRSb"]

    def mat_mult_transpose(a, b):
        """a (3x3 row-major) @ b^T (b given row-major; transposed here)."""
        out = [0.0] * 9
        for i in range(3):
            for j in range(3):
                out[i * 3 + j] = sum(a[i * 3 + k] * b[j * 3 + k] for k in range(3))
        return out

    m = mat_mult_transpose(r_act, r_ref)
    trace = m[0] + m[4] + m[8]
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cos_theta))
