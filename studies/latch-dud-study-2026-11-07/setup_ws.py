"""Build the latch/dud study workspace (scratch copy; RocketFC itself is never touched).

  ws/ld_base  = ws/c_AeroTech_E18W__AeroTech_E18W from motor-chute-study-2026-11-04
                (1.4 kg E18W/E18W, casing eject, sized-canopy fill model, dud
                switch, legs on at ignition confirm, 0.75 s band cut) plus:

  firmware knobs (config.h, all OFF by default = today's code):
    CHUTE_FAST_CYCLES       blind re-cycles right after the first release (0 = off)
    CHUTE_FAST_FIRST_MS     hold at RELEASE after the first release before the first one
    CHUTE_FAST_OPEN_MS      hold at RELEASE between fast cycles
    CHUTE_FAST_LOCK_MS      time commanded toward LOCK per fast cycle
                            (short = a partial wiggle: the servo turns back before LOCK)
    IGNITE_CONFIRM_MS       accel > IGNITE_CONFIRM_G must hold this long (0 = one tick)
    IGNITE_CONFIRM_MIN_ALT_M  ... and KF altitude above this (-1000 = off)
  plant knobs (vehicle.py / run.py, all OFF by default):
    chute_free_delay_s      Step A: the canopy is freed this long after the first release
    latch_rate_dps          chute servo travel (0 = the old instant count-based latch).
                            The first stroke frees the canopy when it passes
                            latch_release_deg; with chute_stuck_releases = N it
                            doesn't, and a later stroke counts as a new attempt when
                            it gets back to OPEN after swinging back at least
                            latch_backswing_deg (30 = any shake frees a sticky pin,
                            135 = it needs a full cycle back to LOCK).

usage: python setup_ws.py
"""
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "motor-chute-study-2026-11-04"))
import mclib as L  # noqa: E402

SRC_WS = os.path.join(L.WS, "c_AeroTech_E18W__AeroTech_E18W")
WS = os.path.join(L.WS, "ld_base2")
IGNORE = shutil.ignore_patterns("build", "*.dll", "__pycache__", "*.exe", "*.json")


def sub(path, old, new):
    t = open(path, encoding="utf-8").read().replace("\r\n", "\n")
    assert t.count(old) == 1, (path, old[:80], t.count(old))
    open(path, "w", encoding="utf-8", newline="\n").write(t.replace(old, new))


def patch_config(ws):
    cfg = os.path.join(ws, "src", "config.h")
    sub(cfg, "constexpr int   CHUTE_MAX_RELEASES    = 4;",
        """constexpr int   CHUTE_MAX_RELEASES    = 4;
// latch-dud-study-2026-11-07: fast blind re-cycles right after the first
// release (0 = off). The first fast cycle waits CHUTE_FAST_FIRST_MS (servo
// travel LOCK -> OPEN), later ones CHUTE_FAST_OPEN_MS; each then commands
// LOCK for CHUTE_FAST_LOCK_MS (short = partial wiggle) and releases again.
constexpr int   CHUTE_FAST_CYCLES     = 0;
constexpr float CHUTE_FAST_FIRST_MS   = 400.0f;
constexpr float CHUTE_FAST_OPEN_MS    = 50.0f;
constexpr float CHUTE_FAST_LOCK_MS    = 400.0f;
// Angle-target wiggles (0 = off): after the first release the latch is
// commanded to WIG_HIGH (dwell FIRST), then WIG_LOW (dwell DOWN), WIG_HIGH
// (dwell UP) ... CHUTE_WIG_CYCLES down-strokes, the last up-stroke going to
// OPEN; then today's canopy check. Dwells cover the servo travel (no feedback).
constexpr int   CHUTE_WIG_CYCLES      = 0;
constexpr float CHUTE_WIG_LOW_DEG     = 70.0f;
constexpr float CHUTE_WIG_HIGH_DEG    = 150.0f;
constexpr float CHUTE_WIG_FIRST_MS    = 300.0f;
constexpr float CHUTE_WIG_DOWN_MS     = 220.0f;
constexpr float CHUTE_WIG_UP_MS       = 220.0f;""")
    sub(cfg, "constexpr float IGNITE_CONFIRM_G    = 1.3f;",
        """constexpr float IGNITE_CONFIRM_G    = 1.3f;
// latch-dud-study-2026-11-07: the accel must hold above IGNITE_CONFIRM_G this
// long (0 = one tick, today) and/or the KF altitude be above this (-1000 = off).
constexpr float IGNITE_CONFIRM_MS   = 0.0f;
constexpr float IGNITE_CONFIRM_MIN_ALT_M = -1000.0f;""")


def patch_chute_deploy(ws):
    p = os.path.join(ws, "src", "core", "chute_deploy.h")
    sub(p, """      case Phase::HOLD:
        break;
    }
    return t;""", """      case Phase::FAST_OPEN:  // study: blind fast re-cycle, at RELEASE
        if (since(ms) >= (releases_ == 1 ? cfg::CHUTE_FAST_FIRST_MS
                                         : cfg::CHUTE_FAST_OPEN_MS)) {
          servoRelease_ = false;
          phase_ = Phase::FAST_LOCK;
          phaseMs_ = ms;
        }
        break;
      case Phase::FAST_LOCK:  // ...toward LOCK, then release again
        if (since(ms) >= cfg::CHUTE_FAST_LOCK_MS) {
          --fastLeft_;
          release(ms);
          t.released = true;
        }
        break;
      case Phase::WIG_HIGH:  // study: angle-target wiggle, at WIG_HIGH
        if (since(ms) >= (releases_ == 1 ? cfg::CHUTE_WIG_FIRST_MS : cfg::CHUTE_WIG_UP_MS)) {
          servoRelease_ = false;
          wig_ = 2;  // -> WIG_LOW
          phase_ = Phase::WIG_LOW;
          phaseMs_ = ms;
        }
        break;
      case Phase::WIG_LOW:  // ...back up (to WIG_HIGH, or OPEN after the last)
        if (since(ms) >= cfg::CHUTE_WIG_DOWN_MS) {
          --wigLeft_;
          release(ms);
          t.released = true;
        }
        break;
      case Phase::HOLD:
        break;
    }
    return t;""")
    sub(p, "          if (releases_ < cfg::CHUTE_MAX_RELEASES) {",
        "          if (releases_ < cfg::CHUTE_MAX_RELEASES + cfg::CHUTE_FAST_CYCLES +\n"
        "                              cfg::CHUTE_WIG_CYCLES) {")
    sub(p, """    started_ = true;
    release(ms);""", """    started_ = true;
    fastLeft_ = cfg::CHUTE_FAST_CYCLES;
    wigLeft_ = cfg::CHUTE_WIG_CYCLES;
    release(ms);""")
    sub(p, """    baselineG_ = lpfG_;
    phase_ = Phase::WAIT;""", """    baselineG_ = lpfG_;
    wig_ = 0;
    phase_ = fastLeft_ > 0 ? Phase::FAST_OPEN : Phase::WAIT;
    if (wigLeft_ > 0) {
      wig_ = 3;  // WIG_HIGH
      phase_ = Phase::WIG_HIGH;
    }""")
    sub(p, """        servoRelease_ = true;  // mid re-cycle? go straight back to RELEASE
        phase_ = Phase::HOLD;""", """        servoRelease_ = true;  // mid re-cycle? go straight back to RELEASE
        wig_ = 0;
        phase_ = Phase::HOLD;""")
    sub(p, """    servoRelease_ = true;
    phase_ = Phase::HOLD;
  }""", """    servoRelease_ = true;
    wig_ = 0;
    phase_ = Phase::HOLD;
  }""")
    sub(p, "  bool servoRelease() const { return servoRelease_; }",
        "  bool servoRelease() const { return servoRelease_; }\n"
        "  // study: 0 LOCK, 1 OPEN, 2 WIG_LOW, 3 WIG_HIGH\n"
        "  int servoCode() const { return wig_ ? wig_ : (servoRelease_ ? 1 : 0); }")
    sub(p, "  enum class Phase : uint8_t { WAIT, LOCK_DWELL, HOLD };",
        "  enum class Phase : uint8_t { WAIT, LOCK_DWELL, HOLD, FAST_OPEN, FAST_LOCK,\n"
        "                               WIG_HIGH, WIG_LOW };")
    sub(p, "  int releases_ = 0;", "  int releases_ = 0;\n  int fastLeft_ = 0;\n"
        "  int wigLeft_ = 0;\n  int wig_ = 0;")


def patch_flight_state(ws):
    p = os.path.join(ws, "src", "core", "flight_state.h")
    sub(p, """      if (sinceEntry(in) > 50.0f && in.accelNormG > cfg::IGNITE_CONFIRM_G) {""",
        """      igniteAccMs_ = (sinceEntry(in) > 50.0f && in.accelNormG > cfg::IGNITE_CONFIRM_G &&
                      in.kfAlt > cfg::IGNITE_CONFIRM_MIN_ALT_M)
                         ? igniteAccMs_ + in.dtMs : 0.0f;
      if (igniteAccMs_ > 0.0f && igniteAccMs_ >= cfg::IGNITE_CONFIRM_MS) {""")
    sub(p, "      ignitionConfirmed_ = false;\n",
        "      ignitionConfirmed_ = false;\n      igniteAccMs_ = 0.0f;\n")
    sub(p, "  bool ignitionConfirmed_ = false, legsOn_ = false;",
        "  bool ignitionConfirmed_ = false, legsOn_ = false;\n  float igniteAccMs_ = 0.0f;")
    # the chute servo code (study wiggle targets) out through the core and the bridge
    sub(p, "  bool chuteRelease = false;   // LEVEL: chute latch servo should be at RELEASE now",
        "  bool chuteRelease = false;   // LEVEL: chute latch servo should be at RELEASE now\n"
        "  int chuteServo = 0;          // study: 0 LOCK, 1 OPEN, 2 WIG_LOW, 3 WIG_HIGH")
    sub(p, "    out.chuteRelease = chute_.servoRelease();",
        "    out.chuteRelease = chute_.servoRelease();\n    out.chuteServo = chute_.servoCode();")
    fc = os.path.join(ws, "src", "core", "flight_core.h")
    sub(fc, "  bool chuteRelease = false;   // LEVEL: chute latch servo at RELEASE now",
        "  bool chuteRelease = false;   // LEVEL: chute latch servo at RELEASE now\n"
        "  int chuteServo = 0;          // study: 0 LOCK, 1 OPEN, 2 WIG_LOW, 3 WIG_HIGH")
    sub(fc, "    out.chuteRelease = fo.chuteRelease;",
        "    out.chuteRelease = fo.chuteRelease;\n    out.chuteServo = fo.chuteServo;")
    br = os.path.join(ws, "tools", "sim", "bridge.cpp")
    sub(br, "  float usA, usB;\n};", "  float usA, usB;\n  int32_t chuteServo;  // study\n};")
    sub(br, "  out->chuteDetected = co.chuteDetected ? 1 : 0;",
        "  out->chuteDetected = co.chuteDetected ? 1 : 0;\n  out->chuteServo = co.chuteServo;")
    cp = os.path.join(ws, "tools", "sim", "core.py")
    sub(cp, '        ("usB", ctypes.c_float),\n',
        '        ("usB", ctypes.c_float),\n        ("chuteServo", ctypes.c_int32),\n')
    sub(cp, '            "us_a": o.usA, "us_b": o.usB,\n',
        '            "us_a": o.usA, "us_b": o.usB, "chute_servo": o.chuteServo,\n')


def patch_sim(ws):
    veh = os.path.join(ws, "tools", "sim", "vehicle.py")
    sub(veh, "    dud_landing: bool = False\n", """    dud_landing: bool = False
    # latch-dud-study-2026-11-07 (0 = off; see the study's setup_ws.py)
    chute_free_delay_s: float = 0.0
    latch_rate_dps: float = 0.0
    latch_lock_deg: float = 40.0
    latch_open_deg: float = 180.0
    latch_release_deg: float = 110.0
    latch_backswing_deg: float = 30.0
    latch_wig_low_deg: float = 70.0     # must match CHUTE_WIG_LOW_DEG / HIGH
    latch_wig_high_deg: float = 150.0
    latch_model: str = "shake"          # "shake" or "repass" (see the study's setup_ws.py)
    latch_repass_margin_deg: float = 5.0
""")
    run = os.path.join(ws, "tools", "sim", "run.py")
    sub(run, "    chute_dep = None\n", """    chute_dep = None
    free_at = None
    latch_rate = getattr(vcfg, "latch_rate_dps", 0.0)
    free_delay = getattr(vcfg, "chute_free_delay_s", 0.0)
    latch_deg = getattr(vcfg, "latch_lock_deg", 40.0)
    latch_attempts = 0
    latch_low = None
    latch_prev = latch_deg
    latch_rising = False
    latch_armed = False
    latch_free_t = []
    chute_dep_t0 = None
""")
    sub(run, """                    if chute_releases > vcfg.chute_stuck_releases:
                        pl.deploy_chute(t)
                        if chute_dep is None:""", """                    if chute_dep_t0 is None:
                        chute_dep_t0 = t
                    if free_delay > 0.0:
                        if free_at is None:
                            free_at = t + free_delay
                    elif latch_rate <= 0.0 and chute_releases > vcfg.chute_stuck_releases:
                        pl.deploy_chute(t)
                        if chute_dep is None:""")
    sub(run, "            # ---- servo sign round-trip (see module docstring) ----\n",
        """            # ---- latch-dud-study: delayed freeing / latch servo travel ----
            _free = False
            if free_at is not None and t >= free_at:
                _free = True
                free_at = None
            if latch_rate > 0.0 and chute_releases > 0:
                _code = out.get("chute_servo", 1 if out["chute_release"] else 0)
                _tgt = {0: vcfg.latch_lock_deg, 1: vcfg.latch_open_deg,
                        2: vcfg.latch_wig_low_deg, 3: vcfg.latch_wig_high_deg}.get(
                            _code, vcfg.latch_open_deg)
                _stp = latch_rate * dt
                latch_deg += max(-_stp, min(_stp, _tgt - latch_deg))
                _rising = latch_deg > latch_prev + 1e-9
                _try = False
                if latch_attempts == 0:
                    if latch_deg >= vcfg.latch_release_deg:
                        latch_attempts = 1
                        latch_low = None        # first stroke: no backswing yet
                        _try = True
                elif vcfg.latch_model == "repass":
                    # a retry = coming back up through the release point
                    if latch_deg < vcfg.latch_release_deg - vcfg.latch_repass_margin_deg:
                        latch_armed = True
                    elif latch_armed and latch_deg >= vcfg.latch_release_deg:
                        latch_armed = False
                        latch_attempts += 1
                        _try = True
                else:
                    # "shake": a retry at each stroke top after a backswing >= latch_backswing_deg
                    if latch_rising is True and not _rising:
                        if (latch_low is not None and
                                latch_deg - latch_low >= vcfg.latch_backswing_deg):
                            latch_attempts += 1
                            _try = True
                        latch_low = latch_deg
                    elif latch_low is not None:
                        latch_low = min(latch_low, latch_deg)
                latch_rising = _rising
                latch_prev = latch_deg
                if _try:
                    latch_free_t.append(round(t - chute_dep_t0, 3) if chute_dep_t0 is not None else None)
                if _try and latch_attempts > vcfg.chute_stuck_releases:
                    _free = True
            if _free and not pl.chute_deployed:
                pl.deploy_chute(t)
                if chute_dep is None:
                    chute_dep = {"t": t, "h": s.pos.z, "vz": s.vel.z,
                                 "speed": s.vel.norm()}

            # ---- servo sign round-trip (see module docstring) ----
""")
    sub(run, """        "chute_deploy_t": chute_dep["t"] if chute_dep else None,
""", """        "chute_deploy_t": chute_dep["t"] if chute_dep else None,
        "latch_attempts": latch_attempts,
        "latch_try_t": latch_free_t,
""")


def build():
    if os.path.isdir(WS):
        shutil.rmtree(WS)
    shutil.copytree(SRC_WS, WS, ignore=IGNORE)
    patch_config(WS)
    patch_chute_deploy(WS)
    patch_flight_state(WS)
    patch_sim(WS)
    print("built", WS)


if __name__ == "__main__":
    build()
