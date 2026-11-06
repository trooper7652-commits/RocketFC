"""Add a GPS velocity aid for the S3 drift loop to the candidate workspace.
Everything is OFF by default (GPS_VEL_GAIN 0 in firmware, gps_hz 0 in the
sim), so existing results must stay bit-identical."""
import sys

root = sys.argv[1]


def patch(rel, pairs):
    p = root + "/" + rel
    s = open(p, encoding="utf-8").read()
    for a, b in pairs:
        assert s.count(a) == 1, (rel, a[:70])
        s = s.replace(a, b)
    open(p, "w", encoding="utf-8").write(s)


patch("src/config.h", [(
    "constexpr float DRIFT_START_S = 0.0f;        // S3 option: only lean after this long into the burn\n",
    "constexpr float DRIFT_START_S = 0.0f;        // S3 option: only lean after this long into the burn\n"
    "// GPS aid for the S3 sideways-velocity estimate: each GPS velocity sample pulls\n"
    "// the dead-reckoned estimate this fraction of the way toward it. 0 = off.\n"
    "constexpr float GPS_VEL_GAIN = 0.0f;\n")])

patch("src/core/flight_core.h", [(
    "  const Ahrs& ahrs() const { return ahrs_; }\n",
    "  const Ahrs& ahrs() const { return ahrs_; }\n\n"
    "  // GPS ground velocity (m/s, world x/y): complementary correction of the\n"
    "  // dead-reckoned sideways velocity used by the drift loop.\n"
    "  void gpsVelocity(float vx, float vy) {\n"
    "    if (cfg::GPS_VEL_GAIN <= 0.0f) return;\n"
    "    velHx_ += cfg::GPS_VEL_GAIN * (vx - velHx_);\n"
    "    velHy_ += cfg::GPS_VEL_GAIN * (vy - velHy_);\n"
    "  }\n")])

patch("tools/sim/bridge.cpp", [(
    "void rfc_zero_altitude(void* h) { static_cast<FlightCore*>(h)->zeroAltitude(); }\n",
    "void rfc_zero_altitude(void* h) { static_cast<FlightCore*>(h)->zeroAltitude(); }\n\n"
    "void rfc_gps_vel(void* h, float vx, float vy) {\n"
    "  static_cast<FlightCore*>(h)->gpsVelocity(vx, vy);\n"
    "}\n")])

patch("tools/sim/core.py", [
    ("    lib.rfc_zero_altitude.argtypes = [ctypes.c_void_p]\n",
     "    lib.rfc_zero_altitude.argtypes = [ctypes.c_void_p]\n"
     "    lib.rfc_gps_vel.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]\n"
     "    lib.rfc_gps_vel.restype = None\n"),
    ("    def state(self):\n        return FlightState(self._lib.rfc_state(self._h))\n",
     "    def state(self):\n        return FlightState(self._lib.rfc_state(self._h))\n\n"
     "    def gps_vel(self, vx, vy):\n"
     "        self._lib.rfc_gps_vel(self._h, vx, vy)\n"),
])

patch("tools/sim/vehicle.py", [(
    "    settle_s: float = 2.0              # must stand still this long after burnout\n",
    "    settle_s: float = 2.0              # must stand still this long after burnout\n\n"
    "    # ---- Simulated GPS velocity (study; 0 Hz = no GPS) ----\n"
    "    gps_hz: float = 0.0                # u-blox M9/M10-class: 10 Hz\n"
    "    gps_vel_sigma: float = 0.1         # [ESTIMATE] horizontal velocity noise (m/s)\n"
    "    gps_latency_s: float = 0.1         # [ESTIMATE] measurement age when it arrives (s)\n")])

patch("tools/sim/run.py", [
    ("    t = 0.0\n    i = 0\n    try:\n",
     "    # Simulated GPS: its own RNG stream, so nothing else changes when it's on.\n"
     "    gps_rng = random.Random(seed * 104729 + 11)\n"
     "    gps_hist = []\n"
     "    gps_next = 0.0\n\n"
     "    t = 0.0\n    i = 0\n    try:\n"),
    ("            pl.legs_frac = legs_frac\n            pl.step(dt, act_x, act_y)\n",
     "            pl.legs_frac = legs_frac\n            pl.step(dt, act_x, act_y)\n"
     "            if vcfg.gps_hz > 0.0 and launched:\n"
     "                gps_hist.append((t, s.vel.x, s.vel.y))\n"
     "                if t >= gps_next:\n"
     "                    gps_next = t + 1.0 / vcfg.gps_hz\n"
     "                    want = t - vcfg.gps_latency_s\n"
     "                    while len(gps_hist) > 1 and gps_hist[1][0] <= want:\n"
     "                        gps_hist.pop(0)\n"
     "                    _, gvx, gvy = gps_hist[0]\n"
     "                    fc.gps_vel(gvx + gps_rng.gauss(0.0, vcfg.gps_vel_sigma),\n"
     "                               gvy + gps_rng.gauss(0.0, vcfg.gps_vel_sigma))\n"),
])
print("gps patched", root)
