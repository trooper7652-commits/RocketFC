"""Add "artificial throttling" (ATH) to the candidate firmware copy: during the
landing burn, lean the rocket off vertical when it has thrust to spare (it is
falling slower than the braking curve says it may), circle the lean so the
sideways push cancels, and straighten up for touchdown. OFF by default
(THR_TILT_MAX_DEG 0), so existing results must stay bit-identical."""
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
    "constexpr float GPS_VEL_GAIN = 0.0f;\n",
    "constexpr float GPS_VEL_GAIN = 0.0f;\n"
    "// Artificial throttling (ATH): during the landing burn, lean up to this far\n"
    "// off vertical (losing 1 - cos(lean) of the upward thrust) whenever the rocket\n"
    "// falls slower than the braking curve v_des = sqrt(2 * THR_RESERVE * a_full * h)\n"
    "// -- i.e. it has thrust to spare and would stop high. The lean direction circles\n"
    "// once per THR_CONE_PERIOD_S so the sideways push cancels, and the allowed lean\n"
    "// fades to 0 between THR_STRAIGHT_H + THR_RAMP_H and THR_STRAIGHT_H (m).\n"
    "constexpr float THR_TILT_MAX_DEG = 0.0f;     // 0 = off\n"
    "constexpr float THR_K = 6.0f;                // deg of lean per m/s of surplus\n"
    "constexpr float THR_RESERVE = 0.85f;         // fraction of upright braking the curve assumes\n"
    "constexpr float THR_CONE_PERIOD_S = 1.5f;\n"
    "constexpr float THR_STRAIGHT_H = 2.0f;\n"
    "constexpr float THR_RAMP_H = 3.0f;\n")])

patch("src/core/flight_core.h", [(
    """        if (cfg::DRIFT_GAIN > 0.0f && aT > 3.0f &&
            (in.ms - burnMs_) * 0.001f >= cfg::DRIFT_START_S) {
          // Want horizontal accel = -K v: lean the thrust axis by -K v / aT.
          float hx = -cfg::DRIFT_GAIN * velHx_ / aT;
          float hy = -cfg::DRIFT_GAIN * velHy_ / aT;
          const float hn = std::sqrt(hx * hx + hy * hy);
          const float cap = std::tan(cfg::DRIFT_MAX_TILT_DEG * cfg::DEG2RAD);
          if (hn > cap) { hx *= cap / hn; hy *= cap / hn; }
          const float n = std::sqrt(hx * hx + hy * hy + 1.0f);
          e = ahrs_.tiltErrorBodyTo(Vec3{hx / n, hy / n, 1.0f / n});
        }""",
    """        float hx = 0.0f, hy = 0.0f;
        bool lean = false;
        if (cfg::DRIFT_GAIN > 0.0f && aT > 3.0f &&
            (in.ms - burnMs_) * 0.001f >= cfg::DRIFT_START_S) {
          // Want horizontal accel = -K v: lean the thrust axis by -K v / aT.
          hx = -cfg::DRIFT_GAIN * velHx_ / aT;
          hy = -cfg::DRIFT_GAIN * velHy_ / aT;
          const float hn = std::sqrt(hx * hx + hy * hy);
          const float cap = std::tan(cfg::DRIFT_MAX_TILT_DEG * cfg::DEG2RAD);
          if (hn > cap) { hx *= cap / hn; hy *= cap / hn; }
          lean = true;
        }
        if (cfg::THR_TILT_MAX_DEG > 0.0f && aT > 3.0f) {
          // Artificial throttling: shed spare thrust by leaning, coning.
          const float aFull = aT - cfg::G0;          // upright braking available
          const float h = kf_.altitude();
          const float vDown = -kf_.velocity();
          float thMax = cfg::THR_TILT_MAX_DEG;
          const float ramp = (h - cfg::THR_STRAIGHT_H) / cfg::THR_RAMP_H;
          if (ramp < 1.0f) thMax *= (ramp > 0.0f ? ramp : 0.0f);
          float thDeg = 0.0f;
          if (aFull > 0.5f && h > 0.0f) {
            const float vDes = std::sqrt(2.0f * cfg::THR_RESERVE * aFull * h);
            thDeg = cfg::THR_K * (vDes - vDown);
            if (thDeg < 0.0f) thDeg = 0.0f;
            if (thDeg > thMax) thDeg = thMax;
          }
          thrDeg_ = thDeg;
          if (thDeg > 0.0f) {
            const float psi = 2.0f * cfg::PI_F * (in.ms - burnMs_) * 0.001f /
                              cfg::THR_CONE_PERIOD_S;
            const float t = std::tan(thDeg * cfg::DEG2RAD);
            hx += t * std::cos(psi);
            hy += t * std::sin(psi);
            lean = true;
          }
        }
        if (lean) {
          const float n = std::sqrt(hx * hx + hy * hy + 1.0f);
          e = ahrs_.tiltErrorBodyTo(Vec3{hx / n, hy / n, 1.0f / n});
        }"""),
    ("""  float velHx_ = 0.0f, velHy_ = 0.0f;
};""", """  float velHx_ = 0.0f, velHy_ = 0.0f;
  float thrDeg_ = 0.0f;   // ATH lean commanded this tick (deg), for logs

 public:
  float throttleLeanDeg() const { return thrDeg_; }
};"""),
])

# Expose the commanded lean to the sim for traces (read-only accessor).
patch("tools/sim/bridge.cpp", [(
    "void rfc_gps_vel(void* h, float vx, float vy) {\n",
    "float rfc_thr_lean_deg(void* h) {\n"
    "  return static_cast<FlightCore*>(h)->throttleLeanDeg();\n"
    "}\n\n"
    "void rfc_gps_vel(void* h, float vx, float vy) {\n")])
patch("tools/sim/core.py", [
    ("    lib.rfc_gps_vel.restype = None\n",
     "    lib.rfc_gps_vel.restype = None\n"
     "    lib.rfc_thr_lean_deg.argtypes = [ctypes.c_void_p]\n"
     "    lib.rfc_thr_lean_deg.restype = ctypes.c_float\n"),
    ("    def gps_vel(self, vx, vy):\n",
     "    def thr_lean_deg(self):\n"
     "        return self._lib.rfc_thr_lean_deg(self._h)\n\n"
     "    def gps_vel(self, vx, vy):\n"),
])
print("ATH patched", root)
patch("tools/sim/run.py", [(
    '                    "thrust_n": thrust_now,\n',
    '                    "thrust_n": thrust_now,\n'
    '                    "thr_lean_deg": fc.thr_lean_deg(),\n')])
print("run.py rows record the ATH lean")
