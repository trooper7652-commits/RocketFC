"""Apply the winning fixes to a RocketFC tree (exact-match edits, keeps line endings).

    python port_fixes.py <rocketfc_dir> [--margin]

S3 drift cancelling (enabled, DRIFT_GAIN 2.0) and the burntable::hIgnite(NaN)
guard always; --margin also sets BURN_TABLE_MARGIN_M to 1.0 (only if it
passed the holdout). Code matches ideas_full.patch's S3 path, so a tree
ported with this flies bit-for-bit like the study copy with DRIFT_GAIN 2.0.
"""
import os
import sys


def edit(root, rel, old, new):
    p = os.path.join(root, rel)
    raw = open(p, "rb").read().decode("utf-8")
    crlf = "\r\n" in raw
    txt = raw.replace("\r\n", "\n")
    n = txt.count(old)
    if n != 1:
        sys.exit("%s: expected 1 match, found %d for:\n%s" % (rel, n, old[:300]))
    txt = txt.replace(old, new)
    if crlf:
        txt = txt.replace("\n", "\r\n")
    open(p, "wb").write(txt.encode("utf-8"))
    print("patched", rel)


root = sys.argv[1]
margin = "--margin" in sys.argv[2:]

edit(root, "src/config.h",
     "constexpr float BURN_TABLE_MARGIN_M = 0.0f;   // + = ignite earlier (higher)\n",
     ("constexpr float BURN_TABLE_MARGIN_M = 1.0f;   // + = ignite earlier (higher); 1 m with S3:\n"
      "                                              // sim holdout 2026-10-24 (motor-study-2026-10-24)\n")
     if margin else
     "constexpr float BURN_TABLE_MARGIN_M = 0.0f;   // + = ignite earlier (higher)\n")
edit(root, "src/config.h",
     "constexpr float KF_UNHEALTHY_ABORT_MS = 500.0f;\n",
     "constexpr float KF_UNHEALTHY_ABORT_MS = 500.0f;\n"
     "\n"
     "// ---------------------------------------------------------------------------\n"
     "// Landing-burn drift cancelling (S3): during the landing burn, lean the TVC\n"
     "// target against the sideways velocity dead-reckoned from launch\n"
     "// (world-frame accelerometer), so the rocket has stopped drifting by\n"
     "// touchdown and doesn't tip over on its legs. Sim study 2026-10-24\n"
     "// (motor-study-2026-10-24): sideways contact speed ~1.4 -> ~0.45 m/s.\n"
     "// ---------------------------------------------------------------------------\n"
     "constexpr float DRIFT_GAIN = 2.0f;           // 1/s; 0 = off\n"
     "constexpr float DRIFT_MAX_TILT_DEG = 5.0f;   // cap on the commanded lean\n")

edit(root, "src/core/ahrs.h",
     "  // Vertical (world-Z) component of specific force, for the altitude KF:\n",
     "  // Tilt error toward an arbitrary world-frame target direction (unit\n"
     "  // vector), same convention as tiltErrorBody() (which is target = up).\n"
     "  Vec3 tiltErrorBodyTo(const Vec3& target) const {\n"
     "    const Vec3 zbWorld = q_.rotate(Vec3{0, 0, 1});\n"
     "    const Vec3 axis = zbWorld.cross(target);\n"
     "    const float s = axis.norm();\n"
     "    if (s < 1e-6f) return {0, 0, 0};\n"
     "    const float ang = std::atan2(s, zbWorld.x * target.x + zbWorld.y * target.y +\n"
     "                                        zbWorld.z * target.z);\n"
     "    return q_.rotateInv(axis * (ang / s));\n"
     "  }\n"
     "\n"
     "  // Vertical (world-Z) component of specific force, for the altitude KF:\n")

edit(root, "src/core/flight_core.h",
     "    kf_.predict(ahrs_.worldVerticalAccel(in.accel, cfg::G0), in.dt);\n",
     "    kf_.predict(ahrs_.worldVerticalAccel(in.accel, cfg::G0), in.dt);\n"
     "    // Sideways velocity, dead-reckoned from launch for the drift loop:\n"
     "    // gravity is vertical, so the world-frame horizontal specific force\n"
     "    // IS the horizontal acceleration.\n"
     "    if (onPad) {\n"
     "      velHx_ = velHy_ = 0.0f;\n"
     "    } else {\n"
     "      const Vec3 aw = ahrs_.quat().rotate(in.accel);\n"
     "      velHx_ += aw.x * in.dt;\n"
     "      velHy_ += aw.y * in.dt;\n"
     "    }\n")
edit(root, "src/core/flight_core.h",
     "      const Vec3 e = ahrs_.tiltErrorBody();\n",
     "      Vec3 e = ahrs_.tiltErrorBody();\n"
     "      if (fo.useLandingGains && cfg::DRIFT_GAIN > 0.0f) {\n"
     "        const float aT = in.accel.norm();   // ~thrust/mass while burning\n"
     "        if (aT > 3.0f) {\n"
     "          // Want horizontal accel = -K v: lean the thrust axis by -K v / aT.\n"
     "          float hx = -cfg::DRIFT_GAIN * velHx_ / aT;\n"
     "          float hy = -cfg::DRIFT_GAIN * velHy_ / aT;\n"
     "          const float hn = std::sqrt(hx * hx + hy * hy);\n"
     "          const float cap = std::tan(cfg::DRIFT_MAX_TILT_DEG * cfg::DEG2RAD);\n"
     "          if (hn > cap) { hx *= cap / hn; hy *= cap / hn; }\n"
     "          const float n = std::sqrt(hx * hx + hy * hy + 1.0f);\n"
     "          e = ahrs_.tiltErrorBodyTo(Vec3{hx / n, hy / n, 1.0f / n});\n"
     "        }\n"
     "      }\n")
edit(root, "src/core/flight_core.h",
     "  bool wasActive_ = false;\n",
     "  bool wasActive_ = false;\n"
     "  float velHx_ = 0.0f, velHy_ = 0.0f;  // dead-reckoned sideways velocity (m/s, world x/y)\n")

edit(root, "src/core/flight_state.h",
     "    if (vDown < cfg::BURN_MIN_SPEED_MS) return;\n",
     "    // Written as !(>=) so a NaN velocity also returns here: hIgnite() casts\n"
     "    // its argument to int, which is undefined behaviour for NaN.\n"
     "    if (!(vDown >= cfg::BURN_MIN_SPEED_MS)) return;\n")
