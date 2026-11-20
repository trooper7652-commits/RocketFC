//
// RocketFC PC replay harness.
//
// Compiles src/core (the EXACT files the Teensy flies) natively, runs built-in
// unit checks on the math, then streams synthetic flight CSVs (from
// tools/synth_flight.py) through FlightCore and asserts:
//   - the state sequence matches the scenario expectation
//   - the landing-motor fire command lands inside the physics-derived window
//   - aborts happen exactly when they should (and never when they shouldn't)
//   - the parachute is released when (and only when) it should be, the canopy
//     is confirmed, and a stuck latch gets re-cycled
//   - the leg-release nichrome burns from fire + LEGS_DELAY_MS to touchdown on
//     a good landing burn, and never otherwise
//   - the Kalman altitude tracks truth within bounds
//
// Build:  g++ -O2 -std=c++17 -Wall -o replay main.cpp   (see Makefile)
// Run:    ./replay cases/nominal_chute.csv cases/nominal_land.csv ...
//
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "../../src/core/flight_core.h"

// ---------------------------------------------------------------------------
// Tiny helpers for the #META json line (format is under our control).
// ---------------------------------------------------------------------------
static std::string metaRaw(const std::string& meta, const std::string& key) {
  const std::string pat = "\"" + key + "\":";
  const size_t k = meta.find(pat);
  if (k == std::string::npos) return "";
  size_t i = k + pat.size();
  while (i < meta.size() && meta[i] == ' ') ++i;
  size_t depth = 0, j = i;
  for (; j < meta.size(); ++j) {
    const char c = meta[j];
    if (c == '[' || c == '{') ++depth;
    else if (c == ']' || c == '}') {
      if (depth == 0) break;
      --depth;
    } else if (c == ',' && depth == 0) break;
  }
  return meta.substr(i, j - i);
}
static std::string metaStr(const std::string& m, const std::string& k) {
  std::string r = metaRaw(m, k);
  if (r.size() >= 2 && r.front() == '"') return r.substr(1, r.size() - 2);
  return r;  // "null" or bare
}
static double metaNum(const std::string& m, const std::string& k, double dflt) {
  const std::string r = metaRaw(m, k);
  if (r.empty() || r == "null") return dflt;
  return atof(r.c_str());
}
static std::vector<std::string> metaStrArr(const std::string& m,
                                           const std::string& k) {
  std::vector<std::string> out;
  const std::string r = metaRaw(m, k);
  size_t i = 0;
  while ((i = r.find('"', i)) != std::string::npos) {
    const size_t j = r.find('"', i + 1);
    if (j == std::string::npos) break;
    out.push_back(r.substr(i + 1, j - i - 1));
    i = j + 1;
  }
  return out;
}
static bool metaPair(const std::string& m, const std::string& k, double& a,
                     double& b) {
  const std::string r = metaRaw(m, k);
  if (r.empty() || r == "null") return false;
  return sscanf(r.c_str(), "[%lf , %lf]", &a, &b) == 2 ||
         sscanf(r.c_str(), "[%lf, %lf]", &a, &b) == 2;
}

// ---------------------------------------------------------------------------
static int failures = 0;
#define CHECK(cond, ...)                          \
  do {                                            \
    if (cond) {                                   \
      printf("  PASS: " __VA_ARGS__);             \
      printf("\n");                               \
    } else {                                      \
      printf("  FAIL: " __VA_ARGS__);             \
      printf("\n");                               \
      ++failures;                                 \
    }                                             \
  } while (0)

// ---------------------------------------------------------------------------
// Drives ChuteDeploy alone at 500 Hz with a scripted |specific force| (g) and
// altitude (m, default well clear of the ground) as functions of seconds
// since release. Release happens at t = 0 after 1 s of pre-release history.
// ---------------------------------------------------------------------------
struct ChuteRun {
  int releases = 0;
  bool detected = false, unconfirmed = false, everLockedAfter = false;
  bool endReleased = false;
  ChuteServo endCmd = ChuteServo::LOCK;
  std::vector<ChuteServo> cmds;  // servo command each time it changed
};
static ChuteRun runChute(
    const std::function<float(float)>& accelG, float seconds,
    const std::function<float(float)>& altM = [](float) { return 100.0f; }) {
  ChuteDeploy cd;
  ChuteRun r;
  const float dtMs = 2.0f;
  uint32_t ms = 10000;
  for (float t = -1.0f; t < seconds; t += dtMs / 1000.0f, ms += 2) {
    if (t >= 0.0f && !cd.started()) cd.start(ms);
    cd.update(ms, dtMs, accelG(t), altM(t));
    if (cd.started() && !cd.servoRelease()) r.everLockedAfter = true;
    if (cd.started() && (r.cmds.empty() || r.cmds.back() != cd.servoCmd()))
      r.cmds.push_back(cd.servoCmd());
  }
  r.endCmd = cd.servoCmd();
  r.releases = cd.releases();
  r.detected = cd.detected();
  r.unconfirmed = cd.unconfirmed();
  r.endReleased = cd.servoRelease();
  return r;
}

// ---------------------------------------------------------------------------
// Unit checks on the math the vehicle's life depends on.
// ---------------------------------------------------------------------------
static void unitChecks() {
  printf("== unit checks ==\n");

  // Quaternion round trip
  {
    Quat q = Quat::fromRotVec({0.3f, -0.2f, 0.5f});
    Vec3 v{1, 2, 3};
    Vec3 r = q.rotateInv(q.rotate(v));
    CHECK(std::fabs(r.x - 1) < 1e-4f && std::fabs(r.y - 2) < 1e-4f &&
              std::fabs(r.z - 3) < 1e-4f,
          "quat rotate/rotateInv round-trip");
  }

  // Tilt error sign: +5 deg rotation about body X -> correction is -5 deg
  // about body X.
  {
    Ahrs a;
    a.initFromAccel({0, 0, 9.81f});
    const float th = 5.0f * cfg::DEG2RAD;
    Ahrs t = a;
    // apply rotation about body X by integrating gyro for 1 s
    for (int i = 0; i < 1000; ++i) t.update({th / 1.0f, 0, 0}, 0.001f);
    const Vec3 e = t.tiltErrorBody();
    CHECK(std::fabs(e.x + th) < 0.15f * th && std::fabs(e.y) < 0.02f,
          "tilt error: +5deg about X -> correction -5deg about X (e=[%.3f %.3f])",
          e.x, e.y);
  }

  // Roll decoupling: same tilt, then 90 deg roll about the body axis. The
  // needed correction must move to the OTHER body axis.
  {
    Ahrs t;
    t.initFromAccel({0, 0, 9.81f});
    const float th = 5.0f * cfg::DEG2RAD;
    for (int i = 0; i < 1000; ++i) t.update({th, 0, 0}, 0.001f);
    for (int i = 0; i < 1000; ++i)
      t.update({0, 0, 90.0f * cfg::DEG2RAD}, 0.001f);  // roll +90 deg
    const Vec3 e = t.tiltErrorBody();
    CHECK(std::fabs(e.y - th) < 0.15f * th && std::fabs(e.x) < 0.02f,
          "roll decoupling: after +90deg roll the correction moves X->Y "
          "(e=[%.3f %.3f], tilt %.2f deg)",
          e.x, e.y, t.tiltRad() * cfg::RAD2DEG);
  }

  // KF: static convergence + ramp tracking
  {
    AltitudeKf kf;
    kf.configure(0.5f, 0.01f, 5.0f, 25);
    kf.reset(0);
    kf.setBaroVar(0.04f);
    // 10 s static with accel bias 0.2 m/s^2
    for (int i = 0; i < 5000; ++i) {
      kf.predict(0.2f, 0.002f);
      if (i % 5 == 0) kf.update(0.0f);
    }
    CHECK(std::fabs(kf.altitude()) < 0.2f && std::fabs(kf.velocity()) < 0.2f &&
              std::fabs(kf.bias() - 0.2f) < 0.1f,
          "KF absorbs accel bias at rest (h=%.3f v=%.3f b=%.3f)",
          kf.altitude(), kf.velocity(), kf.bias());
    // Constant 5 m/s climb seen only by the baro (accel input stays at the
    // learned bias — i.e. a dead accelerometer). The filter overshoots
    // through the transient, then settles; 20 s gives it time to converge.
    float h = 0;
    for (int i = 0; i < 10000; ++i) {
      h += 5.0f * 0.002f;
      kf.predict(0.2f, 0.002f);
      if (i % 5 == 0) kf.update(h);
    }
    CHECK(std::fabs(kf.altitude() - h) < 0.5f &&
              std::fabs(kf.velocity() - 5.0f) < 0.5f,
          "KF tracks a 5 m/s ramp (h=%.2f/%.2f v=%.2f)", kf.altitude(), h,
          kf.velocity());
    // glitch rejection
    const float hBefore = kf.altitude();
    kf.update(hBefore + 40.0f);
    CHECK(std::fabs(kf.altitude() - hBefore) < 1.0f,
          "KF gates a 40 m baro glitch");
  }

  // Parachute deploy supervisor
  {
    // Clean deploy: freefall, canopy opens 0.3 s after release (2.5 g
    // spike, then 1 g under canopy).
    const ChuteRun a = runChute(
        [](float t) { return t < 0.3f ? 0.02f : t < 0.5f ? 2.5f : 1.0f; },
        6.0f);
    // (The blind wiggle may already have made its next stroke: a healthy
    // latch is detected within the wiggle, no slow re-cycle.)
    const int K = cfg::CHUTE_WIG_CYCLES;
    CHECK(a.detected && a.releases <= 1 + K && a.endReleased &&
              a.endCmd == ChuteServo::OPEN,
          "chute: clean deploy -> detected within the wiggle, ends OPEN "
          "(releases=%d, max %d)", a.releases, 1 + K);

    // Healthy latch released near apogee: no opening spike, the g builds up
    // slowly as the rocket picks up speed under the filling canopy (the sim:
    // 0.4 g at 0.34 s, 0.75 g at 0.62 s). Must be confirmed during the wiggle,
    // not lost because a wiggle stroke re-took the baseline on the way up.
    const ChuteRun g = runChute(
        [](float t) { return t < 0.0f ? 0.02f : std::fmin(0.95f, 0.02f + 1.15f * t); },
        6.0f);
    CHECK(g.detected && g.releases <= 1 + K && g.endCmd == ChuteServo::OPEN,
          "chute: slow g build-up after an apogee release -> detected within the "
          "wiggle (releases=%d, max %d)", g.releases, 1 + K);

    // Stuck latch, never comes free: every wiggle and re-cycle is used, latch
    // ends open.
    const ChuteRun b = runChute([](float) { return 0.02f; }, 20.0f);
    CHECK(!b.detected && b.unconfirmed &&
              b.releases == cfg::CHUTE_MAX_RELEASES + K && b.everLockedAfter &&
              b.endReleased && b.endCmd == ChuteServo::OPEN,
          "chute: stuck latch -> %d releases (want %d), unconfirmed, held open",
          b.releases, cfg::CHUTE_MAX_RELEASES + K);
    // ...and the wiggle comes first: WIG_HIGH, WIG_LOW (x K), then OPEN.
    if (K >= 1) {
      std::vector<ChuteServo> want;
      for (int i = 0; i < K; ++i) {
        want.push_back(ChuteServo::WIG_HIGH);
        want.push_back(ChuteServo::WIG_LOW);
      }
      want.push_back(ChuteServo::OPEN);
      want.push_back(ChuteServo::LOCK);
      const bool seqOk = b.cmds.size() >= want.size() &&
                         std::equal(want.begin(), want.end(), b.cmds.begin());
      CHECK(seqOk, "chute: wiggle order HIGH/LOW x%d, then OPEN, then slow "
                   "re-cycles (%d commands)", K, (int)b.cmds.size());
    }

    // Times since the first release: the wiggle ends (last stroke to OPEN),
    // then the canopy check starts.
    const float wigEnd = (K > 0 ? cfg::CHUTE_WIG_FIRST_MS + K * cfg::CHUTE_WIG_DOWN_MS +
                                      (K - 1) * cfg::CHUTE_WIG_UP_MS
                                : 0.0f) / 1000.0f;

    // Latch frees on the 2nd wiggle stroke: detected, no further release.
    if (K >= 1) {
      const float tw = (cfg::CHUTE_WIG_FIRST_MS + cfg::CHUTE_WIG_DOWN_MS) /
                           1000.0f + 0.05f;
      const ChuteRun w = runChute(
          [tw](float t) { return t < tw ? 0.02f : 1.0f; }, 6.0f);
      CHECK(w.detected && w.releases == 2 && w.endCmd == ChuteServo::OPEN,
            "chute: frees on the 2nd wiggle stroke -> detected, releases=%d "
            "(want 2), ends OPEN", w.releases);

      // Canopy while the wiggle is swung back to WIG_LOW: straight to OPEN.
      // (rises so that detection lands 10 ms into the first WIG_LOW)
      const float tl = (cfg::CHUTE_WIG_FIRST_MS + 10.0f - cfg::CHUTE_DETECT_MS) / 1000.0f;
      const ChuteRun x = runChute(
          [tl](float t) { return t < tl ? 0.02f : 1.0f; }, 6.0f);
      CHECK(x.detected && x.releases == 1 && x.endCmd == ChuteServo::OPEN,
            "chute: canopy during a wiggle down-stroke -> straight to OPEN "
            "(releases=%d)", x.releases);
    }

    // Latch frees on the first slow re-cycle: canopy shortly after it.
    const float t2 = wigEnd + (cfg::CHUTE_CONFIRM_MS + cfg::CHUTE_RECYCLE_LOCK_MS) /
                                  1000.0f + 0.3f;
    const ChuteRun c = runChute(
        [t2](float t) { return t < t2 ? 0.02f : 1.0f; }, 10.0f);
    CHECK(c.detected && c.releases == K + 2 && c.endReleased,
          "chute: frees on the 1st slow re-cycle -> detected, releases=%d (want %d)",
          c.releases, K + 2);

    // Post-abort drag build-up (0.3 g rising 0.12 g/s toward 1 g at terminal
    // velocity, never a canopy) must NOT be mistaken for one -- the baseline
    // is re-taken at each release.
    const ChuteRun d = runChute(
        [](float t) { return std::fmin(1.0f, 0.3f + 0.12f * (t > 0 ? t : 0)); },
        15.0f);
    CHECK(!d.detected && d.unconfirmed,
          "chute: slow drag build-up is not a canopy (releases=%d)",
          d.releases);

    // Opening while the servo is swung back to LOCK: straight back to RELEASE.
    const float tl = wigEnd + cfg::CHUTE_CONFIRM_MS / 1000.0f + 0.1f;
    const ChuteRun e = runChute(
        [tl](float t) { return t < tl ? 0.02f : 1.0f; }, 6.0f);
    CHECK(e.detected && e.releases == 1 + K && e.endReleased,
          "chute: canopy during the LOCK dwell -> back to RELEASE (releases=%d)",
          e.releases);

    // Latch never frees and the rocket hits the ground at 2.5 s: the impact
    // jolt (then 1 g lying on the ground) must NOT be logged as a canopy.
    const ChuteRun f = runChute(
        [](float t) { return t < 2.5f ? 0.02f : t < 2.6f ? 30.0f : 1.0f; },
        6.0f, [](float t) { return t < 2.5f ? 30.0f - 12.0f * t : 0.0f; });
    CHECK(!f.detected,
          "chute: ground impact is not mistaken for a canopy (releases=%d)",
          f.releases);
  }
}

// ---------------------------------------------------------------------------
struct Row {
  float t, ax, ay, az, gx, gy, gz;
  int baroNew;
  float pPa, hTrue, vTrue, tiltTrue, thrust;
};

static bool parseRow(const std::string& line, Row& r) {
  Row x{};
  char pbuf[32] = "";
  // p_pa may be empty (non-baro rows)
  const int n = sscanf(line.c_str(),
                       "%f,%f,%f,%f,%f,%f,%f,%d,%31[^,],%f,%f,%f,%f", &x.t,
                       &x.ax, &x.ay, &x.az, &x.gx, &x.gy, &x.gz, &x.baroNew,
                       pbuf, &x.hTrue, &x.vTrue, &x.tiltTrue, &x.thrust);
  if (n == 13) {
    x.pPa = atof(pbuf);
  } else {
    // empty p_pa: sscanf can't match an empty field with %[^,]; re-parse
    const int n2 = sscanf(line.c_str(), "%f,%f,%f,%f,%f,%f,%f,%d,,%f,%f,%f,%f",
                          &x.t, &x.ax, &x.ay, &x.az, &x.gx, &x.gy, &x.gz,
                          &x.baroNew, &x.hTrue, &x.vTrue, &x.tiltTrue,
                          &x.thrust);
    if (n2 != 12) return false;
    x.pPa = 0;
  }
  r = x;
  return true;
}

static const char* abortName(AbortReason r) {
  switch (r) {
    case AbortReason::NONE: return "NONE";
    case AbortReason::TILT: return "TILT";
    case AbortReason::IMU_FAIL: return "IMU_FAIL";
    case AbortReason::KF_UNHEALTHY: return "KF_UNHEALTHY";
    case AbortReason::MISSED_WINDOW: return "MISSED_WINDOW";
    case AbortReason::DUD_IGNITER: return "DUD_IGNITER";
  }
  return "?";
}

// ---------------------------------------------------------------------------
static void runCase(const std::string& path) {
  std::ifstream f(path);
  if (!f) {
    printf("cannot open %s\n", path.c_str());
    ++failures;
    return;
  }
  std::string meta, line;
  std::getline(f, meta);           // #META {...}
  std::getline(f, line);           // header

  std::vector<Row> rows;
  rows.reserve(20000);
  Row r;
  while (std::getline(f, line))
    if (parseRow(line, r)) rows.push_back(r);

  const std::string scenario = metaStr(meta, "scenario");
  const std::string mode = metaStr(meta, "mode");
  const double armT = metaNum(meta, "arm_t", 2.6);
  const double tdTrue = metaNum(meta, "touchdown_t", 1e9);
  const double maxKfErr = metaNum(meta, "max_kf_alt_err", 3.0);
  const std::string expectAbort = metaStr(meta, "expect_abort");
  const auto expectStates = metaStrArr(meta, "expect_states");
  // -1 (not given): any count a healthy latch can give, 1 .. 1 + wiggles.
  const int expectReleases = (int)metaNum(meta, "expect_chute_releases", -1);
  const bool expectDetect = metaStr(meta, "expect_chute_detected") != "false";
  double fw0 = 0, fw1 = 0;
  const bool hasFireWindow = metaPair(meta, "fire_window", fw0, fw1);

  printf("\n== case %s (%zu rows, mode %s) ==\n", scenario.c_str(),
         rows.size(), mode.c_str());

  // --- pad calibration, exactly like the firmware's arming sequence ---
  Vec3 gyroBias{}, accelAvg{};
  int nCal = 0;
  double pM2 = 0, pMean = 0;
  int nP = 0;
  for (const Row& row : rows) {
    if (row.t >= armT - 0.2) break;
    gyroBias = gyroBias + Vec3{row.gx, row.gy, row.gz};
    accelAvg = accelAvg + Vec3{row.ax, row.ay, row.az};
    ++nCal;
    if (row.baroNew && row.pPa > 0) {
      ++nP;
      const double d = row.pPa - pMean;
      pMean += d / nP;
      pM2 += d * (row.pPa - pMean);
    }
  }
  gyroBias = gyroBias * (1.0f / nCal);
  accelAvg = accelAvg * (1.0f / nCal);
  const float p0 = (float)pMean;
  const float mPerPa = 8434.6f / p0;
  const float altVar = (float)(pM2 / (nP - 1)) * mPerPa * mPerPa;

  // --- flight core, wired the same way RocketFC.ino wires it ---
  FlightCore core;
  core.begin();
  core.setMode(mode == "land" ? cfg::FlightMode::FULL_LANDING
                              : cfg::FlightMode::CHUTE_TEST);

  std::vector<std::string> seq{"IDLE"};
  double fireT = -1, abortT = -1, tdT = -1, launchT = -1;
  double chuteT = -1, chuteDetT = -1;
  int chuteReleases = 0;
  double legsOnT = -1, legsOffT = -1, confirmT = -1;
  bool legsEver = false, legsGap = false;
  AbortReason abortReason = AbortReason::NONE;
  float maxErr = 0;
  bool armed = false;

  CoreOutput co;
  for (const Row& row : rows) {
    const uint32_t ms = (uint32_t)(row.t * 1000.0 + 0.5);
    if (!armed && row.t >= armT) {
      core.padLevelInit(accelAvg);
      core.setBaroNoiseVar(altVar > 0.0025f ? altVar : 0.0025f);
      core.zeroAltitude();
      core.requestArm(ms);
      armed = true;
    }

    CoreInput ci;
    ci.ms = ms;
    ci.dt = 0.002f;
    ci.accel = {row.ax, row.ay, row.az};
    ci.gyro = Vec3{row.gx, row.gy, row.gz} - gyroBias;
    ci.baroNew = row.baroNew != 0 && row.pPa > 0;
    if (ci.baroNew)
      ci.baroAlt = 44330.0f * (1.0f - powf(row.pPa / p0, 0.190295f));
    core.step(ci, co);

    const char* sn = FlightStateMachine::stateName(co.state);
    if (seq.empty() || seq.back() != sn) seq.push_back(sn);

    FlightStateMachine::Event ev;
    while (core.fsm().popEvent(ev)) {
      if (ev.code == FlightEvent::LAUNCH) launchT = row.t;
      if (ev.code == FlightEvent::FIRE_LANDING) fireT = row.t;
      if (ev.code == FlightEvent::ABORT_DET) {
        abortT = row.t;
        abortReason = (AbortReason)(uint8_t)ev.value;
      }
      if (ev.code == FlightEvent::TOUCHDOWN_DET && tdT < 0) tdT = row.t;
      if (ev.code == FlightEvent::CHUTE_RELEASE) {
        if (chuteT < 0) chuteT = row.t;
        ++chuteReleases;
      }
      if (ev.code == FlightEvent::CHUTE_DETECTED) chuteDetT = row.t;
      if (ev.code == FlightEvent::LEGS_BURN_ON) legsOnT = row.t;
      if (ev.code == FlightEvent::IGNITION_CONFIRMED) confirmT = row.t;
      if (ev.code == FlightEvent::LEGS_BURN_OFF) legsOffT = row.t;
    }
    legsEver |= co.legsBurn;
    if (legsOnT > 0 && legsOffT < 0 && !co.legsBurn) legsGap = true;

    if (launchT > 0 && tdT < 0) {
      const float err = std::fabs(co.kfAlt - row.hTrue);
      if (err > maxErr) maxErr = err;
    }
  }

  // --- assertions ---
  std::string got;
  for (const auto& s : seq) got += s + " ";
  std::string want;
  for (const auto& s : expectStates) want += s + " ";
  CHECK(seq == expectStates, "state sequence [%s] (want [%s])", got.c_str(),
        want.c_str());

  if (hasFireWindow)
    CHECK(fireT >= fw0 && fireT <= fw1,
          "landing fire cmd at %.2f s inside window [%.2f, %.2f]", fireT, fw0,
          fw1);

  if (expectAbort != "null" && !expectAbort.empty())
    CHECK(abortT > 0 && expectAbort == abortName(abortReason),
          "abort %s at %.2f s (want %s)", abortName(abortReason), abortT,
          expectAbort.c_str());
  else
    CHECK(abortT < 0, "no abort (got %s at %.2f s)", abortName(abortReason),
          abortT);

  bool wantChute = false;
  for (const auto& s : expectStates) wantChute |= s == "DESCENT_CHUTE";
  if (wantChute) {
    // A confirm must come from the canopy itself, never from the ground impact.
    // The latch must be released before contact: on any abort the chute
    // has to come out (whether it can still fill doesn't matter).
    const bool countOk =
        expectReleases > 0
            ? chuteReleases == expectReleases
            : chuteReleases >= 1 && chuteReleases <= 1 + cfg::CHUTE_WIG_CYCLES;
    CHECK(countOk && chuteT > 0 && chuteT < tdTrue &&
              (chuteDetT > 0) == expectDetect &&
              (chuteDetT < 0 || chuteDetT < tdTrue) && co.chuteRelease,
          "chute released at %.2f s (contact %.2f s), %d release%s (want %s), "
          "canopy %s at %.2f s (want %s, before contact), latch held open",
          chuteT, tdTrue, chuteReleases, chuteReleases == 1 ? "" : "s",
          expectReleases > 0 ? std::to_string(expectReleases).c_str() : "1..1+wiggles",
          chuteDetT > 0 ? "confirmed" : "NOT confirmed", chuteDetT,
          expectDetect ? "confirmed" : "not confirmed");
  } else {
    CHECK(chuteReleases == 0 && !co.chuteRelease,
          "chute never released (got %d release%s)", chuteReleases,
          chuteReleases == 1 ? "" : "s");
  }

  // Legs: only on a real landing burn (fire command and no abort).
  const bool wantLegs = hasFireWindow && abortT < 0;
  if (wantLegs) {
    // On at fire + LEGS_DELAY_MS, but never before the ignition confirm.
    const double want = std::max(fireT + cfg::LEGS_DELAY_MS / 1000.0, confirmT);
    CHECK(confirmT > 0 && legsOnT >= want - 0.001 && legsOnT <= want + 0.1 &&
              legsOffT == tdT && !legsGap && !co.legsBurn,
          "legs nichrome on at %.2f s (fire + %.2f s, confirm at %.2f s), off at "
          "%.2f s (touchdown %.2f s), continuous, off at the end",
          legsOnT, legsOnT - fireT, confirmT, legsOffT, tdT);
  } else {
    CHECK(legsOnT < 0 && !legsEver, "legs never fired (on at %.2f s)",
          legsOnT);
  }

  CHECK(tdT > 0 && tdT < tdTrue + 8.0,
        "touchdown detected at %.2f s (true contact %.2f s)", tdT, tdTrue);

  CHECK(maxErr < (float)maxKfErr, "KF altitude max error %.2f m < %.2f m",
        maxErr, maxKfErr);
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  unitChecks();
  for (int i = 1; i < argc; ++i) runCase(argv[i]);
  printf("\n%s (%d failure%s)\n", failures == 0 ? "ALL PASS" : "FAILURES",
         failures, failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}
