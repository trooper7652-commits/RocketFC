//
// RocketFC PC replay harness.
//
// Compiles src/core (the EXACT files the Teensy flies) natively, runs built-in
// unit checks on the math, then streams synthetic flight CSVs (from
// tools/synth_flight.py) through FlightCore and asserts:
//   - the state sequence matches the scenario expectation
//   - the landing-motor fire command lands inside the physics-derived window
//   - aborts happen exactly when they should (and never when they shouldn't)
//   - the Kalman altitude tracks truth within bounds
//
// Build:  g++ -O2 -std=c++17 -Wall -o replay main.cpp   (see Makefile)
// Run:    ./replay cases/nominal_chute.csv cases/nominal_land.csv ...
//
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
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
    }

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
