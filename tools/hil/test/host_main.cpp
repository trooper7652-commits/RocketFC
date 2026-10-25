// Runs the GENERATED HIL sketch (tools/hil/build/RocketFC_HIL) on the PC, with
// tools/hil/test/mock standing in for the Teensy core and its libraries, and
// drives it exactly the way an operator + hil_ctl.py would:
//
//   boot -> `load` + trace bytes on port 2 -> `mode land` / `chute lock` /
//   `arm` on the console -> `go` on port 2 -> run until both logs close.
//
// The rocket sits still (gravity + a little noise on the "real" IMU). When the
// landing pyro gate goes high, real vibration starts on the IMU after
// <ignition delay> ms, so compare.py's ignition-delay estimate can be checked
// against a known answer. Loop passes take a random 3-30 us of simulated time.
//
// usage: host_hil <sd dir> <trace.bin> "<load line>" <mode 0|1>
//                 <ignition delay ms> [abort after go, s]
// Built and run by tools/hil/test_hil.py.
#include <random>

#include "Arduino.h"
#include "BMI088.h"
#include "EEPROM.h"
#include "SD.h"
#include "Servo.h"
#include "Wire.h"

uint64_t g_us = 0;
int g_pinOut[64] = {0};
int g_analog[64] = {0};
float g_imuAccel[3] = {0, 0, 9.80665f};
float g_imuGyro[3] = {0, 0, 0};
uint32_t g_baroD1 = 9085466;  // datasheet example: ~1000 hPa
std::string g_sdRoot;
HostSerial Serial("con");
HostSerial SerialUSB1("hil");
TwoWire Wire;
SDClass SD;
EEPROMClass EEPROM;

#include "RocketFC_HIL.ino"

static uint64_t g_fireUs = 0;
static int g_ignDelayMs = 400;

void hostPinChanged(int pin, int v) {
  if (pin == cfg::PIN_PYRO_LAND) {
    if (v == HIGH && g_fireUs == 0) {
      g_fireUs = g_us;
      ::printf("[%9.3f host] landing pyro gate ON (vibration in %d ms)\n", g_us / 1e6,
               g_ignDelayMs);
    }
    // Gate on pulls the drain low; afterwards the burnt e-match is open.
    g_analog[cfg::PIN_CONT_LAND] = 0;
  }
  if (pin == cfg::PIN_PYRO_LEGS)
    ::printf("[%9.3f host] legs nichrome gate %s\n", g_us / 1e6, v ? "ON" : "OFF");
}

static std::mt19937 rng(12345);
static std::normal_distribution<float> unit(0.0f, 1.0f);
static std::uniform_int_distribution<int> passUs(3, 30);
static std::uniform_int_distribution<int> baroNoise(-80, 80);

static void stepOnce() {
  // Still rocket: gravity along body +z, expressed in the sensor frame
  // (body = R * sensor, R orthonormal -> sensor = R^T * body).
  const float* R = cfg::IMU_R_SB;
  const bool burning = g_fireUs != 0 && g_us >= g_fireUs + (uint64_t)g_ignDelayMs * 1000;
  const float an = burning ? 15.0f : 0.02f;
  for (int i = 0; i < 3; ++i) {
    g_imuAccel[i] = cfg::G0 * R[6 + i] + an * unit(rng);
    g_imuGyro[i] = (burning ? 0.2f : 0.0005f) * unit(rng);
  }
  g_baroD1 = 9085466 + baroNoise(rng);
  loop();
  g_us += passUs(rng);
}

static bool runUntil(double maxS, HostSerial& port, const char* needle) {
  const uint64_t end = g_us + (uint64_t)(maxS * 1e6);
  while (g_us < end) {
    stepOnce();
    while (!port.lines.empty()) {
      const std::string ln = port.lines.front();
      port.lines.pop_front();
      if (needle && ln.find(needle) != std::string::npos) return true;
      if (ln.rfind("ERR", 0) == 0 || ln.find("REFUSED") != std::string::npos) {
        ::printf("host: FAILED on: %s\n", ln.c_str());
        exit(3);
      }
    }
  }
  return needle == nullptr;
}

static void need(bool ok, const char* what) {
  if (!ok) {
    ::printf("host: timed out waiting for %s\n", what);
    exit(2);
  }
}

int main(int argc, char** argv) {
  if (argc < 6) {
    ::printf("usage: host_hil <sd dir> <trace.bin> \"<load line>\" <mode> <ign ms> [abort s]\n");
    return 1;
  }
  g_sdRoot = argv[1];
  FILE* tf = fopen(argv[2], "rb");
  if (!tf) { ::printf("host: can't open %s\n", argv[2]); return 1; }
  std::string blob;
  char buf[65536];
  size_t k;
  while ((k = fread(buf, 1, sizeof(buf), tf)) > 0) blob.append(buf, k);
  fclose(tf);
  const std::string loadLine = argv[3];
  const int mode = atoi(argv[4]);
  g_ignDelayMs = atoi(argv[5]);
  const double abortAt = argc > 6 ? atof(argv[6]) : -1.0;

  // Healthy hardware: both continuity channels see ~8 V on the drain, 7.8 V battery.
  const int contRaw = (int)(8.0f / cfg::CONT_DIVIDER_RATIO / 3.3f * 1023.0f);
  g_analog[cfg::PIN_CONT_LAND] = contRaw;
  g_analog[cfg::PIN_CONT_LEGS] = contRaw;
  g_analog[cfg::PIN_VBAT] = (int)(7.8f / cfg::VBAT_DIVIDER / 3.3f * 1023.0f);

  setup();
  runUntil(1.0, Serial, nullptr);

  SerialUSB1.push(loadLine + "\n");
  need(runUntil(2.0, SerialUSB1, "OK send"), "OK send");
  SerialUSB1.push(blob.data(), blob.size());
  need(runUntil(10.0, SerialUSB1, "OK loaded"), "OK loaded");

  Serial.push(mode == 1 ? "mode land\n" : "mode chute\n");
  runUntil(0.3, Serial, nullptr);
  Serial.push("chute lock\n");
  runUntil(0.5, Serial, nullptr);
  Serial.push("arm\n");
  need(runUntil(30.0, Serial, "ARMED"), "ARMED");
  runUntil(1.0, Serial, nullptr);

  SerialUSB1.push("go\n");
  need(runUntil(1.0, SerialUSB1, "OK go"), "OK go");
  const uint64_t goUs = g_us;

  // After an operator abort the core waits under the chute (no touchdown, so
  // the flight log stays open, flushed every LOG_FLUSH_MS): stop once the
  // real-IMU log has closed.
  bool logClosed = false, imuClosed = false, aborted = false;
  auto finished = [&] { return imuClosed && (logClosed || abortAt >= 0); };
  while (g_us - goUs < 60000000ull && !finished()) {
    stepOnce();
    if (abortAt >= 0 && !aborted && g_us - goUs >= (uint64_t)(abortAt * 1e6)) {
      SerialUSB1.push("abort\n");
      aborted = true;
    }
    while (!Serial.lines.empty()) {
      if (Serial.lines.front().find("flight log closed") != std::string::npos) logClosed = true;
      Serial.lines.pop_front();
    }
    while (!SerialUSB1.lines.empty()) {
      if (SerialUSB1.lines.front().find("imu log closed") != std::string::npos) imuClosed = true;
      SerialUSB1.lines.pop_front();
    }
  }
  ::printf("host: finished %.1f s after go; flight log %s, imu log %s\n", (g_us - goUs) / 1e6,
           logClosed ? "closed" : "OPEN", imuClosed ? "closed" : "OPEN");
  return finished() ? 0 : 4;
}
