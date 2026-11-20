// PC host for tools/hil/test_wiggle.py: boots the GENERATED HIL sketch on the
// simulated clock, sends `chute lock`, `chute wiggle` (disarmed), then `arm`
// and `chute wiggle` again (must be refused), and prints every chute-servo
// pulse the mock Servo saw as "SERVO <ms> <us>" lines, plus the console.
//
// The CLI's blocking loops only feed the watchdog (a no-op on the PC), so the
// host replaces that hook with one that advances the simulated clock.
//
// usage: host_wiggle <sd dir>
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
uint32_t g_baroD1 = 9085466;
std::string g_sdRoot;
HostSerial Serial("con");
HostSerial SerialUSB1("hil");
TwoWire Wire;
SDClass SD;
EEPROMClass EEPROM;

#include "RocketFC_HIL.ino"

void hostReboot() { exit(5); }
void hostPinChanged(int, int) {}

static std::mt19937 rng(12345);
static std::normal_distribution<float> unit(0.0f, 1.0f);
static std::uniform_int_distribution<int> baroNoise(-80, 80);

static void stepOnce() {
  const float* R = cfg::IMU_R_SB;
  for (int i = 0; i < 3; ++i) {
    g_imuAccel[i] = cfg::G0 * R[6 + i] + 0.02f * unit(rng);
    g_imuGyro[i] = 0.0005f * unit(rng);
  }
  g_baroD1 = 9085466 + baroNoise(rng);
  loop();
  g_us += 20;
}

static void hostWdogFeed() { g_us += 20; }  // time passes inside CLI busy loops

static bool runUntil(double maxS, const char* needle) {
  const uint64_t end = g_us + (uint64_t)(maxS * 1e6);
  while (g_us < end) {
    stepOnce();
    while (!Serial.lines.empty()) {
      const std::string ln = Serial.lines.front();
      Serial.lines.pop_front();
      if (needle && ln.find(needle) != std::string::npos) return true;
    }
  }
  return needle == nullptr;
}

int main(int argc, char** argv) {
  if (argc < 2) { ::printf("usage: host_wiggle <sd dir>\n"); return 1; }
  g_sdRoot = argv[1];
  const int contRaw = (int)(8.0f / cfg::CONT_DIVIDER_RATIO / 3.3f * 1023.0f);
  g_analog[cfg::PIN_CONT_LAND] = contRaw;
  g_analog[cfg::PIN_CONT_LEGS] = contRaw;
  g_analog[cfg::PIN_VBAT] = (int)(7.8f / cfg::VBAT_DIVIDER / 3.3f * 1023.0f);

  setup();
  cliCtx.wdogFeed = &hostWdogFeed;
  runUntil(1.0, nullptr);
  servoLogOn() = true;

  Serial.push("chute lock\n");
  runUntil(0.5, nullptr);
  ::printf("MARK wiggle_start\n");
  servoLog().push_back({millis(), -1, -1});  // marker row: pin -1
  Serial.push("chute wiggle\n");
  if (!runUntil(5.0, "done.")) { ::printf("host: wiggle never finished\n"); return 2; }
  ::printf("MARK wiggle_end\n");
  servoLog().push_back({millis(), -2, -2});

  // Armed: must refuse. Needs the chute locked, then `arm`.
  Serial.push("chute lock\n");
  runUntil(0.5, nullptr);
  Serial.push("arm\n");
  if (!runUntil(30.0, "ARMED")) { ::printf("host: ARMED never seen\n"); return 3; }
  runUntil(1.0, nullptr);
  servoLog().push_back({millis(), -3, -3});
  Serial.push("chute wiggle\n");
  runUntil(1.0, nullptr);
  servoLog().push_back({millis(), -4, -4});

  for (const auto& w : servoLog())
    if (w.pin == cfg::PIN_SERVO_CHUTE || w.pin < 0)
      ::printf("SERVO %u %d %d\n", (unsigned)w.ms, w.pin, w.us);
  fflush(stdout);
  return 0;
}
