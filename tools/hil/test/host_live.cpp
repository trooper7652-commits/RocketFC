// Runs the GENERATED HIL sketch (tools/hil/build/RocketFC_HIL) on the PC in
// REAL TIME, with its two USB serial ports bridged to TCP sockets, so the
// closed-loop bench sim (tools/hil/live/) can be tested end to end without a
// Teensy:
//
//   console     (Serial)      127.0.0.1:<port>
//   HIL channel (SerialUSB1)  127.0.0.1:<port + 1>
//
// tools/hil/live/link.py opens them as socket://127.0.0.1:<port>. The "real"
// IMU and barometer sit still (gravity + a little noise), both pyro channels
// read an intact load on 7.8 V, and every change of the two pyro gate pins
// prints on stdout ("[t host] landing gate ON" ...), so tools/hil/test_live.py
// can check that LIVE mode never drives the landing gate and drives the leg
// gate only with live legs. Time is the PC's wall clock.
//
// usage: host_live <sd dir> <port> [max seconds]
// Waits (up to 20 s) for both clients before booting the sketch, so the
// console client sees the boot report. Exits when the HIL client disconnects,
// on `reboot`, or after max seconds. The console output is also written to
// <sd dir>/host_console.txt.
#include <chrono>
#include <random>
#include <thread>

#include "Arduino.h"
#include "BMI088.h"
#include "EEPROM.h"
#include "SD.h"
#include "Servo.h"
#include "Wire.h"
#include "net.h"

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

static net_sock g_con = NET_NONE, g_hil = NET_NONE;
static FILE* g_conLog = nullptr;

static void sinkCon(const uint8_t* b, size_t n, void*) {
  if (g_con != NET_NONE) net_send(g_con, b, (int)n);
  if (g_conLog) fwrite(b, 1, n, g_conLog);
}
static void sinkHil(const uint8_t* b, size_t n, void*) {
  if (g_hil != NET_NONE) net_send(g_hil, b, (int)n);
}

void hostPinChanged(int pin, int v) {
  if (pin == cfg::PIN_PYRO_LAND)
    ::printf("[%9.3f host] landing gate %s\n", g_us / 1e6, v ? "ON" : "OFF");
  if (pin == cfg::PIN_PYRO_LEGS)
    ::printf("[%9.3f host] legs gate %s\n", g_us / 1e6, v ? "ON" : "OFF");
  fflush(stdout);
}

void hostReboot() {  // HIL `reboot`: the caller starts a fresh process
  ::printf("host: reboot\n");
  fflush(stdout);
  if (g_conLog) fclose(g_conLog);
  exit(0);
}

static std::mt19937 rng(12345);
static std::normal_distribution<float> unit(0.0f, 1.0f);
static std::uniform_int_distribution<int> baroNoise(-80, 80);

static void stillSensors() {
  // Still bench: gravity along body +z, in the sensor frame (sensor = R^T body).
  const float* R = cfg::IMU_R_SB;
  for (int i = 0; i < 3; ++i) {
    g_imuAccel[i] = cfg::G0 * R[6 + i] + 0.02f * unit(rng);
    g_imuGyro[i] = 0.0005f * unit(rng);
  }
  g_baroD1 = 9085466 + baroNoise(rng);
}

int main(int argc, char** argv) {
  if (argc < 3) {
    ::printf("usage: host_live <sd dir> <port> [max seconds]\n");
    return 1;
  }
  g_sdRoot = argv[1];
  const int port = atoi(argv[2]);
  const double maxS = argc > 3 ? atof(argv[3]) : 0.0;
  g_conLog = fopen((g_sdRoot + "/host_console.txt").c_str(), "wb");

  if (!net_init()) { ::printf("host: socket init failed\n"); return 1; }
  const net_sock lc = net_listen(port), lh = net_listen(port + 1);
  if (lc == NET_NONE || lh == NET_NONE) {
    ::printf("host: can't listen on 127.0.0.1:%d/%d\n", port, port + 1);
    return 1;
  }
  const int contRaw = (int)(8.0f / cfg::CONT_DIVIDER_RATIO / 3.3f * 1023.0f);
  g_analog[cfg::PIN_CONT_LAND] = contRaw;
  g_analog[cfg::PIN_CONT_LEGS] = contRaw;
  g_analog[cfg::PIN_VBAT] = (int)(7.8f / cfg::VBAT_DIVIDER / 3.3f * 1023.0f);
  Serial.rawSink = sinkCon;
  SerialUSB1.rawSink = sinkHil;
  ::printf("LISTENING %d %d\n", port, port + 1);
  fflush(stdout);

  using clk = std::chrono::steady_clock;
  const auto t0 = clk::now();
  auto wallUs = [&] {
    return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(clk::now() - t0).count();
  };
  while ((g_con == NET_NONE || g_hil == NET_NONE) && wallUs() < 20000000ull) {
    if (g_con == NET_NONE) g_con = net_accept(lc);
    if (g_hil == NET_NONE) g_hil = net_accept(lh);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (g_hil == NET_NONE) { ::printf("host: no HIL client\n"); return 2; }

  stillSensors();
  setup();  // its delay()s advance g_us; the wall clock takes over below
  const uint64_t w0 = wallUs();
  const uint64_t off = g_us > w0 ? g_us - w0 : 0;

  char buf[4096];
  uint64_t lastPollUs = 0;
  while (true) {
    const uint64_t w = wallUs() + off;
    if (w > g_us) g_us = w;
    stillSensors();
    loop();
    if (g_us - lastPollUs >= 50) {
      lastPollUs = g_us;
      if (g_con == NET_NONE) g_con = net_accept(lc);
      if (g_con != NET_NONE) {
        const int k = net_recv(g_con, buf, sizeof(buf));
        if (k > 0) Serial.push(buf, (size_t)k);
        else if (k < 0) { net_close(g_con); g_con = NET_NONE; }
      }
      const int k = net_recv(g_hil, buf, sizeof(buf));
      if (k > 0) {
        SerialUSB1.push(buf, (size_t)k);
      } else if (k < 0) {
        ::printf("host: HIL client closed at %.3f s\n", g_us / 1e6);
        break;
      }
      if (maxS > 0 && g_us > (uint64_t)(maxS * 1e6)) {
        ::printf("host: max time reached\n");
        break;
      }
    }
    std::this_thread::yield();
  }
  if (g_conLog) fclose(g_conLog);
  fflush(stdout);
  return 0;
}
