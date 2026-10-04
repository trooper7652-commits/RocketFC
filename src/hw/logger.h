#pragma once
//
// SD flight logger (Teensy 4.1 built-in microSD).
//
// One CSV per flight: FLIGHTS/flight_NNN.csv, opened when the vehicle ARMs
// (counter persisted in EEPROM by ConfigStore), closed at TOUCHDOWN or
// disarm. 100 Hz data rows plus immediate event rows (state transitions,
// pyro fires, chute releases, aborts) in the same file — the `evt` column is
// empty on data rows. Writes are buffered by the filesystem; we flush every
// LOG_FLUSH_MS so a hard crash loses at most half a second.
//
// Logging failures never affect flight logic — the vehicle flies, log or no.
//
#include <Arduino.h>
#include <SD.h>

#include "../config.h"

struct LogFrame {
  uint32_t ms = 0;
  const char* state = "";
  float ax = 0, ay = 0, az = 0;        // body specific force, m/s^2
  float gx = 0, gy = 0, gz = 0;        // body rates, rad/s
  float baroAlt = 0, baroPress = 0;
  float qw = 1, qx = 0, qy = 0, qz = 0;
  float tiltDeg = 0;
  float kfAlt = 0, kfVel = 0, kfBias = 0, innov = 0;
  float pX = 0, iX = 0, dX = 0, pY = 0, iY = 0, dY = 0;  // PID terms, rad
  float gimXDeg = 0, gimYDeg = 0;
  float usA = 1500, usB = 1500;
  uint8_t pyroFlags = 0;  // bit0 chute latch at RELEASE, bit1 land gate, bit2 legs nichrome
  uint8_t cont = 0;       // bit0 chute canopy detected, bit1 land, bit2 legs continuity
  float vbat = 0;
  uint32_t loopMaxUs = 0;              // worst fast-loop period since last row
};

class Logger {
 public:
  bool begin() {
    sdOk_ = SD.begin(BUILTIN_SDCARD);
    if (sdOk_ && !SD.exists("FLIGHTS")) SD.mkdir("FLIGHTS");
    return sdOk_;
  }

  bool sdOk() const { return sdOk_; }
  bool isOpen() const { return fileOpen_; }
  int flightNumber() const { return flightNum_; }

  bool openFlight(int number) {
    if (!sdOk_) return false;
    close();
    char name[32];
    snprintf(name, sizeof(name), "FLIGHTS/flight_%03d.csv", number);
    file_ = SD.open(name, FILE_WRITE);
    if (!file_) return false;
    flightNum_ = number;
    fileOpen_ = true;
    file_.println(
        "t_ms,state,ax,ay,az,gx,gy,gz,baro_alt,baro_pa,qw,qx,qy,qz,tilt_deg,"
        "kf_alt,kf_vel,kf_bias,innov,pX,iX,dX,pY,iY,dY,gimx_deg,gimy_deg,"
        "servoA_us,servoB_us,pyro,cont,vbat,loop_max_us,evt");
    return true;
  }

  void logRow(const LogFrame& f) {
    if (!fileOpen_) return;
    char buf[512];
    const int n = snprintf(
        buf, sizeof(buf),
        "%lu,%s,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.1f,"
        "%.4f,%.4f,%.4f,%.4f,%.2f,"
        "%.2f,%.2f,%.3f,%.2f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,"
        "%.0f,%.0f,%u,%u,%.2f,%lu,",
        (unsigned long)f.ms, f.state, f.ax, f.ay, f.az, f.gx, f.gy, f.gz,
        f.baroAlt, f.baroPress, f.qw, f.qx, f.qy, f.qz, f.tiltDeg, f.kfAlt,
        f.kfVel, f.kfBias, f.innov, f.pX, f.iX, f.dX, f.pY, f.iY, f.dY,
        f.gimXDeg, f.gimYDeg, f.usA, f.usB, (unsigned)f.pyroFlags,
        (unsigned)f.cont, f.vbat, (unsigned long)f.loopMaxUs);
    if (n > 0) file_.println(buf);
  }

  void logEvent(uint32_t ms, const char* name, float value) {
    if (!fileOpen_) return;
    char buf[96];
    // Event rows: t_ms, then empty data columns, evt in the last column.
    snprintf(buf, sizeof(buf),
             "%lu,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,EVT:%s:%.2f",
             (unsigned long)ms, name, value);
    file_.println(buf);
    dirty_ = true;
  }

  void flushTick(uint32_t ms) {
    if (!fileOpen_) return;
    if (ms - lastFlushMs_ >= (uint32_t)cfg::LOG_FLUSH_MS) {
      file_.flush();
      lastFlushMs_ = ms;
    }
  }

  void close() {
    if (fileOpen_) {
      file_.flush();
      file_.close();
      fileOpen_ = false;
    }
  }

 private:
  File file_;
  bool sdOk_ = false, fileOpen_ = false, dirty_ = false;
  int flightNum_ = 0;
  uint32_t lastFlushMs_ = 0;
};
