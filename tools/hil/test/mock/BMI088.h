#pragma once
// PC stand-in for the Bolder Flight BMI088 library. Readings come from
// g_imuAccel / g_imuGyro (sensor frame), set by host_main.cpp every tick.
#include "Wire.h"

extern float g_imuAccel[3];  // m/s^2, sensor frame
extern float g_imuGyro[3];   // rad/s, sensor frame

class Bmi088Accel {
 public:
  enum Range { RANGE_3G, RANGE_6G, RANGE_12G, RANGE_24G };
  enum Odr { ODR_400HZ_BW_40HZ };
  Bmi088Accel(TwoWire&, uint8_t addr) : addr_(addr) {}
  int begin() { return addr_ == 0x18 ? 1 : -1; }
  bool setRange(Range) { return true; }
  bool setOdr(Odr) { return true; }
  void readSensor() {}
  float getAccelX_mss() { return g_imuAccel[0]; }
  float getAccelY_mss() { return g_imuAccel[1]; }
  float getAccelZ_mss() { return g_imuAccel[2]; }
 private:
  uint8_t addr_;
};

class Bmi088Gyro {
 public:
  enum Range { RANGE_2000DPS };
  enum Odr { ODR_1000HZ_BW_116HZ };
  Bmi088Gyro(TwoWire&, uint8_t addr) : addr_(addr) {}
  int begin() { return addr_ == 0x68 ? 1 : -1; }
  bool setRange(Range) { return true; }
  bool setOdr(Odr) { return true; }
  void readSensor() {}
  float getGyroX_rads() { return g_imuGyro[0]; }
  float getGyroY_rads() { return g_imuGyro[1]; }
  float getGyroZ_rads() { return g_imuGyro[2]; }
 private:
  uint8_t addr_;
};
