#pragma once
// PC stand-in for the I2C bus with one MS5611 at 0x77 (datasheet example
// PROM, so the driver's CRC-4 check and compensation run for real).
#include "Arduino.h"

extern uint32_t g_baroD1;  // pressure ADC value, set by host_main.cpp

class TwoWire {
 public:
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t a) { addr_ = a; tx_.clear(); }
  size_t write(uint8_t b) { tx_.push_back(b); return 1; }
  uint8_t endTransmission() {
    if (addr_ != 0x77) return 2;  // NACK
    if (tx_.empty()) return 0;
    const uint8_t c = tx_[0];
    if (c == 0x46) lastConv_ = 1;       // D1 pressure
    else if (c == 0x56) lastConv_ = 2;  // D2 temperature
    else if (c >= 0xA0 && c <= 0xAE) promIdx_ = (c - 0xA0) / 2;
    lastCmd_ = c;
    return 0;
  }
  uint8_t requestFrom(uint8_t a, uint8_t n) {
    rx_.clear();
    if (a != 0x77) return 0;
    if (lastCmd_ >= 0xA0 && lastCmd_ <= 0xAE && n == 2) {
      const uint16_t v = prom()[promIdx_];
      rx_ = {(uint8_t)(v >> 8), (uint8_t)v};
    } else if (lastCmd_ == 0x00 && n == 3) {
      const uint32_t v = lastConv_ == 1 ? g_baroD1 : 8569150u;  // datasheet D2 (~20 C)
      rx_ = {(uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};
    }
    return (uint8_t)rx_.size();
  }
  int read() {
    if (rx_.empty()) return -1;
    const int v = rx_.front();
    rx_.erase(rx_.begin());
    return v;
  }

 private:
  static const uint16_t* prom() {
    static uint16_t p[8] = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};
    static bool done = false;
    if (!done) {  // the datasheet CRC-4, same algorithm as Ms5611::crc4Ok
      uint16_t q[8];
      memcpy(q, p, sizeof(q));
      uint16_t rem = 0;
      for (int i = 0; i < 16; ++i) {
        rem ^= (i % 2 == 1) ? (q[i >> 1] & 0x00FF) : (q[i >> 1] >> 8);
        for (int b = 8; b > 0; --b) rem = (rem & 0x8000) ? (rem << 1) ^ 0x3000 : (rem << 1);
      }
      p[7] = (p[7] & 0xFFF0) | ((rem >> 12) & 0xF);
      done = true;
    }
    return p;
  }
  uint8_t addr_ = 0, lastCmd_ = 0xFF, lastConv_ = 0;
  int promIdx_ = 0;
  std::basic_string<uint8_t> tx_, rx_;
};

extern TwoWire Wire;
