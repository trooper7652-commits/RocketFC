#pragma once
// PC stand-in for EEPROM: 4 KB of RAM, blank (0xFF) at start, like a new Teensy.
#include "Arduino.h"

class EEPROMClass {
 public:
  EEPROMClass() { memset(mem_, 0xFF, sizeof(mem_)); }
  template <class T> T& get(int addr, T& t) { memcpy(&t, mem_ + addr, sizeof(T)); return t; }
  template <class T> const T& put(int addr, const T& t) { memcpy(mem_ + addr, &t, sizeof(T)); return t; }
 private:
  uint8_t mem_[4096];
};

extern EEPROMClass EEPROM;
