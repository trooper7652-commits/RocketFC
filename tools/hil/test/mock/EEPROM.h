#pragma once
// PC stand-in for EEPROM: 4 KB of RAM, blank (0xFF) at start, like a new Teensy.
// attach(path) makes it persistent: loaded from the file now, saved on every
// write, so tools/hil/test/host_live.cpp keeps it across a HIL `reboot`
// (a fresh host process), the way the Teensy's EEPROM survives a reset.
#include <string>

#include "Arduino.h"

class EEPROMClass {
 public:
  EEPROMClass() { memset(mem_, 0xFF, sizeof(mem_)); }
  template <class T> T& get(int addr, T& t) { memcpy(&t, mem_ + addr, sizeof(T)); return t; }
  template <class T> const T& put(int addr, const T& t) {
    memcpy(mem_ + addr, &t, sizeof(T));
    save();
    return t;
  }
  uint8_t read(int addr) { return mem_[addr]; }
  void write(int addr, uint8_t v) {
    mem_[addr] = v;
    save();
  }
  void update(int addr, uint8_t v) {
    if (mem_[addr] != v) write(addr, v);
  }
  uint16_t length() const { return (uint16_t)sizeof(mem_); }

  void attach(const std::string& path) {
    path_ = path;
    if (FILE* f = fopen(path_.c_str(), "rb")) {
      size_t n = fread(mem_, 1, sizeof(mem_), f);
      (void)n;
      fclose(f);
    }
  }

 private:
  void save() {
    if (path_.empty()) return;
    if (FILE* f = fopen(path_.c_str(), "wb")) {
      fwrite(mem_, 1, sizeof(mem_), f);
      fclose(f);
    }
  }
  uint8_t mem_[4096];
  std::string path_;
};

extern EEPROMClass EEPROM;
