#pragma once
// PC stand-in for the Servo library: remembers the last pulse width.
#include "Arduino.h"

class Servo {
 public:
  uint8_t attach(int pin, int = 544, int = 2400) { pin_ = pin; return 1; }
  void writeMicroseconds(int us) { us_ = us; }
  void write(int) {}
  void detach() {}
  bool attached() const { return pin_ >= 0; }
  int readMicroseconds() const { return us_; }
 private:
  int pin_ = -1, us_ = 1500;
};
