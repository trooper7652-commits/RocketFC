#pragma once
// PC stand-in for the Servo library: remembers the last pulse width, and (when
// a test switches g_servoLogOn on) logs every write as (ms, pin, us).
#include <vector>

#include "Arduino.h"

struct ServoWrite { uint32_t ms; int pin; int us; };
inline std::vector<ServoWrite>& servoLog() { static std::vector<ServoWrite> v; return v; }
inline bool& servoLogOn() { static bool on = false; return on; }

class Servo {
 public:
  uint8_t attach(int pin, int = 544, int = 2400) { pin_ = pin; return 1; }
  void writeMicroseconds(int us) {
    us_ = us;
    if (servoLogOn()) servoLog().push_back({millis(), pin_, us});
  }
  void write(int) {}
  void detach() {}
  bool attached() const { return pin_ >= 0; }
  int readMicroseconds() const { return us_; }
 private:
  int pin_ = -1, us_ = 1500;
};
