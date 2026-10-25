#pragma once
// PC stand-in for the Teensy Arduino core, just enough to compile and run the
// generated HIL sketch (tools/hil/build/RocketFC_HIL) under tools/hil/test.
// Time is simulated: host_main.cpp advances g_us between loop() calls.
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>

#define USB_DUAL_SERIAL 1
#define DMAMEM
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

using std::max;
using std::min;

extern uint64_t g_us;          // simulated time
extern int g_pinOut[64];       // last digitalWrite per pin
extern int g_analog[64];       // analogRead value per pin (10-bit)
void hostPinChanged(int pin, int value);  // host_main.cpp hook

inline uint32_t micros() { return (uint32_t)g_us; }
inline uint32_t millis() { return (uint32_t)(g_us / 1000); }
inline void delay(uint32_t ms) { g_us += (uint64_t)ms * 1000; }
inline void delayMicroseconds(uint32_t us) { g_us += us; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int v) {
  if (pin >= 0 && pin < 64 && g_pinOut[pin] != v) {
    g_pinOut[pin] = v;
    hostPinChanged(pin, v);
  }
}
inline int digitalRead(int pin) { return (pin >= 0 && pin < 64) ? g_pinOut[pin] : 0; }
inline int analogRead(int pin) { return (pin >= 0 && pin < 64) ? g_analog[pin] : 0; }
inline void tone(int, unsigned, unsigned = 0) {}
inline void noTone(int) {}
inline void noInterrupts() {}
inline void interrupts() {}
inline void __disable_irq() {}
inline void __enable_irq() {}

template <class T, class L, class H>
inline T constrain(T x, L lo, H hi) { return x < lo ? (T)lo : (x > hi ? (T)hi : x); }

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(const uint8_t* b, size_t n) = 0;
  size_t write(uint8_t c) { return write(&c, 1); }
  size_t write(const char* s) { return write((const uint8_t*)s, strlen(s)); }
  size_t print(const char* s) { return write(s); }
  size_t println(const char* s) { return write(s) + write("\r\n"); }
  size_t println() { return write("\r\n"); }
  size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return n > 0 ? write((const uint8_t*)buf, (size_t)std::min(n, (int)sizeof(buf) - 1)) : 0;
  }
};

// A USB serial port: the harness pushes input bytes, collects output lines.
class HostSerial : public Print {
 public:
  explicit HostSerial(const char* tag) : tag_(tag) {}
  void begin(unsigned long) {}
  explicit operator bool() const { return true; }
  int available() { return (int)in.size(); }
  int read() {
    if (in.empty()) return -1;
    const int c = (uint8_t)in.front();
    in.pop_front();
    return c;
  }
  size_t readBytes(char* dst, size_t n) {
    size_t k = 0;
    while (k < n && !in.empty()) { dst[k++] = in.front(); in.pop_front(); }
    return k;
  }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; ++i) {
      const char c = (char)b[i];
      if (c == '\r') continue;
      if (c == '\n') {
        lines.push_back(cur);
        if (echo) ::printf("[%9.3f %s] %s\n", g_us / 1e6, tag_, cur.c_str());
        cur.clear();
      } else {
        cur += c;
      }
    }
    return n;
  }
  using Print::write;
  void push(const std::string& s) { in.insert(in.end(), s.begin(), s.end()); }
  void push(const void* p, size_t n) {
    const char* c = (const char*)p;
    in.insert(in.end(), c, c + n);
  }

  std::deque<char> in;
  std::deque<std::string> lines;
  bool echo = true;

 private:
  std::string cur;
  const char* tag_;
};

extern HostSerial Serial;
extern HostSerial SerialUSB1;

class elapsedMicros {
 public:
  elapsedMicros() : t0_(micros()) {}
  operator uint32_t() const { return micros() - t0_; }
  elapsedMicros& operator=(uint32_t v) { t0_ = micros() - v; return *this; }
 private:
  uint32_t t0_;
};
