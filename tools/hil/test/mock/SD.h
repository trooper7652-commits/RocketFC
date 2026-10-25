#pragma once
// PC stand-in for the Teensy SD library: files live under g_sdRoot.
#include <sys/stat.h>

#include "Arduino.h"

#define BUILTIN_SDCARD 254
#define FILE_WRITE 1

extern std::string g_sdRoot;

class File : public Print {
 public:
  File() {}
  explicit File(FILE* fp) : fp_(fp) {}
  explicit operator bool() const { return fp_ != nullptr; }
  size_t write(const uint8_t* b, size_t n) override { return fp_ ? fwrite(b, 1, n, fp_) : 0; }
  using Print::write;
  void flush() { if (fp_) fflush(fp_); }
  void close() { if (fp_) { fclose(fp_); fp_ = nullptr; } }
 private:
  FILE* fp_ = nullptr;
};

class SDClass {
 public:
  bool begin(int) { return true; }
  bool exists(const char* p) {
    struct stat st;
    return stat(path(p).c_str(), &st) == 0;
  }
  bool mkdir(const char* p) { return ::mkdir(path(p).c_str()) == 0; }
  File open(const char* p, int) { return File(fopen(path(p).c_str(), "ab")); }
 private:
  static std::string path(const char* p) { return g_sdRoot + "/" + p; }
};

extern SDClass SD;
