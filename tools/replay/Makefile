# PC replay harness for the RocketFC flight core.
#   make          build
#   make test     regenerate cases and run the full suite
CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra

# Some MSYS2/Windows setups leave TMP/TEMP unset inside make recipes, so g++
# falls back to C:\WINDOWS for its temp files and fails ("Permission denied").
# Point the compiler's temp dir at this build directory (always writable). The
# native (ucrt64) g++ needs a Windows-style path, so convert via cygpath when
# it exists; on Linux/macOS cygpath is absent and TMPDIR is left untouched.
CYGPATH := $(shell command -v cygpath 2>/dev/null)
ifneq ($(CYGPATH),)
WINTMP := $(shell cygpath -w "$(CURDIR)")
export TMPDIR := $(WINTMP)
export TMP := $(WINTMP)
export TEMP := $(WINTMP)
endif

replay: main.cpp ../../src/config.h $(wildcard ../../src/core/*.h)
	$(CXX) $(CXXFLAGS) main.cpp -o replay

cases: ../synth_flight.py ../burn_table.py
	cd .. && python synth_flight.py

test: replay cases
	./replay cases/*.csv

clean:
	rm -f replay replay.exe
