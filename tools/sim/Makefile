# Builds rocketfc_core.dll: a shared-library C ABI over src/core/flight_core.h
# (see bridge.cpp) so tools/sim/core.py can drive the exact flight code via
# ctypes.
#
#   make            build tools/sim/rocketfc_core.dll against the repo's
#                   own ../../src
#   make clean
#
# core.py's config-sweep build manager does NOT use this Makefile: to sweep a
# firmware constexpr without touching the real src/ tree, it mirrors this
# directory's layout (tools/sim/bridge.cpp beside a patched src/ two levels
# up) under build/<hash>/ and invokes g++ directly, so bridge.cpp's
# "../../src/core/flight_core.h" include resolves into the patched copy
# unmodified. See core.py's _build() for the exact command line (same
# CXXFLAGS and TMPDIR workaround as here).
CXX ?= g++
# -static (full, not just -static-libgcc/-libstdc++): MSYS2 g++ also
# dynamically links libwinpthread-1.dll by default, which ctypes.CDLL can't
# resolve from a Python install that doesn't live next to it (Python 3.8+
# no longer searches PATH/CWD for a loaded DLL's own dependencies) -- see
# core.py's _CXXFLAGS comment for the full explanation.
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -shared -static

# Same MSYS2/Windows TMP workaround as tools/replay/Makefile: g++ falls back
# to C:\WINDOWS for temp files and fails ("Permission denied") if TMP/TEMP are
# unset inside a make recipe. Point it at this directory instead.
CYGPATH := $(shell command -v cygpath 2>/dev/null)
ifneq ($(CYGPATH),)
WINTMP := $(shell cygpath -w "$(CURDIR)")
export TMPDIR := $(WINTMP)
export TMP := $(WINTMP)
export TEMP := $(WINTMP)
endif

rocketfc_core.dll: bridge.cpp ../../src/config.h $(wildcard ../../src/core/*.h)
	$(CXX) $(CXXFLAGS) bridge.cpp -o rocketfc_core.dll

clean:
	rm -f rocketfc_core.dll

.PHONY: clean
