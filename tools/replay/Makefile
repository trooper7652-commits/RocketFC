# PC replay harness for the RocketFC flight core.
#   make          build
#   make test     regenerate cases and run the full suite
CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra

replay: main.cpp ../../src/config.h $(wildcard ../../src/core/*.h)
	$(CXX) $(CXXFLAGS) main.cpp -o replay

cases: ../synth_flight.py ../burn_table.py
	cd .. && python synth_flight.py

test: replay cases
	./replay cases/*.csv

clean:
	rm -f replay replay.exe
