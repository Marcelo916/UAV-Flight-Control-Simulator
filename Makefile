CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -pedantic -Iinclude

SIM_SRC := main.cpp src/Controller.cpp src/Sensor.cpp
TEST_SRC := tests/controller_tests.cpp src/Controller.cpp src/Sensor.cpp

.PHONY: all run test clean

all: simulator

simulator: $(SIM_SRC)
	$(CXX) $(CXXFLAGS) $(SIM_SRC) -o $@

run: simulator
	./simulator

test: controller_tests
	./controller_tests

controller_tests: $(TEST_SRC)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) -o $@

clean:
	rm -f simulator controller_tests
