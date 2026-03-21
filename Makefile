CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -pedantic -Iinclude

COMMON_SRC := src/Controller.cpp src/Sensor.cpp src/Plant.cpp src/Simulation.cpp
SIM_SRC := main.cpp $(COMMON_SRC)
CONTROLLER_TEST_SRC := tests/controller_tests.cpp src/Controller.cpp
SIMULATION_TEST_SRC := tests/simulation_tests.cpp $(COMMON_SRC)

.PHONY: all run test clean

all: simulator

simulator: $(SIM_SRC)
	$(CXX) $(CXXFLAGS) $(SIM_SRC) -o $@

run: simulator
	./simulator

test: controller_tests simulation_tests
	./controller_tests
	./simulation_tests

controller_tests: $(CONTROLLER_TEST_SRC)
	$(CXX) $(CXXFLAGS) $(CONTROLLER_TEST_SRC) -o $@

simulation_tests: $(SIMULATION_TEST_SRC)
	$(CXX) $(CXXFLAGS) $(SIMULATION_TEST_SRC) -o $@

clean:
	rm -f simulator controller_tests simulation_tests
