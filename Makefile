CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -pedantic -Iinclude

COMMON_SRC := src/Controller.cpp src/Sensor.cpp src/Plant.cpp src/Simulation.cpp
SIM_SRC := main.cpp $(COMMON_SRC)
CONTROLLER_TEST_SRC := tests/controller_tests.cpp src/Controller.cpp
SIMULATION_TEST_SRC := tests/simulation_tests.cpp $(COMMON_SRC)

.PHONY: all simulator controller_tests simulation_tests test demo run clean

all: simulator

simulator: $(SIM_SRC)
	$(CXX) $(CXXFLAGS) $(SIM_SRC) -o $@

run: simulator
	./simulator

controller_tests: $(CONTROLLER_TEST_SRC)
	$(CXX) $(CXXFLAGS) $(CONTROLLER_TEST_SRC) -o $@

simulation_tests: $(SIMULATION_TEST_SRC)
	$(CXX) $(CXXFLAGS) $(SIMULATION_TEST_SRC) -o $@

test: controller_tests simulation_tests
	./controller_tests
	./simulation_tests

demo: simulator
	./simulator nominal telemetry.csv
	python3 tools/plot_telemetry.py telemetry.csv plots/nominal_response.png

clean:
	rm -f simulator controller_tests simulation_tests telemetry.csv
