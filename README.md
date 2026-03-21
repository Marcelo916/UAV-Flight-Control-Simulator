# UAV Flight Control Simulator

A compact C++17 altitude-control simulator that now separates the **controller**, **plant dynamics**, **sensor**, and **simulation runner** into testable modules.

The project models a proportional altitude controller, a 1D vertical-dynamics plant, deterministic closed-loop simulation, and disturbance injection. It also reports controls-oriented performance metrics such as mean absolute error, RMS error, overshoot, and settling time.

## Why this version is stronger

- **Cleaner flight-software style architecture**: the controller computes commands, the plant owns the state, and the simulation runner orchestrates execution.
- **More aerospace-like plant model**: altitude and vertical velocity are explicit states, and motion comes from thrust, gravity, simple vertical damping, actuator lag, and disturbance acceleration.
- **Deterministic simulation**: a fixed seed makes runs reproducible for debugging and testing.
- **Measurable closed-loop results**: the simulator prints metrics you can discuss in a portfolio or interview.
- **Focused testability**: controller and simulation metrics are tested independently.

## Dynamics assumptions

The plant is intentionally simple enough to explain in an interview:

- State = altitude, vertical velocity, and actual actuator thrust.
- The controller output is treated as a normalized thrust command around a hover trim.
- Actual thrust follows the command through a first-order actuator lag.
- Net vertical acceleration is computed from thrust acceleration minus gravity, plus disturbance acceleration, with a simple linear vertical-damping term.
- Altitude is obtained by integrating vertical velocity, rather than updating altitude directly.

This keeps the simulator lightweight while making it much closer to a real flight-dynamics/control-loop discussion.

## Project structure

- `include/Controller.h`, `src/Controller.cpp`: proportional altitude controller that computes a normalized control command from measured altitude.
- `include/Plant.h`, `src/Plant.cpp`: 1D vertical UAV plant with altitude, vertical velocity, thrust saturation, and actuator lag.
- `include/Sensor.h`, `src/Sensor.cpp`: sensor abstraction for altitude measurement.
- `include/Simulation.h`, `src/Simulation.cpp`: deterministic closed-loop runner, telemetry collection, and performance metrics.
- `tests/controller_tests.cpp`: unit tests for controller behavior.
- `tests/simulation_tests.cpp`: tests for plant dynamics, actuator behavior, and end-to-end convergence.

## Build and run

```bash
make simulator
./simulator
```

## Run tests

```bash
make test
```

The simulator logs representative controller samples and prints a performance summary with:
- final altitude and final error
- mean absolute error
- RMS error
- maximum absolute error
- overshoot
- 2% settling time
