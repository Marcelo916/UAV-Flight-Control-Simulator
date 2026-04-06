# UAV Flight Control Simulator

A compact C++17 vertical-flight simulator with a modular architecture designed to look and feel like an entry-level controls/flight-software project rather than a classroom-only demo.

## Architecture

The code is intentionally split into clean layers:

- `Controller`: proportional altitude controller that maps altitude error to a normalized command in `[-1, 1]`.
- `Plant`: 1D vertical dynamics with altitude, vertical velocity, thrust state, gravity, damping, saturation, and actuator lag.
- `Sensor`: thread-safe altitude measurement abstraction.
- `SimulationRunner`: deterministic closed-loop orchestration, telemetry collection, and metrics computation.

Files:

- `include/Controller.h`, `src/Controller.cpp`
- `include/Plant.h`, `src/Plant.cpp`
- `include/Sensor.h`, `src/Sensor.cpp`
- `include/Simulation.h`, `src/Simulation.cpp`
- `main.cpp`

## Dynamics assumptions (simple but interview-friendly)

The vertical model is intentionally lightweight and explainable:

- State: altitude `z`, vertical velocity `vz`, and actual thrust `u_actual`.
- Command: controller outputs normalized command `u_cmd` in `[-1,1]`.
- Thrust mapping: `u_cmd` is added around hover trim and clamped by thrust limits.
- Actuator lag: first-order response so thrust does not change instantly.
- Acceleration: net vertical acceleration comes from thrust, gravity, linear damping, and disturbance acceleration.
- Integration: update `vz`, then update `z` from `vz`.

## Metrics reported

`SimulationRunner` computes and reports:

- mean absolute error (MAE)
- RMS error
- maximum absolute error
- overshoot
- settling time (2% band)
- final altitude and final error

## Deterministic fault scenarios

All scenarios are deterministic with fixed seeds (`SimulationConfig::randomSeed`) and can be selected from CLI:

- `nominal`
- `sensor_dropout` (holds previous measured value in fault window)
- `stuck_sensor` (locks measurement after fault onset)
- `high_noise_burst` (adds large temporary sensor noise)
- `actuator_degradation` (reduces actuator effectiveness after fault onset)

Default fault window: 5.0s to 8.0s.

## CSV telemetry export

You can optionally export telemetry to CSV for plotting:

```bash
./simulator nominal telemetry.csv
```

CSV includes:

- scenario name
- time
- target altitude
- measured altitude
- true altitude
- vertical velocity
- error
- control command
- actual thrust
- disturbance

## Visualization

This repo includes a lightweight plotting script to turn telemetry CSV output into a recruiter-friendly figure.

Generate telemetry from the simulator:

```bash
make simulator
./simulator nominal telemetry.csv
```

Generate the PNG figure:

```bash
python3 tools/plot_telemetry.py telemetry.csv plots/nominal_response.png
```

After running the command above, your plot will be written to:

```
plots/nominal_response.png
```

The top panel shows target altitude versus true altitude so reviewers can quickly see tracking behavior and convergence.
The lower panels show vertical velocity and actual thrust over time, which helps communicate actuator response and vehicle motion without requiring deep controls background.
Together, the figure makes simulation results easier to evaluate in interviews and portfolio reviews.

## Build and run commands

Build simulator:

```bash
make simulator
```

Run simulator (nominal default):

```bash
./simulator
```

Run a specific scenario:

```bash
./simulator sensor_dropout
./simulator stuck_sensor
./simulator high_noise_burst
./simulator actuator_degradation
```

Build and run tests:

```bash
make controller_tests
make simulation_tests
make test
```

Clean artifacts:

```bash
make clean
```
