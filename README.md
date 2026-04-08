# UAV Flight Control Simulator

This project is a small C++17 UAV altitude-control simulator built to show clean engineering, not just control math. It models a vertical flight loop with realistic effects (lag, saturation, disturbances) and produces deterministic outputs you can test and visualize. In short: it helps demonstrate how a controller behaves under both normal and faulty conditions.

## Quick Demo

Fastest way to see the project working:

```bash
make demo
```

This command builds the simulator, runs a nominal scenario to generate `telemetry.csv`, and creates `plots/nominal_response.png`. It is the quickest end-to-end check for reviewers and recruiters.

## What problem this solves

Simple control demos often show only a happy path and are hard to validate. This simulator provides a repeatable way to test altitude tracking, inject common faults, and inspect results through metrics and plots.

## Key Features

- Modular C++17 design (`Controller`, `Plant`, `Sensor`, `SimulationRunner`)
- Deterministic simulation runs (fixed seed)
- Fault scenarios: nominal, sensor dropout, stuck sensor, high-noise burst, actuator degradation
- Closed-loop performance metrics (MAE, RMS error, overshoot, settling time)
- CSV telemetry export for plotting and review
- Lightweight Python plotting script for quick visual checks

## Quick Start

```bash
make demo
```

## Visualization

1. Generate telemetry:

```bash
./simulator nominal telemetry.csv
```

2. Generate the figure:

```bash
python3 tools/plot_telemetry.py telemetry.csv plots/nominal_response.png
```

Output file:

```text
plots/nominal_response.png
```

The top panel compares target altitude and true altitude, so tracking performance is clear at a glance. The lower panels show vertical velocity and actual thrust, which makes controller/actuator behavior easy to explain in interviews.

## Scenarios

```bash
./simulator nominal
./simulator sensor_dropout
./simulator stuck_sensor
./simulator high_noise_burst
./simulator actuator_degradation
```

## Build and Test

```bash
make simulator
make controller_tests
make simulation_tests
make test
make clean
```
