# UAV Flight Control Simulator

A compact C++17 simulator demonstrating a proportional UAV altitude controller running in a periodic control loop with concurrent sensor noise injection.

The project models a simple altitude sensor, a controller that computes throttle from target error, and multi-threaded execution to show how control and disturbances interact over time.

## Build and run

```bash
make simulator
./simulator
```

## Run tests

```bash
make test
```

The test target verifies:
- zero error gives zero throttle
- large positive/negative error saturates throttle to ±1
- altitude updates match controller logic (`altitude += throttle * maxRate`)
