# UAV Flight Control Simulator

A compact C++17 simulator demonstrating a proportional UAV altitude controller running in a periodic control loop with concurrent sensor noise injection.

The project models a simple altitude sensor, a controller that computes throttle from target error, and multi-threaded execution to show how control and disturbances interact over time.
