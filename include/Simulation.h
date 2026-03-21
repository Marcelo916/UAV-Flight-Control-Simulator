#pragma once
#include <cstddef>
#include <vector>

#include "Controller.h"
#include "Plant.h"
#include "Sensor.h"

struct SimulationConfig {
    double targetAltitude = 1000.0;
    double initialAltitude = 980.0;
    double initialVerticalVelocity = 0.0;
    double controllerGain = 0.05;
    double gravity = 9.81;
    double maxThrustAcceleration = 15.0;
    double actuatorTimeConstant = 0.35;
    double minThrust = 0.0;
    double maxThrust = 1.0;
    double hoverThrust = -1.0;
    double verticalDamping = 1.2;
    double disturbanceSigma = 0.4;
    double durationSeconds = 15.0;
    double dtSeconds = 0.1;
    unsigned randomSeed = 7;
};

struct SimulationSample {
    double timeSeconds = 0.0;
    double targetAltitude = 0.0;
    double measuredAltitude = 0.0;
    double trueAltitude = 0.0;
    double verticalVelocity = 0.0;
    double error = 0.0;
    double controlCommand = 0.0;
    double actualThrust = 0.0;
    double disturbance = 0.0;
};

struct PerformanceMetrics {
    double meanAbsoluteError = 0.0;
    double rmsError = 0.0;
    double maxAbsoluteError = 0.0;
    double overshoot = 0.0;
    double settlingTimeSeconds = 0.0;
    double finalAltitude = 0.0;
    double finalError = 0.0;
};

struct SimulationResult {
    SimulationConfig config;
    PerformanceMetrics metrics;
    std::vector<SimulationSample> samples;
};

class SimulationRunner {
public:
    SimulationResult run(const SimulationConfig& config) const;
};

PerformanceMetrics computeMetrics(const std::vector<SimulationSample>& samples,
                                  double targetAltitude,
                                  double settlingBandFraction = 0.02);
