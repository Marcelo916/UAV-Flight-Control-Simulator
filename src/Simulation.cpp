#include "Simulation.h"

#include <cmath>
#include <limits>
#include <random>

SimulationResult SimulationRunner::run(const SimulationConfig& config) const {
    Controller controller(config.targetAltitude, config.controllerGain);
    Plant plant(config.initialAltitude,
                config.initialVerticalVelocity,
                config.gravity,
                config.maxThrustAcceleration,
                config.actuatorTimeConstant,
                config.minThrust,
                config.maxThrust,
                config.hoverThrust,
                config.verticalDamping,
                config.disturbanceSigma);
    Sensor sensor(config.initialAltitude);

    std::mt19937 rng(config.randomSeed);
    std::normal_distribution<double> disturbanceDist(0.0, plant.disturbanceAmplitude());

    SimulationResult result;
    result.config = config;

    const std::size_t stepCount = static_cast<std::size_t>(config.durationSeconds / config.dtSeconds);
    result.samples.reserve(stepCount + 1);

    for (std::size_t step = 0; step <= stepCount; ++step) {
        const double timeSeconds = step * config.dtSeconds;
        const double measuredAltitude = sensor.readAltitude();
        const auto [error, controlCommand] = controller.evaluate(measuredAltitude);
        const double disturbance = (step == stepCount) ? 0.0 : disturbanceDist(rng);
        const PlantState currentState = plant.state();

        result.samples.push_back({timeSeconds,
                                  config.targetAltitude,
                                  measuredAltitude,
                                  currentState.altitude,
                                  currentState.verticalVelocity,
                                  error,
                                  controlCommand,
                                  currentState.actualThrust,
                                  disturbance});

        if (step == stepCount) {
            break;
        }

        const PlantState updatedState = plant.step(controlCommand, disturbance, config.dtSeconds);
        sensor.setAltitude(updatedState.altitude);
    }

    result.metrics = computeMetrics(result.samples, config.targetAltitude);
    return result;
}

PerformanceMetrics computeMetrics(const std::vector<SimulationSample>& samples,
                                  double targetAltitude,
                                  double settlingBandFraction) {
    PerformanceMetrics metrics;
    if (samples.empty()) {
        return metrics;
    }

    double absErrorSum = 0.0;
    double squaredErrorSum = 0.0;
    double maxAbsError = 0.0;
    double maxAltitude = -std::numeric_limits<double>::infinity();

    for (const auto& sample : samples) {
        const double absError = std::fabs(sample.error);
        absErrorSum += absError;
        squaredErrorSum += sample.error * sample.error;
        maxAbsError = std::max(maxAbsError, absError);
        maxAltitude = std::max(maxAltitude, sample.trueAltitude);
    }

    const auto& finalSample = samples.back();
    metrics.meanAbsoluteError = absErrorSum / static_cast<double>(samples.size());
    metrics.rmsError = std::sqrt(squaredErrorSum / static_cast<double>(samples.size()));
    metrics.maxAbsoluteError = maxAbsError;
    metrics.overshoot = std::max(0.0, maxAltitude - targetAltitude);
    metrics.finalAltitude = finalSample.trueAltitude;
    metrics.finalError = finalSample.error;

    const double settlingBand = std::max(1.0, std::fabs(targetAltitude) * settlingBandFraction);
    metrics.settlingTimeSeconds = finalSample.timeSeconds;

    for (std::size_t i = 0; i < samples.size(); ++i) {
        bool settled = true;
        for (std::size_t j = i; j < samples.size(); ++j) {
            if (std::fabs(samples[j].error) > settlingBand) {
                settled = false;
                break;
            }
        }

        if (settled) {
            metrics.settlingTimeSeconds = samples[i].timeSeconds;
            break;
        }
    }

    return metrics;
}
