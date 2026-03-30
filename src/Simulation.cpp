#include "Simulation.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>

namespace {
constexpr double kFaultStartSeconds = 5.0;
constexpr double kFaultEndSeconds = 8.0;
constexpr double kBurstNoiseSigma = 8.0;
constexpr double kActuatorDegradedEffectiveness = 0.6;

bool isFaultWindow(double t) {
    return t >= kFaultStartSeconds && t <= kFaultEndSeconds;
}

bool equals(const std::string& a, const std::string& b) {
    return a == b;
}
} // namespace

const char* scenarioName(ScenarioKind scenario) {
    switch (scenario) {
        case ScenarioKind::Nominal:
            return "nominal";
        case ScenarioKind::SensorDropout:
            return "sensor_dropout";
        case ScenarioKind::StuckSensor:
            return "stuck_sensor";
        case ScenarioKind::HighNoiseBurst:
            return "high_noise_burst";
        case ScenarioKind::ActuatorDegradation:
            return "actuator_degradation";
    }
    return "nominal";
}

bool parseScenario(const std::string& text, ScenarioKind& outScenario) {
    if (equals(text, "nominal")) {
        outScenario = ScenarioKind::Nominal;
        return true;
    }
    if (equals(text, "sensor_dropout")) {
        outScenario = ScenarioKind::SensorDropout;
        return true;
    }
    if (equals(text, "stuck_sensor")) {
        outScenario = ScenarioKind::StuckSensor;
        return true;
    }
    if (equals(text, "high_noise_burst")) {
        outScenario = ScenarioKind::HighNoiseBurst;
        return true;
    }
    if (equals(text, "actuator_degradation")) {
        outScenario = ScenarioKind::ActuatorDegradation;
        return true;
    }
    return false;
}

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
    std::normal_distribution<double> sensorNoiseDist(0.0, config.sensorNoiseSigma);
    std::normal_distribution<double> burstNoiseDist(0.0, kBurstNoiseSigma);

    SimulationResult result;
    result.config = config;

    const std::size_t stepCount = static_cast<std::size_t>(config.durationSeconds / config.dtSeconds);
    result.samples.reserve(stepCount + 1);

    bool stuckInitialized = false;
    double stuckValue = sensor.readAltitude();
    double lastMeasured = sensor.readAltitude();

    for (std::size_t step = 0; step <= stepCount; ++step) {
        const double timeSeconds = step * config.dtSeconds;
        const PlantState currentState = plant.state();

        double measuredAltitude = sensor.readAltitude();
        measuredAltitude += sensorNoiseDist(rng);

        if (config.scenario == ScenarioKind::SensorDropout && isFaultWindow(timeSeconds)) {
            measuredAltitude = lastMeasured;
        }

        if (config.scenario == ScenarioKind::StuckSensor && timeSeconds >= kFaultStartSeconds) {
            if (!stuckInitialized) {
                stuckValue = measuredAltitude;
                stuckInitialized = true;
            }
            measuredAltitude = stuckValue;
        }

        if (config.scenario == ScenarioKind::HighNoiseBurst && isFaultWindow(timeSeconds)) {
            measuredAltitude += burstNoiseDist(rng);
        }

        if (config.scenario == ScenarioKind::ActuatorDegradation && timeSeconds >= kFaultStartSeconds) {
            plant.setActuatorEffectiveness(kActuatorDegradedEffectiveness);
        }

        const auto [error, controlCommand] = controller.evaluate(measuredAltitude);
        const double disturbance = (step == stepCount) ? 0.0 : disturbanceDist(rng);

        result.samples.push_back({timeSeconds,
                                  config.targetAltitude,
                                  measuredAltitude,
                                  currentState.altitude,
                                  currentState.verticalVelocity,
                                  error,
                                  controlCommand,
                                  currentState.actualThrust,
                                  disturbance});

        lastMeasured = measuredAltitude;

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

bool writeTelemetryCsv(const std::string& path, const SimulationResult& result) {
    std::ofstream out(path);
    if (!out) {
        return false;
    }

    out << "scenario,time_seconds,target_altitude,measured_altitude,true_altitude,vertical_velocity,error,control_command,actual_thrust,disturbance\n";
    out << std::fixed << std::setprecision(6);

    for (const auto& s : result.samples) {
        out << scenarioName(result.config.scenario) << ','
            << s.timeSeconds << ','
            << s.targetAltitude << ','
            << s.measuredAltitude << ','
            << s.trueAltitude << ','
            << s.verticalVelocity << ','
            << s.error << ','
            << s.controlCommand << ','
            << s.actualThrust << ','
            << s.disturbance << '\n';
    }

    return true;
}
